#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""ViNT ROS2 Policy Node for CostNav.

This node subscribes to camera images, runs ViNT inference,
and publishes trajectory to /vint_trajectory for the trajectory follower node.

Supports three navigation modes (selected via ``goal_type``):
1. **no_goal** (default) — exploration without a goal image.
2. **image_goal** — navigate toward a single goal image.
3. **topomap** — follow a topological map generated
   online in simulation by the NavMesh-to-Topomap pipeline
   (see ``TOPOMAP_PIPELINE.md``).

Topomap Navigation
------------------
The topomap is a directory of sequentially numbered PNGs (``0.png``, ``1.png``,
…, ``N.png``) produced by ``TopomapGenerator``.  At each inference step the node:

1. Selects a local window of subgoal images around ``closest_node``
   (controlled by ``--topomap_radius``).
2. Runs batched ViNT inference to predict distances from the current
   observation to each subgoal in the window.
3. Localises to the closest node (``argmin`` of predicted distances).
4. If the distance to the closest node is below ``--topomap_close_threshold``,
   advances ``closest_node`` by one.
5. Publishes the trajectory toward the chosen subgoal.
6. Publishes ``reached_goal=True`` on ``/vint_reached_goal`` when
   ``closest_node == goal_node``.

This mirrors the navigation loop in ViNT's ``navigate.py`` but runs inside
the CostNav ROS 2 stack.

For ImageGoal mode, the node receives goal images via:
1. /goal_image topic - Published by the mission manager when a new mission starts
2. /set_goal_image service - Service to set the goal image programmatically

Following the NavDP pattern, when a new goal image is received:
- The agent's memory queue is reset (new mission = fresh start)
- The new goal image is stored and used for subsequent inference

Usage:
    # Default (Get goal type from model config)
    python3 vint_policy_node.py \\
        --checkpoint /path/to/model.pth \\
        --model_config /path/to/config.yaml \\
        --robot_config /path/to/robot.yaml

    # Topomap mode (--goal_type overrides goal_type from vint_eval.yaml)
    python3 vint_policy_node.py \\
        --checkpoint /path/to/model.pth \\
        --model_config /path/to/config.yaml \\
        --robot_config /path/to/robot.yaml \\
        --goal_type topomap --topomap_dir /tmp/costnav_topomap
"""

import argparse
import os
from typing import List, Optional

from PIL import Image as PILImage

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult, quat2mat

from il_evaluation.agents.vint_agent import ViNTAgent


class ViNTPolicyNode(Node):
    """ROS2 Node for ViNT policy inference.

    Supports three navigation modes (mutually exclusive, selected via ``goal_type``):

    * **no_goal** — exploration without a goal.
    * **image_goal** — navigate toward a single goal image received on a topic.
    * **topomap** — follow a topological map (directory of numbered PNGs).
      The topomap is generated online in simulation by ``TopomapGenerator``
      from the NavMesh-to-Topomap pipeline.

    Subscribes to:
        - /front_stereo_camera/left/image_raw (sensor_msgs/Image)
        - /goal_image (sensor_msgs/Image) - for image_goal mode (latched/transient local QoS)
        - /vint_enable (std_msgs/Bool) - enable/disable policy execution

    Services:
        - /vint_reset_agent (std_srvs/Trigger) - reset agent memory queue for new mission

    Publishes:
        - /vint_trajectory (nav_msgs/Path) - full trajectory for trajectory follower node
        - /vint_reached_goal (std_msgs/Bool) - True when topomap goal node is reached

    CLI Parameters:
        - checkpoint: Path to trained model weights
        - model_config: Path to model configuration YAML (inference params)
        - robot_config: Path to robot configuration YAML (topics)
        - goal_type: Override navigation mode ("no_goal", "image_goal", "topomap")
        - topomap_dir: Directory containing topomap images

    From model_config (vint_eval.yaml):
        - inference_rate, device, goal_type, visualize_debug_images
        - topomap_goal_node, topomap_radius, topomap_close_threshold

    From robot_config (robot_*.yaml):
        - image topic, goal_image topic
    """

    #: Valid navigation mode values for ``goal_type``.
    VALID_GOAL_TYPES = ("no_goal", "image_goal", "topomap")

    def __init__(
        self,
        checkpoint: str,
        model_config: str,
        robot_config: str,
        goal_type: Optional[str] = None,
        topomap_dir: str = "",
    ):
        super().__init__("vint_policy_node")

        # Validate file paths
        if not checkpoint or not os.path.exists(checkpoint):
            self.get_logger().error(f"Checkpoint not found: {checkpoint}")
            raise ValueError("Invalid checkpoint path")
        if not model_config or not os.path.exists(model_config):
            self.get_logger().error(f"Model config not found: {model_config}")
            raise ValueError("Invalid model config path")
        if not robot_config or not os.path.exists(robot_config):
            self.get_logger().error(f"Robot config not found: {robot_config}")
            raise ValueError("Invalid robot config path")

        # Load model config — inference parameters live here
        import yaml

        with open(model_config, "r") as f:
            model_cfg = yaml.safe_load(f)
        with open(robot_config, "r") as f:
            robot_cfg = yaml.safe_load(f)

        # Parameters from model config
        self.inference_rate = model_cfg.get("inference_rate", 4.0)
        device = model_cfg.get("device", "cuda:0")
        self.visualize_debug_images = model_cfg.get("visualize_debug_images", False)
        topomap_goal_node = model_cfg.get("topomap_goal_node", -1)
        topomap_radius = model_cfg.get("topomap_radius", 4)
        topomap_close_threshold = model_cfg.get("topomap_close_threshold", 3.0)

        # Navigation mode: CLI --goal_type overrides config goal_type
        if goal_type is not None:
            self.goal_type = goal_type
        else:
            self.goal_type = model_cfg.get("goal_type", "no_goal")
        if self.goal_type not in self.VALID_GOAL_TYPES:
            raise ValueError(f"Invalid goal_type '{self.goal_type}'. Must be one of {self.VALID_GOAL_TYPES}")

        # Parameters from robot config
        topics = robot_cfg.get("topics", {})
        image_topic = topics.get("image", "/front_stereo_camera/left/image_raw")
        self.goal_image_topic = topics.get("goal_image", "/goal_image")

        # Initialize ViNT agent
        self.get_logger().info(f"Loading ViNT model from {checkpoint}")
        self.agent = ViNTAgent(
            model_path=checkpoint,
            model_config_path=model_config,
            device=device,
        )
        self.agent.reset(batch_size=1)
        self.get_logger().info("ViNT agent initialized successfully")

        # CV bridge for image conversion
        self.bridge = CvBridge()

        # State variables
        self.current_image: Optional[np.ndarray] = None
        self.goal_image: Optional[np.ndarray] = None
        self.enabled = True  # Control enable flag
        self._goal_image_received = False  # Track if goal image was received for current mission

        # Topomap state
        self.topomap: List[PILImage.Image] = []
        self._topomap_closest_node: int = 0
        self._topomap_goal_node: int = 0
        self._topomap_radius: int = topomap_radius
        self._topomap_close_threshold: float = topomap_close_threshold
        self._topomap_reached_goal: bool = False

        # Store topomap parameters for deferred loading.
        # The topomap is generated by the Isaac Sim mission manager at mission
        # start, so it won't exist yet when this node initialises.  Loading is
        # triggered later via the /vint_load_topomap service.
        self._topomap_dir: str = topomap_dir
        self._topomap_goal_node_param: int = topomap_goal_node

        # Logging counters for interval-based logging
        self._trajectory_publish_count = 0
        self._log_interval = 10  # Log every N trajectory publishes

        # QoS profile for sensor data (camera images)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS profile for goal image with TRANSIENT_LOCAL durability
        # This ensures the subscriber receives the last published goal image
        # even if it was published before the subscription was created
        goal_image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, sensor_qos)

        if self.goal_type == "image_goal":
            # Subscribe to goal image with transient local durability for reliable delivery
            self.goal_sub = self.create_subscription(
                Image, self.goal_image_topic, self.goal_image_callback, goal_image_qos
            )
            self.get_logger().info(f"image_goal mode enabled. Subscribing to: {self.goal_image_topic}")

        # Enable/disable subscription
        self.enable_sub = self.create_subscription(Bool, "/vint_enable", self.enable_callback, 10)

        # Services for mission control
        self._reset_service = self.create_service(Trigger, "/vint_reset_agent", self._handle_reset_agent)

        # Topomap (re)load service — called by the mission manager after it
        # generates a fresh topomap on each mission start.
        if self.goal_type == "topomap":
            self._load_topomap_service = self.create_service(Trigger, "/vint_load_topomap", self._handle_load_topomap)

        # TF2 buffer and listener for base_link -> map transform (RViz visualization)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.trajectory_pub = self.create_publisher(Path, "/vint_trajectory", 10)
        self.trajectory_map_pub = self.create_publisher(Path, "/vint_trajectory_map", 10)
        self.reached_goal_pub = self.create_publisher(Bool, "/vint_reached_goal", 10)

        # Debug publishers (all gated by the single visualize_debug_images flag)
        self._goal_image_pub = None
        self._localization_image_pub = None
        if self.visualize_debug_images:
            self._goal_image_pub = self.create_publisher(Image, "/vint_goal_image", 10)
            self._localization_image_pub = self.create_publisher(Image, "/vint_localization_image", 10)
            self.get_logger().info(
                "Debug image visualization enabled. Publishing to: /vint_goal_image, /vint_localization_image"
            )

        # Inference timer
        timer_period = 1.0 / self.inference_rate
        self.timer = self.create_timer(timer_period, self.inference_callback)

        self.get_logger().info(
            f"ViNT policy node started (goal_type={self.goal_type}). Inference rate: {self.inference_rate} Hz"
        )
        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info("Publishing trajectory to: /vint_trajectory")
        if self.goal_type == "topomap":
            self.get_logger().info(
                f"Topomap mode enabled. Waiting for /vint_load_topomap service call (topomap_dir={self._topomap_dir})"
            )
        elif self.goal_type == "image_goal":
            self.get_logger().info("Waiting for goal image to start image-goal navigation...")

    # ------------------------------------------------------------------
    # Topomap helpers
    # ------------------------------------------------------------------

    def _load_topomap(self, topomap_dir: str, goal_node: int) -> None:
        """Load a topomap from a directory of numbered PNG images.

        Images are sorted by their integer filename prefix (``0.png``,
        ``1.png``, …) which matches the format produced by
        ``TopomapGenerator`` and expected by ViNT's ``navigate.py``.

        Args:
            topomap_dir: Path to the directory containing topomap images.
            goal_node: Goal node index.  ``-1`` means the last node.

        Raises:
            ValueError: If the directory is empty or does not exist.
        """
        if not topomap_dir or not os.path.isdir(topomap_dir):
            raise ValueError(f"Topomap directory not found: {topomap_dir}")

        filenames = sorted(
            os.listdir(topomap_dir),
            key=lambda x: int(x.split(".")[0]),
        )
        if not filenames:
            raise ValueError(f"Topomap directory is empty: {topomap_dir}")

        self.topomap = []
        for fname in filenames:
            img_path = os.path.join(topomap_dir, fname)
            self.topomap.append(PILImage.open(img_path).convert("RGB"))

        num_nodes = len(self.topomap)
        if goal_node == -1:
            self._topomap_goal_node = num_nodes - 1
        else:
            assert 0 <= goal_node < num_nodes, f"Invalid goal_node {goal_node} for topomap with {num_nodes} nodes"
            self._topomap_goal_node = goal_node

        self._topomap_closest_node = 0
        self._topomap_reached_goal = False

        self.get_logger().info(
            f"Loaded topomap from {topomap_dir}: {num_nodes} nodes, goal_node={self._topomap_goal_node}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def image_callback(self, msg: Image):
        """Process incoming camera image."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def goal_image_callback(self, msg: Image):
        """Process incoming goal image (for ImageGoal mode).

        When a new goal image is received, this indicates the start of a new mission.
        Following the NavDP pattern, we reset the agent's memory queue to ensure
        a fresh start for the new navigation task.

        The goal image is published by the mission manager (or goal image publisher)
        at the beginning of each mission.
        """
        try:
            new_goal_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # Reset agent memory queue for new mission (following NavDP pattern)
            # This ensures the agent starts fresh with new goal context
            self.agent.reset(batch_size=1)
            self.get_logger().info("Agent memory queue reset for new mission")

            # Store the new goal image
            self.goal_image = new_goal_image
            self._goal_image_received = True

            self.get_logger().info(
                f"Received new goal image (shape: {self.goal_image.shape}). Starting image-goal navigation."
            )

            # Publish received goal image for debugging visualization
            if self._goal_image_pub is not None:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(new_goal_image, "rgb8")
                    debug_msg.header.stamp = self.get_clock().now().to_msg()
                    debug_msg.header.frame_id = "goal_image"
                    self._goal_image_pub.publish(debug_msg)
                    self.get_logger().info("Published received goal image to /vint_goal_image")
                except Exception as pub_err:
                    self.get_logger().warn(f"Failed to publish debug goal image: {pub_err}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert goal image: {e}")

    def _handle_reset_agent(self, _request, response):
        """Handle agent reset service request.

        This service can be called by the mission manager to reset the agent's
        memory queue without providing a new goal image. Useful for debugging
        or when the goal image will be sent separately.

        Returns:
            Trigger response with success status and message.
        """
        try:
            self.agent.reset(batch_size=1)
            self.goal_image = None
            self._goal_image_received = False
            # Reset topomap navigation state
            self._topomap_closest_node = 0
            self._topomap_reached_goal = False
            response.success = True
            response.message = "ViNT agent memory queue reset successfully"
            self.get_logger().info("Agent reset via service call")
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset agent: {e}"
            self.get_logger().error(f"Agent reset failed: {e}")
        return response

    def _handle_load_topomap(self, _request, response):
        """Handle topomap (re)load service request.

        Called by the mission manager after it generates a fresh topomap for
        the current mission.  This clears the previous topomap and loads the
        new images from ``self._topomap_dir``, then resets the agent memory
        queue so navigation starts fresh.

        Returns:
            Trigger response with success status and message.
        """
        try:
            self._load_topomap(self._topomap_dir, self._topomap_goal_node_param)
            # Reset agent memory for the new mission
            self.agent.reset(batch_size=1)
            response.success = True
            response.message = f"Topomap loaded: {len(self.topomap)} nodes from {self._topomap_dir}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to load topomap: {e}"
            self.get_logger().error(response.message)
        return response

    def enable_callback(self, msg: Bool):
        """Enable/disable policy execution.

        In topomap mode, transitioning from disabled → enabled triggers a
        topomap reload.  The mission manager publishes ``True`` on
        ``/vint_enable`` at the start of every mission (after generating a
        fresh topomap), so this is the reliable per-mission reload hook.
        """
        was_enabled = self.enabled
        self.enabled = msg.data
        status = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"ViNT policy {status}")

        # Reload topomap on every disabled → enabled transition
        if self.goal_type == "topomap" and self.enabled and not was_enabled:
            try:
                self._load_topomap(self._topomap_dir, self._topomap_goal_node_param)
                self.agent.reset(batch_size=1)
                self.get_logger().info(
                    f"Topomap reloaded on enable: {len(self.topomap)} nodes from {self._topomap_dir}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to reload topomap on enable: {e}")

    def inference_callback(self):
        """Run ViNT inference and publish trajectory."""
        if not self.enabled:
            return

        if self.current_image is None:
            return

        # In image_goal mode, wait for goal image before starting inference
        if self.goal_type == "image_goal" and self.goal_image is None:
            return

        # In topomap mode, wait until the topomap has been loaded via service
        if self.goal_type == "topomap" and not self.topomap:
            return

        try:
            if self.goal_type == "topomap":
                self._inference_topomap()
            elif self.goal_type == "image_goal" and self.goal_image is not None:
                self._inference_imagegoal()
            else:
                self._inference_nogoal()
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")

    # ------------------------------------------------------------------
    # Per-mode inference helpers
    # ------------------------------------------------------------------

    def _inference_imagegoal(self) -> None:
        """ImageGoal mode: navigate toward a single goal image."""
        _, trajectory, distance = self.agent.step_imagegoal([self.goal_image], [self.current_image])
        self._publish_trajectory(trajectory[0], distance=float(distance[0]))

        # Publish goal image for visualization
        if self._goal_image_pub is not None:
            self._publish_debug_image(self._goal_image_pub, self.goal_image, "goal_image")

    def _inference_nogoal(self) -> None:
        """NoGoal mode: exploration without a goal."""
        _, trajectory = self.agent.step_nogoal([self.current_image])
        self._publish_trajectory(trajectory[0])

    def _inference_topomap(self) -> None:
        """Topomap mode: follow a topological map.

        Implements the navigation loop from ViNT's ``navigate.py``:
        1. Select a local window of subgoal images around ``closest_node``.
        2. Run batched inference to predict distances to each subgoal.
        3. Localise to the closest node and optionally advance.
        4. Publish the trajectory toward the chosen subgoal.
        5. Publish ``reached_goal`` status.
        """
        if self._topomap_reached_goal:
            # Already reached goal — keep publishing reached_goal=True
            goal_msg = Bool()
            goal_msg.data = True
            self.reached_goal_pub.publish(goal_msg)
            return

        # Build local window around closest_node
        start = max(self._topomap_closest_node - self._topomap_radius, 0)
        end = min(
            self._topomap_closest_node + self._topomap_radius + 1,
            self._topomap_goal_node,
        )
        subgoal_images = self.topomap[start : end + 1]

        if not subgoal_images:
            return

        # Batched inference against the subgoal window
        _, trajectory, distances_np, chosen_idx = self.agent.step_topomap(
            [self.current_image],
            subgoal_images,
            close_threshold=self._topomap_close_threshold,
        )

        # Update closest_node in global topomap coordinates
        min_dist_idx = int(np.argmin(distances_np))
        min_dist = float(distances_np[min_dist_idx])
        if min_dist > self._topomap_close_threshold:
            action = "track"
            self._topomap_closest_node = start + min_dist_idx
        else:
            action = "advance"
            self._topomap_closest_node = min(start + min_dist_idx + 1, self._topomap_goal_node)

        # Enhanced logging for topomap inference
        if self._trajectory_publish_count % self._log_interval == 0:
            dists_str = ", ".join(f"{d:.2f}" for d in distances_np)
            self.get_logger().info(
                f"[Topomap] window=[{start}:{end}] ({len(subgoal_images)} nodes), "
                f"distances=[{dists_str}], "
                f"min_dist={min_dist:.2f} @local={min_dist_idx} "
                f"(global={start + min_dist_idx}), "
                f"chosen_idx={chosen_idx} (global={start + chosen_idx}), "
                f"action={action}, "
                f"closest_node={self._topomap_closest_node}/{self._topomap_goal_node}"
            )

        # Check if goal reached
        self._topomap_reached_goal = self._topomap_closest_node == self._topomap_goal_node

        # Publish trajectory
        self._publish_trajectory(
            trajectory[0],
            distance=float(distances_np[chosen_idx]),
            closest_node=self._topomap_closest_node,
        )

        # Publish reached_goal status
        goal_msg = Bool()
        goal_msg.data = self._topomap_reached_goal
        self.reached_goal_pub.publish(goal_msg)

        if self._topomap_reached_goal:
            self.get_logger().info("Topomap goal reached!")

        # Publish goal (chosen subgoal) and localization (closest node) images
        if self._goal_image_pub is not None:
            chosen_global_idx = start + chosen_idx
            if 0 <= chosen_global_idx < len(self.topomap):
                goal_img_np = np.array(self.topomap[chosen_global_idx])
                self._publish_debug_image(self._goal_image_pub, goal_img_np, "goal_image")
        if self._localization_image_pub is not None:
            closest_node = self._topomap_closest_node
            if 0 <= closest_node < len(self.topomap):
                loc_img_np = np.array(self.topomap[closest_node])
                self._publish_debug_image(self._localization_image_pub, loc_img_np, "localization_image")

    def _publish_debug_image(self, publisher, image_np: np.ndarray, frame_id: str) -> None:
        """Publish a debug image on the given publisher.

        Args:
            publisher: ROS2 image publisher.
            image_np: RGB numpy image (H, W, 3).
            frame_id: Frame ID string for the image header.
        """
        try:
            msg = self.bridge.cv2_to_imgmsg(image_np, "rgb8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image ({frame_id}): {e}")

    def _publish_trajectory(
        self,
        trajectory,
        distance: Optional[float] = None,
        closest_node: Optional[int] = None,
    ) -> None:
        """Publish trajectory and log at interval."""
        trajectory_msg = self.trajectory_to_path(trajectory)
        self.trajectory_pub.publish(trajectory_msg)

        # Also publish in map frame for RViz visualization
        self._publish_trajectory_map(trajectory_msg)

        self._trajectory_publish_count += 1
        if self._trajectory_publish_count % self._log_interval == 0:
            traj = trajectory
            if hasattr(traj, "cpu"):
                traj = traj.cpu().numpy()
            traj_len = len(traj)
            last_wp = traj[-1] if traj_len > 0 else [0, 0]

            extra = ""
            if distance is not None:
                extra += f", dist={distance:.2f}"
            if closest_node is not None:
                extra += f", node={closest_node}/{self._topomap_goal_node}"
            self.get_logger().info(
                f"[ViNT] #{self._trajectory_publish_count}: traj_last=({last_wp[0]:.2f}, {last_wp[1]:.2f}){extra}"
            )

    def trajectory_to_path(self, trajectory: np.ndarray) -> Path:
        """Convert ViNT trajectory output to nav_msgs/Path message.

        The trajectory is in robot-local frame (x forward, y left). This is
        published for the trajectory follower node to execute at higher rate.

        Args:
            trajectory: Smoothed trajectory [M, 3] (x, y, theta) in local frame.

        Returns:
            Path message with waypoints in robot-local frame.
        """
        # Convert from torch tensor to numpy if needed
        if hasattr(trajectory, "cpu"):
            trajectory = trajectory.cpu().numpy()

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "base_link"  # Local frame

        for i in range(len(trajectory)):
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(trajectory[i, 0])
            pose_stamped.pose.position.y = float(trajectory[i, 1])
            pose_stamped.pose.position.z = 0.0

            # Set orientation from theta (yaw) using transforms3d
            # euler2quat returns (w, x, y, z) for given (ai, aj, ak) = (roll, pitch, yaw)
            if trajectory.shape[1] > 2:
                theta = float(trajectory[i, 2])
                quat = euler2quat(0, 0, theta)  # roll=0, pitch=0, yaw=theta
                pose_stamped.pose.orientation.w = quat[0]
                pose_stamped.pose.orientation.x = quat[1]
                pose_stamped.pose.orientation.y = quat[2]
                pose_stamped.pose.orientation.z = quat[3]
            else:
                pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        return path_msg

    def _publish_trajectory_map(self, base_link_path: Path) -> None:
        """Publish trajectory transformed to map frame for RViz visualization.

        Looks up the latest base_link -> map transform via TF2 and applies it
        to every pose in *base_link_path*.  Published on ``/vint_trajectory_map``.
        """
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        tf_quat = [q.w, q.x, q.y, q.z]
        rot_matrix = quat2mat(tf_quat)

        map_path = Path()
        map_path.header.stamp = base_link_path.header.stamp
        map_path.header.frame_id = "map"

        for pose_stamped in base_link_path.poses:
            p = pose_stamped.pose.position
            rotated = rot_matrix @ np.array([p.x, p.y, 0.0])

            map_pose = PoseStamped()
            map_pose.header = map_path.header
            map_pose.pose.position.x = t.x + rotated[0]
            map_pose.pose.position.y = t.y + rotated[1]
            map_pose.pose.position.z = 0.0

            # Compose orientations: q_map = q_tf * q_local
            o = pose_stamped.pose.orientation
            local_quat = [o.w, o.x, o.y, o.z]
            map_quat = qmult(tf_quat, local_quat)
            map_pose.pose.orientation.w = map_quat[0]
            map_pose.pose.orientation.x = map_quat[1]
            map_pose.pose.orientation.y = map_quat[2]
            map_pose.pose.orientation.z = map_quat[3]

            map_path.poses.append(map_pose)

        self.trajectory_map_pub.publish(map_path)


def parse_args():
    """Parse command line arguments.

    Most parameters are read from model_config (vint_eval.yaml) and
    robot_config (robot_*.yaml).  Only the file paths, optional goal_type
    override, topomap_dir, and log level remain as CLI arguments.
    """
    parser = argparse.ArgumentParser(description="ViNT ROS2 Policy Node for CostNav")
    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="Path to trained model weights (.pth file)",
    )
    parser.add_argument(
        "--model_config",
        type=str,
        required=True,
        help="Path to model configuration YAML (contains inference params)",
    )
    parser.add_argument(
        "--robot_config",
        type=str,
        required=True,
        help="Path to robot configuration YAML (contains topics)",
    )
    parser.add_argument(
        "--goal_type",
        type=str,
        default=None,
        choices=ViNTPolicyNode.VALID_GOAL_TYPES,
        help="Override navigation mode from vint_eval.yaml (no_goal | image_goal | topomap)",
    )
    parser.add_argument(
        "--topomap_dir",
        type=str,
        default="/tmp/costnav_topomap",
        help="Directory containing topomap images (default: /tmp/costnav_topomap)",
    )
    parser.add_argument(
        "--log_level",
        type=str,
        default="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        help="Log level (default: info)",
    )
    return parser.parse_args()


def main():
    """Main entry point for the ViNT policy node."""
    args = parse_args()

    rclpy.init()

    try:
        node = ViNTPolicyNode(
            checkpoint=args.checkpoint,
            model_config=args.model_config,
            robot_config=args.robot_config,
            goal_type=args.goal_type,
            topomap_dir=args.topomap_dir,
        )
        # Set log level
        log_level_map = {
            "debug": rclpy.logging.LoggingSeverity.DEBUG,
            "info": rclpy.logging.LoggingSeverity.INFO,
            "warn": rclpy.logging.LoggingSeverity.WARN,
            "error": rclpy.logging.LoggingSeverity.ERROR,
            "fatal": rclpy.logging.LoggingSeverity.FATAL,
        }
        node.get_logger().set_level(log_level_map[args.log_level])
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error starting ViNT policy node: {e}")
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
