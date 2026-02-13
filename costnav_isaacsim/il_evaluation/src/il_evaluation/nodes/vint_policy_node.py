#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""ViNT ROS2 Policy Node for CostNav.

This node subscribes to camera images, runs ViNT inference,
and publishes trajectory to /vint_trajectory for the trajectory follower node.

For ImageGoal mode, the node receives goal images via:
1. /goal_image topic - Published by the mission manager when a new mission starts
2. /set_goal_image service - Service to set the goal image programmatically

Following the NavDP pattern, when a new goal image is received:
- The agent's memory queue is reset (new mission = fresh start)
- The new goal image is stored and used for subsequent inference

Usage:
    python3 vint_policy_node.py \
        --checkpoint /path/to/model.pth \
        --model_config /path/to/config.yaml \
        --robot_config /path/to/robot.yaml \
        --use_imagegoal
"""

import argparse
import os
from typing import Optional

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
from transforms3d.euler import euler2quat

from il_evaluation.agents.vint_agent import ViNTAgent


class ViNTPolicyNode(Node):
    """ROS2 Node for ViNT policy inference.

    Subscribes to:
        - /front_stereo_camera/left/image_raw (sensor_msgs/Image)
        - /goal_image (sensor_msgs/Image) - for ImageGoal mode (latched/transient local QoS)
        - /vint_enable (std_msgs/Bool) - enable/disable policy execution

    Services:
        - /set_goal_image (std_srvs/SetGoalImage) - set goal image programmatically
        - /reset_agent (std_srvs/Trigger) - reset agent memory queue for new mission

    Publishes:
        - /vint_trajectory (nav_msgs/Path) - full trajectory for trajectory follower node

    Parameters:
        - checkpoint: Path to trained model weights
        - model_config: Path to model configuration YAML
        - robot_config: Path to robot configuration YAML
        - inference_rate: Inference frequency in Hz (default: 10.0)
        - image_topic: Camera image topic (default: /front_stereo_camera/left/image_raw)
        - use_imagegoal: Whether to use image goal navigation (default: False)
    """

    def __init__(
        self,
        checkpoint: str,
        model_config: str,
        robot_config: str,
        inference_rate: float = 10.0,
        image_topic: str = "/front_stereo_camera/left/image_raw",
        use_imagegoal: bool = False,
        device: str = "cuda:0",
        goal_image_topic: str = "/goal_image",
        visualize_goal_image: bool = False,
    ):
        super().__init__("vint_policy_node")

        # Store parameters
        self.inference_rate = inference_rate
        self.use_imagegoal = use_imagegoal
        self.goal_image_topic = goal_image_topic
        self.visualize_goal_image = visualize_goal_image

        # Validate parameters
        if not checkpoint or not os.path.exists(checkpoint):
            self.get_logger().error(f"Checkpoint not found: {checkpoint}")
            raise ValueError("Invalid checkpoint path")
        if not model_config or not os.path.exists(model_config):
            self.get_logger().error(f"Model config not found: {model_config}")
            raise ValueError("Invalid model config path")
        if not robot_config or not os.path.exists(robot_config):
            self.get_logger().error(f"Robot config not found: {robot_config}")
            raise ValueError("Invalid robot config path")

        # Initialize ViNT agent
        self.get_logger().info(f"Loading ViNT model from {checkpoint}")
        self.agent = ViNTAgent(
            model_path=checkpoint,
            model_config_path=model_config,
            robot_config_path=robot_config,
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

        if self.use_imagegoal:
            # Subscribe to goal image with transient local durability for reliable delivery
            self.goal_sub = self.create_subscription(
                Image, self.goal_image_topic, self.goal_image_callback, goal_image_qos
            )
            self.get_logger().info(f"ImageGoal mode enabled. Subscribing to: {self.goal_image_topic}")

        # Enable/disable subscription
        self.enable_sub = self.create_subscription(Bool, "/vint_enable", self.enable_callback, 10)

        # Services for mission control
        self._reset_service = self.create_service(Trigger, "/vint_reset_agent", self._handle_reset_agent)

        # Publishers
        self.trajectory_pub = self.create_publisher(Path, "/vint_trajectory", 10)

        # Debug publisher for visualizing received goal image
        self._received_goal_image_pub = None
        if self.visualize_goal_image:
            self._received_goal_image_pub = self.create_publisher(Image, "/received_goal_image", 10)
            self.get_logger().info("Goal image visualization enabled. Publishing to: /received_goal_image")

        # Inference timer
        timer_period = 1.0 / self.inference_rate
        self.timer = self.create_timer(timer_period, self.inference_callback)

        self.get_logger().info(f"ViNT policy node started. Inference rate: {self.inference_rate} Hz")
        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info("Publishing trajectory to: /vint_trajectory")
        if self.use_imagegoal:
            self.get_logger().info("Waiting for goal image to start image-goal navigation...")

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
            if self._received_goal_image_pub is not None:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(new_goal_image, "rgb8")
                    debug_msg.header.stamp = self.get_clock().now().to_msg()
                    debug_msg.header.frame_id = "goal_image"
                    self._received_goal_image_pub.publish(debug_msg)
                    self.get_logger().info("Published received goal image to /received_goal_image")
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
            response.success = True
            response.message = "ViNT agent memory queue reset successfully"
            self.get_logger().info("Agent reset via service call")
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset agent: {e}"
            self.get_logger().error(f"Agent reset failed: {e}")
        return response

    def enable_callback(self, msg: Bool):
        """Enable/disable policy execution."""
        self.enabled = msg.data
        status = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"ViNT policy {status}")

    def inference_callback(self):
        """Run ViNT inference and publish trajectory."""
        if not self.enabled:
            return

        if self.current_image is None:
            return

        # In imagegoal mode, wait for goal image before starting inference
        if self.use_imagegoal and self.goal_image is None:
            # Don't log every iteration, just skip silently
            return

        try:
            # Run inference based on mode
            if self.use_imagegoal and self.goal_image is not None:
                _, trajectory, distance = self.agent.step_imagegoal([self.goal_image], [self.current_image])
            else:
                _, trajectory = self.agent.step_nogoal([self.current_image])
                distance = None

            # Publish full trajectory for trajectory follower node
            trajectory_msg = self.trajectory_to_path(trajectory[0])
            self.trajectory_pub.publish(trajectory_msg)

            # Log trajectory publish at interval
            self._trajectory_publish_count += 1
            if self._trajectory_publish_count % self._log_interval == 0:
                traj = trajectory[0]
                if hasattr(traj, "cpu"):
                    traj = traj.cpu().numpy()
                traj_len = len(traj)
                # Get last waypoint from smoothed trajectory
                last_wp = traj[-1] if traj_len > 0 else [0, 0]

                dist_str = f", dist={float(distance[0]):.2f}" if distance is not None else ""
                self.get_logger().info(
                    f"[ViNT] #{self._trajectory_publish_count}: "
                    f"traj_last=({last_wp[0]:.2f}, {last_wp[1]:.2f}){dist_str}"
                )

        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")

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


def parse_args():
    """Parse command line arguments."""
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
        help="Path to model configuration YAML",
    )
    parser.add_argument(
        "--robot_config",
        type=str,
        required=True,
        help="Path to robot configuration YAML",
    )
    parser.add_argument(
        "--inference_rate",
        type=float,
        default=10.0,
        help="Inference frequency in Hz (default: 10.0)",
    )
    parser.add_argument(
        "--image_topic",
        type=str,
        default="/front_stereo_camera/left/image_raw",
        help="Camera image topic (default: /front_stereo_camera/left/image_raw)",
    )
    parser.add_argument(
        "--use_imagegoal",
        action="store_true",
        help="Use image goal navigation mode",
    )
    parser.add_argument(
        "--goal_image_topic",
        type=str,
        default="/goal_image",
        help="Goal image topic for ImageGoal mode (default: /goal_image)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Device for inference (default: cuda:0)",
    )
    parser.add_argument(
        "--visualize_goal_image",
        action="store_true",
        help="Publish received goal image to /received_goal_image for debugging (default: False)",
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
            inference_rate=args.inference_rate,
            image_topic=args.image_topic,
            use_imagegoal=args.use_imagegoal,
            device=args.device,
            goal_image_topic=args.goal_image_topic,
            visualize_goal_image=args.visualize_goal_image,
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
