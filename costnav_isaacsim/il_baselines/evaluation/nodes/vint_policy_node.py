#!/usr/bin/env python3
# Copyright (c) 2024 CostNav Authors
# Licensed under the MIT License

"""ViNT ROS2 Policy Node for CostNav.

This node subscribes to camera images and odometry, runs ViNT inference,
and publishes velocity commands to /cmd_vel_model which is picked up by
the teleop node when model control is enabled.

Usage:
    ros2 run costnav_il_baselines vint_policy_node \
        --ros-args \
        -p checkpoint:=/path/to/model.pth \
        -p model_config:=/path/to/config.yaml \
        -p robot_config:=/path/to/robot.yaml
"""

import os
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Import ViNT agent (relative import when used as package)
try:
    from costnav_isaacsim.il_baselines.evaluation.agents import ViNTAgent
except ImportError:
    from ..agents import ViNTAgent


class ViNTPolicyNode(Node):
    """ROS2 Node for ViNT policy inference.

    Subscribes to:
        - /front_stereo_camera/left/image_raw (sensor_msgs/Image)
        - /odom (nav_msgs/Odometry)
        - /goal_image (sensor_msgs/Image) - optional for ImageGoal mode

    Publishes:
        - /cmd_vel_model (geometry_msgs/Twist)

    Parameters:
        - checkpoint: Path to trained model weights
        - model_config: Path to model configuration YAML
        - robot_config: Path to robot configuration YAML
        - inference_rate: Inference frequency in Hz (default: 10.0)
        - image_topic: Camera image topic (default: /front_stereo_camera/left/image_raw)
        - use_imagegoal: Whether to use image goal navigation (default: False)
    """

    def __init__(self):
        super().__init__("vint_policy_node")

        # Declare parameters
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("model_config", "")
        self.declare_parameter("robot_config", "")
        self.declare_parameter("inference_rate", 10.0)
        self.declare_parameter("image_topic", "/front_stereo_camera/left/image_raw")
        self.declare_parameter("use_imagegoal", False)
        self.declare_parameter("device", "cuda:0")

        # Get parameters
        checkpoint = self.get_parameter("checkpoint").value
        model_config = self.get_parameter("model_config").value
        robot_config = self.get_parameter("robot_config").value
        self.inference_rate = self.get_parameter("inference_rate").value
        image_topic = self.get_parameter("image_topic").value
        self.use_imagegoal = self.get_parameter("use_imagegoal").value
        device = self.get_parameter("device").value

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
        self.current_odom: Optional[Odometry] = None
        self.enabled = True  # Control enable flag

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, sensor_qos)

        if self.use_imagegoal:
            self.goal_sub = self.create_subscription(Image, "/goal_image", self.goal_image_callback, 10)

        # Enable/disable subscription
        self.enable_sub = self.create_subscription(Bool, "/vint_enable", self.enable_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_model", 10)

        # Inference timer
        timer_period = 1.0 / self.inference_rate
        self.timer = self.create_timer(timer_period, self.inference_callback)

        self.get_logger().info(f"ViNT policy node started. Inference rate: {self.inference_rate} Hz")
        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info("Publishing to: /cmd_vel_model")

    def image_callback(self, msg: Image):
        """Process incoming camera image."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def goal_image_callback(self, msg: Image):
        """Process incoming goal image (for ImageGoal mode)."""
        try:
            self.goal_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.get_logger().info("Received new goal image")
        except Exception as e:
            self.get_logger().error(f"Failed to convert goal image: {e}")

    def odom_callback(self, msg: Odometry):
        """Process incoming odometry."""
        self.current_odom = msg

    def enable_callback(self, msg: Bool):
        """Enable/disable policy execution."""
        self.enabled = msg.data
        status = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"ViNT policy {status}")

    def inference_callback(self):
        """Run ViNT inference and publish velocity command."""
        if not self.enabled:
            return

        if self.current_image is None:
            return

        try:
            # Run inference based on mode
            if self.use_imagegoal and self.goal_image is not None:
                waypoints, trajectory = self.agent.step_imagegoal([self.goal_image], [self.current_image])
            else:
                waypoints, trajectory = self.agent.step_nogoal([self.current_image])

            # Convert trajectory to velocity command
            cmd_vel = self.trajectory_to_twist(waypoints[0], trajectory[0])
            self.cmd_vel_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            # Publish zero velocity on error
            self.cmd_vel_pub.publish(Twist())

    def trajectory_to_twist(self, waypoints: np.ndarray, trajectory: np.ndarray) -> Twist:
        """Convert ViNT trajectory output to Twist velocity command.

        Uses the first waypoint to compute desired velocity. This is a simple
        proportional controller - can be replaced with MPC for better tracking.

        Args:
            waypoints: Predicted waypoints [N, 3] (x, y, theta).
            trajectory: Smoothed trajectory [M, 3].

        Returns:
            Twist message with linear and angular velocities.
        """
        twist = Twist()

        # Get first waypoint (closest target)
        if len(waypoints) == 0:
            return twist

        # Convert from torch tensor to numpy if needed
        if hasattr(waypoints, "cpu"):
            waypoints = waypoints.cpu().numpy()

        dx = float(waypoints[0, 0])  # Forward displacement
        dy = float(waypoints[0, 1])  # Lateral displacement

        # Compute angle to first waypoint
        target_angle = np.arctan2(dy, dx)

        # Proportional control gains
        KP_LINEAR = 2.0  # Linear velocity gain
        KP_ANGULAR = 1.5  # Angular velocity gain

        # Compute velocities
        distance = np.sqrt(dx**2 + dy**2)
        linear_vel = KP_LINEAR * distance
        angular_vel = KP_ANGULAR * target_angle

        # Apply velocity limits
        max_linear = self.agent.MAX_V
        max_angular = self.agent.MAX_W
        linear_vel = np.clip(linear_vel, -max_linear, max_linear)
        angular_vel = np.clip(angular_vel, -max_angular, max_angular)

        twist.linear.x = linear_vel
        twist.angular.z = angular_vel

        return twist


def main(args=None):
    """Main entry point for the ViNT policy node."""
    rclpy.init(args=args)

    try:
        node = ViNTPolicyNode()
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
