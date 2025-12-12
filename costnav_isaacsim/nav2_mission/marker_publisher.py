# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""RViz Marker Publisher for Nav2 Mission Visualization.

This module provides a ROS2 node that publishes visualization markers for:
- Start position (green sphere/arrow)
- Goal position (red sphere/arrow)
- Current robot position (blue arrow, real-time updates)

Topics:
    /start_marker (visualization_msgs/Marker): Start position marker
    /goal_marker (visualization_msgs/Marker): Goal position marker
    /robot_marker (visualization_msgs/Marker): Current robot position marker
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

if TYPE_CHECKING:
    from .navmesh_sampler import SampledPosition


@dataclass
class MarkerConfig:
    """Configuration for a visualization marker."""

    r: float
    g: float
    b: float
    a: float = 1.0
    scale_x: float = 0.5
    scale_y: float = 0.5
    scale_z: float = 0.5
    marker_type: int = Marker.SPHERE
    frame_id: str = "map"


# Predefined marker configurations
START_MARKER_CONFIG = MarkerConfig(
    r=0.0, g=1.0, b=0.0, marker_type=Marker.ARROW, scale_x=0.8, scale_y=0.15, scale_z=0.15
)
GOAL_MARKER_CONFIG = MarkerConfig(
    r=1.0, g=0.0, b=0.0, marker_type=Marker.ARROW, scale_x=0.8, scale_y=0.15, scale_z=0.15
)
ROBOT_MARKER_CONFIG = MarkerConfig(
    r=0.0, g=0.0, b=1.0, marker_type=Marker.ARROW, scale_x=0.6, scale_y=0.1, scale_z=0.1
)


class MarkerPublisher(Node):
    """ROS2 Node for publishing Nav2 mission visualization markers.

    This node publishes markers for start, goal, and robot positions to RViz2.
    It subscribes to odometry for real-time robot position updates.

    Example usage:
        rclpy.init()
        marker_pub = MarkerPublisher()

        # Publish start and goal markers
        marker_pub.publish_start_marker(x=0.0, y=0.0, z=0.0, heading=0.0)
        marker_pub.publish_goal_marker(x=10.0, y=5.0, z=0.0, heading=0.5)

        # Robot marker is updated automatically from /odom
        rclpy.spin(marker_pub)
    """

    def __init__(self, node_name: str = "nav2_marker_publisher"):
        """Initialize the marker publisher node.

        Args:
            node_name: Name of the ROS2 node.
        """
        super().__init__(node_name)

        # QoS profile for markers (latched/transient local for persistence)
        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers
        self.start_marker_pub = self.create_publisher(Marker, "/start_marker", marker_qos)
        self.goal_marker_pub = self.create_publisher(Marker, "/goal_marker", marker_qos)
        self.robot_marker_pub = self.create_publisher(Marker, "/robot_marker", 10)

        # Subscriber for robot odometry
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

        # Timer for robot marker updates (10 Hz)
        self._robot_pose: Optional[tuple] = None
        self.robot_marker_timer = self.create_timer(0.1, self._publish_robot_marker)

        # Marker IDs
        self._start_marker_id = 0
        self._goal_marker_id = 1
        self._robot_marker_id = 2

        self.get_logger().info("MarkerPublisher initialized.")

    def _create_marker(
        self,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        heading: float,
        config: MarkerConfig,
        label: str = "",
    ) -> Marker:
        """Create a visualization marker.

        Args:
            marker_id: Unique marker ID.
            x, y, z: Position coordinates.
            heading: Orientation in radians (yaw).
            config: Marker configuration.
            label: Optional text label.

        Returns:
            Configured Marker message.
        """
        marker = Marker()
        marker.header.frame_id = config.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nav2_mission"
        marker.id = marker_id
        marker.type = config.marker_type
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation (quaternion from yaw)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(heading / 2.0)
        marker.pose.orientation.w = math.cos(heading / 2.0)

        # Scale
        marker.scale.x = config.scale_x
        marker.scale.y = config.scale_y
        marker.scale.z = config.scale_z

        # Color
        marker.color = ColorRGBA(r=config.r, g=config.g, b=config.b, a=config.a)

        # Lifetime (0 = forever for start/goal, short for robot)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    def publish_start_marker(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        heading: float = 0.0,
    ) -> None:
        """Publish the start position marker.

        Args:
            x, y, z: Start position coordinates.
            heading: Start orientation in radians.
        """
        marker = self._create_marker(
            marker_id=self._start_marker_id,
            x=x,
            y=y,
            z=z,
            heading=heading,
            config=START_MARKER_CONFIG,
            label="Start",
        )
        self.start_marker_pub.publish(marker)
        self.get_logger().info(f"Published start marker at ({x:.2f}, {y:.2f}, {z:.2f})")

    def publish_goal_marker(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        heading: float = 0.0,
    ) -> None:
        """Publish the goal position marker.

        Args:
            x, y, z: Goal position coordinates.
            heading: Goal orientation in radians.
        """
        marker = self._create_marker(
            marker_id=self._goal_marker_id,
            x=x,
            y=y,
            z=z,
            heading=heading,
            config=GOAL_MARKER_CONFIG,
            label="Goal",
        )
        self.goal_marker_pub.publish(marker)
        self.get_logger().info(f"Published goal marker at ({x:.2f}, {y:.2f}, {z:.2f})")

    def publish_start_goal_from_sampled(
        self,
        start: "SampledPosition",
        goal: "SampledPosition",
    ) -> None:
        """Publish start and goal markers from SampledPosition objects.

        Args:
            start: Start position from NavMeshSampler.
            goal: Goal position from NavMeshSampler.
        """
        self.publish_start_marker(start.x, start.y, start.z, start.heading)
        self.publish_goal_marker(goal.x, goal.y, goal.z, goal.heading)

    def _odom_callback(self, msg: Odometry) -> None:
        """Handle odometry messages for robot position tracking."""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        # Extract yaw from quaternion
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._robot_pose = (pos.x, pos.y, pos.z, yaw)

    def _publish_robot_marker(self) -> None:
        """Publish robot position marker (called by timer)."""
        if self._robot_pose is None:
            return

        x, y, z, heading = self._robot_pose
        marker = self._create_marker(
            marker_id=self._robot_marker_id,
            x=x,
            y=y,
            z=z,
            heading=heading,
            config=ROBOT_MARKER_CONFIG,
            label="Robot",
        )
        # Short lifetime for robot marker (updates frequently)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200_000_000  # 200ms

        self.robot_marker_pub.publish(marker)

    def clear_markers(self) -> None:
        """Clear all published markers."""
        for marker_id, publisher in [
            (self._start_marker_id, self.start_marker_pub),
            (self._goal_marker_id, self.goal_marker_pub),
            (self._robot_marker_id, self.robot_marker_pub),
        ]:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nav2_mission"
            marker.id = marker_id
            marker.action = Marker.DELETE
            publisher.publish(marker)

        self.get_logger().info("Cleared all markers.")


def main(args=None):
    """Main entry point for standalone marker publisher node."""
    rclpy.init(args=args)
    node = MarkerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
