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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

if TYPE_CHECKING:
    from .navmesh_sampler import SampledPosition


@dataclass
class MarkerConfig:
    """Configuration for a visualization marker.

    Attributes:
        r: Red channel (0.0-1.0).
        g: Green channel (0.0-1.0).
        b: Blue channel (0.0-1.0).
        a: Alpha channel (0.0-1.0).
        scale_x: Length of the marker.
        scale_y: Width of the marker.
        scale_z: Height of the marker.
        marker_type: visualization_msgs/Marker type.
        frame_id: Frame for the marker.
        heading_offset: Extra yaw offset (radians).
        fixed_heading: Fixed yaw override (radians) when set.
        head_on_pose: Place arrow head at the pose.
    """

    r: float
    g: float
    b: float
    a: float = 1.0
    scale_x: float = 0.5
    scale_y: float = 0.5
    scale_z: float = 0.5
    marker_type: int = Marker.SPHERE
    frame_id: str = "map"
    heading_offset: float = 0.0
    fixed_heading: Optional[float] = None
    head_on_pose: bool = False


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

    def __init__(
        self,
        node_name: str = "nav2_marker_publisher",
        arrow_length: float = 0.8,
        arrow_width: float = 0.15,
        arrow_height: float = 0.15,
        start_topic: str = "/start_marker",
        goal_topic: str = "/goal_marker",
        robot_topic: str = "/robot_marker",
        odom_topic: str = "/chassis/odom",
        enabled: bool = True,
    ):
        """Initialize the marker publisher node.

        Args:
            node_name: Name of the ROS2 node.
            arrow_length: Length of arrow markers (scale_x).
            arrow_width: Width of arrow markers (scale_y).
            arrow_height: Height of arrow markers (scale_z).
            start_topic: Topic name for start position marker.
            goal_topic: Topic name for goal position marker.
            robot_topic: Topic name for robot position marker.
            odom_topic: Topic name for odometry subscription.
            enabled: Whether to enable marker publishing.
        """
        super().__init__(node_name)

        # Store marker configuration
        self._enabled = enabled
        self._arrow_length = arrow_length
        self._arrow_width = arrow_width
        self._arrow_height = arrow_height

        # QoS profile for markers (latched/transient local for persistence)
        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers (using topic names from config)
        self.start_marker_pub = self.create_publisher(Marker, start_topic, marker_qos)
        self.goal_marker_pub = self.create_publisher(Marker, goal_topic, marker_qos)
        self.robot_marker_pub = self.create_publisher(Marker, robot_topic, 10)

        # Subscriber for robot odometry (using topic name from config)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        # Timer for robot marker updates (10 Hz)
        self._robot_pose: Optional[tuple] = None
        self.robot_marker_timer = self.create_timer(0.1, self._publish_robot_marker)

        # Marker IDs
        self._start_marker_id = 0
        self._goal_marker_id = 1
        self._robot_marker_id = 2

        self.get_logger().info("MarkerPublisher initialized.")

    def _get_start_marker_config(self) -> MarkerConfig:
        """Get marker configuration for start position."""
        return MarkerConfig(
            r=0.0,
            g=1.0,
            b=0.0,
            a=0.5,
            marker_type=Marker.ARROW,
            scale_x=self._arrow_length,
            scale_y=self._arrow_width,
            scale_z=self._arrow_height,
            fixed_heading=math.pi,
            head_on_pose=True,
        )

    def _get_goal_marker_config(self) -> MarkerConfig:
        """Get marker configuration for goal position."""
        return MarkerConfig(
            r=1.0,
            g=0.0,
            b=0.0,
            a=0.5,
            marker_type=Marker.ARROW,
            scale_x=self._arrow_length,
            scale_y=self._arrow_width,
            scale_z=self._arrow_height,
            fixed_heading=math.pi,
            head_on_pose=True,
        )

    def _get_robot_marker_config(self) -> MarkerConfig:
        """Get marker configuration for robot position."""
        return MarkerConfig(
            r=0.0,
            g=0.0,
            b=1.0,
            a=0.5,
            marker_type=Marker.ARROW,
            scale_x=self._arrow_length * 0.75,  # Slightly smaller than start/goal
            scale_y=self._arrow_width * 0.67,
            scale_z=self._arrow_height * 0.67,
            heading_offset=math.pi,
            head_on_pose=True,
        )

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

        base_heading = config.fixed_heading if config.fixed_heading is not None else heading
        arrow_heading = base_heading + config.heading_offset

        # Position (ensure float type for ROS2 compatibility)
        if config.marker_type == Marker.ARROW and config.head_on_pose:
            offset = float(config.scale_x)
            marker.pose.position.x = float(x) - math.cos(arrow_heading) * offset
            marker.pose.position.y = float(y) - math.sin(arrow_heading) * offset
            marker.pose.position.z = float(z)
        else:
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)

        # Orientation (quaternion from yaw)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(arrow_heading / 2.0)
        marker.pose.orientation.w = math.cos(arrow_heading / 2.0)

        # Scale (ensure float type for ROS2 compatibility)
        marker.scale.x = float(config.scale_x)
        marker.scale.y = float(config.scale_y)
        marker.scale.z = float(config.scale_z)

        # Color (ensure float type for ROS2 compatibility)
        marker.color = ColorRGBA(r=float(config.r), g=float(config.g), b=float(config.b), a=float(config.a))

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
        if not self._enabled:
            return

        marker = self._create_marker(
            marker_id=self._start_marker_id,
            x=x,
            y=y,
            z=z,
            heading=heading,
            config=self._get_start_marker_config(),
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
        if not self._enabled:
            return

        marker = self._create_marker(
            marker_id=self._goal_marker_id,
            x=x,
            y=y,
            z=z,
            heading=heading,
            config=self._get_goal_marker_config(),
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
        if not self._enabled or self._robot_pose is None:
            return

        x, y, z, heading = self._robot_pose
        marker = self._create_marker(
            marker_id=self._robot_marker_id,
            x=x,
            y=y,
            z=z,
            heading=heading,
            config=self._get_robot_marker_config(),
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
