# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Mission Orchestrator for Nav2 Navigation Testing.

This module provides the main orchestration logic for Nav2 navigation missions:
1. Sample valid start/goal positions using NavMesh
2. Teleport robot to start position (Isaac Sim)
3. Set initial pose for AMCL localization
4. Publish navigation goal to Nav2
5. Visualize start, goal, and robot positions in RViz2

Usage:
    # From Isaac Sim environment
    from costnav_isaacsim.nav2_mission import MissionOrchestrator

    orchestrator = MissionOrchestrator()
    orchestrator.run_mission()
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .marker_publisher import MarkerPublisher
from .navmesh_sampler import NavMeshSampler, SampledPosition

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


@dataclass
class OrchestratorConfig:
    """Configuration for MissionOrchestrator runtime settings."""

    min_distance: float = 5.0  # Minimum distance between start and goal (meters)
    max_distance: float = 100.0  # Maximum distance between start and goal (meters)
    initial_pose_delay: float = 1.0  # Delay after setting initial pose (seconds)
    goal_delay: float = 0.5  # Delay after publishing goal (seconds)
    teleport_height: float = 0.1  # Height offset for teleportation (meters)
    robot_prim_path: Optional[str] = None  # Robot prim path in Isaac Sim USD stage
    teleport_settle_steps: int = 5  # Number of simulation steps after teleportation for physics to settle


class MissionOrchestrator(Node):
    """Orchestrates Nav2 navigation missions with NavMesh-based sampling.

    This class coordinates the full mission workflow:
    1. NavMesh position sampling with distance constraints
    2. Robot teleportation in Isaac Sim (auto-setup if robot_prim_path provided)
    3. AMCL initial pose publication
    4. Nav2 goal publication
    5. RViz marker visualization

    Example usage:
        import rclpy
        from nav2_mission import MissionOrchestrator, OrchestratorConfig

        rclpy.init()

        # Auto-setup Isaac Sim teleportation by providing robot_prim_path
        config = OrchestratorConfig(
            min_distance=5.0,
            max_distance=50.0,
            robot_prim_path="/World/Nova_Carter_ROS"
        )
        orchestrator = MissionOrchestrator(config=config)

        # Run missions
        for i in range(10):
            orchestrator.run_mission()
            time.sleep(30)  # Wait for navigation to complete

        orchestrator.destroy_node()
        rclpy.shutdown()
    """

    def __init__(
        self,
        node_name: str = "nav2_mission_orchestrator",
        config: Optional[OrchestratorConfig] = None,
        teleport_callback: Optional[Callable[[SampledPosition], bool]] = None,
        simulation_context: Optional[object] = None,
    ):
        """Initialize the mission orchestrator.

        Args:
            node_name: Name of the ROS2 node.
            config: Mission configuration parameters.
            teleport_callback: Optional callback for robot teleportation.
                              Signature: (position: SampledPosition) -> bool
                              If not provided and config.robot_prim_path is set,
                              will attempt to auto-setup Isaac Sim teleportation.
            simulation_context: Optional Isaac Sim SimulationContext for stepping
                              physics after teleportation. Required for proper
                              physics settling after teleportation.
        """
        super().__init__(node_name)

        self.config = config or OrchestratorConfig()
        self._teleport_callback = teleport_callback
        self._simulation_context = simulation_context

        # Initialize NavMesh sampler
        self.sampler = NavMeshSampler(
            min_distance=self.config.min_distance,
            max_distance=self.config.max_distance,
        )

        # Initialize marker publisher (as a separate node)
        self.marker_publisher = MarkerPublisher(node_name=f"{node_name}_markers")

        # Auto-setup Isaac Sim teleport callback if robot_prim_path is provided
        if self._teleport_callback is None and self.config.robot_prim_path:
            self._setup_isaac_sim_teleport()

        # QoS for pose topics
        pose_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", pose_qos)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", pose_qos)

        # Current mission state
        self._current_start: Optional[SampledPosition] = None
        self._current_goal: Optional[SampledPosition] = None
        self._mission_count = 0

        self.get_logger().info(
            f"MissionOrchestrator initialized: min_dist={self.config.min_distance}m, "
            f"max_dist={self.config.max_distance}m"
        )

    def set_teleport_callback(
        self,
        callback: Callable[[SampledPosition], bool],
    ) -> None:
        """Set the teleportation callback function.

        Args:
            callback: Function that teleports robot to given position.
                     Returns True on success, False on failure.
        """
        self._teleport_callback = callback
        self.get_logger().info("Teleport callback registered.")

    def _setup_isaac_sim_teleport(self) -> None:
        """Setup Isaac Sim teleportation using XFormPrim.

        This method uses XFormPrim for teleportation which provides:
        - Physics-aware positioning for articulated robots
        - Proper integration with Isaac Sim physics simulation

        If any step fails, logs a warning and continues without teleportation.
        """
        if not self.config.robot_prim_path:
            return

        robot_prim_path = self.config.robot_prim_path
        self.get_logger().info(f"Setting up Isaac Sim teleport for robot at: {robot_prim_path}")

        try:
            # Import Isaac Sim modules
            from isaacsim.core.prims import XFormPrim

            # Store robot prim path
            self._robot_prim_path = robot_prim_path

            # Initialize XFormPrim for the robot
            self._xform_prim = XFormPrim(robot_prim_path)
            self._xform_prim.initialize()

            # Create teleport callback using XFormPrim approach
            teleport_callback = self._create_xform_prim_teleport_callback()
            self._teleport_callback = teleport_callback

            self.get_logger().info("Isaac Sim teleport callback successfully registered")

        except ImportError as e:
            self.get_logger().warning(f"Isaac Sim modules not available: {e}")
            self.get_logger().warning("Teleportation will be skipped for missions")
        except Exception as e:
            self.get_logger().error(f"Failed to setup Isaac Sim teleport callback: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            self.get_logger().warning("Teleportation will be skipped for missions")

    def _create_xform_prim_teleport_callback(self):
        """Create teleport callback using XFormPrim (works for articulations and rigid bodies).

        This uses XFormPrim for physics-aware teleportation.
        This method works for both articulated robots and simple rigid bodies.

        Returns:
            Callable that teleports the robot to a given position.
        """
        import numpy as np

        def teleport_callback(position: SampledPosition) -> bool:
            try:
                # Position array
                pos = np.array([position.x, position.y, position.z], dtype=np.float64)

                # Quaternion (w, x, y, z) format for Isaac Sim XFormPrim
                qx, qy, qz, qw = self._yaw_to_quaternion(position.heading)
                # XFormPrim expects (w, x, y, z) order
                orientation = np.array([qw, qx, qy, qz], dtype=np.float64)

                # Set pose using XFormPrim
                self._xform_prim.set_world_pose(position=pos, orientation=orientation)

                self.get_logger().info(
                    f"Teleported robot to ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), "
                    f"heading={position.heading:.2f}"
                )
                return True

            except Exception as e:
                self.get_logger().error(f"XFormPrim teleportation failed: {e}")
                import traceback

                self.get_logger().error(traceback.format_exc())
                return False

        return teleport_callback

    def _yaw_to_quaternion(self, yaw: float) -> tuple:
        """Convert yaw angle to quaternion (x, y, z, w)."""
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def publish_initial_pose(self, position: SampledPosition) -> None:
        """Publish initial pose for AMCL localization.

        Args:
            position: Robot's initial position.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = position.x
        msg.pose.pose.position.y = position.y
        msg.pose.pose.position.z = position.z

        qx, qy, qz, qw = self._yaw_to_quaternion(position.heading)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Set covariance (small values for good initial estimate)
        # Covariance is 6x6 matrix in row-major order
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.0685  # yaw variance (~15 degrees)

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"Published initial pose: ({position.x:.2f}, {position.y:.2f}), "
            f"heading={math.degrees(position.heading):.1f}°"
        )

    def publish_goal_pose(self, position: SampledPosition) -> None:
        """Publish navigation goal to Nav2.

        Args:
            position: Goal position.
        """
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = position.x
        msg.pose.position.y = position.y
        msg.pose.position.z = position.z

        qx, qy, qz, qw = self._yaw_to_quaternion(position.heading)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.goal_pose_pub.publish(msg)
        self.get_logger().info(
            f"Published goal pose: ({position.x:.2f}, {position.y:.2f}), heading={math.degrees(position.heading):.1f}°"
        )

    def teleport_robot(self, position: SampledPosition) -> bool:
        """Teleport robot to the specified position.

        Args:
            position: Target position for teleportation.

        Returns:
            True if teleportation succeeded, False otherwise.
        """
        if self._teleport_callback is None:
            self.get_logger().warning("No teleport callback registered. Skipping teleportation.")
            return True  # Continue without teleportation

        try:
            # Add height offset for teleportation
            teleport_pos = SampledPosition(
                x=position.x,
                y=position.y,
                z=position.z + self.config.teleport_height,
                heading=position.heading,
            )
            success = self._teleport_callback(teleport_pos)

            if success:
                self.get_logger().info(f"Robot teleported to ({position.x:.2f}, {position.y:.2f})")
            else:
                self.get_logger().error("Teleportation failed.")

            return success
        except Exception as e:
            self.get_logger().error(f"Teleportation error: {e}")
            return False

    def run_mission(
        self,
        fixed_start: Optional[SampledPosition] = None,
        fixed_goal: Optional[SampledPosition] = None,
    ) -> bool:
        """Run a complete navigation mission.

        This method:
        1. Samples start/goal positions (or uses provided ones)
        2. Teleports robot to start position
        3. Publishes initial pose for AMCL
        4. Publishes goal to Nav2
        5. Updates RViz markers

        Args:
            fixed_start: Optional fixed start position (skips sampling).
            fixed_goal: Optional fixed goal position (skips sampling).

        Returns:
            True if mission was initiated successfully, False otherwise.
        """
        self._mission_count += 1
        self.get_logger().info(f"=== Starting Mission #{self._mission_count} ===")

        # Step 1: Sample or use provided positions
        if fixed_start is not None and fixed_goal is not None:
            start, goal = fixed_start, fixed_goal
            self.get_logger().info("Using provided start/goal positions.")
        elif fixed_start is not None:
            start, goal = self.sampler.sample_start_goal_pair(fixed_start=fixed_start)
        else:
            start, goal = self.sampler.sample_start_goal_pair()

        if start is None or goal is None:
            self.get_logger().error("Failed to sample valid start/goal positions.")
            return False

        self._current_start = start
        self._current_goal = goal

        distance = start.distance_to(goal)
        self.get_logger().info(f"Mission positions: start={start}, goal={goal}, distance={distance:.2f}m")

        # Step 2: Teleport robot to start position
        if not self.teleport_robot(start):
            self.get_logger().error("Mission aborted: teleportation failed.")
            return False

        # Step physics to settle after teleportation
        # This is critical for proper physics state updates
        if self._simulation_context is not None:
            for _ in range(self.config.teleport_settle_steps):
                self._simulation_context.step(render=True)
            self.get_logger().info(f"Physics settled after {self.config.teleport_settle_steps} simulation steps")
        else:
            # Fallback to time-based delay if no simulation context
            time.sleep(0.5)

        # Step 3: Publish initial pose for AMCL
        self.publish_initial_pose(start)
        time.sleep(self.config.initial_pose_delay)

        # Step 4: Publish goal to Nav2
        self.publish_goal_pose(goal)
        time.sleep(self.config.goal_delay)

        # Step 5: Update RViz markers
        self.marker_publisher.publish_start_goal_from_sampled(start, goal)

        self.get_logger().info(f"=== Mission #{self._mission_count} initiated ===")
        return True

    def get_current_mission(self) -> tuple:
        """Get current mission start and goal positions.

        Returns:
            Tuple of (start, goal) SampledPosition objects.
        """
        return self._current_start, self._current_goal

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.marker_publisher.clear_markers()
        self.marker_publisher.destroy_node()
        super().destroy_node()


def main(args=None):
    """Main entry point for standalone mission orchestrator.

    This can be run as a ROS2 node for testing without Isaac Sim teleportation.
    """
    rclpy.init(args=args)

    config = OrchestratorConfig(
        min_distance=5.0,
        max_distance=50.0,
    )
    orchestrator = MissionOrchestrator(config=config)

    try:
        # Run a single test mission (without teleportation)
        orchestrator.get_logger().info("Running test mission (no teleportation)...")

        # For testing without NavMesh, create manual positions
        test_start = SampledPosition(x=0.0, y=0.0, z=0.0, heading=0.0)
        test_goal = SampledPosition(x=10.0, y=5.0, z=0.0, heading=0.5)

        # Publish markers and poses
        orchestrator.publish_initial_pose(test_start)
        time.sleep(1.0)
        orchestrator.publish_goal_pose(test_goal)
        orchestrator.marker_publisher.publish_start_goal_from_sampled(test_start, test_goal)

        orchestrator.get_logger().info("Test mission published. Spinning...")
        rclpy.spin(orchestrator)

    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
