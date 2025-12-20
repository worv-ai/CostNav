"""Mission manager that runs in the main simulation loop.

This module provides MissionManager which integrates with the main simulation
loop instead of running in a background thread. This ensures proper synchronization
between teleportation and physics simulation steps.

The MissionManager is a ROS2 node that provides:
- State machine-based mission workflow execution
- NavMesh position sampling with distance constraints
- Robot teleportation in Isaac Sim (auto-setup if robot_prim_path provided)
- AMCL initial pose publication
- Nav2 goal publication
- RViz marker visualization
- Proper physics synchronization via main loop integration
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from costnav_isaacsim.config import MissionConfig

logger = logging.getLogger("costnav_mission_manager")


@dataclass
class MissionManagerConfig:
    """Configuration for MissionManager runtime settings."""

    min_distance: float = 5.0  # Minimum distance between start and goal (meters)
    max_distance: float = 100.0  # Maximum distance between start and goal (meters)
    initial_pose_delay: float = 1.0  # Delay after setting initial pose (seconds)
    goal_delay: float = 0.5  # Delay after publishing goal (seconds)
    teleport_height: float = 0.1  # Height offset for teleportation (meters)
    robot_prim_path: Optional[str] = None  # Robot prim path in Isaac Sim USD stage
    teleport_settle_steps: int = 5  # Number of simulation steps after teleportation for physics to settle


class MissionState(Enum):
    """State machine for mission execution."""

    INIT = "init"
    WAITING_FOR_NAV2 = "waiting_for_nav2"
    READY = "ready"
    TELEPORTING = "teleporting"
    SETTLING = "settling"  # Waiting for physics to settle after teleport
    PUBLISHING_INITIAL_POSE = "publishing_initial_pose"
    PUBLISHING_GOAL = "publishing_goal"
    WAITING_FOR_COMPLETION = "waiting_for_completion"
    COMPLETED = "completed"


class MissionManager:
    """Mission manager that runs in the main simulation loop.

    This class manages mission execution by being stepped in the main simulation
    loop, ensuring proper synchronization between teleportation and physics steps.

    It combines state machine-based workflow execution with ROS2 publishing capabilities:
    - NavMesh position sampling with distance constraints
    - Robot teleportation in Isaac Sim (auto-setup if robot_prim_path provided)
    - AMCL initial pose publication
    - Nav2 goal publication
    - RViz marker visualization

    Usage:
        from costnav_isaacsim.config import MissionConfig
        from costnav_isaacsim.nav2_mission import MissionManager

        config = MissionConfig(count=5, delay=30.0)
        manager = MissionManager(config, simulation_context)

        # In main simulation loop:
        while running:
            simulation_context.step(render=True)
            manager.step()  # Step mission manager after physics step
    """

    def __init__(
        self,
        mission_config: "MissionConfig",
        simulation_context,
        node_name: str = "nav2_mission_manager",
        manager_config: Optional[MissionManagerConfig] = None,
        teleport_callback: Optional[Callable[[object], bool]] = None,
    ):
        """Initialize mission manager.

        Args:
            mission_config: Mission configuration object (count, delay, distances, etc.).
            simulation_context: Isaac Sim SimulationContext for stepping physics.
            node_name: Name of the ROS2 node.
            manager_config: Optional MissionManagerConfig for fine-tuning. If not provided,
                          will be created from mission_config.
            teleport_callback: Optional callback for robot teleportation.
                             Signature: (position: SampledPosition) -> bool
                             If not provided and manager_config.robot_prim_path is set,
                             will attempt to auto-setup Isaac Sim teleportation.
        """
        self.mission_config = mission_config
        self.simulation_context = simulation_context
        self.node_name = node_name

        # Create manager config from mission config if not provided
        if manager_config is None:
            manager_config = MissionManagerConfig(
                min_distance=mission_config.min_distance,
                max_distance=mission_config.max_distance,
                teleport_height=mission_config.teleport.height_offset,
                robot_prim_path=mission_config.teleport.robot_prim,
                # Use default value of 5 for teleport_settle_steps (not in TeleportConfig)
                teleport_settle_steps=5,
            )
        self.config = manager_config

        self._teleport_callback = teleport_callback

        # ROS2 node and publishers (initialized later)
        self._node = None
        self._initial_pose_pub = None
        self._goal_pose_pub = None
        self._marker_publisher = None
        self._sampler = None
        self._stage = None
        self._robot_prim_path = None

        # State machine
        self._state = MissionState.INIT
        self._current_mission = 0
        self._mission_start_time = None
        self._wait_start_time = None
        self._settle_steps_remaining = 0
        self._initialized = False

        # Current mission positions
        self._current_start = None
        self._current_goal = None

    def step(self):
        """Step the mission manager (called from main simulation loop)."""
        if not self._initialized:
            self._initialize()
            return

        if self._state == MissionState.WAITING_FOR_NAV2:
            self._step_waiting_for_nav2()
        elif self._state == MissionState.READY:
            self._step_ready()
        elif self._state == MissionState.TELEPORTING:
            self._step_teleporting()
        elif self._state == MissionState.SETTLING:
            self._step_settling()
        elif self._state == MissionState.PUBLISHING_INITIAL_POSE:
            self._step_publishing_initial_pose()
        elif self._state == MissionState.PUBLISHING_GOAL:
            self._step_publishing_goal()
        elif self._state == MissionState.WAITING_FOR_COMPLETION:
            self._step_waiting_for_completion()
        elif self._state == MissionState.COMPLETED:
            pass  # All missions completed

    def _initialize(self):
        """Initialize ROS2 node and all components (called once on first step)."""
        import rclpy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        from .marker_publisher import MarkerPublisher
        from .navmesh_sampler import NavMeshSampler

        try:
            # Initialize ROS2
            rclpy.init()

            # Create ROS2 node
            self._node = Node(self.node_name)

            # Initialize NavMesh sampler
            self._sampler = NavMeshSampler(
                min_distance=self.config.min_distance,
                max_distance=self.config.max_distance,
            )

            # Initialize marker publisher (as a separate node)
            self._marker_publisher = MarkerPublisher(node_name=f"{self.node_name}_markers")

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
            self._initial_pose_pub = self._node.create_publisher(PoseWithCovarianceStamped, "/initialpose", pose_qos)
            self._goal_pose_pub = self._node.create_publisher(PoseStamped, "/goal_pose", pose_qos)

            self._initialized = True
            self._state = MissionState.WAITING_FOR_NAV2
            self._wait_start_time = time.time()

            logger.info(f"[{self._state.name}] Mission manager initialized")
            logger.info(f"[{self._state.name}] Will run {self.mission_config.count} mission(s)")
            logger.info(
                f"[{self._state.name}] Distance range: {self.config.min_distance}m - {self.config.max_distance}m"
            )

        except Exception as e:
            logger.error(f"[{self._state.name}] Failed to initialize mission manager: {e}")
            import traceback

            logger.error(f"[{self._state.name}] {traceback.format_exc()}")
            self._state = MissionState.COMPLETED

    def _setup_isaac_sim_teleport(self) -> None:
        """Setup Isaac Sim teleportation using UsdGeom.Xformable.

        This method uses USD xform operations for teleportation.
        This approach works for all prim types including articulated robots.

        If any step fails, logs a warning and continues without teleportation.
        """
        if not self.config.robot_prim_path:
            return

        robot_prim_path = self.config.robot_prim_path
        logger.info(f"[{self._state.name}] Setting up Isaac Sim teleport for robot at: {robot_prim_path}")

        try:
            # Import Isaac Sim modules
            import omni.usd

            # Get the stage and prim
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(robot_prim_path)

            if not prim.IsValid():
                raise ValueError(f"Invalid robot prim at path: {robot_prim_path}")

            # Store stage and prim path for teleport callback
            self._stage = stage
            self._robot_prim_path = robot_prim_path

            # Create teleport callback using UsdGeom.Xformable
            teleport_callback = self._create_xform_teleport_callback()
            self._teleport_callback = teleport_callback

            logger.info(f"[{self._state.name}] Isaac Sim teleport callback successfully registered")

        except ImportError as e:
            logger.warning(f"[{self._state.name}] Isaac Sim modules not available: {e}")
            logger.warning(f"[{self._state.name}] Teleportation will be skipped for missions")
        except Exception as e:
            logger.error(f"[{self._state.name}] Failed to setup Isaac Sim teleport callback: {e}")
            import traceback

            logger.error(f"[{self._state.name}] {traceback.format_exc()}")
            logger.warning(f"[{self._state.name}] Teleportation will be skipped for missions")

    def _create_xform_teleport_callback(self):
        """Create teleport callback using UsdGeom.Xformable.

        This uses USD xform operations for teleportation.
        This method works for all prim types including articulated robots.

        Returns:
            Callable that teleports the robot to a given position.
        """
        from pxr import Gf, UsdGeom

        def teleport_callback(position) -> bool:
            try:
                prim = self._stage.GetPrimAtPath(self._robot_prim_path)
                if not prim.IsValid():
                    logger.error(f"[TELEPORTING] Teleport target prim not found: {self._robot_prim_path}")
                    return False

                # Create UsdGeom.Xformable from prim
                xform = UsdGeom.Xformable(prim)

                # Get or create translate operation
                translate_ops = [
                    op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
                ]
                translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()

                # Get or create orient operation
                orient_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeOrient]
                orient_op = orient_ops[0] if orient_ops else xform.AddOrientOp()

                # Set position
                pos = Gf.Vec3d(position.x, position.y, position.z)
                translate_op.Set(pos)

                # Convert yaw to quaternion (w, x, y, z) format for Gf.Quatd
                qx, qy, qz, qw = self._yaw_to_quaternion(position.heading)
                # Gf.Quatd expects (real, imaginary) = (w, Vec3d(x, y, z))
                orientation = Gf.Quatd(float(qw), Gf.Vec3d(float(qx), float(qy), float(qz)))
                orient_op.Set(orientation)

                logger.info(
                    f"[TELEPORTING] Teleported robot to ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), "
                    f"heading={position.heading:.2f}"
                )
                return True

            except Exception as e:
                logger.error(f"[TELEPORTING] USD xform teleportation failed: {e}")
                import traceback

                logger.error(f"[TELEPORTING] {traceback.format_exc()}")
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

    def _teleport_robot(self, position) -> bool:
        """Teleport robot to the specified position.

        Args:
            position: Target position for teleportation (SampledPosition).

        Returns:
            True if teleportation succeeded, False otherwise.
        """
        if self._teleport_callback is None:
            logger.warning(f"[{self._state.name}] No teleport callback registered. Skipping teleportation.")
            return True  # Continue without teleportation

        try:
            # Import here to avoid circular dependency
            from .navmesh_sampler import SampledPosition

            # Add height offset for teleportation
            teleport_pos = SampledPosition(
                x=position.x,
                y=position.y,
                z=position.z + self.config.teleport_height,
                heading=position.heading,
            )
            success = self._teleport_callback(teleport_pos)

            if success:
                logger.info(f"[{self._state.name}] Robot teleported to ({position.x:.2f}, {position.y:.2f})")
            else:
                logger.error(f"[{self._state.name}] Teleportation failed.")

            return success
        except Exception as e:
            logger.error(f"[{self._state.name}] Teleportation error: {e}")
            return False

    def _publish_initial_pose(self, position) -> None:
        """Publish initial pose for AMCL localization.

        Args:
            position: Robot's initial position (SampledPosition).
        """
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()

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

        self._initial_pose_pub.publish(msg)
        logger.info(
            f"[{self._state.name}] Published initial pose: ({position.x:.2f}, {position.y:.2f}), "
            f"heading={math.degrees(position.heading):.1f}°"
        )

    def _publish_goal_pose(self, position) -> None:
        """Publish navigation goal to Nav2.

        Args:
            position: Goal position (SampledPosition).
        """
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()

        msg.pose.position.x = position.x
        msg.pose.position.y = position.y
        msg.pose.position.z = position.z

        qx, qy, qz, qw = self._yaw_to_quaternion(position.heading)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._goal_pose_pub.publish(msg)
        logger.info(
            f"[{self._state.name}] Published goal pose: ({position.x:.2f}, {position.y:.2f}), heading={math.degrees(position.heading):.1f}°"
        )

    def _step_waiting_for_nav2(self):
        """Wait for Nav2 stack to be ready."""
        elapsed = time.time() - self._wait_start_time
        if elapsed >= self.mission_config.nav2.wait_time:
            logger.info(f"[{self._state.name}] Nav2 wait time complete, ready to start missions")
            self._state = MissionState.READY

    def _step_ready(self):
        """Start next mission or complete if all done."""
        if self._current_mission >= self.mission_config.count:
            logger.info(f"[{self._state.name}] All missions completed!")
            self._state = MissionState.COMPLETED
            self._cleanup()
            return

        self._current_mission += 1
        logger.info(f"[{self._state.name}] \n{'=' * 50}")
        logger.info(f"[{self._state.name}] Mission {self._current_mission}/{self.mission_config.count}")
        logger.info(f"[{self._state.name}] {'=' * 50}")

        # Step 1: Sample positions using sampler
        start, goal = self._sampler.sample_start_goal_pair()
        if start is None or goal is None:
            logger.error(f"[{self._state.name}] Failed to sample valid start/goal positions")
            self._state = MissionState.READY  # Try again next step
            return

        # Store positions locally
        self._current_start = start
        self._current_goal = goal

        distance = start.distance_to(goal)
        logger.info(f"[{self._state.name}] Mission positions: start={start}, goal={goal}, distance={distance:.2f}m")

        self._state = MissionState.TELEPORTING

    def _step_teleporting(self):
        """Teleport robot to start position."""
        if not self._teleport_robot(self._current_start):
            logger.error(f"[{self._state.name}] Teleportation failed, skipping mission")
            self._state = MissionState.READY
            return

        # After teleportation, we need to settle physics
        self._settle_steps_remaining = self.config.teleport_settle_steps
        self._state = MissionState.SETTLING
        logger.info(f"[SETTLING] Teleportation complete, settling physics for {self._settle_steps_remaining} steps")

    def _step_settling(self):
        """Wait for physics to settle after teleportation.

        This is critical: after XFormPrim.set_world_pose(), we must step the
        simulation multiple times to allow the physics engine to process the
        new pose and update all internal states.
        """
        if self._settle_steps_remaining > 0:
            # Simulation step is already called in main loop before this
            # We just count down the steps
            self._settle_steps_remaining -= 1
            if self._settle_steps_remaining == 0:
                logger.info(f"[{self._state.name}] Physics settled, publishing initial pose")
                self._state = MissionState.PUBLISHING_INITIAL_POSE

    def _step_publishing_initial_pose(self):
        """Publish initial pose for AMCL."""
        self._publish_initial_pose(self._current_start)
        self._wait_start_time = time.time()
        self._state = MissionState.PUBLISHING_GOAL

    def _step_publishing_goal(self):
        """Publish goal pose to Nav2 after initial pose delay."""
        elapsed = time.time() - self._wait_start_time
        if elapsed >= self.config.initial_pose_delay:
            self._publish_goal_pose(self._current_goal)

            # Update RViz markers
            self._marker_publisher.publish_start_goal_from_sampled(self._current_start, self._current_goal)

            logger.info(f"[{self._state.name}] Mission {self._current_mission} initiated successfully")
            self._mission_start_time = time.time()
            self._state = MissionState.WAITING_FOR_COMPLETION

    def _step_waiting_for_completion(self):
        """Wait for mission delay before starting next mission."""
        elapsed = time.time() - self._mission_start_time
        if elapsed >= self.mission_config.delay:
            self._state = MissionState.READY

    def _cleanup(self):
        """Clean up ROS2 resources."""
        try:
            if self._node:
                self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._marker_publisher:
                self._marker_publisher.destroy_node()
        except Exception:
            pass
        try:
            import rclpy

            rclpy.shutdown()
        except Exception:
            pass
