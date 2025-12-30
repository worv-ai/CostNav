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
    edge_margin: float = 0.5  # Minimum distance from navmesh edges (meters)
    initial_pose_delay: float = 1.0  # Delay after setting initial pose (seconds)
    goal_delay: float = 0.5  # Delay after publishing goal (seconds)
    teleport_height: float = 0.1  # Height offset for teleportation (meters)
    robot_prim_path: Optional[str] = None  # Robot prim path in Isaac Sim USD stage
    teleport_settle_steps: int = 5  # Number of simulation steps after teleportation for physics to settle


class MissionState(Enum):
    """State machine for mission execution."""

    INIT = "init"
    WAITING_FOR_NAV2 = "waiting_for_nav2"
    WAITING_FOR_START = "waiting_for_start"
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

        config = MissionConfig(timeout=3600.0)
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
            mission_config: Mission configuration object (timeout, distances, etc.).
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
                edge_margin=mission_config.sampling.edge_margin,
                initial_pose_delay=mission_config.nav2.initial_pose_delay,
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
        self._executor = None
        self._start_service = None

        # State machine
        self._state = MissionState.INIT
        self._current_mission = 0
        self._mission_start_time = None
        self._wait_start_time = None
        self._settle_steps_remaining = 0
        self._initialized = False
        self._start_requested = False
        self._restart_requested = False

        # Current mission positions
        self._current_start = None
        self._current_goal = None

    def _get_current_time_seconds(self) -> float:
        """Get current time in seconds from ROS clock.

        Returns:
            Current time in seconds.
        """
        if self._node is None:
            return 0.0
        return self._node.get_clock().now().nanoseconds / 1e9

    def step(self):
        """Step the mission manager (called from main simulation loop)."""
        if not self._initialized:
            self._initialize()
            return

        self._spin_ros_once()

        if self._restart_requested and self._is_mission_active():
            self._reset_mission_state()
            self._restart_requested = False
            self._state = MissionState.READY

        if self._state == MissionState.WAITING_FOR_NAV2:
            self._step_waiting_for_nav2()
        elif self._state == MissionState.WAITING_FOR_START:
            self._step_waiting_for_start()
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
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import Trigger

        from .marker_publisher import MarkerPublisher
        from .navmesh_sampler import NavMeshSampler

        try:
            # Initialize ROS2
            rclpy.init()

            # Create ROS2 node
            self._node = Node(self.node_name)

            # Start mission service (manual trigger)
            self._start_service = self._node.create_service(Trigger, "/start_mission", self._handle_start_mission)

            # Initialize NavMesh sampler with config values
            self._sampler = NavMeshSampler(
                min_distance=self.config.min_distance,
                max_distance=self.config.max_distance,
                edge_margin=self.config.edge_margin,
                max_sampling_attempts=self.mission_config.sampling.max_attempts,
                validate_path=self.mission_config.sampling.validate_path,
            )

            # Initialize marker publisher (as a separate node) with config values
            self._marker_publisher = MarkerPublisher(
                node_name=f"{self.node_name}_markers",
                arrow_length=self.mission_config.markers.arrow_length,
                arrow_width=self.mission_config.markers.arrow_width,
                arrow_height=self.mission_config.markers.arrow_height,
                start_topic=self.mission_config.markers.start_topic,
                goal_topic=self.mission_config.markers.goal_topic,
                robot_topic=self.mission_config.markers.robot_topic,
                odom_topic=self.mission_config.nav2.odom_topic,
                enabled=self.mission_config.markers.enabled,
            )

            # ROS2 executor to service timers/subscriptions without blocking the sim loop.
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor.add_node(self._marker_publisher)

            # Auto-setup Isaac Sim teleport callback if robot_prim_path is provided
            if self._teleport_callback is None and self.config.robot_prim_path:
                self._setup_isaac_sim_teleport()

            # QoS for pose topics
            pose_qos = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )

            # Publishers (using topic names from config)
            self._initial_pose_pub = self._node.create_publisher(
                PoseWithCovarianceStamped, self.mission_config.nav2.initial_pose_topic, pose_qos
            )
            self._goal_pose_pub = self._node.create_publisher(
                PoseStamped, self.mission_config.nav2.goal_pose_topic, pose_qos
            )

            self._initialized = True
            self._state = MissionState.WAITING_FOR_NAV2
            self._wait_start_time = self._get_current_time_seconds()

            logger.info(f"[{self._state.name}] Mission manager initialized")
            logger.info(
                f"[{self._state.name}] Distance range: {self.config.min_distance}m - {self.config.max_distance}m"
            )

        except Exception as e:
            logger.error(f"[{self._state.name}] Failed to initialize mission manager: {e}")
            import traceback

            logger.error(f"[{self._state.name}] {traceback.format_exc()}")
            self._state = MissionState.COMPLETED

    def _is_mission_active(self) -> bool:
        """Return True if a mission is in progress or about to start."""
        return self._state in {
            MissionState.TELEPORTING,
            MissionState.SETTLING,
            MissionState.PUBLISHING_INITIAL_POSE,
            MissionState.PUBLISHING_GOAL,
            MissionState.WAITING_FOR_COMPLETION,
        }

    def _reset_mission_state(self) -> None:
        """Reset mission state so a new mission can start cleanly."""
        self._current_start = None
        self._current_goal = None
        self._mission_start_time = None
        self._settle_steps_remaining = 0

    def _handle_start_mission(self, request, response):
        """Handle external mission start trigger."""
        if self._state == MissionState.COMPLETED:
            response.success = False
            response.message = "Missions already completed."
            return response

        self._start_requested = True
        if self._is_mission_active():
            self._restart_requested = True
            logger.info(f"[{self._state.name}] Start signal received. Restarting mission.")
        elif self._state == MissionState.WAITING_FOR_START:
            self._state = MissionState.READY
            logger.info("[WAITING_FOR_START] Start signal received. Beginning mission.")
        else:
            logger.info(f"[{self._state.name}] Start signal received. Will begin when ready.")

        response.success = True
        response.message = "Mission start requested."
        return response

    def _spin_ros_once(self) -> None:
        """Service ROS2 callbacks and timers without blocking the sim loop."""
        if self._executor is None:
            return

        try:
            self._executor.spin_once(timeout_sec=0.0)
        except Exception as exc:
            logger.warning(f"[{self._state.name}] ROS2 spin_once failed: {exc}")

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
        The teleport operation simply translates and orients the target prim
        to the specified position without modifying any articulation states.

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
        elapsed = self._get_current_time_seconds() - self._wait_start_time
        if elapsed >= self.mission_config.nav2.wait_time:
            if self._start_requested:
                logger.info(f"[{self._state.name}] Nav2 wait time complete, starting missions")
                self._state = MissionState.READY
            else:
                logger.info(f"[{self._state.name}] Nav2 ready. Waiting for /start_mission.")
                self._state = MissionState.WAITING_FOR_START

    def _step_waiting_for_start(self):
        """Wait for an external mission start trigger."""
        if self._start_requested:
            self._start_requested = False
            self._state = MissionState.READY

    def _step_ready(self):
        """Start next mission or complete if all done."""
        self._current_mission += 1
        self._start_requested = False
        self._restart_requested = False
        logger.info(f"[{self._state.name}] \n{'=' * 50}")
        logger.info(f"[{self._state.name}] Mission {self._current_mission}")
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
        self._wait_start_time = self._get_current_time_seconds()
        self._state = MissionState.PUBLISHING_GOAL

    def _step_publishing_goal(self):
        """Publish goal pose to Nav2 after initial pose delay."""
        elapsed = self._get_current_time_seconds() - self._wait_start_time
        if elapsed >= self.config.initial_pose_delay:
            self._publish_goal_pose(self._current_goal)

            # Update RViz markers
            self._marker_publisher.publish_start_goal_from_sampled(self._current_start, self._current_goal)

            logger.info(f"[{self._state.name}] Mission {self._current_mission} initiated successfully")
            self._mission_start_time = self._get_current_time_seconds()
            self._state = MissionState.WAITING_FOR_COMPLETION

    def _step_waiting_for_completion(self):
        """Wait for mission timeout before returning to wait-for-start."""
        elapsed = self._get_current_time_seconds() - self._mission_start_time
        if elapsed >= self.mission_config.timeout:
            self._state = MissionState.WAITING_FOR_START
            logger.info(f"[{self._state.name}] Mission {self._current_mission} timed out {self.mission_config.timeout=}, waiting for start signal")

    def _cleanup(self):
        """Clean up ROS2 resources."""
        try:
            if self._executor:
                if self._node:
                    self._executor.remove_node(self._node)
                if self._marker_publisher:
                    self._executor.remove_node(self._marker_publisher)
                self._executor.shutdown()
        except Exception:
            pass
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
