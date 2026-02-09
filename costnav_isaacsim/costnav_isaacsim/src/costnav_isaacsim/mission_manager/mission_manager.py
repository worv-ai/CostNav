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
from enum import Enum
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from costnav_isaacsim.config import MissionConfig

logger = logging.getLogger("costnav_mission_manager")

PROPERTY_PRIM_PATHS = {
    "fire_hydrant": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section19",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section53",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section57",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section55",
    ],
    "traffic_light": [
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section32",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section31",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section31",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section12",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section39",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section39",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section41",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section41",
    ],
    "street_lamp": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section1",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section1",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section1",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section1",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section78",
    ],
    "bollard": [
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section77",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section31",
    ],
    # Building prims are matched broadly via prefix; keep only explicit non-standard
    # building-related prims here (e.g., cubes that represent building parts).
    "building": [
        "/World/box/Cube",
        "/World/box/Cube_01",
        "/World/box/Cube_02",
        "/World/box/Cube_03",
        "/World/box/Cube_04",
        "/World/box/Cube_05",
        "/World/box/Cube_06",
        "/World/box/Cube_07",
        "/World/box/Cube_08",
        "/World/box/Cube_09",
        "/World/box/Cube_10",
        "/World/box/Cube_11",
        "/World/box/Cube_12",
        "/World/box/Cube_13",
    ],
    "trash_bin": [
        "/World/Environment/SM_Buidlng_032/SM_Buidlng_032/Section26",
    ],
    "mail_box": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section20",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section85",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section55",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section18",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section18",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section18",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section57",
    ],
    "newspaper_box": [
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section21",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section21",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section21",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section23",
    ],
    "bus_stop": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section8",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section6",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section6",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section6",
    ],
}

# Property categories used for cost comparison.
_COST_PROPERTY_CATEGORIES = {"mail_box", "trash_bin", "building", "bollard"}


class MissionState(Enum):
    """State machine for mission execution."""

    INIT = "init"
    WAITING_FOR_NAV2 = "waiting_for_nav2"
    WAITING_FOR_START = "waiting_for_start"
    READY = "ready"
    TELEPORTING = "teleporting"
    SETTLING = "settling"  # Waiting for physics to settle after teleport
    PUBLISHING_INITIAL_POSE = "publishing_initial_pose"
    CLEARING_COSTMAPS = "clearing_costmaps"
    PUBLISHING_GOAL = "publishing_goal"
    WAITING_FOR_COMPLETION = "waiting_for_completion"
    COMPLETED = "completed"


class MissionResult(Enum):
    """Result of a mission execution."""

    PENDING = "pending"  # Mission not yet started or in progress
    SUCCESS = "success"  # Robot reached goal within tolerance
    FAILURE_TIMEOUT = "failure_timeout"  # Timeout reached before reaching goal
    FAILURE_PHYSICALASSISTANCE = (
        "failure_physicalassistance"  # Robot fell down (bad orientation) or impulse health depleted
    )
    FAILURE_FOODSPOILED = "failure_foodspoiled"  # Food spoiled during delivery


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
        from costnav_isaacsim.isaac_sim.mission_manager import MissionManager

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
        node_name: str = "mission_manager",
        teleport_callback: Optional[Callable[[object], bool]] = None,
    ):
        """Initialize mission manager.

        Args:
            mission_config: Mission configuration object (timeout, distances, etc.).
            simulation_context: Isaac Sim SimulationContext for stepping physics.
            node_name: Name of the ROS2 node.
            teleport_callback: Optional callback for robot teleportation.
                             Signature: (position: SampledPosition) -> bool
                             If not provided and config.manager.robot_prim_path is set,
                             will attempt to auto-setup Isaac Sim teleportation.
        """
        self.config = mission_config
        self.simulation_context = simulation_context
        self.node_name = node_name

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
        self._result_service = None
        self._odom_sub = None

        # IL baseline node control publishers
        self._vint_enable_pub = None
        self._trajectory_follower_enable_pub = None

        # Nav2 costmap clear service clients (initialized in _initialize)
        self._clear_costmap_global_srv = None
        self._clear_costmap_local_srv = None
        self._costmap_clear_futures = {}
        self._costmap_clear_start_time = None

        # State machine
        self._state = MissionState.INIT
        self._current_mission = 0
        self._mission_start_time = None
        self._mission_end_time = None
        self._wait_start_time = None
        self._settle_steps_remaining = 0
        self._initialized = False
        self._start_requested = False
        self._restart_requested = False

        # Current mission positions
        self._current_start = None
        self._current_goal = None

        # Robot position tracking (from odom)
        self._robot_position = None  # (x, y, z) tuple
        self._robot_orientation = None  # (x, y, z, w) quaternion tuple
        self._prev_robot_position = None  # Previous position for distance tracking

        # Mission result tracking
        self._last_mission_result = MissionResult.PENDING
        self._last_mission_result_reason = (
            None  # Specific reason for result (e.g., "orientation", "impulse_health_depletion")
        )
        self._last_mission_distance = None  # Distance to goal at end

        # Mission metrics tracking
        self._traveled_distance = 0.0  # Cumulative distance traveled (meters)
        self._last_traveled_distance = None  # Final traveled distance for last mission (meters)
        self._last_elapsed_time = None  # Elapsed time for last mission (seconds)

        # Impulse health tracking (structural damage)
        self._impulse_min_threshold = 50.0
        self._impulse_health_max = 177.8
        self._impulse_health = self._impulse_health_max
        self._impulse_damage_accumulated = 0.0
        self._contact_report_subscription = None
        self._contact_report_targets = set()

        # Contact count and total impulse tracking (for evaluation metrics)
        self._contact_count = 0  # Number of contact events during mission
        self._total_impulse = 0.0  # Total impulse accumulated during mission
        self._last_contact_count = None  # Final contact count for last mission
        self._last_total_impulse = None  # Final total impulse for last mission
        self._people_contact_count = 0  # People collisions (subset of contacts)
        self._last_people_contact_count = None
        self._property_contact_counts = {key: 0 for key in PROPERTY_PRIM_PATHS}
        self._last_property_contact_counts = None
        self._property_contact_impulse_min_threshold = 100.0

        # Damager cooldown tracking
        self._last_damage_steps_remaining = 0
        self._damage_cooldown_steps = 30

        # Delta-v and injury cost tracking (computed from impulse/mass)
        self._delta_v_magnitudes_mps = []  # List of delta-v magnitudes in m/s per mission
        self._last_delta_v_magnitudes_mps = None  # Final list for last mission
        self._injury_costs = []  # List of MAIS-based injury costs per collision
        self._total_injury_cost = 0.0  # Sum of all injury costs during mission
        self._last_injury_costs = None  # Final list for last mission
        self._last_total_injury_cost = None  # Final total for last mission

        # Food tracking for spoilage evaluation
        self._food_root_prim_path = None  # Root prim path for food assets
        self._food_prefix_path = None  # Pre-calculated prefix for startswith check
        self._food_pieces_prim_path = None  # Full prim path to food pieces
        self._food_bucket_prim_path = None  # Full prim path to food bucket
        self._initial_food_piece_count = 0  # Piece count at mission start
        self._final_food_piece_count = None  # Piece count at mission end

        # Goal image capture for ViNT ImageGoal mode
        self._goal_camera_prim = None  # USD camera prim for goal image capture
        self._goal_render_product = None  # Omni replicator render product
        self._goal_rgb_annotator = None  # RGB annotator for goal image capture
        self._goal_image_pub = None  # ROS2 publisher for goal images

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
        elif self._state == MissionState.CLEARING_COSTMAPS:
            self._step_clearing_costmaps()
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
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64
        from std_srvs.srv import Trigger

        from .marker_publisher import MarkerPublisher
        from .mission_result_srv import GetMissionResult
        from .navmesh_sampler import NavMeshSampler

        try:
            # Initialize ROS2
            rclpy.init()

            # Create ROS2 node
            self._node = Node(self.node_name)

            # Nav2 costmap clear services (best-effort; keep running even if nav2_msgs isn't available)
            try:
                from nav2_msgs.srv import ClearEntireCostmap

                # Use the same service names as nav2_simple_commander (relative names)
                self._clear_costmap_global_srv = self._node.create_client(
                    ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap"
                )
                self._clear_costmap_local_srv = self._node.create_client(
                    ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap"
                )
            except Exception as exc:
                logger.warning(f"[INIT] Nav2 costmap clear services unavailable: {exc}")
                self._clear_costmap_global_srv = None
                self._clear_costmap_local_srv = None

            # Start mission service (manual trigger)
            self._start_service = self._node.create_service(Trigger, "/start_mission", self._handle_start_mission)

            # Mission result service
            self._result_service = self._node.create_service(
                GetMissionResult, "/get_mission_result", self._handle_get_mission_result
            )

            # Timeout configuration subscriber (allows dynamic timeout setting)
            self._timeout_sub = self._node.create_subscription(
                Float64, "/set_mission_timeout", self._handle_set_timeout, 10
            )

            # Initialize NavMesh sampler with config values
            self._sampler = NavMeshSampler(
                min_distance=self.config.manager.min_distance,
                max_distance=self.config.manager.max_distance,
                edge_margin=self.config.manager.edge_margin,
                max_sampling_attempts=self.config.sampling.max_attempts,
                validate_path=self.config.sampling.validate_path,
            )

            # Initialize marker publisher (as a separate node) with config values
            self._marker_publisher = MarkerPublisher(
                node_name=f"{self.node_name}_markers",
                arrow_length=self.config.markers.arrow_length,
                arrow_width=self.config.markers.arrow_width,
                arrow_height=self.config.markers.arrow_height,
                robot_length=self.config.markers.robot_length,
                robot_width=self.config.markers.robot_width,
                robot_height=self.config.markers.robot_height,
                start_topic=self.config.markers.start_topic,
                goal_topic=self.config.markers.goal_topic,
                robot_topic=self.config.markers.robot_topic,
                odom_topic=self.config.nav2.odom_topic,
                enabled=self.config.markers.enabled,
            )

            # ROS2 executor to service timers/subscriptions without blocking the sim loop.
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor.add_node(self._marker_publisher)

            # Auto-setup Isaac Sim teleport callback if robot_prim_path is provided
            if self._teleport_callback is None and self.config.manager.robot_prim_path:
                self._setup_isaac_sim_teleport()

            # QoS for pose topics
            pose_qos = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )

            # Publishers (using topic names from config)
            self._initial_pose_pub = self._node.create_publisher(
                PoseWithCovarianceStamped, self.config.nav2.initial_pose_topic, pose_qos
            )
            self._goal_pose_pub = self._node.create_publisher(PoseStamped, self.config.nav2.goal_pose_topic, pose_qos)

            # IL baseline node control publishers (to disable ViNT and trajectory follower when mission starts)
            from std_msgs.msg import Bool

            self._vint_enable_pub = self._node.create_publisher(Bool, "/vint_enable", 10)
            self._trajectory_follower_enable_pub = self._node.create_publisher(Bool, "/trajectory_follower_enable", 10)

            # Subscribe to odometry for robot position tracking
            self._odom_sub = self._node.create_subscription(
                Odometry, self.config.nav2.odom_topic, self._odom_callback, 10
            )

            # Setup goal image capture if enabled
            if self.config.goal_image.enabled:
                self._setup_goal_image_publisher(pose_qos)
                self._setup_goal_camera()

            # Setup food tracking if enabled
            self._setup_food_tracking()

            # Setup contact reporting for impulse health tracking
            self._setup_contact_reporting()

            self._initialized = True
            self._state = MissionState.WAITING_FOR_NAV2
            self._wait_start_time = self._get_current_time_seconds()

            logger.info(f"[{self._state.name}] Mission manager initialized")
            logger.info(
                f"[{self._state.name}] Distance range: {self.config.manager.min_distance}m - {self.config.manager.max_distance}m"
            )
            logger.info(f"[{self._state.name}] Goal tolerance: {self.config.goal_tolerance}m")

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
            MissionState.CLEARING_COSTMAPS,
            MissionState.PUBLISHING_GOAL,
            MissionState.WAITING_FOR_COMPLETION,
        }

    def _reset_mission_state(self) -> None:
        """Reset mission state so a new mission can start cleanly."""
        self._current_start = None
        self._current_goal = None
        self._mission_start_time = None
        self._mission_end_time = None
        self._settle_steps_remaining = 0
        self._last_mission_result = MissionResult.PENDING
        self._last_mission_result_reason = None
        self._last_mission_distance = None
        self._traveled_distance = 0.0
        self._prev_robot_position = None
        self._last_traveled_distance = None
        self._last_elapsed_time = None
        self._last_contact_count = None
        self._last_total_impulse = None
        self._last_people_contact_count = None
        self._reset_impulse_health()
        # Reset food tracking
        self._initial_food_piece_count = 0
        self._final_food_piece_count = None

        # Reset costmap clear tracking
        self._costmap_clear_futures = {}
        self._costmap_clear_start_time = None

    def _odom_callback(self, msg) -> None:
        """Handle odometry messages for robot position and orientation tracking."""
        pos = msg.pose.pose.position
        self._robot_position = (pos.x, pos.y, pos.z)
        orient = msg.pose.pose.orientation
        self._robot_orientation = (orient.x, orient.y, orient.z, orient.w)

        """Handle odometry messages for robot position tracking and distance accumulation."""
        new_position = (pos.x, pos.y, pos.z)

        # Accumulate traveled distance during active mission
        if self._state == MissionState.WAITING_FOR_COMPLETION and self._prev_robot_position is not None:
            dx = new_position[0] - self._prev_robot_position[0]
            dy = new_position[1] - self._prev_robot_position[1]
            step_distance = math.sqrt(dx * dx + dy * dy)
            # Only accumulate reasonable movements (filter out noise and teleportation)
            if step_distance < 1.0:  # Max 1m per odom update (reasonable for robot)
                self._traveled_distance += step_distance

        self._prev_robot_position = new_position
        self._robot_position = new_position

    def _get_distance_to_goal(self) -> Optional[float]:
        """Calculate distance from robot to goal.

        Returns:
            Distance in meters, or None if positions unknown.
        """
        if self._robot_position is None or self._current_goal is None:
            return None

        rx, ry, _ = self._robot_position
        gx, gy = self._current_goal.x, self._current_goal.y
        return math.sqrt((rx - gx) ** 2 + (ry - gy) ** 2)

    def _is_robot_fallen(self, limit_angle: float = 0.5) -> bool:
        """Check if the robot has fallen based on orientation.

        This checks if the robot's orientation deviates too much from upright.
        It computes the angle between the robot's up vector (z-axis in body frame)
        and the world up vector (gravity direction).

        Args:
            limit_angle: Maximum allowed tilt angle in radians (default: ~28.6 degrees).
                        If the robot tilts beyond this, it's considered fallen.

        Returns:
            True if the robot has fallen, False otherwise.
        """
        if self._robot_orientation is None:
            return False

        qx, qy, _qz, _qw = self._robot_orientation

        # Compute the z-component of the body z-axis projected to world z
        # For a quaternion (qx, qy, qz, qw), this is:
        # gz_body_z = 1 - 2 * (qx*qx + qy*qy)
        # When robot is upright: gz_body_z = 1, tilt_angle = 0
        # When robot is tilted 90°: gz_body_z = 0, tilt_angle = π/2
        # When robot is upside down: gz_body_z = -1, tilt_angle = π
        gz_body_z = 1.0 - 2.0 * (qx * qx + qy * qy)

        # Compute tilt angle from upright (angle between body z and world z)
        # gz_body_z is the cosine of the tilt angle
        tilt_angle = math.acos(min(1.0, max(-1.0, gz_body_z)))

        return tilt_angle > limit_angle

    def _reset_impulse_health(self) -> None:
        self._impulse_damage_accumulated = 0.0
        self._impulse_health = self._impulse_health_max
        self._last_damage_steps_remaining = 0
        self._contact_count = 0
        self._people_contact_count = 0
        self._total_impulse = 0.0
        self._property_contact_counts = {key: 0 for key in PROPERTY_PRIM_PATHS}
        # Reset delta-v and injury cost tracking
        self._delta_v_magnitudes_mps = []
        self._injury_costs = []
        self._total_injury_cost = 0.0

    def _setup_contact_reporting(self) -> None:
        food_root = self.config.food.prim_path
        if food_root:
            self._food_root_prim_path = food_root.rstrip("/")
            self._food_prefix_path = f"{self._food_root_prim_path}/"
        else:
            self._food_root_prim_path = None
            self._food_prefix_path = None

        base_link_path = self.config.teleport.robot_prim or self.config.manager.robot_prim_path
        if base_link_path:
            base_link_path = base_link_path.rstrip("/")

        robot_hint = base_link_path.lower() if base_link_path else ""
        if "segway" in robot_hint:
            base_link_path = "/World/Segway_E1_ROS2/base_link"
        elif "nova_carter" in robot_hint or "carter" in robot_hint:
            base_link_path = "/World/Nova_Carter_ROS/chassis_link"

        if not base_link_path:
            logger.warning("[CONTACT] Robot prim path not configured; skipping contact reporting")
            return

        try:
            import omni.usd
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysxSchema

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(base_link_path)
            if not prim.IsValid():
                logger.warning(f"[CONTACT] Base link prim not found: {base_link_path}")
                return

            contact_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contact_api.CreateThresholdAttr().Set(self._impulse_min_threshold)
            self._contact_report_targets = {base_link_path}

            if self._contact_report_subscription is None:
                sim_interface = get_physx_simulation_interface()
                self._contact_report_subscription = sim_interface.subscribe_contact_report_events(
                    self._on_contact_report
                )

            logger.info(f"[CONTACT] Contact reporting enabled for {base_link_path}")
        except ImportError as exc:
            logger.warning(f"[CONTACT] Isaac Sim modules not available: {exc}")
        except Exception as exc:
            logger.error(f"[CONTACT] Failed to setup contact reporting: {exc}")

    def _setup_goal_image_publisher(self, qos_profile) -> None:
        """Setup ROS2 publisher for goal images.

        Args:
            qos_profile: QoS profile with TRANSIENT_LOCAL durability for reliable delivery.
        """
        from sensor_msgs.msg import Image

        self._goal_image_pub = self._node.create_publisher(Image, self.config.goal_image.topic, qos_profile)
        logger.info(f"[GOAL_IMAGE] Goal image publisher initialized on {self.config.goal_image.topic}")

    def _setup_goal_camera(self) -> None:
        """Setup Isaac Sim camera at goal location for goal image capture.

        Creates a camera prim at the configured path and sets up replicator
        render product and RGB annotator for image capture.
        """
        try:
            import omni.replicator.core as rep
            from isaacsim.core.utils import prims as prim_utils
            from pxr import Gf, UsdGeom

            goal_image_cfg = self.config.goal_image
            cam_prim_path = goal_image_cfg.camera_prim_path
            resolution = (goal_image_cfg.width, goal_image_cfg.height)

            # Create camera prim if it doesn't exist
            stage = self._stage
            if stage is None:
                import omni.usd

                stage = omni.usd.get_context().get_stage()
                self._stage = stage

            existing_prim = stage.GetPrimAtPath(cam_prim_path)
            if existing_prim.IsValid():
                logger.info(f"[GOAL_IMAGE] Reusing existing camera at {cam_prim_path}")
                self._goal_camera_prim = existing_prim
            else:
                # Create camera prim at a default position (will be moved to goal pose)
                # Orientation includes 180-degree rotation around viewing axis to match rgb_left camera
                self._goal_camera_prim = prim_utils.create_prim(
                    cam_prim_path,
                    prim_type="Camera",
                    translation=(0.0, 0.0, goal_image_cfg.camera_height_offset),
                    orientation=(0.5, 0.5, 0.5, 0.5),  # ROS convention + 180deg rotation
                )
                # Configure camera properties (matching rgb_left.usda parameters)
                camera_geom = UsdGeom.Camera(self._goal_camera_prim)
                camera_geom.GetFocalLengthAttr().Set(2.87343)
                camera_geom.GetFocusDistanceAttr().Set(0.6)
                camera_geom.GetHorizontalApertureAttr().Set(5.76)
                camera_geom.GetVerticalApertureAttr().Set(3.6)
                camera_geom.GetClippingRangeAttr().Set(Gf.Vec2f(0.076, 100000.0))
                logger.info(f"[GOAL_IMAGE] Created goal camera at {cam_prim_path}")

            # Create render product for the camera
            self._goal_render_product = rep.create.render_product(cam_prim_path, resolution=resolution)

            # Create RGB annotator and attach to render product
            self._goal_rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
            self._goal_rgb_annotator.attach(self._goal_render_product)

            logger.info(f"[GOAL_IMAGE] Goal camera setup complete: {resolution[0]}x{resolution[1]} @ {cam_prim_path}")

        except ImportError as exc:
            logger.warning(f"[GOAL_IMAGE] Isaac Sim modules not available: {exc}")
            self.config.goal_image.enabled = False
        except Exception as exc:
            logger.error(f"[GOAL_IMAGE] Failed to setup goal camera: {exc}")
            import traceback

            logger.error(f"[GOAL_IMAGE] {traceback.format_exc()}")
            self.config.goal_image.enabled = False

    def _capture_and_publish_goal_image(self, goal_position) -> bool:
        """Capture goal image from camera at goal position and publish to ROS2.

        Positions the goal camera at the goal location, renders the scene,
        captures the RGB image, and publishes it to the goal image topic.

        Args:
            goal_position: SampledPosition with x, y, yaw for goal pose.

        Returns:
            True if goal image was captured and published successfully, False otherwise.
        """
        if not self.config.goal_image.enabled:
            return False

        if self._goal_camera_prim is None or self._goal_rgb_annotator is None:
            logger.warning("[GOAL_IMAGE] Goal camera not initialized")
            return False

        try:
            import numpy as np
            from pxr import Gf, UsdGeom

            # Position camera at goal location
            xform = UsdGeom.Xformable(self._goal_camera_prim)

            # Get or create translate operation
            translate_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
            translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()

            # Get or create orient operation
            orient_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeOrient]
            orient_op = orient_ops[0] if orient_ops else xform.AddOrientOp()

            # Set camera position (goal x, y with configured height)
            camera_pos = Gf.Vec3d(goal_position.x, goal_position.y, self.config.goal_image.camera_height_offset)
            translate_op.Set(camera_pos)

            # Set camera orientation based on goal heading using _yaw_to_quaternion
            # Convert heading to quaternion and apply to camera
            quat_xyzw = self._yaw_to_quaternion(goal_position.heading)
            # _yaw_to_quaternion returns (x, y, z, w), Gf.Quatd expects (w, x, y, z)
            yaw_quat = Gf.Quatd(quat_xyzw[3], Gf.Vec3d(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]))
            # Base camera orientation with 180-degree rotation to match rgb_left camera
            # Original ROS convention (0.5, -0.5, 0.5, -0.5) + 180deg rotation = (0.5, 0.5, 0.5, 0.5)
            base_quat = Gf.Quatd(0.5, Gf.Vec3d(0.5, 0.5, 0.5))
            final_quat = yaw_quat * base_quat
            orient_op.Set(final_quat)

            # Step simulation with rendering to ensure the annotator buffer is updated
            # The render pipeline is asynchronous, so we need multiple steps to flush the pipeline
            # Using step(render=True) instead of just render() to properly update annotator buffers
            for _ in range(3):
                self.simulation_context.step(render=True)

            # Capture image from annotator
            rgb_data = self._goal_rgb_annotator.get_data()

            if rgb_data is None:
                logger.warning("[GOAL_IMAGE] No image data from annotator")
                return False

            # Convert to numpy array if needed and ensure RGB format (drop alpha if present)
            if isinstance(rgb_data, np.ndarray):
                if rgb_data.ndim == 3 and rgb_data.shape[2] == 4:
                    rgb_data = rgb_data[:, :, :3]  # Drop alpha channel
                # Note: 180-degree rotation is now handled by camera orientation, no image flip needed
            else:
                logger.warning(f"[GOAL_IMAGE] Unexpected data type: {type(rgb_data)}")
                return False

            # Convert to ROS Image message and publish (without cv_bridge)
            from sensor_msgs.msg import Image

            img_msg = Image()
            img_msg.height = rgb_data.shape[0]
            img_msg.width = rgb_data.shape[1]
            img_msg.encoding = "rgb8"
            img_msg.is_bigendian = False
            img_msg.step = rgb_data.shape[1] * 3  # 3 bytes per pixel for RGB
            img_msg.data = rgb_data.astype(np.uint8).tobytes()
            img_msg.header.stamp = self._node.get_clock().now().to_msg()
            img_msg.header.frame_id = "goal_camera"
            self._goal_image_pub.publish(img_msg)

            logger.info(
                f"[GOAL_IMAGE] Published goal image ({rgb_data.shape[1]}x{rgb_data.shape[0]}) "
                f"from position ({goal_position.x:.2f}, {goal_position.y:.2f}, yaw={goal_position.heading:.2f})"
            )
            return True

        except Exception as exc:
            logger.error(f"[GOAL_IMAGE] Failed to capture/publish goal image: {exc}")
            import traceback

            logger.error(f"[GOAL_IMAGE] {traceback.format_exc()}")
            return False

    def _classify_property_from_prim_path(self, prim_path: str) -> Optional[str]:
        if not prim_path:
            return None
        # Exception in trash bin path.
        if prim_path.startswith("/World/Environment/SM_Buidlng_032/SM_Buidlng_032/Section26"):
            return "trash_bin"
        # Broad match for any building prims (handle both SM_Buidlng_ and SM_Buildlng_)
        if prim_path.startswith("/World/Environment/SM_Buidlng_") or prim_path.startswith(
            "/World/Environment/SM_Buildlng_"
        ):
            return "building"
        for category, paths in PROPERTY_PRIM_PATHS.items():
            for base_path in paths:
                if prim_path == base_path or prim_path.startswith(base_path + "/"):
                    return category
        return None

    def _record_property_contact_from_pair(
        self,
        actor0_path: str,
        actor1_path: str,
        impulse_amount: float,
    ) -> Optional[str]:
        """Record property contact from a pair of actor prim paths."""
        if impulse_amount < self._property_contact_impulse_min_threshold:
            return None

        target = next(iter(self._contact_report_targets), None)
        if target:
            if actor0_path == target or actor0_path.startswith(target + "/"):
                category = self._classify_property_from_prim_path(actor1_path)
            elif actor1_path == target or actor1_path.startswith(target + "/"):
                category = self._classify_property_from_prim_path(actor0_path)
            else:
                category = None
        else:
            category = None

        if category is None:
            category = self._classify_property_from_prim_path(actor0_path)
        if category is None:
            category = self._classify_property_from_prim_path(actor1_path)

        if category is None:
            return None
        if category not in _COST_PROPERTY_CATEGORIES:
            return None

        print(f"[CONTACT] Property contact: {category} {impulse_amount}")

        self._property_contact_counts[category] += 1
        return category

    def _apply_impulse_damage(
        self, impulse_amount: float, injury_info: "tuple[float, float, float] | None" = None
    ) -> None:
        # Only calculate health damage when mission is active
        if not self._is_mission_active():
            msg = f"[CONTACT] Impulse: {impulse_amount:.2f}"
            if injury_info:
                delta_v_mps, injury_cost, total_injury_cost = injury_info
                msg += (
                    f" | delta_v={delta_v_mps:.4f} m/s ({delta_v_mps * self._MPS_TO_MPH:.2f} mph), "
                    f"{injury_cost=:.2f}, {total_injury_cost=:.2f}"
                )
            print(msg)
            return

        if self._impulse_health <= 0.0:
            return

        # Track contact count and total impulse for evaluation metrics
        self._contact_count += 1
        self._total_impulse += impulse_amount

        self._impulse_damage_accumulated += impulse_amount
        self._impulse_health = max(0.0, self._impulse_health_max - self._impulse_damage_accumulated)
        msg = (
            f"[CONTACT] Impulse: {impulse_amount:.2f}, Health: {self._impulse_health:.2f}, Count: {self._contact_count}"
        )
        if injury_info:
            delta_v_mps, injury_cost, total_injury_cost = injury_info
            msg += (
                f" | delta_v={delta_v_mps:.4f} m/s ({delta_v_mps * self._MPS_TO_MPH:.2f} mph), "
                f"cost={injury_cost:.2f}, total={total_injury_cost:.2f}"
            )
        print(msg)

    # MAIS logistic regression coefficients from crash data
    # Format: (intercept, slope) for P(MAIS >= level) = 1 / (1 + exp(-(intercept + slope * delta_v_mph)))
    _MAIS_COEFFICIENTS = {
        "all": {
            "mais_1": (-1.3925, 0.0815),
            "mais_2": (-5.1331, 0.1479),
            "mais_3": (-6.9540, 0.1637),
            "mais_4": (-8.2070, 0.1564),
            "mais_5": (-8.7927, 0.1598),
            "fatality": (-8.9819, 0.1603),
        },
    }
    _MPS_TO_MPH = 2.23694
    _MAIS_LEVELS = ("mais_0", "mais_1", "mais_2", "mais_3", "mais_4", "mais_5", "fatality")

    @staticmethod
    def _logistic_probability(intercept: float, slope: float, delta_v_mph: float) -> float:
        """Compute logistic probability P(MAIS >= level)."""
        exponent = intercept + slope * delta_v_mph
        return math.exp(exponent) / (1.0 + math.exp(exponent))

    def _compute_mais_probabilities(self, delta_v_mps: float) -> dict:
        """Compute MAIS level probabilities from delta-v using logistic regression."""
        if delta_v_mps <= 0.0:
            return {level: 1.0 if level == "mais_0" else 0.0 for level in self._MAIS_LEVELS}

        delta_v_mph = delta_v_mps * self._MPS_TO_MPH
        crash_mode = self.config.injury.crash_mode or "all"
        coefficients = self._MAIS_COEFFICIENTS.get(crash_mode, self._MAIS_COEFFICIENTS["all"])

        # Compute cumulative probabilities P(MAIS >= level)
        mais_1_plus = self._logistic_probability(*coefficients["mais_1"], delta_v_mph)
        mais_2_plus = self._logistic_probability(*coefficients["mais_2"], delta_v_mph)
        mais_3_plus = self._logistic_probability(*coefficients["mais_3"], delta_v_mph)
        mais_4_plus = self._logistic_probability(*coefficients["mais_4"], delta_v_mph)
        mais_5_plus = self._logistic_probability(*coefficients["mais_5"], delta_v_mph)
        fatality = self._logistic_probability(*coefficients["fatality"], delta_v_mph)

        # Ensure monotonicity
        mais_2_plus = min(mais_2_plus, mais_1_plus)
        mais_3_plus = min(mais_3_plus, mais_2_plus)
        mais_4_plus = min(mais_4_plus, mais_3_plus)
        mais_5_plus = min(mais_5_plus, mais_4_plus)
        fatality = min(fatality, mais_5_plus)

        # Convert cumulative to individual level probabilities
        return {
            "mais_0": 1.0 - mais_1_plus,
            "mais_1": mais_1_plus - mais_2_plus,
            "mais_2": mais_2_plus - mais_3_plus,
            "mais_3": mais_3_plus - mais_4_plus,
            "mais_4": mais_4_plus - mais_5_plus,
            "mais_5": mais_5_plus - fatality,
            "fatality": fatality,
        }

    # Injury cost adjustment factor to scale expected costs
    _INJURY_COST_ADJUSTMENT_FACTOR = 0.0110

    def _compute_expected_injury_cost(self, probabilities: dict) -> float:
        """Compute expected injury cost from MAIS probabilities and configured costs."""
        costs = self.config.injury.costs
        raw_cost = (
            probabilities.get("mais_0", 0.0) * costs.mais_0
            + probabilities.get("mais_1", 0.0) * costs.mais_1
            + probabilities.get("mais_2", 0.0) * costs.mais_2
            + probabilities.get("mais_3", 0.0) * costs.mais_3
            + probabilities.get("mais_4", 0.0) * costs.mais_4
            + probabilities.get("mais_5", 0.0) * costs.mais_5
            + probabilities.get("fatality", 0.0) * costs.fatality
        )
        return raw_cost * self._INJURY_COST_ADJUSTMENT_FACTOR

    def _process_collision_injury(
        self, impulse_amount: float, is_character_collision: bool
    ) -> "tuple[float, float, float] | None":
        """Compute delta-v from impulse/mass and calculate injury cost.

        Returns:
            A tuple of (delta_v_mps, injury_cost, total_injury_cost) if injury tracking
            is enabled, otherwise None.
        """
        if not self.config.injury.enabled or not is_character_collision:
            return None

        robot_mass = self.config.injury.robot_mass
        delta_v_mps = impulse_amount / robot_mass
        self._delta_v_magnitudes_mps.append(delta_v_mps)

        probabilities = self._compute_mais_probabilities(delta_v_mps)
        injury_cost = self._compute_expected_injury_cost(probabilities)
        self._injury_costs.append(injury_cost)
        self._total_injury_cost += injury_cost

        return (delta_v_mps, injury_cost, self._total_injury_cost)

    def _on_contact_report(self, contact_headers, contact_data) -> None:
        if not self._contact_report_targets:
            return

        if self._last_damage_steps_remaining > 0:
            self._last_damage_steps_remaining = max(0, self._last_damage_steps_remaining - 1)
            return

        try:
            from pxr import PhysicsSchemaTools
        except ImportError:
            return

        food_root: str = self._food_root_prim_path or ""
        for header in contact_headers:
            actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            if (actor0 not in self._contact_report_targets) and (actor1 not in self._contact_report_targets):
                continue

            if food_root:
                food_prefix_str: str = f"{food_root}/"
                if actor0 in self._contact_report_targets and (
                    actor1 == food_root or actor1[: len(food_prefix_str)] == food_prefix_str
                ):
                    continue
                if actor1 in self._contact_report_targets and (
                    actor0 == food_root or actor0[: len(food_prefix_str)] == food_prefix_str
                ):
                    continue

            if header.num_contact_data == 0:
                continue

            is_character_collision = False
            if "/World/Characters/" in actor0 or "/World/Characters/" in actor1:
                is_character_collision = True

            contact_range = range(
                header.contact_data_offset,
                header.contact_data_offset + header.num_contact_data,
            )
            for idx in contact_range:
                impulse = contact_data[idx].impulse
                impulse_amount = (impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z) ** 0.5
                if impulse_amount < self._impulse_min_threshold:
                    continue
                if is_character_collision:
                    self._people_contact_count += 1
                self._record_property_contact_from_pair(actor0, actor1, impulse_amount)
                # Compute delta-v from impulse/mass and calculate injury cost
                injury_info = self._process_collision_injury(impulse_amount, is_character_collision)
                self._apply_impulse_damage(impulse_amount, injury_info)
                self._last_damage_steps_remaining = self._damage_cooldown_steps
                return

    def handle_simulation_restart(self, stage_reloaded: bool = False) -> None:
        """Refresh cached sim handles after stop/reset or stage reload."""
        self._reset_impulse_health()
        self._contact_report_subscription = None
        self._contact_report_targets = set()
        self._setup_contact_reporting()

        if stage_reloaded or self._teleport_callback is None:
            self._setup_isaac_sim_teleport()

        if stage_reloaded:
            self._setup_food_tracking()

    def _count_food_pieces_in_bucket(self) -> int:
        """Count the number of food pieces currently inside the bucket.

        Uses bounding box comparison to determine if pieces are within the bucket.
        This method iterates through all piece prims under the pieces path and
        checks if their positions fall within the bucket's bounding box.

        Returns:
            Number of pieces inside the bucket, or 0 if food tracking is disabled.
        """
        if not self.config.food.enabled or self._food_pieces_prim_path is None:
            return 0

        try:
            import omni.usd
            from isaacsim.core.utils import bounds as bounds_utils
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()

            # Get bucket bounding box
            bucket_prim = stage.GetPrimAtPath(self._food_bucket_prim_path)
            if not bucket_prim.IsValid():
                logger.warning(f"[FOOD] Bucket prim not found: {self._food_bucket_prim_path}")
                return 0

            bbox_cache = bounds_utils.create_bbox_cache()
            bucket_bounds = bounds_utils.compute_aabb(bbox_cache, prim_path=self._food_bucket_prim_path)
            min_xyz = bucket_bounds[:3]
            max_xyz = bucket_bounds[3:]

            # Get pieces parent prim and count children inside bucket
            pieces_prim = stage.GetPrimAtPath(self._food_pieces_prim_path)
            if not pieces_prim.IsValid():
                logger.warning(f"[FOOD] Pieces prim not found: {self._food_pieces_prim_path}")
                return 0

            count = 0
            for child in pieces_prim.GetChildren():
                if not child.IsValid():
                    continue
                xformable = UsdGeom.Xformable(child)
                if not xformable:
                    continue

                # Get world position of the piece
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                position = world_transform.ExtractTranslation()

                # Check if position is within bucket bounds
                if (
                    min_xyz[0] <= position[0] <= max_xyz[0]
                    and min_xyz[1] <= position[1] <= max_xyz[1]
                    and min_xyz[2] <= position[2] <= max_xyz[2]
                ):
                    count += 1

            return count

        except Exception as e:
            logger.error(f"[FOOD] Error counting food pieces: {e}")
            return 0

    def _get_robot_z_offset(self) -> Optional[float]:
        """Get the robot-specific z offset for positioning.

        This offset is used for both robot teleportation and food positioning
        to ensure consistent height placement above the ground.

        Returns:
            Z offset in meters, or None if robot not supported.

        Robot-specific offsets:
        - segway_e1: 0.33m
        - nova_carter: Uses config teleport.height_offset (default behavior)
        - Other robots: Not yet implemented
        """
        robot_prim = self.config.teleport.robot_prim.lower()

        if "segway" in robot_prim:
            return 0.33  # Segway E1 height offset
        elif "nova_carter" in robot_prim:
            # Nova Carter uses the config teleport.height_offset
            return self.config.manager.teleport_height
        else:
            # Other robots: return None to indicate not supported for food
            # but teleportation can still use config value
            return None

    def _spawn_food_at_position(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0, remove_existing: bool = False
    ) -> bool:
        """Spawn food USD asset at the specified position.

        This is the internal method used by both _setup_food_tracking and
        _reset_food_for_teleport to spawn or respawn the food asset.

        Args:
            x: X position in world coordinates.
            y: Y position in world coordinates.
            z: Z position in world coordinates (z_offset will be added).
            remove_existing: If True, remove existing food prim before spawning.

        Returns:
            True if food was successfully spawned, False otherwise.
        """
        food_config = self.config.food
        base_path = food_config.prim_path.rstrip("/")

        z_offset = self._get_robot_z_offset()
        if z_offset is None:
            # Robot not supported for food spawning
            robot_prim = self.config.teleport.robot_prim.lower()
            logger.warning(
                f"[FOOD] Food spawning not implemented for robot: {robot_prim}. "
                f"Currently only segway_e1 is supported. Disabling food tracking."
            )
            self.config.food.enabled = False
            return False

        try:
            import omni.usd
            from pxr import Gf, UsdGeom

            stage = omni.usd.get_context().get_stage()

            # Remove existing food prim if requested
            if remove_existing:
                food_prim = stage.GetPrimAtPath(base_path)
                if food_prim.IsValid():
                    stage.RemovePrim(base_path)
                    logger.info(f"[FOOD] Removed existing food prim at: {base_path}")

            # Check if food prim already exists (if not removing)
            food_prim = stage.GetPrimAtPath(base_path)
            if food_prim.IsValid():
                logger.info(f"[FOOD] Food prim already exists at: {base_path}")
                return True

            # Create Xform prim and add USD reference
            food_prim = stage.DefinePrim(base_path, "Xform")
            if not food_prim.IsValid():
                logger.error(f"[FOOD] Failed to create prim at: {base_path}")
                return False

            # Add reference to the food USD asset
            success = food_prim.GetReferences().AddReference(food_config.usd_path)
            if not success:
                logger.error(f"[FOOD] Failed to add USD reference: {food_config.usd_path}")
                return False

            # Apply position with robot-specific z offset
            final_z = z + z_offset
            xformable = UsdGeom.Xformable(food_prim)
            xformable.ClearXformOpOrder()
            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(x, y, final_z))

            logger.info(f"[FOOD] Spawned food asset from: {food_config.usd_path}")
            logger.info(f"[FOOD] Position: ({x:.2f}, {y:.2f}, {final_z:.2f})")
            return True

        except Exception as e:
            logger.error(f"[FOOD] Error spawning food asset: {e}")
            return False

    def _setup_food_tracking(self) -> None:
        """Setup food tracking by spawning the food USD asset and configuring paths.

        Spawns the food USD asset as a reference at the configured prim path,
        then constructs full prim paths for the food pieces and bucket.
        """
        if not self.config.food.enabled:
            return

        # Spawn food at origin (will be repositioned on first teleport)
        if not self._spawn_food_at_position(x=0.0, y=0.0, z=0.0):
            return

        # Setup prim paths for tracking
        food_config = self.config.food
        base_path = food_config.prim_path.rstrip("/")
        self._food_pieces_prim_path = f"{base_path}/{food_config.pieces_prim_path}"
        self._food_bucket_prim_path = f"{base_path}/{food_config.bucket_prim_path}"

        logger.info("[FOOD] Food tracking enabled")
        logger.info(f"[FOOD] Pieces path: {self._food_pieces_prim_path}")
        logger.info(f"[FOOD] Bucket path: {self._food_bucket_prim_path}")

    def _check_food_spoilage(self) -> bool:
        """Check if food has spoiled (pieces lost from bucket).

        Compares the initial piece count to the current count and determines
        if the loss exceeds the configured threshold.

        Returns:
            True if food has spoiled (too many pieces lost), False otherwise.
        """
        if not self.config.food.enabled:
            return False

        self._final_food_piece_count = self._count_food_pieces_in_bucket()

        if self._initial_food_piece_count == 0:
            return False

        pieces_lost = self._initial_food_piece_count - self._final_food_piece_count
        loss_fraction = pieces_lost / self._initial_food_piece_count

        logger.info(
            f"[FOOD] Pieces: initial={self._initial_food_piece_count}, "
            f"final={self._final_food_piece_count}, lost={pieces_lost} ({loss_fraction:.1%})"
        )

        return loss_fraction > self.config.food.spoilage_threshold

    def _reset_food_for_teleport(self) -> bool:
        """Reset food by removing and respawning at robot's current position.

        When the robot teleports to a new start position and settles, the food
        (e.g., popcorn bucket) must be repositioned to match. This method reads
        the robot's actual position from its prim (base_link or chassis_link)
        and spawns the food at that location.

        Returns:
            True if food was successfully reset, False otherwise.
        """
        if not self.config.food.enabled:
            return True

        # Get robot's actual position from its prim after settling
        try:
            import omni.usd
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()
            robot_prim_path = self.config.teleport.robot_prim
            robot_prim = stage.GetPrimAtPath(robot_prim_path)

            if not robot_prim.IsValid():
                logger.error(f"[FOOD] Robot prim not found: {robot_prim_path}")
                return False

            # Get robot's world transform
            xformable = UsdGeom.Xformable(robot_prim)
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            robot_pos = world_transform.ExtractTranslation()

            logger.info(
                f"[FOOD] Robot position from prim: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f})"
            )

            # Spawn food at robot's actual position (z_offset will be added)
            return self._spawn_food_at_position(
                x=robot_pos[0],
                y=robot_pos[1],
                z=robot_pos[2],
                remove_existing=True,
            )

        except Exception as e:
            logger.error(f"[FOOD] Error getting robot position: {e}")
            return False

    def _handle_set_timeout(self, msg):
        """Handle dynamic timeout configuration.

        Args:
            msg: Float64 message with timeout value in seconds.
                 Use 0 or negative value to disable timeout (infinite).
        """
        timeout_value = msg.data
        if timeout_value <= 0:
            self.config.timeout = None
            logger.info("[CONFIG] Mission timeout disabled (infinite)")
        else:
            self.config.timeout = timeout_value
            logger.info(f"[CONFIG] Mission timeout set to {timeout_value}s")

    def _handle_start_mission(self, _request, response):
        """Handle external mission start trigger."""
        if self._state == MissionState.COMPLETED:
            response.success = False
            response.message = "Missions already completed."
            return response

        # Disable IL baseline nodes (ViNT and trajectory follower) when mission starts
        # This stops the nodes from publishing trajectories and cmd_vel
        self._disable_il_baseline_nodes()

        # Reset mission result immediately to prevent race condition
        # where the eval script sees the previous mission's SUCCESS result
        self._last_mission_result = MissionResult.PENDING
        self._last_mission_result_reason = None
        self._last_mission_distance = None
        self._last_elapsed_time = None
        self._last_traveled_distance = None
        self._last_contact_count = None
        self._last_total_impulse = None
        self._last_people_contact_count = None
        self._last_property_contact_counts = None

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

    def _disable_il_baseline_nodes(self) -> None:
        """Disable IL baseline nodes (ViNT and trajectory follower).

        Publishes False to /vint_enable and /trajectory_follower_enable topics
        to stop the nodes from publishing trajectories and cmd_vel commands.
        """
        from std_msgs.msg import Bool

        disable_msg = Bool()
        disable_msg.data = False

        if self._vint_enable_pub is not None:
            self._vint_enable_pub.publish(disable_msg)
            logger.info("[START_MISSION] Disabled ViNT policy node")

        if self._trajectory_follower_enable_pub is not None:
            self._trajectory_follower_enable_pub.publish(disable_msg)
            logger.info("[START_MISSION] Disabled trajectory follower node")

    def _enable_il_baseline_nodes(self) -> None:
        """Enable IL baseline nodes (ViNT and trajectory follower).

        Publishes True to /vint_enable and /trajectory_follower_enable topics
        to allow the nodes to resume publishing trajectories and cmd_vel commands.
        Called after mission setup is complete (goal published, markers updated).
        """
        from std_msgs.msg import Bool

        enable_msg = Bool()
        enable_msg.data = True

        if self._vint_enable_pub is not None:
            self._vint_enable_pub.publish(enable_msg)
            logger.info("[PUBLISHING_GOAL] Enabled ViNT policy node")

        if self._trajectory_follower_enable_pub is not None:
            self._trajectory_follower_enable_pub.publish(enable_msg)
            logger.info("[PUBLISHING_GOAL] Enabled trajectory follower node")

    def _handle_get_mission_result(self, _request, response):
        """Handle mission result query service.

        Returns the result of the last completed mission as JSON in the message field.
        Response format:
        {
            "result": "pending" | "success" | "failure",
            "result_reason": str | null,  # specific reason (e.g., "orientation", "impulse_health_depletion")
            "mission_number": int,
            "distance_to_goal": float,
            "in_progress": bool,
            "traveled_distance": float,  # meters
            "elapsed_time": float,  # seconds
            "total_contact_count": int,  # number of contact events
            "total_impulse": float,  # accumulated impulse in N*s
            "property_contact_bollard": int,
            "property_contact_building": int,
            "property_contact_trash_bin": int,
            "property_contact_mail_box": int,
            "property_contact_total": int,
            "people_contact_count": int,
            "food_enabled": bool,
            "food_initial_pieces": int,
            "food_final_pieces": int,
            "food_loss_fraction": float,
            "food_spoiled": bool
        }
        """
        import json

        in_progress = self._state == MissionState.WAITING_FOR_COMPLETION
        distance = self._last_mission_distance if self._last_mission_distance is not None else -1.0

        if in_progress:
            current_dist = self._get_distance_to_goal()
            distance = current_dist if current_dist is not None else -1.0

        # Calculate elapsed time
        if in_progress and self._mission_start_time is not None:
            elapsed_time = self._get_current_time_seconds() - self._mission_start_time
        elif self._last_elapsed_time is not None:
            elapsed_time = self._last_elapsed_time
        else:
            elapsed_time = -1.0

        # Get traveled distance (current if in progress, or final from last mission)
        if in_progress:
            traveled_distance = self._traveled_distance
        elif self._last_traveled_distance is not None:
            traveled_distance = self._last_traveled_distance
        else:
            traveled_distance = self._traveled_distance  # Fallback to current

        # Get contact count and total impulse (current if in progress, or final from last mission)
        if in_progress:
            total_contact_count = self._contact_count
            total_impulse = self._total_impulse
            people_contact_count = self._people_contact_count
            property_counts = dict(self._property_contact_counts)
            delta_v_list = list(self._delta_v_magnitudes_mps)
            injury_costs = list(self._injury_costs)
            total_injury_cost = self._total_injury_cost
        elif self._last_contact_count is not None:
            total_contact_count = self._last_contact_count
            total_impulse = self._last_total_impulse if self._last_total_impulse is not None else 0.0
            people_contact_count = self._last_people_contact_count if self._last_people_contact_count is not None else 0
            property_counts = dict(self._last_property_contact_counts or {})
            delta_v_list = list(self._last_delta_v_magnitudes_mps or [])
            injury_costs = list(self._last_injury_costs or [])
            total_injury_cost = self._last_total_injury_cost if self._last_total_injury_cost is not None else 0.0
        else:
            total_contact_count = self._contact_count
            total_impulse = self._total_impulse
            people_contact_count = self._people_contact_count
            property_counts = dict(self._property_contact_counts)
            delta_v_list = list(self._delta_v_magnitudes_mps)
            injury_costs = list(self._injury_costs)
            total_injury_cost = self._total_injury_cost

        # Compute delta-v statistics
        delta_v_count = len(delta_v_list)
        if delta_v_count > 0:
            delta_v_avg_mps = sum(delta_v_list) / delta_v_count
            delta_v_avg_mph = delta_v_avg_mps * self._MPS_TO_MPH
        else:
            delta_v_avg_mps = 0.0
            delta_v_avg_mph = 0.0

        # Food metrics
        food_enabled = bool(self.config.food.enabled)
        food_initial_pieces = self._initial_food_piece_count if food_enabled else -1
        if food_enabled and self._final_food_piece_count is not None:
            food_final_pieces = self._final_food_piece_count
        else:
            food_final_pieces = -1

        food_loss_fraction = -1.0
        if food_enabled and self._initial_food_piece_count > 0 and self._final_food_piece_count is not None:
            food_loss_fraction = (
                self._initial_food_piece_count - self._final_food_piece_count
            ) / self._initial_food_piece_count

        food_spoiled = self._last_mission_result == MissionResult.FAILURE_FOODSPOILED

        result_data = {
            "result": self._last_mission_result.value,
            "result_reason": self._last_mission_result_reason,
            "mission_number": self._current_mission,
            "distance_to_goal": distance,
            "in_progress": in_progress,
            "traveled_distance": traveled_distance,
            "elapsed_time": elapsed_time,
            "total_contact_count": total_contact_count,
            "total_impulse": total_impulse,
            "property_contact_bollard": property_counts.get("bollard", 0),
            "property_contact_building": property_counts.get("building", 0),
            "property_contact_trash_bin": property_counts.get("trash_bin", 0),
            "property_contact_mail_box": property_counts.get("mail_box", 0),
            "property_contact_total": sum(property_counts.values()),
            "people_contact_count": people_contact_count,
            "delta_v_count": delta_v_count,
            "delta_v_avg_mps": delta_v_avg_mps,
            "delta_v_avg_mph": delta_v_avg_mph,
            "injury_costs": injury_costs,
            "total_injury_cost": total_injury_cost,
            "food_enabled": food_enabled,
            "food_initial_pieces": food_initial_pieces,
            "food_final_pieces": food_final_pieces,
            "food_loss_fraction": food_loss_fraction,
            "food_spoiled": food_spoiled,
        }

        response.success = True
        response.message = json.dumps(result_data)
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
        # Read robot_prim from teleport config (set by launch.py based on robot selection)
        robot_prim_path = self.config.teleport.robot_prim
        if not robot_prim_path:
            logger.warning(
                f"[{self._state.name}] robot_prim not set in mission_config.teleport. "
                f"Teleport callback will not be registered. "
                f"teleport.robot_prim={robot_prim_path!r}"
            )
            return
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
        from pxr import Gf, Sdf, UsdGeom

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

                # Convert yaw to quaternion (w, x, y, z) format
                qx, qy, qz, qw = self._yaw_to_quaternion(position.heading)

                # Detect the expected quaternion type from the orient operation's attribute
                # Some USD prims use GfQuatf (float) while others use GfQuatd (double)
                orient_attr = orient_op.GetAttr()
                type_name = orient_attr.GetTypeName()

                if type_name == Sdf.ValueTypeNames.Quatf:
                    # Use float precision quaternion (GfQuatf)
                    orientation = Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz)))
                else:
                    # Use double precision quaternion (GfQuatd) - default
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

            # Get robot-specific z offset, fall back to config value if not supported
            z_offset = self._get_robot_z_offset()
            if z_offset is None:
                z_offset = self.config.manager.teleport_height

            # Add height offset for teleportation
            teleport_pos = SampledPosition(
                x=position.x,
                y=position.y,
                z=position.z + z_offset,
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
        if elapsed >= self.config.nav2.wait_time:
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
        self._reset_impulse_health()
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

        # After teleportation, we need to settle physics before spawning food
        self._settle_steps_remaining = self.config.manager.teleport_settle_steps
        self._state = MissionState.SETTLING
        logger.info(f"[SETTLING] Teleportation complete, settling physics for {self._settle_steps_remaining} steps")

    def _step_settling(self):
        """Wait for physics to settle after teleportation.

        This is critical: after XFormPrim.set_world_pose(), we must step the
        simulation multiple times to allow the physics engine to process the
        new pose and update all internal states.

        After settling, food is spawned at the robot's position.
        """
        if self._settle_steps_remaining > 0:
            # Simulation step is already called in main loop before this
            # We just count down the steps
            self._settle_steps_remaining -= 1
            if self._settle_steps_remaining == 0:
                logger.info(f"[{self._state.name}] Physics settled")

                # Spawn food at robot's actual position after robot has settled
                if self.config.food.enabled:
                    if not self._reset_food_for_teleport():
                        logger.warning(f"[{self._state.name}] Food reset failed, continuing without food tracking")

                logger.info(f"[{self._state.name}] Publishing initial pose")
                self._state = MissionState.PUBLISHING_INITIAL_POSE

    def _step_publishing_initial_pose(self):
        """Publish initial pose for AMCL."""
        self._publish_initial_pose(self._current_start)
        self._wait_start_time = self._get_current_time_seconds()
        # If we teleported, it's useful to clear Nav2 costmaps before sending a new goal.
        if self.config.manager.clear_costmaps_on_mission_start:
            self._state = MissionState.CLEARING_COSTMAPS
        else:
            self._state = MissionState.PUBLISHING_GOAL

    def _step_clearing_costmaps(self):
        """Clear Nav2 global/local costmaps (best-effort) before publishing a new goal."""

        # Defensive: allow disabling at runtime.
        if not self.config.manager.clear_costmaps_on_mission_start:
            self._state = MissionState.PUBLISHING_GOAL
            return

        if self._costmap_clear_start_time is None:
            self._costmap_clear_start_time = self._get_current_time_seconds()
            self._costmap_clear_futures = {}
            logger.info(f"[{self._state.name}] Clearing Nav2 costmaps")

        # If clients weren't created (nav2_msgs missing, or init failed), continue.
        if self._clear_costmap_global_srv is None and self._clear_costmap_local_srv is None:
            logger.warning(f"[{self._state.name}] Costmap clear clients not available, continuing")
            # Reset for next mission
            self._costmap_clear_start_time = None
            self._costmap_clear_futures = {}
            self._state = MissionState.PUBLISHING_GOAL
            return

        # Import locally so unit tests that don't run ROS2 don't require nav2_msgs.
        try:
            from nav2_msgs.srv import ClearEntireCostmap
        except Exception as exc:
            logger.warning(f"[{self._state.name}] nav2_msgs not available ({exc}); continuing without costmap clear")
            # Reset for next mission
            self._costmap_clear_start_time = None
            self._costmap_clear_futures = {}
            self._state = MissionState.PUBLISHING_GOAL
            return

        now = self._get_current_time_seconds()
        if (now - self._costmap_clear_start_time) >= self.config.manager.costmap_clear_timeout_sec:
            logger.info(
                f"[{self._state.name}] Timed out clearing costmaps after {self.config.manager.costmap_clear_timeout_sec:.2f}s, continuing"
            )
            # Reset for next mission
            self._costmap_clear_start_time = None
            self._costmap_clear_futures = {}
            self._state = MissionState.PUBLISHING_GOAL
            return

        def _kickoff(name: str, client) -> None:
            if client is None or name in self._costmap_clear_futures:
                return
            # Non-blocking wait; we retry each sim step until timeout.
            if not client.wait_for_service(timeout_sec=0.0):
                return
            req = ClearEntireCostmap.Request()
            self._costmap_clear_futures[name] = client.call_async(req)

        _kickoff("global", self._clear_costmap_global_srv)
        _kickoff("local", self._clear_costmap_local_srv)

        def _complete(name: str, client) -> bool:
            if client is None:
                return True
            fut = self._costmap_clear_futures.get(name)
            return fut is not None and fut.done()

        if _complete("global", self._clear_costmap_global_srv) and _complete("local", self._clear_costmap_local_srv):
            logger.info(f"[{self._state.name}] Costmaps cleared")
            # Reset for next mission
            self._costmap_clear_start_time = None
            self._costmap_clear_futures = {}
            self._state = MissionState.PUBLISHING_GOAL

    def _step_publishing_goal(self):
        """Publish goal pose to Nav2 after initial pose delay."""
        elapsed = self._get_current_time_seconds() - self._wait_start_time
        if elapsed >= self.config.manager.initial_pose_delay:
            self._publish_goal_pose(self._current_goal)

            # Capture and publish goal image for ViNT ImageGoal mode
            if self.config.goal_image.enabled:
                self._capture_and_publish_goal_image(self._current_goal)

            # Update RViz markers
            self._marker_publisher.publish_start_goal_from_sampled(self._current_start, self._current_goal)

            # Record initial food piece count for spoilage evaluation
            if self.config.food.enabled:
                self._initial_food_piece_count = self._count_food_pieces_in_bucket()
                logger.info(f"[FOOD] Initial piece count: {self._initial_food_piece_count}")

            # Enable IL baseline nodes now that mission setup is complete
            self._enable_il_baseline_nodes()

            logger.info(f"[{self._state.name}] Mission {self._current_mission} initiated successfully")
            self._mission_start_time = self._get_current_time_seconds()
            # Reset distance tracking for this mission
            self._traveled_distance = 0.0
            self._prev_robot_position = self._robot_position
            self._state = MissionState.WAITING_FOR_COMPLETION

    def _step_waiting_for_completion(self):
        """Wait for goal completion, timeout, or robot fall.

        Success: Robot within goal_tolerance of goal position.
        Failure (timeout): Timeout reached before reaching goal.
        Failure (physical assistance): Robot fell down (bad orientation).
        """
        elapsed = self._get_current_time_seconds() - self._mission_start_time
        distance = self._get_distance_to_goal()

        # Check for goal completion (success or food spoiled)
        if distance is not None and distance <= self.config.goal_tolerance:
            self._mission_end_time = self._get_current_time_seconds()
            self._last_mission_distance = distance
            self._last_elapsed_time = elapsed
            self._last_traveled_distance = self._traveled_distance
            self._last_contact_count = self._contact_count
            self._last_people_contact_count = self._people_contact_count
            self._last_total_impulse = self._total_impulse
            self._last_property_contact_counts = dict(self._property_contact_counts)
            self._last_delta_v_magnitudes_mps = list(self._delta_v_magnitudes_mps)
            self._last_injury_costs = list(self._injury_costs)
            self._last_total_injury_cost = self._total_injury_cost

            # Check for food spoilage before declaring success
            if self._check_food_spoilage():
                self._last_mission_result = MissionResult.FAILURE_FOODSPOILED
                self._state = MissionState.WAITING_FOR_START
                logger.info(
                    f"[FAILURE_FOODSPOILED] Mission {self._current_mission} failed - food spoiled! "
                    f"Distance to goal: {distance:.2f}m, elapsed: {elapsed:.1f}s, "
                    f"pieces: {self._initial_food_piece_count} -> {self._final_food_piece_count}"
                )
                return

            self._last_mission_result = MissionResult.SUCCESS
            self._state = MissionState.WAITING_FOR_START
            logger.info(
                f"[SUCCESS] Mission {self._current_mission} completed! "
                f"Distance to goal: {distance:.2f}m (tolerance: {self.config.goal_tolerance}m), "
                f"elapsed: {elapsed:.1f}s, traveled: {self._traveled_distance:.2f}m"
            )
            return

        # Check for robot fall (requires physical assistance)
        if self._is_robot_fallen():
            self._mission_end_time = self._get_current_time_seconds()
            if self.config.food.enabled:
                self._final_food_piece_count = self._count_food_pieces_in_bucket()
            self._last_mission_result = MissionResult.FAILURE_PHYSICALASSISTANCE
            self._last_mission_result_reason = "orientation"
            self._last_mission_distance = distance if distance is not None else -1.0
            self._last_elapsed_time = elapsed
            self._last_traveled_distance = self._traveled_distance
            self._last_contact_count = self._contact_count
            self._last_people_contact_count = self._people_contact_count
            self._last_total_impulse = self._total_impulse
            self._last_property_contact_counts = dict(self._property_contact_counts)
            self._last_delta_v_magnitudes_mps = list(self._delta_v_magnitudes_mps)
            self._last_injury_costs = list(self._injury_costs)
            self._last_total_injury_cost = self._total_injury_cost
            self._state = MissionState.WAITING_FOR_START
            logger.info(
                f"[FAILURE_PHYSICALASSISTANCE] Mission {self._current_mission} failed - robot fell down! "
                f"(reason: orientation) Distance to goal: {distance:.2f}m, elapsed: {elapsed:.1f}s"
            )
            return

        # Check for impulse health depletion (requires physical assistance)
        if self._impulse_health <= 0.0:
            self._mission_end_time = self._get_current_time_seconds()
            if self.config.food.enabled:
                self._final_food_piece_count = self._count_food_pieces_in_bucket()
            self._last_mission_result = MissionResult.FAILURE_PHYSICALASSISTANCE
            self._last_mission_result_reason = "impulse_health_depletion"
            self._last_mission_distance = distance if distance is not None else -1.0
            self._last_elapsed_time = elapsed
            self._last_traveled_distance = self._traveled_distance
            self._last_contact_count = self._contact_count
            self._last_people_contact_count = self._people_contact_count
            self._last_total_impulse = self._total_impulse
            self._last_property_contact_counts = dict(self._property_contact_counts)
            self._last_delta_v_magnitudes_mps = list(self._delta_v_magnitudes_mps)
            self._last_injury_costs = list(self._injury_costs)
            self._last_total_injury_cost = self._total_injury_cost
            self._state = MissionState.WAITING_FOR_START
            logger.info(
                f"[FAILURE_PHYSICALASSISTANCE] Mission {self._current_mission} failed - impulse health depleted! "
                f"(reason: impulse_health_depletion) Distance to goal: {distance:.2f}m, elapsed: {elapsed:.1f}s"
            )
            return

        # Check for timeout (failure)
        if self.config.timeout is not None and elapsed >= self.config.timeout:
            self._mission_end_time = self._get_current_time_seconds()
            if self.config.food.enabled:
                self._final_food_piece_count = self._count_food_pieces_in_bucket()
            self._last_mission_result = MissionResult.FAILURE_TIMEOUT
            self._last_mission_distance = distance if distance is not None else -1.0
            self._last_elapsed_time = elapsed
            self._last_traveled_distance = self._traveled_distance
            self._last_contact_count = self._contact_count
            self._last_people_contact_count = self._people_contact_count
            self._last_total_impulse = self._total_impulse
            self._last_property_contact_counts = dict(self._property_contact_counts)
            self._last_delta_v_magnitudes_mps = list(self._delta_v_magnitudes_mps)
            self._last_injury_costs = list(self._injury_costs)
            self._last_total_injury_cost = self._total_injury_cost
            self._state = MissionState.WAITING_FOR_START
            logger.info(
                f"[FAILURE_TIMEOUT] Mission {self._current_mission} timed out! "
                f"Distance to goal: {distance:.2f}m (needed: {self.config.goal_tolerance}m), "
                f"timeout: {self.config.timeout}s, traveled: {self._traveled_distance:.2f}m"
            )

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
        if self._contact_report_subscription is not None:
            self._contact_report_subscription = None
        try:
            import rclpy

            rclpy.shutdown()
        except Exception:
            pass
