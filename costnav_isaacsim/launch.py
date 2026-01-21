#!/isaac-sim/python.sh
"""
Isaac Sim launcher for CostNav project.
Runs Street_sidewalk.usd or street_sidewalk_segwaye1.usd based on robot selection.

Usage:
    # Basic simulation
    python launch.py

    # Use custom mission config file
    python launch.py --config /path/to/config.yaml

    # Override mission config values via CLI
    python launch.py --mission-timeout 600 --min-distance 10

    # Headless mode
    python launch.py --headless

    # Select robot preset
    python launch.py --robot segway_e1

Missions are triggered manually via /start_mission (e.g. `make start-mission`).
"""

import argparse
import logging
import os
import time
from typing import TYPE_CHECKING, Optional

from isaacsim import SimulationApp

if TYPE_CHECKING:
    from config import MissionConfig

logger = logging.getLogger("costnav_launch")

# Default constants
DEFAULT_PHYSICS_DT = 1.0 / 60.0
DEFAULT_RENDERING_DT = 1.0 / 30.0
WARMUP_STEPS = 100
DEFAULT_ROBOT_NAME = "nova_carter"
DEFAULT_USD_PATHS = {
    "nova_carter": "omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
    "segway_e1": "omniverse://10.50.2.21/Users/worv/costnav/street_sidewalk_segwaye1.usd",
}
DEFAULT_ROBOT_PRIM_PATHS = {
    "nova_carter": "/World/Nova_Carter_ROS/chassis_link",
    "segway_e1": "/World/Segway_E1_ROS2",
}
ROBOT_NAME_ALIASES = {
    "segway": "segway_e1",
    "segway-e1": "segway_e1",
    "segwaye1": "segway_e1",
}
SEGWAY_BUCKET_PRIM_PATH = "/World/Segway_E1_ROS2/PopcornBucket"
SEGWAY_PIECES_PRIM_PATH = "/World/Segway_E1_ROS2/PopcornPieces"
SEGWAY_BUCKET_HEIGHT_OFFSET = 0.33
LEGACY_BUCKET_PRIM_PATHS = (
    "/World/Segway_E1_ROS2/PopcornBucket",
    "/World/Segway_E1_ROS2/PopcornPieces",
)


def parse_args():
    """Parse command line arguments before SimulationApp creation."""
    parser = argparse.ArgumentParser(description="CostNav Isaac Sim Launcher")

    # Simulation arguments
    sim_group = parser.add_argument_group("Simulation")
    sim_group.add_argument(
        "--usd_path",
        type=str,
        default=None,
        help="Path to USD file (overrides --robot)",
    )
    sim_group.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Robot name to select a default USD (nova_carter, segway_e1)",
    )
    sim_group.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI",
    )
    sim_group.add_argument(
        "--physics_dt",
        type=float,
        default=DEFAULT_PHYSICS_DT,
        help="Physics time step (default: 1/60)",
    )
    sim_group.add_argument(
        "--rendering_dt",
        type=float,
        default=DEFAULT_RENDERING_DT,
        help="Rendering time step (default: 1/30)",
    )
    sim_group.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging",
    )

    # Mission arguments
    mission_group = parser.add_argument_group("Nav2 Mission")
    mission_group.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to mission config YAML file (default: config/mission_config.yaml)",
    )
    # CLI overrides for config values
    mission_group.add_argument(
        "--mission-timeout",
        type=float,
        default=None,
        help="Override: Mission timeout in seconds",
    )
    mission_group.add_argument(
        "--min-distance",
        type=float,
        default=None,
        help="Override: Minimum start-goal distance in meters",
    )
    mission_group.add_argument(
        "--max-distance",
        type=float,
        default=None,
        help="Override: Maximum start-goal distance in meters",
    )
    mission_group.add_argument(
        "--nav2-wait",
        type=float,
        default=None,
        help="Override: Seconds to wait for Nav2 stack",
    )

    # People arguments
    people_group = parser.add_argument_group("People")
    people_group.add_argument(
        "--people",
        type=int,
        default=0,
        help="Number of people to spawn in the scene (default: 0, disabled)",
    )

    return parser.parse_args()


class CostNavSimLauncher:
    """Isaac Sim launcher for CostNav with Nav2 navigation support."""

    def __init__(
        self,
        usd_path: str,
        robot_name: str,
        headless: bool,
        physics_dt: float,
        rendering_dt: float,
        mission_config: "Optional[MissionConfig]" = None,
        num_people: int = 0,
    ):
        self.usd_path = usd_path
        self.robot_name = robot_name
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt
        self.mission_config = mission_config
        self.num_people = num_people

        # Setup simulation app
        self.simulation_app = self._setup_simulation_app(headless)

        # Configure app and extensions
        self._config_app()
        self._enable_extensions()
        self._setup_carb_settings()

        # Load USD stage
        self._load_stage()

        # Setup simulation context
        self.simulation_context = self._setup_simulation_context()

        # Initialize people manager (will be setup after warmup)
        self.people_manager = None
        if self.num_people > 0:
            from people_manager import PeopleManager

            self.people_manager = PeopleManager(
                num_people=self.num_people,
                robot_prim_path="/World/Nova_Carter_ROS",
                character_root="/World/Characters",
            )

        # Final app update
        self.simulation_app.update()

    def _setup_simulation_app(self, headless: bool) -> SimulationApp:
        """Create and configure SimulationApp."""
        config = {
            "renderer": "RayTracedLighting",
            "headless": headless,
            "extra_args": [
                "--/app/omni.graph.scriptnode/opt_in=true",
            ],
        }
        return SimulationApp(config)

    def _config_app(self):
        """Configure app settings (disable viewport for performance)."""
        from omni.kit.viewport.utility import get_active_viewport_window

        active_viewport = get_active_viewport_window()
        if active_viewport:
            active_viewport.visible = True

    def _enable_extensions(self):
        """Enable required Isaac Sim extensions."""
        from isaacsim.core.utils.extensions import enable_extension
        import omni.kit.app

        # Navigation extension (must be enabled before using navmesh)
        enable_extension("omni.anim.navigation.core")

        # Core extensions
        enable_extension("omni.isaac.sensor")
        enable_extension("omni.replicator.core")

        # ROS2 bridge for Nav2 communication
        enable_extension("isaacsim.ros2.bridge")

        # People API extension (if people spawning is enabled)
        if self.num_people > 0:
            logger.info("Enabling PeopleAPI extension...")

            # Add extension search path for PeopleAPI
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_path = "/isaac-sim/extsUser"
            logger.info(f"Adding extension search path: {ext_path}")
            ext_manager.add_path(ext_path)

            # Verify the extension can be found
            if not ext_manager.is_extension_enabled("omni.anim.people_api"):
                logger.info("PeopleAPI extension not yet enabled, enabling dependencies...")

            # Enable required dependencies for PeopleAPI (in order)
            try:
                enable_extension("omni.anim.graph.core")
                self.simulation_app.update()
                enable_extension("omni.anim.graph.schema")
                self.simulation_app.update()
                enable_extension("omni.anim.retarget.core")
                self.simulation_app.update()
                enable_extension("omni.kit.scripting")
                self.simulation_app.update()
                enable_extension("omni.metropolis.utils")
                self.simulation_app.update()

                # Finally enable PeopleAPI
                enable_extension("omni.anim.people_api")
                self.simulation_app.update()
                logger.info("PeopleAPI extension enabled successfully")

                # Give extensions time to fully initialize
                for _ in range(10):
                    self.simulation_app.update()

            except Exception as e:
                logger.error(f"Failed to enable PeopleAPI extension: {e}")
                logger.warning("People spawning will be disabled")
                self.num_people = 0

        self.simulation_app.update()

    def _setup_carb_settings(self):
        """Configure carb settings."""
        import carb.settings

        settings = carb.settings.get_settings()
        # Turn off GPU for navmesh baking
        settings.set_bool("/persistent/exts/omni.anim.navigation.core/navMesh/useGpu", False)

    def _load_stage(self):
        """Load the USD stage."""
        import omni.usd

        logger.info(f"Loading: {self.usd_path}")

        usd_context = omni.usd.get_context()
        result = usd_context.open_stage(self.usd_path)

        if not result:
            raise RuntimeError(f"Failed to open USD file: {self.usd_path}")

        logger.info("Stage loaded successfully")

    def _setup_simulation_context(self):
        """Setup SimulationContext with proper physics timing."""
        from isaacsim.core.api import SimulationContext

        simulation_context = SimulationContext(
            physics_dt=self.physics_dt,
            rendering_dt=self.rendering_dt,
            stage_units_in_meters=1.0,
        )
        return simulation_context

    def _resolve_robot_prim_path(self, stage) -> Optional[str]:
        """Resolve a robot prim path for pose lookups."""
        env_override = os.environ.get("ROBOT_PRIM_PATH")
        if env_override:
            prim = stage.GetPrimAtPath(env_override)
            if prim.IsValid():
                return env_override
            logger.warning("ROBOT_PRIM_PATH not found on stage: %s", env_override)
            return None

        default_path = DEFAULT_ROBOT_PRIM_PATHS.get(self.robot_name)
        if default_path:
            prim = stage.GetPrimAtPath(default_path)
            if prim.IsValid():
                return default_path
            logger.warning("Default robot prim path not found: %s", default_path)

        if self.robot_name == "segway_e1":
            return self._find_robot_prim_by_tokens(stage, ("Segway", "segway"))

        return None

    def _find_robot_prim_by_tokens(self, stage, tokens: tuple[str, ...]) -> Optional[str]:
        """Find a prim path containing any of the supplied tokens."""
        from pxr import UsdGeom

        for prim in stage.Traverse():
            name = prim.GetName()
            if not name:
                continue
            if not any(token in name for token in tokens):
                continue
            if not (prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Xformable)):
                continue
            return prim.GetPath().pathString

        return None

    def _get_prim_world_translation(self, stage, prim_path: str) -> Optional[tuple[float, float, float]]:
        """Return the world translation for a prim path."""
        from pxr import UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            logger.warning("Prim not found for translation lookup: %s", prim_path)
            return None

        xform_cache = UsdGeom.XformCache()
        transform = xform_cache.GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        return (float(translation[0]), float(translation[1]), float(translation[2]))

    def _set_prim_world_translation(self, stage, prim_path: str, position: tuple[float, float, float]) -> None:
        """Set the world translation for a prim path."""
        from pxr import Gf, UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            logger.warning("Prim not found for translation update: %s", prim_path)
            return

        xform = UsdGeom.Xformable(prim)
        translate_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))

    def _update_mission_robot_prim(self, robot_prim_path: Optional[str]) -> None:
        """Update mission config robot prim for teleportation."""
        if not self.mission_config:
            return

        if robot_prim_path:
            self.mission_config.teleport.robot_prim = robot_prim_path
        else:
            self.mission_config.teleport.robot_prim = ""

    def _step_simulation(self, mission_manager=None, throttle: bool = False):
        """Advance simulation one tick with optional throttling.

        Args:
            mission_manager: Optional mission manager to step after simulation step.
            throttle: Whether to throttle to maintain rendering_dt timing.
        """
        tick_start = time.perf_counter()
        self.simulation_context.step(render=True)

        # Step mission manager after simulation step (similar to ros_manager.step() pattern)
        if mission_manager is not None:
            mission_manager.step()

        if throttle:
            elapsed = time.perf_counter() - tick_start
            sleep_duration = self.rendering_dt - elapsed
            if sleep_duration > 0.0:
                time.sleep(sleep_duration)

    def _check_healthy(self):
        """Check simulation health and write ready file for Docker healthcheck."""
        if not self.simulation_app.is_running():
            raise RuntimeError("Isaac Sim failed to start.")

        # Write health file - Docker Compose will detect this and start ROS2 container
        with open("/tmp/.isaac_sim_running", "w") as f:
            f.write("READY\n")
        logger.info("Health check file written - ROS2 container will start automatically")

    def run(self):
        """Run the simulation loop."""
        # Initial app update
        self.simulation_app.update()

        # Start the simulation (play)
        self.simulation_context.play()
        logger.info("Simulation playing")

        # Warm-up steps to initialize deltaTime properly
        # This prevents "Invalid deltaTime 0.000000" warnings
        logger.info(f"Running {WARMUP_STEPS} warm-up steps...")
        for _ in range(WARMUP_STEPS):
            self.simulation_context.step(render=True)
        logger.info("Warm-up complete")

        # Initialize people manager after warmup (if enabled)
        if self.people_manager is not None:
            logger.info("Initializing people manager...")
            self.people_manager.initialize(self.simulation_app, self.simulation_context)

        # Mark as healthy - triggers Docker Compose to start ROS2 container
        self._check_healthy()
        logger.info("Simulation started")

        # Setup mission manager if configured (instead of background thread)
        mission_manager = None
        if self.mission_config:
            mission_manager = self._setup_mission_manager()

        # Main simulation loop
        while self.simulation_app.is_running():
            self._step_simulation(mission_manager=mission_manager, throttle=True)

    def _setup_mission_manager(self):
        """Setup mission manager to run in main simulation loop (not background thread).

        This follows the Isaac Sim pattern where mission management is integrated
        into the main simulation loop via step() calls, ensuring proper synchronization
        between teleportation and physics simulation steps.

        Returns:
            MissionManager instance that will be stepped in the main loop.
        """
        if not self.mission_config:
            return None

        # Import here to avoid circular imports and only when needed
        from nav2_mission import MissionManager

        mission_manager = MissionManager(
            mission_config=self.mission_config,
            simulation_context=self.simulation_context,
        )
        logger.info("Mission manager initialized (will run in main simulation loop)")
        return mission_manager

    def close(self):
        """Cleanup and close simulation."""
        # Cleanup people manager
        if self.people_manager is not None:
            try:
                self.people_manager.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down people manager: {e}")

        # Remove health check file
        if os.path.exists("/tmp/.isaac_sim_running"):
            os.remove("/tmp/.isaac_sim_running")

        self.simulation_context.stop()
        self.simulation_app.close()


def resolve_robot_name(robot_name: Optional[str]) -> str:
    """Resolve the robot name from CLI or environment.

    Args:
        robot_name: Optional robot name override.

    Returns:
        Normalized robot name.
    """
    selected_robot = robot_name or os.environ.get("SIM_ROBOT") or DEFAULT_ROBOT_NAME
    return ROBOT_NAME_ALIASES.get(selected_robot, selected_robot)


def resolve_usd_path(usd_path: Optional[str], robot_name: Optional[str]) -> str:
    """Resolve the USD path based on CLI or robot selection.

    Args:
        usd_path: Optional USD path override.
        robot_name: Optional robot name for default selection.

    Returns:
        Resolved USD path string.
    """
    if usd_path:
        return usd_path

    selected_robot = resolve_robot_name(robot_name)
    if selected_robot not in DEFAULT_USD_PATHS:
        supported = ", ".join(sorted(DEFAULT_USD_PATHS.keys()))
        raise ValueError(f"Unknown robot '{selected_robot}'. Supported: {supported}")

    return DEFAULT_USD_PATHS[selected_robot]


def load_and_override_config(args) -> "MissionConfig":
    """Load mission config from file and apply CLI overrides.

    Args:
        args: Parsed command line arguments.

    Returns:
        MissionConfig instance with loaded settings.
    """
    from config import load_mission_config

    # Load from config file (or default)
    config = load_mission_config(args.config)

    # Apply CLI overrides (only if explicitly provided)
    if args.mission_timeout is not None:
        config.timeout = args.mission_timeout
    if args.min_distance is not None:
        config.min_distance = args.min_distance
    if args.max_distance is not None:
        config.max_distance = args.max_distance
    if args.nav2_wait is not None:
        config.nav2.wait_time = args.nav2_wait

    return config


def main():
    args = parse_args()

    # Configure logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

    # Load mission config (missions are triggered manually)
    mission_config = load_and_override_config(args)

    logger.info("Mission manager armed (manual start via /start_mission)")
    logger.info(f"  Config: {args.config or 'default'}")
    logger.info(f"  Timeout: {mission_config.timeout}s")
    logger.info(f"  Distance: {mission_config.min_distance}m - {mission_config.max_distance}m")
    logger.info(f"  Nav2 wait: {mission_config.nav2.wait_time}s")

    robot_name = resolve_robot_name(args.robot)
    usd_path = resolve_usd_path(args.usd_path, robot_name)
    logger.info(f"  Robot: {robot_name}")
    logger.info(f"  USD path: {usd_path}")

    robot_name = resolve_robot_name(args.robot)
    usd_path = resolve_usd_path(args.usd_path, robot_name)
    logger.info(f"  Robot: {robot_name}")
    logger.info(f"  USD path: {usd_path}")

    if args.people > 0:
        logger.info(f"People spawning enabled: {args.people} people")
    else:
        logger.info("People spawning disabled")

    launcher = CostNavSimLauncher(
        usd_path=usd_path,
        robot_name=robot_name,
        headless=args.headless,
        physics_dt=args.physics_dt,
        rendering_dt=args.rendering_dt,
        mission_config=mission_config,
        num_people=args.people,
    )

    try:
        launcher.run()
    except Exception:
        logger.exception("CostNav Isaac Sim launcher crashed")
    finally:
        launcher.close()


if __name__ == "__main__":
    main()
