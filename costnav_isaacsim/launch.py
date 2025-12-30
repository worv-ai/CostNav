#!/isaac-sim/python.sh
"""
Isaac Sim launcher for CostNav project.
Runs Street_sidewalk.usd with Nova Carter robot and Nav2 navigation.

Usage:
    # Basic simulation
    python launch.py

    # Use custom mission config file
    python launch.py --config /path/to/config.yaml

    # Override mission config values via CLI
    python launch.py --mission-timeout 600 --min-distance 10

    # Headless mode
    python launch.py --headless

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


def parse_args():
    """Parse command line arguments before SimulationApp creation."""
    parser = argparse.ArgumentParser(description="CostNav Isaac Sim Launcher")

    # Simulation arguments
    sim_group = parser.add_argument_group("Simulation")
    sim_group.add_argument(
        "--usd_path",
        type=str,
        default="omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
        help="Path to USD file",
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

    return parser.parse_args()


class CostNavSimLauncher:
    """Isaac Sim launcher for CostNav with Nav2 navigation support."""

    def __init__(
        self,
        usd_path: str,
        headless: bool,
        physics_dt: float,
        rendering_dt: float,
        mission_config: "Optional[MissionConfig]" = None,
    ):
        self.usd_path = usd_path
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt
        self.mission_config = mission_config

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

        # Navigation extension (must be enabled before using navmesh)
        enable_extension("omni.anim.navigation.core")

        # Core extensions
        enable_extension("omni.isaac.sensor")
        enable_extension("omni.replicator.core")

        # ROS2 bridge for Nav2 communication
        enable_extension("isaacsim.ros2.bridge")

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
        # Remove health check file
        if os.path.exists("/tmp/.isaac_sim_running"):
            os.remove("/tmp/.isaac_sim_running")

        self.simulation_context.stop()
        self.simulation_app.close()


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

    launcher = CostNavSimLauncher(
        usd_path=args.usd_path,
        headless=args.headless,
        physics_dt=args.physics_dt,
        rendering_dt=args.rendering_dt,
        mission_config=mission_config,
    )

    try:
        launcher.run()
    except Exception:
        logger.exception("CostNav Isaac Sim launcher crashed")
    finally:
        launcher.close()


if __name__ == "__main__":
    main()
