#!/isaac-sim/python.sh
"""
Isaac Sim launcher for CostNav project.
Runs Street_sidewalk.usd with Nova Carter robot and Nav2 navigation.

Usage:
    # Basic simulation (no missions)
    python launch.py

    # Run with Nav2 mission orchestration (uses config file)
    python launch.py --mission

    # Use custom config file
    python launch.py --mission --config /path/to/config.yaml

    # Override config values via CLI
    python launch.py --mission --mission-count 5 --min-distance 10

    # Headless mode with missions
    python launch.py --headless --mission
"""

import argparse
import logging
import os
import time
from typing import TYPE_CHECKING, Optional

from isaacsim import SimulationApp

if TYPE_CHECKING:
    from config import MissionConfig
    from nav2_mission import MissionRunner

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
        "--mission",
        action="store_true",
        help="Enable Nav2 mission orchestration",
    )
    mission_group.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to mission config YAML file (default: config/mission_config.yaml)",
    )
    # CLI overrides for config values
    mission_group.add_argument(
        "--mission-count",
        type=int,
        default=None,
        help="Override: Number of missions to run",
    )
    mission_group.add_argument(
        "--mission-delay",
        type=float,
        default=None,
        help="Override: Delay between missions in seconds",
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

        # Mission runner (initialized later if needed)
        self._mission_runner: "Optional[MissionRunner]" = None

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

    def _step_simulation(self, throttle: bool = False):
        """Advance simulation one tick with optional throttling."""
        tick_start = time.perf_counter()
        self.simulation_context.step(render=True)

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

        # Start mission runner if configured
        if self.mission_config:
            self._start_mission_runner()

        # Main simulation loop
        while self.simulation_app.is_running():
            self._step_simulation(throttle=True)

    def _start_mission_runner(self):
        """Start the mission runner in a background thread."""
        if not self.mission_config:
            return

        # Import here to avoid circular imports and only when needed
        from nav2_mission import MissionRunner

        self._mission_runner = MissionRunner(self.mission_config)
        self._mission_runner.start()

    def close(self):
        """Cleanup and close simulation."""
        # Stop mission runner if running
        if self._mission_runner:
            self._mission_runner.stop()

        # Remove health check file
        if os.path.exists("/tmp/.isaac_sim_running"):
            os.remove("/tmp/.isaac_sim_running")

        self.simulation_context.stop()
        self.simulation_app.close()


def load_and_override_config(args) -> "Optional[MissionConfig]":
    """Load mission config from file and apply CLI overrides.

    Args:
        args: Parsed command line arguments.

    Returns:
        MissionConfig if mission mode is enabled, None otherwise.
    """
    if not args.mission:
        return None

    from config import load_mission_config

    # Load from config file (or default)
    config = load_mission_config(args.config)

    # Apply CLI overrides (only if explicitly provided)
    if args.mission_count is not None:
        config.count = args.mission_count
    if args.mission_delay is not None:
        config.delay = args.mission_delay
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

    # Load mission config if mission mode is enabled
    mission_config = load_and_override_config(args)

    if mission_config:
        logger.info("Mission mode enabled")
        logger.info(f"  Config: {args.config or 'default'}")
        logger.info(f"  Count: {mission_config.count}")
        logger.info(f"  Delay: {mission_config.delay}s")
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
