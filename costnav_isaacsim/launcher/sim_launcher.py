# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""CostNav Isaac Sim launcher class."""

import logging
import os
import time
from typing import TYPE_CHECKING, Optional

from isaacsim import SimulationApp

from ..utils import (
    WARMUP_STEPS,
    resolve_people_robot_prim,
    resolve_robot_prim_path,
)
from .extensions import enable_isaac_sim_extensions

if TYPE_CHECKING:
    from config import MissionConfig

logger = logging.getLogger("costnav_launch")


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

        self._stage_open_subscription = None
        self._pending_physics_reinit = False
        self._pending_stage_reload = False
        self._last_physics_reset_time = None

        # Setup simulation app
        self.simulation_app = self._setup_simulation_app(headless)

        # Configure app and extensions
        self._config_app()
        if not enable_isaac_sim_extensions(self.simulation_app, self.num_people):
            self.num_people = 0
        self._setup_carb_settings()

        # Load USD stage
        self._load_stage()

        # Setup simulation context
        self.simulation_context = self._setup_simulation_context()
        self._pending_physics_reinit = True

        robot_prim_path = self._resolve_and_set_robot_prim()

        # Initialize people manager (will be setup after warmup)
        self.people_manager = None
        self._people_initialized = False
        if self.num_people > 0:
            from people_manager import PeopleManager

            people_robot_prim_path = resolve_people_robot_prim(robot_prim_path)
            self.people_manager = PeopleManager(
                num_people=self.num_people,
                robot_prim_path=people_robot_prim_path,
                character_root="/World/Characters",
            )

        self._register_simulation_event_handlers()

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

    def _setup_carb_settings(self):
        """Configure carb settings."""
        import carb.settings

        settings = carb.settings.get_settings()
        settings.set("/log/channels/omni.physx.plugin", "error")
        settings.set_bool("/persistent/exts/omni.anim.navigation.core/navMesh/useGpu", False)

    def _load_stage(self):
        """Load the USD stage."""
        import omni.usd

        logger.info(f"Loading: {self.usd_path}")

        usd_context = omni.usd.get_context()
        result = usd_context.open_stage(self.usd_path)

        if not result:
            raise RuntimeError(f"Failed to open USD file: {self.usd_path}")

        from isaacsim.core.utils.stage import is_stage_loading

        while is_stage_loading():
            self.simulation_app.update()

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

    def _resolve_and_set_robot_prim(self) -> Optional[str]:
        """Resolve robot prim path and update environment/config."""
        robot_prim_path = None
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            env_override = os.environ.get("ROBOT_PRIM_PATH")
            robot_prim_path = resolve_robot_prim_path(stage, self.robot_name, env_override)
        except Exception as exc:
            logger.warning("Failed to resolve robot prim path: %s", exc)

        if robot_prim_path:
            if not os.environ.get("ROBOT_PRIM_PATH"):
                os.environ["ROBOT_PRIM_PATH"] = robot_prim_path
            self._update_mission_robot_prim(robot_prim_path)
        else:
            logger.warning("Robot prim path could not be resolved; teleportation may be disabled")

        return robot_prim_path

    def _update_mission_robot_prim(self, robot_prim_path: Optional[str]) -> None:
        """Update mission config robot prim for teleportation."""
        if not self.mission_config:
            return

        if robot_prim_path:
            self.mission_config.teleport.robot_prim = robot_prim_path
        else:
            self.mission_config.teleport.robot_prim = ""

    def _register_simulation_event_handlers(self) -> None:
        """Register stage event handlers for reinitialization."""
        try:
            import carb.eventdispatcher
            import omni.usd

            dispatcher = carb.eventdispatcher.get_eventdispatcher()
            self._stage_open_subscription = dispatcher.observe_event(
                event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_open,
                observer_name="costnav.launcher.stage_open",
            )
        except Exception as exc:
            logger.warning("Failed to register simulation event handlers: %s", exc)

    def _on_stage_open(self, _event) -> None:
        self._pending_stage_reload = True
        self._pending_physics_reinit = True
        self._people_initialized = False

    def _ensure_simulation_context(self) -> None:
        from isaacsim.core.api import SimulationContext

        sim_instance = SimulationContext.instance()
        if sim_instance is None:
            self.simulation_context = self._setup_simulation_context()
        else:
            self.simulation_context = sim_instance

    def _refresh_robot_prim_paths(self) -> None:
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            env_override = os.environ.get("ROBOT_PRIM_PATH")
            robot_prim_path = resolve_robot_prim_path(stage, self.robot_name, env_override)
        except Exception as exc:
            logger.warning("Failed to resolve robot prim path after stage reload: %s", exc)
            return

        if robot_prim_path:
            if not os.environ.get("ROBOT_PRIM_PATH"):
                os.environ["ROBOT_PRIM_PATH"] = robot_prim_path
            self._update_mission_robot_prim(robot_prim_path)
        else:
            logger.warning("Robot prim path could not be resolved after stage reload")

        if self.people_manager is not None:
            self.people_manager.robot_prim_path = resolve_people_robot_prim(robot_prim_path)

    def _handle_pending_simulation_reinit(self, mission_manager=None) -> None:
        self._ensure_simulation_context()

        try:
            from isaacsim.core.utils.stage import is_stage_loading

            if is_stage_loading():
                return
        except Exception as exc:
            logger.warning("Failed to query stage loading state: %s", exc)

        force_reset = False
        sim_view = self.simulation_context.physics_sim_view
        if sim_view is not None:
            try:
                if hasattr(sim_view, "is_valid") and not sim_view.is_valid:
                    self._pending_physics_reinit = True
                    force_reset = True
                elif hasattr(sim_view, "check") and not sim_view.check():
                    self._pending_physics_reinit = True
                    force_reset = True
            except Exception:
                self._pending_physics_reinit = True
                force_reset = True

        if not self._pending_physics_reinit and not self._pending_stage_reload:
            return

        stage_reloaded = self._pending_stage_reload
        needs_people_restart = stage_reloaded or force_reset
        if stage_reloaded:
            self._refresh_robot_prim_paths()

        if force_reset:
            now = time.perf_counter()
            if self._last_physics_reset_time is None or (now - self._last_physics_reset_time) > 1.0:
                self._last_physics_reset_time = now
                try:
                    self.simulation_context.reset()
                except Exception as exc:
                    logger.warning("Failed to reset simulation after view invalidation: %s", exc)

        try:
            self.simulation_context.initialize_physics()
        except Exception as exc:
            logger.warning("Failed to initialize physics after restart: %s", exc)

        if mission_manager is not None:
            try:
                mission_manager.handle_simulation_restart(stage_reloaded=stage_reloaded)
            except Exception as exc:
                logger.warning("Failed to refresh mission manager after restart: %s", exc)

        should_restart_people = (
            needs_people_restart
            and self.people_manager is not None
            and (stage_reloaded or not self._people_initialized)
        )
        if should_restart_people:
            try:
                self.people_manager.shutdown()
                self.people_manager.initialize(self.simulation_app, self.simulation_context)
                self._people_initialized = True
            except Exception as exc:
                logger.warning("Failed to reinitialize PeopleManager after physics reset: %s", exc)

        self._pending_physics_reinit = False
        self._pending_stage_reload = False

    def _step_simulation(self, mission_manager=None, throttle: bool = False):
        """Advance simulation one tick with optional throttling.

        Args:
            mission_manager: Optional mission manager to step after simulation step.
            throttle: Whether to throttle to maintain rendering_dt timing.
        """
        self._handle_pending_simulation_reinit(mission_manager)
        tick_start = time.perf_counter()
        self.simulation_context.step(render=True)

        # Step mission manager after simulation step
        if mission_manager is not None:
            mission_manager.step()

        # Update people manager for stuck detection/recovery
        if self.people_manager is not None:
            current_time = self.simulation_context.current_time
            self.people_manager.update(current_time)

        if throttle:
            elapsed = time.perf_counter() - tick_start
            sleep_duration = self.rendering_dt - elapsed
            if sleep_duration > 0.0:
                time.sleep(sleep_duration)

    def _check_healthy(self):
        """Check simulation health and write ready file for Docker healthcheck."""
        if not self.simulation_app.is_running():
            raise RuntimeError("Isaac Sim failed to start.")

        with open("/tmp/.isaac_sim_running", "w") as f:
            f.write("READY\n")
        logger.info("Health check file written - ROS2 container will start automatically")

    def run(self):
        """Run the simulation loop."""
        self.simulation_app.update()

        self.simulation_context.play()
        logger.info("Simulation playing")

        self._handle_pending_simulation_reinit()

        logger.info(f"Running {WARMUP_STEPS} warm-up steps...")
        for _ in range(WARMUP_STEPS):
            self.simulation_context.step(render=True)
        logger.info("Warm-up complete")

        if self.people_manager is not None:
            logger.info("Initializing people manager...")
            self.people_manager.initialize(self.simulation_app, self.simulation_context)
            self._people_initialized = True

        self._check_healthy()
        logger.info("Simulation started")

        mission_manager = None
        if self.mission_config:
            mission_manager = self._setup_mission_manager()

        while self.simulation_app.is_running():
            self._step_simulation(mission_manager=mission_manager, throttle=True)

    def _setup_mission_manager(self):
        """Setup mission manager to run in main simulation loop.

        Returns:
            MissionManager instance that will be stepped in the main loop.
        """
        if not self.mission_config:
            return None

        from nav2_mission import MissionManager

        mission_manager = MissionManager(
            mission_config=self.mission_config,
            simulation_context=self.simulation_context,
        )
        logger.info("Mission manager initialized (will run in main simulation loop)")
        return mission_manager

    def close(self):
        """Cleanup and close simulation."""
        if self.people_manager is not None:
            try:
                self.people_manager.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down people manager: {e}")

        if os.path.exists("/tmp/.isaac_sim_running"):
            os.remove("/tmp/.isaac_sim_running")

        if self._stage_open_subscription is not None:
            self._stage_open_subscription = None

        self.simulation_context.stop()
        self.simulation_app.close()

