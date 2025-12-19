"""Background thread runner for Nav2 missions."""

import logging
import threading
import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from costnav_isaacsim.config import MissionConfig

logger = logging.getLogger("costnav_mission_runner")


class MissionRunner:
    """Background thread runner for Nav2 missions.

    This class manages mission execution in a separate thread,
    allowing the main simulation loop to continue stepping while
    missions are being orchestrated.

    Usage:
        from costnav_isaacsim.config import MissionConfig
        from costnav_isaacsim.nav2_mission import MissionRunner

        config = MissionConfig(count=5, delay=30.0)
        runner = MissionRunner(config)
        runner.start()
        # ... simulation loop ...
        runner.stop()
    """

    def __init__(self, config: "MissionConfig"):
        """Initialize mission runner.

        Args:
            config: Mission configuration object.
        """
        self.config = config
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._orchestrator = None

    @property
    def is_running(self) -> bool:
        """Check if mission runner thread is active."""
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        """Start missions in a background thread."""
        if self.is_running:
            logger.warning("Mission runner already running")
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_missions, daemon=True)
        self._thread.start()
        logger.info("Mission runner thread started")

    def stop(self):
        """Stop the mission runner gracefully."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        if self._orchestrator:
            try:
                self._orchestrator.destroy_node()
            except Exception:
                pass
        logger.info("Mission runner stopped")

    def _run_missions(self):
        """Run missions in background thread."""
        import rclpy

        from .mission_orchestrator import MissionOrchestrator, OrchestratorConfig

        try:
            # Initialize ROS2 in this thread
            rclpy.init()

            # Wait for Nav2 stack to be ready
            wait_time = self.config.nav2.wait_time
            logger.info(f"Waiting {wait_time}s for Nav2 stack to initialize...")
            time.sleep(wait_time)

            # Create orchestrator config (subset of full config)
            orch_config = OrchestratorConfig(
                min_distance=self.config.min_distance,
                max_distance=self.config.max_distance,
                teleport_height=self.config.teleport.height_offset,
                robot_prim_path=self.config.teleport.robot_prim,
            )

            # Create orchestrator (will auto-setup teleport callback if robot_prim_path is set)
            self._orchestrator = MissionOrchestrator(config=orch_config)

            mission_count = self.config.count
            mission_delay = self.config.delay

            logger.info(f"Starting {mission_count} mission(s)...")
            logger.info(f"Distance range: {self.config.min_distance}m - {self.config.max_distance}m")

            for i in range(mission_count):
                if self._stop_event.is_set():
                    logger.info("Mission runner stopped by request")
                    break

                logger.info(f"\n{'=' * 50}")
                logger.info(f"Mission {i + 1}/{mission_count}")
                logger.info(f"{'=' * 50}")

                success = self._orchestrator.run_mission()

                if success:
                    logger.info(f"Mission {i + 1} initiated successfully")
                else:
                    logger.error(f"Mission {i + 1} failed to start")

                # Wait between missions (except for last one)
                if i < mission_count - 1 and not self._stop_event.is_set():
                    logger.info(f"Waiting {mission_delay}s before next mission...")
                    self._interruptible_sleep(mission_delay)

            logger.info("All missions completed!")

        except Exception as e:
            logger.error(f"Mission runner error: {e}")
        finally:
            self._cleanup()

    def _interruptible_sleep(self, duration: float):
        """Sleep that can be interrupted by stop event."""
        for _ in range(int(duration)):
            if self._stop_event.is_set():
                break
            time.sleep(1.0)
        # Handle fractional part
        remainder = duration - int(duration)
        if remainder > 0 and not self._stop_event.is_set():
            time.sleep(remainder)

    def _cleanup(self):
        """Clean up ROS2 resources."""
        if self._orchestrator:
            try:
                self._orchestrator.destroy_node()
            except Exception:
                pass
        try:
            import rclpy

            rclpy.shutdown()
        except Exception:
            pass
