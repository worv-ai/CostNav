"""Mission manager that runs in the main simulation loop.

This module provides MissionManager which integrates with the main simulation
loop instead of running in a background thread. This ensures proper synchronization
between teleportation and physics simulation steps.

The key difference from MissionRunner:
- MissionRunner: Runs in background thread (timing issues with teleportation)
- MissionManager: Runs in main loop via step() calls (proper physics synchronization)
"""

import logging
import time
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from costnav_isaacsim.config import MissionConfig

logger = logging.getLogger("costnav_mission_manager")


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

    def __init__(self, config: "MissionConfig", simulation_context):
        """Initialize mission manager.

        Args:
            config: Mission configuration object.
            simulation_context: Isaac Sim SimulationContext for stepping physics.
        """
        self.config = config
        self.simulation_context = simulation_context
        self._orchestrator = None
        self._state = MissionState.INIT
        self._current_mission = 0
        self._mission_start_time = None
        self._wait_start_time = None
        self._settle_steps_remaining = 0
        self._initialized = False

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
        """Initialize ROS2 and orchestrator (called once on first step)."""
        import rclpy

        from .mission_orchestrator import MissionOrchestrator, OrchestratorConfig

        try:
            # Initialize ROS2
            rclpy.init()

            # Create orchestrator config
            orch_config = OrchestratorConfig(
                min_distance=self.config.min_distance,
                max_distance=self.config.max_distance,
                teleport_height=self.config.teleport.height_offset,
                robot_prim_path=self.config.teleport.robot_prim,
                teleport_settle_steps=self.config.teleport.get("settle_steps", 5),
            )

            # Create orchestrator with simulation context
            self._orchestrator = MissionOrchestrator(
                config=orch_config,
                simulation_context=self.simulation_context,
            )

            self._initialized = True
            self._state = MissionState.WAITING_FOR_NAV2
            self._wait_start_time = time.time()

            logger.info("Mission manager initialized")
            logger.info(f"Will run {self.config.count} mission(s)")
            logger.info(f"Distance range: {self.config.min_distance}m - {self.config.max_distance}m")

        except Exception as e:
            logger.error(f"Failed to initialize mission manager: {e}")
            self._state = MissionState.COMPLETED

    def _step_waiting_for_nav2(self):
        """Wait for Nav2 stack to be ready."""
        elapsed = time.time() - self._wait_start_time
        if elapsed >= self.config.nav2.wait_time:
            logger.info("Nav2 wait time complete, ready to start missions")
            self._state = MissionState.READY

    def _step_ready(self):
        """Start next mission or complete if all done."""
        if self._current_mission >= self.config.count:
            logger.info("All missions completed!")
            self._state = MissionState.COMPLETED
            self._cleanup()
            return

        self._current_mission += 1
        logger.info(f"\n{'=' * 50}")
        logger.info(f"Mission {self._current_mission}/{self.config.count}")
        logger.info(f"{'=' * 50}")

        # Sample positions
        start, goal = self._orchestrator.sampler.sample_start_goal_pair()
        if start is None or goal is None:
            logger.error("Failed to sample valid start/goal positions")
            self._state = MissionState.READY  # Try again next step
            return

        self._orchestrator._current_start = start
        self._orchestrator._current_goal = goal

        distance = start.distance_to(goal)
        logger.info(f"Mission positions: start={start}, goal={goal}, distance={distance:.2f}m")

        self._state = MissionState.TELEPORTING

    def _step_teleporting(self):
        """Teleport robot to start position."""
        start = self._orchestrator._current_start
        if not self._orchestrator.teleport_robot(start):
            logger.error("Teleportation failed, skipping mission")
            self._state = MissionState.READY
            return

        # After teleportation, we need to settle physics
        self._settle_steps_remaining = self._orchestrator.config.teleport_settle_steps
        self._state = MissionState.SETTLING
        logger.info(f"Teleportation complete, settling physics for {self._settle_steps_remaining} steps")

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
                logger.info("Physics settled, publishing initial pose")
                self._state = MissionState.PUBLISHING_INITIAL_POSE

    def _step_publishing_initial_pose(self):
        """Publish initial pose for AMCL."""
        start = self._orchestrator._current_start
        self._orchestrator.publish_initial_pose(start)
        self._wait_start_time = time.time()
        self._state = MissionState.PUBLISHING_GOAL

    def _step_publishing_goal(self):
        """Publish goal pose to Nav2 after initial pose delay."""
        elapsed = time.time() - self._wait_start_time
        if elapsed >= self._orchestrator.config.initial_pose_delay:
            goal = self._orchestrator._current_goal
            self._orchestrator.publish_goal_pose(goal)

            # Update RViz markers
            self._orchestrator.marker_publisher.publish_start_goal_from_sampled(
                self._orchestrator._current_start, self._orchestrator._current_goal
            )

            logger.info(f"Mission {self._current_mission} initiated successfully")
            self._mission_start_time = time.time()
            self._state = MissionState.WAITING_FOR_COMPLETION

    def _step_waiting_for_completion(self):
        """Wait for mission delay before starting next mission."""
        elapsed = time.time() - self._mission_start_time
        if elapsed >= self.config.delay:
            self._state = MissionState.READY

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
