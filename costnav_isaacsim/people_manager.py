#!/usr/bin/env python3
"""
People Manager for CostNav Isaac Sim.

This module handles spawning and managing animated people in the simulation
using the PeopleAPI extension. People walk naturally in NavMesh-enabled areas.
"""

import logging
from typing import Dict, Optional, Tuple

import carb
import omni.kit.commands
from pxr import Sdf, Usd

logger = logging.getLogger("people_manager")


class CharacterStuckTracker:
    """Lightweight tracker for detecting and recovering stuck characters.

    This class tracks character positions over time to detect when characters
    stop moving unexpectedly. It avoids NavMesh API calls by only reading
    prim transforms which are already computed by the simulation.

    Attributes:
        stuck_time_threshold: Seconds a character must be stationary to be considered stuck
        min_movement_distance: Minimum distance (meters) to be considered "moving"
        recovery_cooldown: Seconds to wait between recovery attempts for the same character
    """

    def __init__(
        self,
        stuck_time_threshold: float = 10.0,
        min_movement_distance: float = 0.1,
        recovery_cooldown: float = 30.0,
    ):
        """Initialize the stuck tracker.

        Args:
            stuck_time_threshold: Time in seconds before a character is considered stuck
            min_movement_distance: Minimum movement distance to reset stuck timer
            recovery_cooldown: Minimum time between recovery attempts per character
        """
        self.stuck_time_threshold = stuck_time_threshold
        self.min_movement_distance = min_movement_distance
        self.recovery_cooldown = recovery_cooldown

        # Tracking data: {character_name: (last_position, last_move_time, last_recovery_time)}
        self._tracking_data: Dict[str, Tuple[Tuple[float, float, float], float, float]] = {}

        # Statistics
        self.total_recoveries = 0
        self.recovery_count_per_character: Dict[str, int] = {}

    def update_character(
        self,
        character_name: str,
        position: Tuple[float, float, float],
        current_time: float,
    ) -> bool:
        """Update tracking for a character and check if stuck.

        This method should be called periodically (not every frame) to track
        character movement. It returns True if the character appears stuck
        and should be recovered.

        Args:
            character_name: Unique identifier for the character
            position: Current (x, y, z) position of the character
            current_time: Current simulation time in seconds

        Returns:
            True if the character is stuck and should be recovered, False otherwise
        """
        if character_name not in self._tracking_data:
            # First time seeing this character - initialize tracking
            self._tracking_data[character_name] = (position, current_time, 0.0)
            self.recovery_count_per_character[character_name] = 0
            return False

        last_position, last_move_time, last_recovery_time = self._tracking_data[character_name]

        # Calculate distance moved since last recorded position
        dx = position[0] - last_position[0]
        dy = position[1] - last_position[1]
        distance_moved = (dx * dx + dy * dy) ** 0.5

        if distance_moved >= self.min_movement_distance:
            # Character has moved - update position and reset stuck timer
            self._tracking_data[character_name] = (position, current_time, last_recovery_time)
            return False

        # Character hasn't moved significantly
        time_stationary = current_time - last_move_time

        # Check if stuck (stationary for too long)
        if time_stationary >= self.stuck_time_threshold:
            # Check recovery cooldown
            time_since_last_recovery = current_time - last_recovery_time
            if time_since_last_recovery >= self.recovery_cooldown:
                logger.warning(
                    f"Character '{character_name}' stuck at {position} "
                    f"for {time_stationary:.1f}s - triggering recovery"
                )
                return True

        return False

    def mark_recovered(self, character_name: str, current_time: float) -> None:
        """Mark a character as having been recovered.

        This updates the recovery timestamp to prevent immediate re-recovery
        and resets the movement tracking.

        Args:
            character_name: The character that was recovered
            current_time: Current simulation time
        """
        if character_name in self._tracking_data:
            last_position, _, _ = self._tracking_data[character_name]
            # Reset with current time as last move time and update recovery time
            self._tracking_data[character_name] = (last_position, current_time, current_time)
            self.total_recoveries += 1
            self.recovery_count_per_character[character_name] = (
                self.recovery_count_per_character.get(character_name, 0) + 1
            )
            logger.info(f"Marked '{character_name}' as recovered (total recoveries: {self.total_recoveries})")

    def remove_character(self, character_name: str) -> None:
        """Stop tracking a character.

        Args:
            character_name: The character to stop tracking
        """
        self._tracking_data.pop(character_name, None)
        self.recovery_count_per_character.pop(character_name, None)

    def get_stats(self) -> Dict:
        """Get tracking statistics.

        Returns:
            Dictionary with tracking statistics
        """
        return {
            "tracked_characters": len(self._tracking_data),
            "total_recoveries": self.total_recoveries,
            "per_character_recoveries": dict(self.recovery_count_per_character),
        }


class PeopleManager:
    """Manages animated people in the simulation using PeopleAPI.

    This class handles:
    - Waiting for PeopleAPI to be ready
    - Ensuring NavMesh is baked
    - Spawning people with RANDOM_GOTO behavior
    - Setting up animation graphs and behavior scripts
    """

    def __init__(
        self,
        num_people: int,
        robot_prim_path: str = "/World/Nova_Carter_ROS",
        character_root: str = "/World/Characters",
        performance_mode: bool = True,
        stuck_detection_enabled: bool = True,
        stuck_time_threshold: float = 10.0,
        stuck_recovery_cooldown: float = 30.0,
    ):
        """Initialize the people manager.

        Args:
            num_people: Number of people to spawn
            robot_prim_path: Path to the robot prim (required for CharacterSetup API)
            character_root: Root path for character prims
            performance_mode: If True, disables dynamic avoidance for better FPS (default: True)
            stuck_detection_enabled: If True, enables stuck character detection and recovery
            stuck_time_threshold: Seconds a character must be stationary to be considered stuck
            stuck_recovery_cooldown: Minimum seconds between recovery attempts per character
        """
        self.num_people = num_people
        self.robot_prim_path = robot_prim_path
        self.character_root = character_root
        self.performance_mode = performance_mode

        self.character_setup = None
        self.character_names = []
        self.initialized = False

        # Stuck detection and recovery
        self.stuck_detection_enabled = stuck_detection_enabled
        self._stuck_tracker: Optional[CharacterStuckTracker] = None
        self._last_stuck_check_time: float = 0.0
        self._stuck_check_interval: float = 2.0  # Check every 2 seconds to minimize overhead
        self._skelroot_cache: Dict[str, any] = {}  # Cache SkelRoot prims for recovery

        if stuck_detection_enabled:
            self._stuck_tracker = CharacterStuckTracker(
                stuck_time_threshold=stuck_time_threshold,
                min_movement_distance=0.1,
                recovery_cooldown=stuck_recovery_cooldown,
            )
            logger.info(
                f"Stuck detection enabled: threshold={stuck_time_threshold}s, cooldown={stuck_recovery_cooldown}s"
            )

        logger.info(f"PeopleManager created: num_people={num_people}, performance_mode={performance_mode}")

    def initialize(self, simulation_app, simulation_context):
        """Initialize people spawning after simulation is ready.

        This should be called after the stage is loaded and simulation context is created.

        Args:
            simulation_app: SimulationApp instance
            simulation_context: SimulationContext instance
        """
        if self.num_people <= 0:
            logger.info("People spawning disabled (num_people=0)")
            return

        logger.info("Initializing PeopleManager...")

        try:
            # Wait a bit for extensions to fully load
            logger.info("Waiting for PeopleAPI extension to load...")
            for _ in range(30):
                simulation_app.update()

            # Import PeopleAPI modules
            try:
                from omni.anim.people_api.settings import PeopleSettings
                from omni.anim.people_api.scripts.character_setup import CharacterSetup, CharacterBehavior

                logger.info("PeopleAPI modules imported successfully")
            except ImportError as e:
                logger.error(f"Failed to import PeopleAPI modules: {e}")
                logger.error("Make sure the PeopleAPI extension is enabled and loaded.")
                logger.error("Extension should be at: /isaac-sim/extsUser/omni.anim.people_api")
                raise

            from isaacsim.core.utils import prims
            from isaacsim.core.utils.stage import is_stage_loading

            # Configure people settings
            settings = carb.settings.get_settings()
            settings.set(PeopleSettings.CHARACTER_PRIM_PATH, self.character_root)
            settings.set(PeopleSettings.BEHAVIOR_SCRIPT_PATH, CharacterBehavior.RANDOM_GOTO.value.script_path)
            settings.set(PeopleSettings.NAVMESH_ENABLED, True)
            # PERFORMANCE: Disable dynamic avoidance in performance mode
            # Dynamic avoidance causes frequent NavMesh queries per frame per character
            settings.set(PeopleSettings.DYNAMIC_AVOIDANCE_ENABLED, not self.performance_mode)
            settings.set(PeopleSettings.NUMBER_OF_LOOP, "inf")  # Walk forever

            if self.performance_mode:
                logger.info("Performance mode ENABLED: dynamic avoidance disabled for better FPS")

            logger.info("Waiting for PeopleAPI to be ready...")
            if not self._wait_for_people_api(simulation_app, max_updates=300):
                logger.error("PeopleAPI not ready; CustomCommandManager not initialized.")
                return

            # Ensure robot prim exists
            if not prims.is_prim_path_valid(self.robot_prim_path):
                logger.warning(f"Robot prim not found at {self.robot_prim_path}, creating placeholder")
                prims.create_prim(self.robot_prim_path, "Xform")

            # Check if NavMesh is already baked, if not, bake it
            logger.info("Checking NavMesh status...")
            if not self._check_navmesh_baked():
                logger.warning("NavMesh is not baked. Attempting to bake NavMesh...")
                if not self._bake_navmesh(simulation_app):
                    logger.error("Failed to bake NavMesh. People spawning will be disabled.")
                    return
                logger.info("NavMesh baked successfully")
            else:
                logger.info("NavMesh is already baked")

            # Create character setup with warmup to prevent animation system crashes
            # Note: CharacterSetup uses starting_point only for distance validation when spawning
            # The actual spawn positions are random NavMesh points throughout the map
            logger.info("Creating CharacterSetup with warmup enabled...")
            self.character_setup = CharacterSetup(
                self.robot_prim_path,
                num_characters=0,  # We'll load them manually
                starting_point=(0.0, 0.0, 0.0),  # Not used for spawn positions, only for validation
                is_warmup=True,  # Enable warmup to initialize animation system properly
            )

            # Additional warmup steps for animation system
            logger.info("Running additional warmup steps for animation system...")
            for _ in range(50):
                simulation_app.update()

            # Wait for animation graph
            stage = omni.usd.get_context().get_stage()
            anim_graph = self._wait_for_animation_graph(simulation_app, stage, max_updates=600)
            if anim_graph is None:
                logger.error("AnimationGraph prim not found; cannot animate characters.")
                return

            self.character_setup.anim_graph_prim = anim_graph
            logger.info(f"Animation graph found: {anim_graph.GetPrimPath()}")

            # Generate random positions using NavMesh sampling
            logger.info(f"Generating {self.num_people} random positions from NavMesh...")
            positions = self._generate_random_positions(self.num_people)
            logger.info(f"Generated {len(positions)} positions: {positions[:5]}{'...' if len(positions) > 5 else ''}")

            # Load characters in batches to prevent animation system crashes
            batch_size = 3
            self.character_names = []

            for batch_start in range(0, len(positions), batch_size):
                batch_end = min(batch_start + batch_size, len(positions))
                batch_positions = positions[batch_start:batch_end]

                logger.info(
                    f"Loading batch {batch_start // batch_size + 1}/{(len(positions) + batch_size - 1) // batch_size}: "
                    f"{len(batch_positions)} characters (total: {batch_start + len(batch_positions)}/{len(positions)})"
                )

                batch_names = self.character_setup.load_characters(batch_positions, CharacterBehavior.RANDOM_GOTO)
                self.character_names.extend(batch_names)

                # Wait for batch assets to load before loading next batch
                logger.info(f"Waiting for batch {batch_start // batch_size + 1} assets to load...")
                for _ in range(60):
                    if not is_stage_loading():
                        break
                    simulation_app.update()

                for _ in range(30):
                    simulation_app.update()

            logger.info(f"All {len(self.character_names)} characters loaded successfully")

            # Apply animation graph and behavior scripts to each character
            logger.info("Applying animation graphs and behavior scripts...")
            script_path = CharacterBehavior.RANDOM_GOTO.value.script_path
            skelroots = []

            for name in self.character_names:
                root_path = f"{self.character_root}/{name}"
                root_prim = stage.GetPrimAtPath(root_path)
                if not root_prim or not root_prim.IsValid():
                    logger.warning(f"Root prim not found: {root_path}")
                    continue

                # Clear any existing behavior scripts
                cleared = self._clear_behavior_scripts(root_prim)
                if cleared:
                    logger.debug(f"Cleared scripts under {root_path}: {cleared}")

                for _ in range(5):
                    simulation_app.update()

                # Find SkelRoot
                skelroot = None
                for _ in range(120):
                    skelroot = self._find_skelroot(stage, root_path)
                    if skelroot:
                        break
                    simulation_app.update()

                if skelroot is None:
                    logger.warning(f"SkelRoot not found for {root_path}")
                    continue

                logger.debug(f"Using SkelRoot {skelroot.GetPrimPath()} for {root_path}")
                self._apply_behavior_to_skelroot(skelroot, anim_graph, script_path)
                skelroots.append(skelroot)

                # Cache SkelRoot for stuck recovery
                self._skelroot_cache[name] = skelroot

            logger.info(f"Spawned {len(skelroots)} people: {self.character_names}")

            # Cache animation graph and script path for recovery
            self._cached_anim_graph = anim_graph
            self._cached_script_path = script_path

            self.initialized = True
            logger.info("PeopleManager initialization complete!")

        except Exception as e:
            logger.exception(f"Failed to initialize PeopleManager: {e}")

    def _generate_random_positions(self, num_positions):
        """Generate random (x, y) positions using NavMesh random sampling.

        OPTIMIZED: Uses batch sampling with spatial hashing for O(1) distance checks.
        Returns a list of (x, y) tuples that will be converted to (x, y, z) by CharacterSetup.
        """
        import random
        import omni.anim.navigation.core as nav

        logger.info(f"Sampling {num_positions} random positions from NavMesh (optimized)...")

        inav = nav.acquire_interface()
        navmesh = inav.get_navmesh()

        if navmesh is None:
            logger.error("NavMesh is None, cannot sample positions!")
            # Fallback to simple grid positions
            return [(i * 2.0, 0.0) for i in range(num_positions)]

        positions = []
        # Reduced max attempts - accept slightly closer positions if needed
        max_attempts = num_positions * 5
        attempts = 0

        # Get area count for area mask
        area_count = inav.get_area_count()
        area_mask = [1] * max(area_count, 1)

        logger.info(f"NavMesh has {area_count} areas")

        # Spatial hash grid for O(1) distance checking instead of O(n)
        min_distance = 1.5  # Reduced from 2.0m for faster placement
        cell_size = min_distance
        occupied_cells = set()

        def get_cell(x, y):
            return (int(x / cell_size), int(y / cell_size))

        def is_position_valid(x, y):
            """Check if position is far enough from existing positions using spatial hash."""
            cell = get_cell(x, y)
            # Check this cell and 8 neighboring cells
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    neighbor = (cell[0] + dx, cell[1] + dy)
                    if neighbor in occupied_cells:
                        # Get actual positions in this cell and check distance
                        for px, py in positions:
                            if get_cell(px, py) == neighbor:
                                if (x - px) ** 2 + (y - py) ** 2 < min_distance**2:
                                    return False
            return True

        # Use a single random ID prefix for efficiency
        base_id = f"spawn_{random.randint(1000, 99999)}"

        while len(positions) < num_positions and attempts < max_attempts:
            attempts += 1

            # Query random point from NavMesh
            point = navmesh.query_random_point(f"{base_id}_{attempts}", area_mask)

            if point is not None:
                x, y = float(point[0]), float(point[1])

                if is_position_valid(x, y):
                    positions.append((x, y))
                    occupied_cells.add(get_cell(x, y))
                    if len(positions) % 10 == 0:
                        logger.info(f"  Generated {len(positions)}/{num_positions} positions")

        if len(positions) < num_positions:
            logger.warning(f"Only generated {len(positions)}/{num_positions} positions after {attempts} attempts")
        else:
            logger.info(f"Generated all {len(positions)} positions in {attempts} attempts")

        return positions

    def _wait_for_people_api(self, simulation_app, max_updates=300):
        """Wait for PeopleAPI CustomCommandManager to be ready."""
        from omni.anim.people_api.scripts.custom_command.command_manager import CustomCommandManager  # noqa: F401

        for _ in range(max_updates):
            if CustomCommandManager.get_instance() is not None:
                return True
            simulation_app.update()
        return False

    def _check_navmesh_baked(self):
        """Check if NavMesh is already baked in the scene."""
        try:
            import omni.anim.navigation.core as nav

            inav = nav.acquire_interface()
            navmesh = inav.get_navmesh()
            return navmesh is not None
        except Exception as e:
            logger.warning(f"Could not check NavMesh status: {e}")
            return False

    def _ensure_navmesh_volume(self):
        """Ensure a NavMeshVolume exists in the scene."""
        try:
            import omni.kit.commands
            import NavSchema
            from pxr import Sdf, Gf, UsdGeom

            stage = omni.usd.get_context().get_stage()

            # Check if NavMeshVolume already exists
            for prim in stage.Traverse():
                if prim.GetTypeName() == "NavMeshVolume" or prim.IsA(NavSchema.NavMeshVolume):
                    logger.info(f"Found existing NavMeshVolume at {prim.GetPath()}")
                    return prim

            # Create NavMeshVolume if it doesn't exist
            logger.info("Creating NavMeshVolume...")
            omni.kit.commands.execute(
                "CreateNavMeshVolumeCommand",
                parent_prim_path=Sdf.Path.emptyPath,
                volume_type=0,
                position=(0.0, 0.0, 0.0),
            )

            # Wait for volume to be created and configure it
            for _ in range(120):
                for prim in stage.Traverse():
                    if prim.GetTypeName() == "NavMeshVolume" or prim.IsA(NavSchema.NavMeshVolume):
                        nav_volume = NavSchema.NavMeshVolume(prim)
                        nav_type_attr = nav_volume.GetNavVolumeTypeAttr()
                        if nav_type_attr:
                            nav_type_attr.Set("Include")

                        # Set a large scale to cover the entire scene
                        xform = UsdGeom.Xformable(prim)
                        scale_op = next(
                            (op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeScale),
                            None,
                        )
                        if scale_op is None:
                            scale_op = xform.AddScaleOp()
                        scale_op.Set(Gf.Vec3f(100.0, 100.0, 10.0))  # Large volume to cover the map

                        logger.info(f"NavMeshVolume created at {prim.GetPath()}")
                        return prim

            logger.error("Failed to create NavMeshVolume")
            return None

        except Exception as e:
            logger.error(f"Failed to ensure NavMeshVolume: {e}")
            return None

    def _bake_navmesh(self, simulation_app):
        """Bake NavMesh for the scene."""
        try:
            import omni.anim.navigation.core as nav

            # Ensure NavMeshVolume exists
            if self._ensure_navmesh_volume() is None:
                logger.error("Cannot bake NavMesh without NavMeshVolume")
                return False

            logger.info("Starting NavMesh baking...")
            inav = nav.acquire_interface()
            inav.start_navmesh_baking()

            # Wait for NavMesh to be baked (max 600 updates = ~10 seconds)
            for i in range(600):
                simulation_app.update()
                navmesh = inav.get_navmesh()
                if navmesh is not None:
                    logger.info(f"NavMesh baked successfully after {i} updates")
                    return True

            logger.error("NavMesh baking timed out after 600 updates")
            return False

        except Exception as e:
            logger.error(f"Failed to bake NavMesh: {e}")
            return False

    def _wait_for_animation_graph(self, simulation_app, stage, max_updates=600):
        """Wait for AnimationGraph prim to appear in the stage."""
        for _ in range(max_updates):
            for prim in stage.Traverse():
                if prim.GetTypeName() == "AnimationGraph":
                    return prim
            simulation_app.update()
        return None

    def _find_skelroot(self, stage, character_root_path):
        """Find SkelRoot prim under a character root path."""
        root_prim = stage.GetPrimAtPath(character_root_path)
        if not root_prim or not root_prim.IsValid():
            return None
        for prim in Usd.PrimRange(root_prim):
            if prim.GetTypeName() == "SkelRoot":
                return prim
        return None

    def _clear_behavior_scripts(self, root_prim):
        """Clear any existing behavior scripts from a prim hierarchy."""
        cleared = []
        for prim in Usd.PrimRange(root_prim):
            attr = prim.GetAttribute("omni:scripting:scripts")
            if not attr or not attr.IsValid():
                continue
            scripts = attr.Get()
            if scripts:
                attr.Set([])
                cleared.append((str(prim.GetPrimPath()), scripts))
        return cleared

    def _apply_behavior_to_skelroot(self, skelroot_prim, anim_graph_prim, script_path):
        """Apply animation graph and behavior script to a SkelRoot prim."""
        omni.kit.commands.execute(
            "ApplyAnimationGraphAPICommand",
            paths=[Sdf.Path(skelroot_prim.GetPrimPath())],
            animation_graph_path=Sdf.Path(anim_graph_prim.GetPrimPath()),
        )
        omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(skelroot_prim.GetPrimPath())])
        attr = skelroot_prim.GetAttribute("omni:scripting:scripts")
        attr.Set([r"{}".format(script_path)])

    def update(self, current_time: float) -> None:
        """Periodic update for stuck detection and recovery.

        This method should be called periodically (e.g., every frame or every few frames)
        to check for stuck characters and trigger recovery. It is designed to be lightweight
        and only performs work at the configured check interval.

        The method avoids NavMesh API calls by only reading prim transforms which are
        already computed by the simulation.

        Args:
            current_time: Current simulation time in seconds
        """
        if not self.initialized or not self.stuck_detection_enabled or self._stuck_tracker is None:
            return

        # Throttle checks to reduce overhead
        if current_time - self._last_stuck_check_time < self._stuck_check_interval:
            return

        self._last_stuck_check_time = current_time

        # Check each character for stuck status
        for character_name in self.character_names:
            position = self._get_character_position(character_name)
            if position is None:
                continue

            is_stuck = self._stuck_tracker.update_character(character_name, position, current_time)

            if is_stuck:
                success = self._recover_stuck_character(character_name)
                if success:
                    self._stuck_tracker.mark_recovered(character_name, current_time)

    def _get_character_position(self, character_name: str) -> Optional[Tuple[float, float, float]]:
        """Get the current position of a character without NavMesh queries.

        This method reads the prim transform directly, which is already computed
        by the simulation, avoiding expensive NavMesh API calls.

        Args:
            character_name: Name of the character

        Returns:
            (x, y, z) position tuple, or None if position cannot be determined
        """
        try:
            # Get SkelRoot from cache
            skelroot = self._skelroot_cache.get(character_name)
            if skelroot is None or not skelroot.IsValid():
                return None

            # Try to get position from the behavior script instance (most accurate)
            try:
                from omni.metropolis.utils.simulation_util import SimulationUtil

                script_instance = SimulationUtil.get_agent_script_instance_by_path(str(skelroot.GetPrimPath()))
                if script_instance and hasattr(script_instance, "get_current_position"):
                    pos = script_instance.get_current_position()
                    if pos is not None:
                        return (float(pos[0]), float(pos[1]), float(pos[2]))
            except Exception:
                pass  # Fall back to prim transform

            # Fallback: Read prim transform directly (no NavMesh query)
            from pxr import UsdGeom

            xformable = UsdGeom.Xformable(skelroot)
            if xformable:
                transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = transform.ExtractTranslation()
                return (float(translation[0]), float(translation[1]), float(translation[2]))

            return None

        except Exception as e:
            logger.debug(f"Failed to get position for '{character_name}': {e}")
            return None

    def _recover_stuck_character(self, character_name: str) -> bool:
        """Attempt to recover a stuck character by resetting its behavior script.

        This recovery mechanism is lightweight and avoids NavMesh API calls:
        1. Clears the current command queue of the behavior script
        2. Forces the script to regenerate new random commands
        3. Does NOT re-query NavMesh or recalculate paths

        The behavior script's get_simulation_commands() will naturally use the
        pre-cached NavMesh positions when it regenerates commands.

        Args:
            character_name: Name of the stuck character

        Returns:
            True if recovery was successful, False otherwise
        """
        try:
            logger.info(f"Attempting to recover stuck character: {character_name}")

            # Get the behavior script instance
            skelroot = self._skelroot_cache.get(character_name)
            if skelroot is None or not skelroot.IsValid():
                logger.warning(f"Cannot recover '{character_name}': SkelRoot not found in cache")
                return False

            try:
                from omni.metropolis.utils.simulation_util import SimulationUtil

                script_instance = SimulationUtil.get_agent_script_instance_by_path(str(skelroot.GetPrimPath()))

                if script_instance is None:
                    logger.warning(f"Cannot recover '{character_name}': behavior script not found")
                    return False

                # Method 1: Try to end current command and clear command queue
                # This forces the script to regenerate new random commands
                if hasattr(script_instance, "end_current_command"):
                    script_instance.end_current_command(set_status=False)

                if hasattr(script_instance, "commands"):
                    # Clear the command queue - the script will regenerate on next update
                    script_instance.commands = []

                if hasattr(script_instance, "current_command"):
                    script_instance.current_command = None

                # Reset any stuck state in navigation manager if present
                if hasattr(script_instance, "navigation_manager") and script_instance.navigation_manager:
                    nav_mgr = script_instance.navigation_manager
                    if hasattr(nav_mgr, "path_targets"):
                        nav_mgr.path_targets = []
                    if hasattr(nav_mgr, "path_points"):
                        nav_mgr.path_points = []

                logger.info(f"Successfully reset behavior script for '{character_name}'")
                return True

            except ImportError:
                # SimulationUtil not available, try alternative recovery
                logger.debug("SimulationUtil not available, trying alternative recovery")

            # Alternative recovery: Re-apply behavior script (more heavyweight but reliable)
            # This is a last resort and still avoids NavMesh queries
            if hasattr(self, "_cached_anim_graph") and hasattr(self, "_cached_script_path"):
                stage = omni.usd.get_context().get_stage()
                root_path = f"{self.character_root}/{character_name}"
                root_prim = stage.GetPrimAtPath(root_path)

                if root_prim and root_prim.IsValid():
                    # Clear and re-apply behavior script
                    self._clear_behavior_scripts(root_prim)
                    self._apply_behavior_to_skelroot(
                        skelroot,
                        self._cached_anim_graph,
                        self._cached_script_path,
                    )
                    logger.info(f"Re-applied behavior script for '{character_name}'")
                    return True

            return False

        except Exception as e:
            logger.error(f"Failed to recover stuck character '{character_name}': {e}")
            return False

    def get_stuck_detection_stats(self) -> Dict:
        """Get statistics about stuck detection and recoveries.

        Returns:
            Dictionary with statistics, or empty dict if stuck detection is disabled
        """
        if self._stuck_tracker is None:
            return {}
        return self._stuck_tracker.get_stats()

    def shutdown(self):
        """Cleanup people manager resources."""
        if self.character_setup:
            try:
                self.character_setup.shutdown()
            except Exception as e:
                logger.warning(f"Error during character_setup shutdown: {e}")

        self.character_setup = None
        self.character_names = []
        self.initialized = False
        logger.info("PeopleManager shutdown complete")
