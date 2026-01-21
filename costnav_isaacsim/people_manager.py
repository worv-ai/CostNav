#!/usr/bin/env python3
"""
People Manager for CostNav Isaac Sim.

This module handles spawning and managing animated people in the simulation
using the PeopleAPI extension. People walk naturally in NavMesh-enabled areas.
"""

import logging

import carb
import omni.kit.commands
from pxr import Sdf, Usd

logger = logging.getLogger("people_manager")


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
    ):
        """Initialize the people manager.

        Args:
            num_people: Number of people to spawn
            robot_prim_path: Path to the robot prim (required for CharacterSetup API)
            character_root: Root path for character prims
            performance_mode: If True, disables dynamic avoidance for better FPS (default: True)
        """
        self.num_people = num_people
        self.robot_prim_path = robot_prim_path
        self.character_root = character_root
        self.performance_mode = performance_mode

        self.character_setup = None
        self.character_names = []
        self.initialized = False

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
                from omni.anim.people_api.scripts.custom_command.command_manager import CustomCommandManager
                from omni.anim.people_api.scripts.utils import Utils

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
            logger.info(f"Creating CharacterSetup with warmup enabled...")
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
            # OPTIMIZED: Increased batch size from 3 to 10, reduced wait cycles
            batch_size = 10  # Load 10 characters at a time (was 3)
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
                # OPTIMIZED: Reduced from 60 to 30 max wait cycles
                logger.info(f"Waiting for batch {batch_start // batch_size + 1} assets to load...")
                for _ in range(30):
                    if not is_stage_loading():
                        break
                    simulation_app.update()

                # OPTIMIZED: Reduced stabilization time from 30 to 15 cycles
                for _ in range(15):
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
                self._apply_behavior_to_skelroot(skelroot, anim_graph, script_path, Utils)
                skelroots.append(skelroot)

            logger.info(f"Spawned {len(skelroots)} people: {self.character_names}")

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
        from omni.anim.people_api.scripts.custom_command.command_manager import CustomCommandManager

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

    def _apply_behavior_to_skelroot(self, skelroot_prim, anim_graph_prim, script_path, Utils):
        """Apply animation graph and behavior script to a SkelRoot prim."""
        omni.kit.commands.execute(
            "ApplyAnimationGraphAPICommand",
            paths=[Sdf.Path(skelroot_prim.GetPrimPath())],
            animation_graph_path=Sdf.Path(anim_graph_prim.GetPrimPath()),
        )
        omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(skelroot_prim.GetPrimPath())])
        attr = skelroot_prim.GetAttribute("omni:scripting:scripts")
        attr.Set([r"{}".format(script_path)])
        Utils.add_colliders(skelroot_prim)
        Utils.add_rigid_body_dynamics(skelroot_prim)

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
