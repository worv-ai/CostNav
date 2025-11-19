# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Safe area validation using raycast collision detection."""

from __future__ import annotations

from typing import TYPE_CHECKING

import omni.physx  # type: ignore
import torch
from pxr import Gf

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

# Enable NavMesh extensions
try:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.anim.navigation.core")
    enable_extension("omni.anim.navigation.meshtools")
    enable_extension("omni.anim.navigation.ui")
    enable_extension("omni.anim.navigation.bundle")
    import omni.anim.navigation.core as nav
    import omni.kit.commands

    NAVMESH_AVAILABLE = True
except Exception as e:
    print(f"[SafeAreaValidator] Warning: NavMesh extensions not available: {e}")
    NAVMESH_AVAILABLE = False


class SafeAreaValidator:
    """Validates safe spawn areas by raycasting to the sky.

    The idea: If a point can "see" the sky (raycast upward with no collision),
    it's a safe outdoor position. If blocked, it's inside a building or underground.

    Example usage:
        validator = SafeAreaValidator(env)

        # Check if specific positions are safe
        positions = torch.tensor([[1.0, 2.0, 0.5], [3.0, 4.0, 0.5]])
        is_safe = validator.validate_positions(positions)

        # Generate safe positions from a grid
        safe_positions = validator.generate_safe_grid(
            x_range=(0.0, 10.0),
            y_range=(0.0, 10.0),
            z_height=0.5,
            grid_spacing=1.0,
        )
    """

    def __init__(
        self,
        env: ManagerBasedEnv,
        raycast_height: float = 100.0,
        min_clearance: float = 2.0,
        visualize_raycasts: bool = False,
        check_navmesh_reachability: bool = False,
        navmesh_origin: tuple[float, float, float] = (0.0, 0.0, 0.5),
        debug: bool = True,
    ):
        """Initialize the safe area validator.

        Args:
            env: The environment instance.
            raycast_height: How high to raycast (meters). Default 100m should reach sky.
            min_clearance: Minimum clearance required above position (meters).
            visualize_raycasts: Whether to visualize raycasts with lines and markers.
            check_navmesh_reachability: Whether to check if position is reachable via NavMesh from origin.
            navmesh_origin: Origin point (x, y, z) to check NavMesh reachability from.
            debug: Whether to print debug information.
        """
        self.env = env
        self.raycast_height = raycast_height
        self.min_clearance = min_clearance
        self.visualize_raycasts = visualize_raycasts
        self.check_navmesh_reachability = check_navmesh_reachability
        self.navmesh_origin = navmesh_origin
        self.debug = debug

        # Get PhysX scene for raycasting
        self.physx_scene_query_interface = omni.physx.get_physx_scene_query_interface()

        # Storage for raycast visualization data
        self.raycast_origins = []
        self.raycast_hits = []
        self.raycast_misses = []

        # NavMesh interface
        self.navmesh_interface = None
        if self.check_navmesh_reachability:
            if not NAVMESH_AVAILABLE:
                print("[SafeAreaValidator] Warning: NavMesh reachability requested but extensions not available!")
                print("  Disabling NavMesh checks.")
                self.check_navmesh_reachability = False
            else:
                try:
                    self.navmesh_interface = nav.acquire_interface()
                    if self.debug:
                        print("[SafeAreaValidator] NavMesh interface acquired successfully")
                except Exception as e:
                    print(f"[SafeAreaValidator] Failed to acquire NavMesh interface: {e}")
                    print("  Disabling NavMesh checks.")
                    self.check_navmesh_reachability = False

    def validate_positions(
        self,
        positions: torch.Tensor,
        batch_size: int = 100,
    ) -> torch.Tensor:
        """Validate if positions are safe using upward raycasting and optionally NavMesh.

        Args:
            positions: Tensor of shape (N, 3) with (x, y, z) positions to check.
            batch_size: Number of positions to check at once (for performance).

        Returns:
            Boolean tensor of shape (N,) indicating which positions are safe.
        """
        num_positions = positions.shape[0]
        is_safe = torch.zeros(num_positions, dtype=torch.bool, device=positions.device)

        if self.debug:
            print(f"\n[SafeAreaValidator] Validating {num_positions} positions...")
            if self.check_navmesh_reachability:
                print("  - Raycast check: Upward to sky")
                print(f"  - NavMesh check: Reachability from {self.navmesh_origin}")

        # Process in batches for performance
        for batch_start in range(0, num_positions, batch_size):
            batch_end = min(batch_start + batch_size, num_positions)
            batch_positions = positions[batch_start:batch_end]

            # Check each position in the batch
            for i, pos in enumerate(batch_positions):
                global_idx = batch_start + i

                # First check: raycast to sky
                raycast_safe = self._raycast_to_sky(pos)

                # Second check: NavMesh reachability (if enabled and raycast passed)
                if raycast_safe and self.check_navmesh_reachability:
                    navmesh_safe = self._check_navmesh_reachability(pos)
                    is_safe[global_idx] = navmesh_safe
                else:
                    is_safe[global_idx] = raycast_safe

                if self.debug and (global_idx + 1) % 50 == 0:
                    safe_count = is_safe[: global_idx + 1].sum().item()
                    print(f"  Checked {global_idx + 1}/{num_positions} - Safe: {safe_count}")

        safe_count = is_safe.sum().item()
        if self.debug:
            print(
                f"[SafeAreaValidator] Result: {safe_count}/{num_positions} positions are safe "
                f"({100.0 * safe_count / num_positions:.1f}%)\n"
            )

        return is_safe

    def _raycast_to_sky(self, position: torch.Tensor) -> bool:
        """Raycast upward from position to check if it can see the sky.

        Args:
            position: Tensor of shape (3,) with (x, y, z) position.

        Returns:
            True if position can see the sky (no collision), False otherwise.
        """
        # Convert to numpy for PhysX API
        pos = position.cpu().numpy()

        # Raycast origin (slightly above the position to avoid ground collision)
        origin = Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2]) + 0.1)

        # Raycast direction (straight up)
        direction = Gf.Vec3f(0.0, 0.0, 1.0)

        # Raycast distance
        distance = self.raycast_height

        # Perform raycast with bothSides=True to hit both sides of mesh faces
        # This is critical - without it, raycasts can pass through meshes depending on face orientation
        hit = self.physx_scene_query_interface.raycast_closest(origin, direction, distance, bothSides=True)

        # Store visualization data
        if self.visualize_raycasts:
            origin_pos = (float(pos[0]), float(pos[1]), float(pos[2]) + 0.1)
            self.raycast_origins.append(origin_pos)

            if hit["hit"]:
                # Hit something - store hit point
                hit_distance = hit["distance"]
                hit_pos = (float(pos[0]), float(pos[1]), float(pos[2]) + 0.1 + hit_distance)
                self.raycast_hits.append((origin_pos, hit_pos, hit_distance))
            else:
                # No hit - raycast went to max distance
                end_pos = (float(pos[0]), float(pos[1]), float(pos[2]) + 0.1 + distance)
                self.raycast_misses.append((origin_pos, end_pos))

        # If no hit, can see the sky (safe)
        if not hit["hit"]:
            return True

        # If hit, check if clearance is sufficient
        hit_distance = hit["distance"]
        if hit_distance >= self.min_clearance:
            return True  # Enough clearance above

        return False  # Blocked or insufficient clearance

    def bake_navmesh(
        self,
        volume_size: tuple[float, float, float] = (100.0, 100.0, 10.0),
        volume_position: tuple[float, float, float] = (0.0, 0.0, 5.0),
        agent_height: float = 1.8,
        agent_radius: float = 0.5,
        agent_max_slope: float = 45.0,
    ) -> bool:
        """Bake NavMesh for the scene.

        This method triggers NavMesh baking using the navigation interface.
        The NavMesh is used for reachability checking in the validation process.

        Args:
            volume_size: Size of the NavMesh volume (x, y, z) in meters (currently unused).
            volume_position: Position of the NavMesh volume center (x, y, z) in meters (currently unused).
            agent_height: Height of the navigation agent in meters (currently unused).
            agent_radius: Radius of the navigation agent in meters (currently unused).
            agent_max_slope: Maximum slope angle in degrees that the agent can traverse (currently unused).

        Returns:
            True if NavMesh was baked successfully, False otherwise.

        Note:
            This method assumes NavMesh volumes are already set up in the scene.
            You need to create NavMesh Include/Exclude volumes via the UI or USD authoring
            before calling this method.
        """
        if not NAVMESH_AVAILABLE:
            print("[SafeAreaValidator] ✗ NavMesh extensions not available. Cannot bake NavMesh.")
            return False

        try:
            import time

            print("[SafeAreaValidator] Checking NavMesh status...")

            # Get the current navmesh state
            navmesh = self.navmesh_interface.get_navmesh()
            if navmesh is not None:
                print("[SafeAreaValidator] ✓ NavMesh already exists and is baked!")
                return True

            print("[SafeAreaValidator] NavMesh not found. Attempting to bake...")
            print("[SafeAreaValidator] NOTE: This requires NavMesh volumes to be set up in the scene.")
            print("[SafeAreaValidator]       (Create > Navigation > NavMesh Include Volume in Isaac Sim UI)")

            # Stop simulation before baking
            print("[SafeAreaValidator] Stopping simulation for NavMesh baking...")
            self.env.sim.stop()

            # Start NavMesh baking
            print("[SafeAreaValidator] Triggering NavMesh baking via navigation interface...")
            self.navmesh_interface.start_navmesh_baking()

            # Wait for NavMesh to be baked
            print("[SafeAreaValidator] Waiting for NavMesh to be baked (max 10 seconds)...")
            max_wait_time = 300.0  # Reduced to 10 seconds
            start_time = time.time()
            check_count = 0
            while navmesh is None:
                navmesh = self.navmesh_interface.get_navmesh()
                if navmesh is not None:
                    break

                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > max_wait_time:
                    print(f"[SafeAreaValidator] ✗ NavMesh baking timed out after {max_wait_time}s")
                    print("[SafeAreaValidator]   This likely means no NavMesh volumes exist in the scene.")
                    print("[SafeAreaValidator]   NavMesh reachability checking will be DISABLED.")
                    print("[SafeAreaValidator]   Continuing with raycast-only validation...")
                    self.env.sim.play()
                    return False

                check_count += 1
                if check_count % 10 == 0:  # Print progress every second
                    print(f"[SafeAreaValidator]   Still waiting... ({elapsed:.1f}s elapsed)")

                time.sleep(0.1)

            # Restart simulation
            print("[SafeAreaValidator] Restarting simulation...")
            self.env.sim.play()

            print("[SafeAreaValidator] ✓ NavMesh baked successfully!")
            return True

        except Exception as e:
            print(f"[SafeAreaValidator] ✗ Error baking NavMesh: {e}")
            import traceback

            traceback.print_exc()

            # Make sure simulation is restarted even if there's an error
            try:
                self.env.sim.play()
            except Exception:
                pass

            return False

    def _check_navmesh_reachability(self, position: torch.Tensor) -> bool:
        """Check if position is reachable from origin via NavMesh pathfinding.

        Args:
            position: Tensor of shape (3,) with (x, y, z) position.

        Returns:
            True if position is reachable from origin via NavMesh, False otherwise.
        """
        if not self.check_navmesh_reachability or self.navmesh_interface is None:
            return True  # Skip check if not enabled

        try:
            # Get the NavMesh object
            navmesh = self.navmesh_interface.get_navmesh()
            if navmesh is None:
                # NavMesh not baked yet
                if self.debug:
                    print("[SafeAreaValidator] Warning: NavMesh not baked. Skipping reachability check.")
                return True

            # Convert to carb.Float3 for NavMesh API
            import carb

            pos = position.cpu().numpy()
            start = carb.Float3(
                float(self.navmesh_origin[0]),
                float(self.navmesh_origin[1]),
                float(self.navmesh_origin[2]),
            )
            end = carb.Float3(float(pos[0]), float(pos[1]), float(pos[2]))

            # Query NavMesh for shortest path from origin to target
            navmesh_path = navmesh.query_shortest_path(start_pos=start, end_pos=end)

            # Check if path exists and is valid
            if navmesh_path is not None:
                points = navmesh_path.get_points()
                if points and len(points) > 0:
                    return True

            return False

        except Exception as e:
            # If NavMesh query fails, log and assume reachable (fail-safe)
            if self.debug:
                print(f"[SafeAreaValidator] NavMesh query failed for position {position}: {e}")
            return True

    def visualize_raycast_results(self):
        """Visualize raycast results using USD lines.

        Creates visual lines showing:
        - Green lines: Raycasts that didn't hit anything (safe)
        - Red lines: Raycasts that hit something (unsafe)
        - Yellow spheres: Hit points
        """
        if not self.visualize_raycasts:
            print("[SafeAreaValidator] Visualization not enabled. Set visualize_raycasts=True")
            return

        from pxr import UsdGeom

        stage = self.env.sim.stage

        # Create a parent Xform for all raycast visualizations
        raycast_viz_path = "/World/Visuals/RaycastVisualization"
        if stage.GetPrimAtPath(raycast_viz_path):
            stage.RemovePrim(raycast_viz_path)

        raycast_viz = UsdGeom.Xform.Define(stage, raycast_viz_path)  # noqa: F841

        print("\n[Raycast Visualization]")
        print(f"  Green lines: {len(self.raycast_misses)} raycasts with no hit (SAFE)")
        print(f"  Red lines: {len(self.raycast_hits)} raycasts with hit (UNSAFE)")

        # Debug: Show some sample hit distances
        if len(self.raycast_hits) > 0:
            sample_distances = [dist for _, _, dist in self.raycast_hits[:5]]
            print(f"  Sample hit distances: {sample_distances}")

        # Visualize hits (red lines)
        for idx, (origin, hit_pos, distance) in enumerate(self.raycast_hits):
            line_path = f"{raycast_viz_path}/HitRay_{idx}"
            line = UsdGeom.BasisCurves.Define(stage, line_path)

            # Set points
            points = [origin, hit_pos]
            line.GetPointsAttr().Set(points)

            # Set curve type
            line.GetTypeAttr().Set("linear")
            line.GetCurveVertexCountsAttr().Set([2])

            # Set color to red
            line.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])
            line.CreateWidthsAttr([0.05])

            # Add a small sphere at hit point (yellow)
            sphere_path = f"{raycast_viz_path}/HitPoint_{idx}"
            sphere = UsdGeom.Sphere.Define(stage, sphere_path)
            sphere.GetRadiusAttr().Set(0.1)
            sphere.AddTranslateOp().Set(hit_pos)
            sphere.CreateDisplayColorAttr([(1.0, 1.0, 0.0)])  # Yellow

        # Visualize misses (green lines) - show all of them
        print(f"  Creating {len(self.raycast_misses)} green lines...")
        for idx, (origin, end_pos) in enumerate(self.raycast_misses):
            line_path = f"{raycast_viz_path}/MissRay_{idx}"
            line = UsdGeom.BasisCurves.Define(stage, line_path)

            # Set points (show 30m upward to make them clearly visible)
            short_end = (origin[0], origin[1], origin[2] + 100.0)
            points = [origin, short_end]
            line.GetPointsAttr().Set(points)

            # Set curve type
            line.GetTypeAttr().Set("linear")
            line.GetCurveVertexCountsAttr().Set([2])

            # Set color to bright green
            line.CreateDisplayColorAttr([(0.0, 1.0, 0.0)])
            line.CreateWidthsAttr([0.05])  # Make them thick to be very visible

        print("\n  Legend:")
        print("    🟢 Green lines = Raycast to sky (no obstacle)")
        print("    🔴 Red lines = Raycast hit obstacle")
        print("    🟡 Yellow spheres = Hit points")
        print(f"\n  Visualization created at: {raycast_viz_path}")

    def generate_safe_grid(
        self,
        x_range: tuple[float, float],
        y_range: tuple[float, float],
        z_height: float,
        grid_spacing: float,
        max_positions: int | None = None,
    ) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
        """Generate a grid of positions and filter for safe ones.

        Args:
            x_range: (min_x, max_x) range for the grid.
            y_range: (min_y, max_y) range for the grid.
            z_height: Height (z-coordinate) for all positions.
            grid_spacing: Spacing between grid points.
            max_positions: Maximum number of safe positions to return (None = all).

        Returns:
            Tuple of (safe_positions, unsafe_positions) as lists of (x, y, z) tuples.
        """
        if self.debug:
            print("\n[SafeAreaValidator] Generating safe grid...")
            print(f"  X range: {x_range}")
            print(f"  Y range: {y_range}")
            print(f"  Z height: {z_height}")
            print(f"  Grid spacing: {grid_spacing}")

        # Create grid points
        x_points = torch.arange(x_range[0], x_range[1] + grid_spacing, grid_spacing, device=self.env.device)
        y_points = torch.arange(y_range[0], y_range[1] + grid_spacing, grid_spacing, device=self.env.device)

        # Create meshgrid
        xx, yy = torch.meshgrid(x_points, y_points, indexing="ij")

        # Flatten and create positions
        positions = torch.stack(
            [
                xx.flatten(),
                yy.flatten(),
                torch.full_like(xx.flatten(), z_height),
            ],
            dim=1,
        )

        if self.debug:
            print(f"  Total grid points: {positions.shape[0]}")

        # Validate positions
        is_safe = self.validate_positions(positions)

        # Split into safe and unsafe
        safe_positions = positions[is_safe].cpu().numpy().tolist()
        unsafe_positions = positions[~is_safe].cpu().numpy().tolist()

        # Convert to list of tuples
        safe_positions = [(float(x), float(y), float(z)) for x, y, z in safe_positions]
        unsafe_positions = [(float(x), float(y), float(z)) for x, y, z in unsafe_positions]

        # Limit if requested
        if max_positions is not None and len(safe_positions) > max_positions:
            if self.debug:
                print(f"  Limiting to {max_positions} positions (from {len(safe_positions)})")
            safe_positions = safe_positions[:max_positions]

        return safe_positions, unsafe_positions

    def validate_and_save(
        self,
        x_range: tuple[float, float],
        y_range: tuple[float, float],
        z_height: float,
        grid_spacing: float,
        output_file: str = "safe_positions.py",
    ):
        """Generate safe positions and save to a Python file.

        Args:
            x_range: (min_x, max_x) range for the grid.
            y_range: (min_y, max_y) range for the grid.
            z_height: Height (z-coordinate) for all positions.
            grid_spacing: Spacing between grid points.
            output_file: Path to save the positions.
        """
        safe_positions, unsafe_positions = self.generate_safe_grid(x_range, y_range, z_height, grid_spacing)

        # Generate Python code
        code = f"""# Auto-generated safe positions for custom map
# Generated by SafeAreaValidator

# Safe spawn positions ({len(safe_positions)} total)
INIT_POSITIONS = [
"""
        for pos in safe_positions[: len(safe_positions) // 2]:  # Use half for spawns
            code += f"    {pos},\n"

        code += """]

# Safe goal positions ({} total)
TARGET_POSITIONS = [
""".format(len(safe_positions) - len(safe_positions) // 2)

        for pos in safe_positions[len(safe_positions) // 2 :]:  # Use other half for goals
            code += f"    {pos},\n"

        code += """]

# Unsafe positions (for reference, {0} total)
# These positions are inside buildings or have insufficient clearance
UNSAFE_POSITIONS = [
""".format(len(unsafe_positions))

        # Only include first 20 unsafe positions to keep file small
        for pos in unsafe_positions[:20]:
            code += f"    {pos},\n"
        if len(unsafe_positions) > 20:
            code += f"    # ... and {len(unsafe_positions) - 20} more\n"

        code += "]\n"

        # Save to file
        with open(output_file, "w") as f:
            f.write(code)

        if self.debug:
            print(f"\n[SafeAreaValidator] Saved positions to: {output_file}")
            print(f"  Safe positions: {len(safe_positions)}")
            print(f"  - Spawn positions: {len(safe_positions) // 2}")
            print(f"  - Goal positions: {len(safe_positions) - len(safe_positions) // 2}")
            print(f"  Unsafe positions: {len(unsafe_positions)}")
