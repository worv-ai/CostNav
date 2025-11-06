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
        debug: bool = True,
    ):
        """Initialize the safe area validator.

        Args:
            env: The environment instance.
            raycast_height: How high to raycast (meters). Default 100m should reach sky.
            min_clearance: Minimum clearance required above position (meters).
            debug: Whether to print debug information.
        """
        self.env = env
        self.raycast_height = raycast_height
        self.min_clearance = min_clearance
        self.debug = debug

        # Get PhysX scene for raycasting
        self.physx_scene_query_interface = omni.physx.get_physx_scene_query_interface()

    def validate_positions(
        self,
        positions: torch.Tensor,
        batch_size: int = 100,
    ) -> torch.Tensor:
        """Validate if positions are safe using upward raycasting.

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

        # Process in batches for performance
        for batch_start in range(0, num_positions, batch_size):
            batch_end = min(batch_start + batch_size, num_positions)
            batch_positions = positions[batch_start:batch_end]

            # Check each position in the batch
            for i, pos in enumerate(batch_positions):
                global_idx = batch_start + i
                is_safe[global_idx] = self._raycast_to_sky(pos)

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

        # Perform raycast
        hit = self.physx_scene_query_interface.raycast_closest(origin, direction, distance)

        # If no hit, can see the sky (safe)
        if not hit["hit"]:
            return True

        # If hit, check if clearance is sufficient
        hit_distance = hit["distance"]
        if hit_distance >= self.min_clearance:
            return True  # Enough clearance above

        return False  # Blocked or insufficient clearance

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
        x_points = torch.arange(
            x_range[0], x_range[1] + grid_spacing, grid_spacing, device=self.env.device
        )
        y_points = torch.arange(
            y_range[0], y_range[1] + grid_spacing, grid_spacing, device=self.env.device
        )

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
        safe_positions, unsafe_positions = self.generate_safe_grid(
            x_range, y_range, z_height, grid_spacing
        )

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
""".format(
            len(safe_positions) - len(safe_positions) // 2
        )

        for pos in safe_positions[len(safe_positions) // 2 :]:  # Use other half for goals
            code += f"    {pos},\n"

        code += """]

# Unsafe positions (for reference, {0} total)
# These positions are inside buildings or have insufficient clearance
UNSAFE_POSITIONS = [
""".format(
            len(unsafe_positions)
        )

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
            print(f"  - Spawn positions: {len(safe_positions)//2}")
            print(f"  - Goal positions: {len(safe_positions) - len(safe_positions)//2}")
            print(f"  Unsafe positions: {len(unsafe_positions)}")
