#!/usr/bin/env python3

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Automatically find safe spawn positions on your custom map using raycast validation.

This script:
1. Loads your custom map
2. Creates a grid of candidate positions
3. Raycasts upward from each position to check if it can "see" the sky
4. Filters out positions inside buildings or underground
5. Saves safe positions to a file

Usage:
    python find_safe_positions.py --x_range 0 10 --y_range 0 10 --spacing 0.5
"""

from __future__ import annotations

import argparse
import os

from isaaclab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(description="Find safe spawn positions on custom map")
parser.add_argument(
    "--x_range",
    type=float,
    nargs=2,
    default=None,
    help="X range to search (min max). If not specified, auto-detect from map bounds.",
)
parser.add_argument(
    "--y_range",
    type=float,
    nargs=2,
    default=None,
    help="Y range to search (min max). If not specified, auto-detect from map bounds.",
)
parser.add_argument("--z_height", type=float, default=0.5, help="Height above ground to spawn (default: 0.5m)")
parser.add_argument("--spacing", type=float, default=4.0, help="Grid spacing in meters (default: 4.0m)")
parser.add_argument(
    "--raycast_height",
    type=float,
    default=10000.0,
    help="Raycast distance upward (default: 10000m)",
)
parser.add_argument(
    "--min_clearance",
    type=float,
    default=10000.0,
    help="Minimum clearance above position (default: 10000.0m)",
)
parser.add_argument(
    "--output",
    type=str,
    default="safe_positions_auto_generated.py",
    help="Output file for safe positions",
)
parser.add_argument(
    "--margin",
    type=float,
    default=3.0,
    help="Margin to shrink from map edges (default: 3.0m)",
)
parser.add_argument(
    "--visualize_raycasts",
    action="store_true",
    help="Visualize raycasts with colored lines (green=safe, red=unsafe)",
)
parser.add_argument(
    "--check_navmesh",
    action="store_true",
    help="Check NavMesh reachability from origin (0, 0, 0.5)",
)
parser.add_argument(
    "--navmesh_origin",
    type=float,
    nargs=3,
    default=[0.0, 0.0, 0.5],
    help="Origin point (x, y, z) for NavMesh reachability check (default: 0 0 0.5)",
)
parser.add_argument(
    "--bake_navmesh",
    action="store_true",
    help="Automatically bake NavMesh before validation (required for --check_navmesh)",
)
parser.add_argument(
    "--navmesh_volume_size",
    type=float,
    nargs=3,
    default=[100.0, 100.0, 10.0],
    help="NavMesh volume size (x, y, z) in meters (default: 100 100 10)",
)
parser.add_argument(
    "--navmesh_agent_height",
    type=float,
    default=1.8,
    help="NavMesh agent height in meters (default: 1.8)",
)
parser.add_argument(
    "--navmesh_agent_radius",
    type=float,
    default=0.5,
    help="NavMesh agent radius in meters (default: 0.5)",
)

# Append AppLauncher cli args (this adds --headless and other args)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import isaaclab.sim as sim_utils
import numpy as np
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from pxr import Usd, UsdGeom

# Import the safe area validator
from costnav_isaaclab.source.costnav_isaaclab.costnav_isaaclab.tasks.manager_based.costnav_isaaclab_v2_NavRL.safe_area_validator import (
    SafeAreaValidator,
)

# Import the configuration
from costnav_isaaclab.tasks.manager_based.costnav_isaaclab_v2_NavRL.costnav_isaaclab_env_cfg import (
    CostnavIsaaclabEnvCfg,
)


def compute_map_bounds(env, map_prim_path: str = "/World/custom_map", margin: float = 1.0) -> tuple:
    """
    Compute the bounding box of the custom map.

    Args:
        env: The environment instance
        map_prim_path: Path to the custom map prim
        margin: Margin to shrink from edges (meters)

    Returns:
        Tuple of (x_min, x_max, y_min, y_max)
    """
    stage = env.sim.stage

    # Get the map prim
    map_prim = stage.GetPrimAtPath(map_prim_path)

    if not map_prim.IsValid():
        print(f"[WARNING] Map prim '{map_prim_path}' not found. Using default range.")
        return (0.0, 10.0, 0.0, 10.0)

    # Create bounding box cache
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

    # Compute world bounding box
    bound = bbox_cache.ComputeWorldBound(map_prim)
    bound_range = bound.ComputeAlignedBox()

    # Extract min and max points
    min_point = bound_range.GetMin()
    max_point = bound_range.GetMax()

    # Apply margin
    x_min = min_point[0] + margin
    x_max = max_point[0] - margin
    y_min = min_point[1] + margin
    y_max = max_point[1] - margin

    print("\n[Map Bounds Detection]")
    print(
        f"  Raw bounds: X=[{min_point[0]:.2f}, {max_point[0]:.2f}], Y=[{min_point[1]:.2f}, {max_point[1]:.2f}], Z=[{min_point[2]:.2f}, {max_point[2]:.2f}]"
    )
    print(f"  With margin ({margin}m): X=[{x_min:.2f}, {x_max:.2f}], Y=[{y_min:.2f}, {y_max:.2f}]")

    return (x_min, x_max, y_min, y_max)


def visualize_positions(safe_positions: list, unsafe_positions: list):
    """
    Visualize safe and unsafe positions with colored spheres using Isaac Lab markers.

    Args:
        safe_positions: List of safe position tuples (x, y, z)
        unsafe_positions: List of unsafe position tuples (x, y, z)

    Returns:
        VisualizationMarkers: The marker instance for visualization
    """
    print("\n[Visualization] Creating visual markers...")

    # Create marker configuration with green and red spheres
    marker_cfg = VisualizationMarkersCfg(
        prim_path="/World/Visuals/SafePositionMarkers",
        markers={
            "safe": sim_utils.SphereCfg(
                radius=0.15,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),  # Green
            ),
            "unsafe": sim_utils.SphereCfg(
                radius=0.2,  # Slightly larger to stand out
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),  # Red
            ),
        },
    )

    # Create the marker instance
    markers = VisualizationMarkers(marker_cfg)

    # Prepare positions and marker indices
    all_positions = []
    marker_indices = []

    # Add safe positions (marker index 0 = "safe")
    print(f"  Adding {len(safe_positions)} green markers for safe positions...")
    for pos in safe_positions:
        all_positions.append(pos)
        marker_indices.append(0)  # Index for "safe" marker

    # Add unsafe positions (marker index 1 = "unsafe")
    print(f"  Adding {len(unsafe_positions)} red markers for unsafe positions...")
    for pos in unsafe_positions:
        all_positions.append(pos)
        marker_indices.append(1)  # Index for "unsafe" marker

    # Convert to numpy array
    all_positions_np = np.array(all_positions, dtype=np.float32)
    marker_indices_np = np.array(marker_indices, dtype=np.int32)

    # Visualize all markers at once
    markers.visualize(translations=all_positions_np, marker_indices=marker_indices_np)

    print("  ✓ Visualization complete!")
    print("\nLEGEND:")
    print("  🟢 Green spheres = SAFE positions (can see sky)")
    print("  🔴 Red spheres = UNSAFE positions (blocked/inside building)")

    return markers


def main():
    """Main function to find safe positions."""

    print("\n" + "=" * 80)
    print("AUTOMATIC SAFE POSITION FINDER")
    print("=" * 80)

    # Create environment configuration (single environment for validation)
    env_cfg = CostnavIsaaclabEnvCfg()
    env_cfg.scene.num_envs = 1  # Only need one environment for validation

    # Remove camera from scene and observations
    # Camera is not needed for safe position finding
    if hasattr(env_cfg.scene, "camera"):
        env_cfg.scene.camera = None
        # Also remove RGB observation term that references the camera
        if hasattr(env_cfg.observations, "policy") and hasattr(env_cfg.observations.policy, "rgb"):
            delattr(env_cfg.observations.policy, "rgb")
        print("  [Memory optimization] Disabled camera and RGB observations (not needed for position finding)")

    # Keep robot but it won't be used - just needed for env initialization
    # The robot is lightweight compared to the map rendering

    print("[1/5] Creating environment and loading map...")
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # Let the simulation settle
    print("[2/5] Waiting for simulation to settle...")
    for _ in range(10):
        env.sim.step()

    # Auto-detect map bounds if not specified
    print("[3/5] Determining search area...")
    if args_cli.x_range is None or args_cli.y_range is None:
        print("  Auto-detecting map bounds...")
        x_min, x_max, y_min, y_max = compute_map_bounds(env, margin=args_cli.margin)

        # Use detected bounds if not specified
        if args_cli.x_range is None:
            args_cli.x_range = [x_min, x_max]
        if args_cli.y_range is None:
            args_cli.y_range = [y_min, y_max]

    print(
        f"Search area: X=[{args_cli.x_range[0]:.2f}, {args_cli.x_range[1]:.2f}], "
        f"Y=[{args_cli.y_range[0]:.2f}, {args_cli.y_range[1]:.2f}]"
    )
    print(f"Z height: {args_cli.z_height}m")
    print(f"Grid spacing: {args_cli.spacing}m")
    print(f"Raycast height: {args_cli.raycast_height}m")
    print(f"Min clearance: {args_cli.min_clearance}m")

    # Handle NavMesh baking and verification
    if args_cli.bake_navmesh or args_cli.check_navmesh:
        print("\n[3.5/5] Setting up NavMesh...")

        # Create a temporary validator to check/bake NavMesh
        temp_validator = SafeAreaValidator(
            env=env,
            raycast_height=args_cli.raycast_height,
            min_clearance=args_cli.min_clearance,
            visualize_raycasts=False,
            check_navmesh_reachability=True,
            navmesh_origin=tuple(args_cli.navmesh_origin),
            debug=True,
        )

        # Check if NavMesh already exists
        navmesh_exists = False
        if temp_validator.navmesh_interface is not None:
            existing_navmesh = temp_validator.navmesh_interface.get_navmesh()
            if existing_navmesh is not None:
                print("[SafeAreaValidator] ✓ NavMesh already exists in the scene!")
                navmesh_exists = True

        # Bake NavMesh if requested and it doesn't exist
        if args_cli.bake_navmesh and not navmesh_exists:
            print("[SafeAreaValidator] Baking NavMesh...")

            # Calculate NavMesh volume position (center of search area)
            volume_center_x = (args_cli.x_range[0] + args_cli.x_range[1]) / 2.0
            volume_center_y = (args_cli.y_range[0] + args_cli.y_range[1]) / 2.0
            volume_position = (
                volume_center_x,
                volume_center_y,
                args_cli.navmesh_volume_size[2] / 2.0,
            )

            # Bake NavMesh
            navmesh_baked = temp_validator.bake_navmesh(
                volume_size=tuple(args_cli.navmesh_volume_size),
                volume_position=volume_position,
                agent_height=args_cli.navmesh_agent_height,
                agent_radius=args_cli.navmesh_agent_radius,
            )

            if navmesh_baked:
                navmesh_exists = True
            elif args_cli.check_navmesh:
                print("[WARNING] NavMesh baking failed but --check_navmesh was requested.")
                print("          Continuing with raycast-only validation...")
                args_cli.check_navmesh = False

        # If check_navmesh is requested but NavMesh doesn't exist, disable it
        if args_cli.check_navmesh and not navmesh_exists:
            print("[WARNING] --check_navmesh requested but NavMesh is not available.")
            print("          Use --bake_navmesh to bake NavMesh, or ensure it's already baked in the scene.")
            print("          Continuing with raycast-only validation...")
            args_cli.check_navmesh = False

    print("\n[4/5] Validating positions (this may take a while)...")

    # Create validator
    validator = SafeAreaValidator(
        env=env,
        raycast_height=args_cli.raycast_height,
        min_clearance=args_cli.min_clearance,
        visualize_raycasts=args_cli.visualize_raycasts,
        check_navmesh_reachability=args_cli.check_navmesh,
        navmesh_origin=tuple(args_cli.navmesh_origin),
        debug=True,
    )

    # Generate safe positions
    safe_positions, unsafe_positions = validator.generate_safe_grid(
        x_range=tuple(args_cli.x_range),
        y_range=tuple(args_cli.y_range),
        z_height=args_cli.z_height,
        grid_spacing=args_cli.spacing,
    )

    # Visualize raycasts if requested
    if args_cli.visualize_raycasts:
        print("\n[Visualizing Raycasts]")
        validator.visualize_raycast_results()

    print("[5/5] Saving results...")

    # Save to file
    output_path = os.path.join(os.path.dirname(__file__), args_cli.output)

    # Get the map URL from the environment configuration
    map_url = env_cfg.scene.custom_map.spawn.usd_path

    # Generate Python code
    code = f"""# Auto-generated safe positions for custom map
# Generated by find_safe_positions.py
#
# Map: {map_url}
#
# Search parameters:
#   X range: [{args_cli.x_range[0]}, {args_cli.x_range[1]}]
#   Y range: [{args_cli.y_range[0]}, {args_cli.y_range[1]}]
#   Z height: {args_cli.z_height}m
#   Grid spacing: {args_cli.spacing}m
#   Raycast height: {args_cli.raycast_height}m
#   Min clearance: {args_cli.min_clearance}m
#
# Results:
#   Total safe positions: {len(safe_positions)}
#   Total unsafe positions: {len(unsafe_positions)}

# Safe positions ({len(safe_positions)} positions)
# These positions can "see" the sky (not inside buildings)
SAFE_POSITIONS = [
"""

    for pos in safe_positions:
        code += f"    {pos},\n"

    code += f"""]

# Statistics
TOTAL_SAFE = {len(safe_positions)}
TOTAL_UNSAFE = {len(unsafe_positions)}
SAFE_PERCENTAGE = {100.0 * len(safe_positions) / (len(safe_positions) + len(unsafe_positions)) if (len(safe_positions) + len(unsafe_positions)) > 0 else 0:.1f}

# Example usage:
# from {args_cli.output.replace(".py", "")} import SAFE_POSITIONS
#
# Use SAFE_POSITIONS for spawning robots and goals
"""

    # Save to file
    with open(output_path, "w") as f:
        f.write(code)

    # Print summary
    print("\n" + "=" * 80)
    print("RESULTS")
    print("=" * 80)
    print(f"✓ Safe positions found: {len(safe_positions)}")
    print(f"✗ Unsafe positions: {len(unsafe_positions)}")
    if len(safe_positions) + len(unsafe_positions) > 0:
        safe_pct = 100.0 * len(safe_positions) / (len(safe_positions) + len(unsafe_positions))
        print(f"  Safe percentage: {safe_pct:.1f}%")
    print(f"\n✓ Saved to: {output_path}")
    print("=" * 80)

    # Print recommendations
    print("\nRECOMMENDATIONS:")
    if len(safe_positions) < 10:
        print("  ⚠️  Few safe positions found. Consider:")
        print("     - Expanding search area (--x_range, --y_range)")
        print("     - Reducing grid spacing (--spacing)")
        print("     - Reducing min clearance (--min_clearance)")
    elif len(safe_positions) < 50:
        print("  ✓ Decent number of positions, but you might want more variety")
        print("    Consider expanding the search area or reducing grid spacing")
    else:
        print("  ✓ Great! You have plenty of safe positions for training")

    print("\nNEXT STEPS:")
    print(f"  1. Review the generated file: {args_cli.output}")
    print("  2. Import in your environment:")
    print(f"     from {args_cli.output.replace('.py', '')} import SAFE_POSITIONS")
    print("  3. Use SAFE_POSITIONS for spawning robots and goals")
    print("=" * 80 + "\n")

    # Visualize the positions
    print("\n" + "=" * 80)
    print("VISUALIZATION")
    print("=" * 80)
    markers = visualize_positions(safe_positions, unsafe_positions)  # noqa: F841

    # Keep simulation running for visualization
    print("\n" + "=" * 80)
    print("VISUALIZATION ACTIVE")
    print("=" * 80)
    print("The simulator is now showing:")
    print("  🟢 Green spheres = Safe positions")
    print("  🔴 Red spheres = Unsafe positions")
    if args_cli.visualize_raycasts:
        print("\nRaycast visualization:")
        print("  🟢 Green lines = Raycasts that reached the sky (no obstacle)")
        print("  🔴 Red lines = Raycasts that hit obstacles")
        print("  🟡 Yellow spheres = Hit points on obstacles")
    if args_cli.check_navmesh:
        print("\nNavMesh validation:")
        print(f"  ✓ Positions checked for reachability from {args_cli.navmesh_origin}")
        print("  ✓ Only positions reachable via NavMesh are marked as safe")
    print("\nThe visualization is now active. You can:")
    print("  - Inspect the scene in the viewport")
    print("  - Navigate around to see all positions")
    if args_cli.visualize_raycasts:
        print("  - Examine raycast lines to understand why positions are safe/unsafe")
    if args_cli.check_navmesh:
        print("  - Verify that safe positions are reachable from the origin")
    print("  - Press Ctrl+C or close the window to exit")
    print("=" * 80 + "\n")

    # Keep the simulation running until user interrupts
    try:
        print("Running simulation... (Press Ctrl+C to exit)")
        while simulation_app.is_running():
            env.sim.step()
    except KeyboardInterrupt:
        print("\n\nExiting...")

    # Close environment
    env.close()


if __name__ == "__main__":
    # Run the main function
    main()
    # Close the simulator
    simulation_app.close()
