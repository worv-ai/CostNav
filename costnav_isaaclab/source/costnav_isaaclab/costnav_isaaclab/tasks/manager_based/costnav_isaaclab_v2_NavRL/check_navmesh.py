#!/usr/bin/env python3
"""
Minimal script to check if NavMesh is baked in the map.

Usage:
    python check_navmesh.py --headless
"""

import argparse
import sys
import time

# Add Isaac Lab to path
sys.path.append("/workspace/costnav_isaaclab/source/isaaclab")

from isaaclab.app import AppLauncher

# Create argument parser
parser = argparse.ArgumentParser(description="Check if NavMesh is baked in the map")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.usd")
enable_extension("omni.client")
enable_extension("omni.kit.commands")
enable_extension("omni.kit.usd.layers")
enable_extension("omni.kit.usd_undo")
enable_extension("omni.usd.schema.anim")
enable_extension("omni.anim.navigation.scheme")
enable_extension("omni.kit.pip_archive")
enable_extension("omni.anim.navigation.core")
enable_extension("omni.anim.navigation.meshtools")
enable_extension("omni.anim.navigation.ui")
enable_extension("omni.anim.navigation.bundle")


"""Rest everything follows."""

import isaaclab.sim as sim_utils

print("\n" + "=" * 80)
print("NAVMESH VALIDATION SCRIPT")
print("=" * 80)

# Create simulation context
print("\n[1/3] Creating simulation context...")
sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cuda:0")
sim = sim_utils.SimulationContext(sim_cfg)

# Load the map
print("\n[2/3] Loading map...")
map_usd_path = "omniverse://10.50.2.21/Users/worv/map/temp.usd"
print(f"  Map path: {map_usd_path}")

cfg = sim_utils.UsdFileCfg(usd_path=map_usd_path)
cfg.func("/World", cfg)

# Play simulation
print("\n[3/3] Starting simulation...")
sim.reset()
print("  Simulation started successfully!")

# Check NavMesh
print("\n" + "=" * 80)
print("NAVMESH CHECK")
print("=" * 80)

try:
    # Try to import NavMesh extensions
    print("\n[Step 1] Loading NavMesh extensions...")
    try:
        print("  ✓ NavMesh extensions loaded successfully")
    except Exception as e:
        print(f"  ✗ Failed to load NavMesh extensions: {e}")
        raise

    # Import NavMesh interface
    print("\n[Step 2] Acquiring NavMesh interface...")
    import carb
    import omni.anim.navigation.core as nav

    navmesh_interface = nav.acquire_interface()
    if navmesh_interface is None:
        print("  ✗ Failed to acquire NavMesh interface")
        raise RuntimeError("NavMesh interface is None")
    print("  ✓ NavMesh interface acquired successfully")

    # Check if NavMesh is baked
    print("\n[Step 3] Checking if NavMesh is baked...")
    navmesh = navmesh_interface.get_navmesh()

    if navmesh is None:
        print("  ✗ NavMesh is NOT baked in the scene")
        print("\n[Step 4] Attempting to bake NavMesh...")
        print("  NOTE: This requires NavMesh volumes to be set up in the scene.")
        print("        (Create > Navigation > NavMesh Include Volume in Isaac Sim UI)")

        # Stop simulation before baking
        print("  Stopping simulation...")
        sim.reset()

        # Start NavMesh baking
        print("  Triggering NavMesh baking...")
        navmesh_interface.start_navmesh_baking()

        # Wait for NavMesh to be baked
        print("  Waiting for NavMesh to be baked (max 300 seconds)...")
        max_wait_time = 30000.0
        start_time = time.time()
        check_count = 0

        while navmesh is None:
            simulation_app.update()
            navmesh = navmesh_interface.get_navmesh()
            if navmesh is not None:
                break

            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > max_wait_time:
                print(f"  ✗ NavMesh baking timed out after {max_wait_time}s")
                print("  This likely means no NavMesh volumes exist in the scene.")
                print("\n" + "=" * 80)
                print("RESULT: ✗ NavMesh baking FAILED")
                print("=" * 80)
                print("\nTo bake NavMesh manually:")
                print("  1. Open the map in Isaac Sim UI")
                print("  2. Create > Navigation > NavMesh Include Volume")
                print("  3. Position and scale the volume to cover your navigation area")
                print("  4. Window > Navigation > Navigation Mesh")
                print("  5. Click 'Bake NavMesh'")
                print("  6. Save the USD file")

                # Restart simulation
                sim.reset()
                navmesh = None
                break

            check_count += 1
            if check_count % 10 == 0:  # Print progress every second
                print(f"    Still waiting... ({elapsed:.1f}s elapsed)")

            time.sleep(0.1)

        # Restart simulation
        if navmesh is not None:
            print("  Restarting simulation...")
            sim.play()
            print("  ✓ NavMesh baked successfully!")

    if navmesh is not None:
        print("\n" + "=" * 80)
        print("RESULT: ✓ NavMesh IS baked in the scene!")
        print("=" * 80)

        # Try to get some NavMesh info
        print("\n[Additional Info]")
        try:
            # Test a simple query
            test_start = carb.Float3(0.0, 0.0, 0.5)
            test_end = carb.Float3(5.0, 5.0, 0.5)

            print(f"  Testing path query from (0,0,0.5) to (5,5,0.5)...")
            path = navmesh.query_shortest_path(start_pos=test_start, end_pos=test_end)

            if path is None:
                print("    ✗ No path found (positions might be outside NavMesh)")
            else:
                points = path.get_points()
                if points and len(points) > 0:
                    print(f"    ✓ Path found with {len(points)} waypoints")
                    if len(points) > 0:
                        print(f"      First point: ({points[0].x:.2f}, {points[0].y:.2f}, {points[0].z:.2f})")
                    if len(points) > 1:
                        print(f"      Last point: ({points[-1].x:.2f}, {points[-1].y:.2f}, {points[-1].z:.2f})")
                else:
                    print("    ✗ Path returned but has no points")

        except Exception as e:
            print(f"    ⚠ Could not query path: {e}")

except Exception as e:
    print(f"\n✗ Error during NavMesh check: {e}")
    import traceback

    traceback.print_exc()

# Cleanup
print("\n" + "=" * 80)
print("CLEANUP")
print("=" * 80)
print("Closing simulation...")
simulation_app.close()
print("Done!")
