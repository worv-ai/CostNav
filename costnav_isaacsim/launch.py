#!/usr/bin/env python3
"""
Essential script to open Street_sidewalk.usd using Isaac Sim.
Combines simple USD loading with optional simulation and robot support.

Usage:
    # Simple load
    python launch.py

    # With simulation
    python launch.py --simulate

    # With robot
    python launch.py --simulate --robot carter

For more details, see /workspace/docs/nav2/isaac_sim_launch.md
"""

import argparse

from isaacsim import SimulationApp

# Parse arguments before creating SimulationApp
parser = argparse.ArgumentParser(description="Load Street_sidewalk.usd in Isaac Sim")
parser.add_argument(
    "--usd_path",
    type=str,
    default="omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
    help="Path to USD file",
)
parser.add_argument("--simulate", action="store_true", help="Enable physics simulation")
parser.add_argument("--robot", type=str, choices=["carter", "franka", "ur10"], help="Add robot to scene")
parser.add_argument("--headless", action="store_true", help="Run without GUI")

args = parser.parse_args()

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": args.headless})

# Import after SimulationApp
import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Usd


def main():
    print(f"Loading: {args.usd_path}")

    if args.simulate:
        # Create World for simulation
        world = World()
        world.initialize_physics()
        world.scene.add_default_ground_plane()

        # Load USD as reference
        prim = add_reference_to_stage(usd_path=args.usd_path, prim_path="/World/Environment")
        if prim and prim.IsValid():
            print(f"✓ Loaded at /World/Environment")

        # Add robot if requested
        if args.robot:
            robot_paths = {
                "carter": f"{get_assets_root_path()}/Isaac/Robots/Carter/carter_v2.usd",
                "franka": f"{get_assets_root_path()}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
                "ur10": f"{get_assets_root_path()}/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
            }
            add_reference_to_stage(usd_path=robot_paths[args.robot], prim_path=f"/World/{args.robot}")
            print(f"✓ Added {args.robot} robot")

        world.set_camera_view([5.0, 5.0, 5.0], [0.0, 0.0, 0.0])
        world.reset()

        print("Simulation running. Press Ctrl+C to exit...")
        while simulation_app.is_running():
            world.step(render=True)
    else:
        # Simple load without simulation
        if not Usd.Stage.IsSupportedFile(args.usd_path):
            print(f"✗ Not a valid USD file: {args.usd_path}")
            simulation_app.close()
            return

        usd_context = omni.usd.get_context()
        result = usd_context.open_stage(args.usd_path)

        if result:
            print(f"✓ Successfully opened")
            print("Press Ctrl+C to exit...")
            while simulation_app.is_running():
                simulation_app.update()
        else:
            print(f"✗ Failed to open")

    simulation_app.close()


if __name__ == "__main__":
    main()
