#!/usr/bin/env python3
# Copyright (c) 2025, COCO Navigation Project.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to test the robot controller with varying v and w values."""

"""Launch Isaac Sim Simulator first."""

import argparse
import time

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Test controller for COCO robot.")
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Template-Costnav-Isaaclab-v2-NavRL", help="Name of the task.")
parser.add_argument(
    "--mode",
    type=str,
    default="interactive",
    choices=["interactive", "sweep"],
    help="Test mode: interactive (keyboard) or sweep (automated test)",
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys
import threading

import costnav_isaaclab.tasks  # noqa: F401
import gymnasium as gym
import isaaclab_tasks  # noqa: F401
import numpy as np
import torch
from isaaclab_tasks.utils import parse_env_cfg

# Global variable to store keyboard input
keyboard_buffer = []
keyboard_lock = threading.Lock()
stop_keyboard_thread = False


def print_instructions():
    """Print keyboard control instructions."""
    print("\n" + "=" * 80)
    print("CONTROLLER TEST - KEYBOARD CONTROLS")
    print("=" * 80)
    print("\nVelocity Controls:")
    print("  W/S: Increase/Decrease linear velocity (v)")
    print("  A/D: Increase/Decrease steering angle (left/right)")
    print("  X:   Stop (v=0, steering=0)")
    print("\nPreset Commands:")
    print("  1: Forward straight (v=1.0, steering=0.0)")
    print("  2: Forward + right turn (v=1.0, steering=0.35)")
    print("  3: Forward + left turn (v=1.0, steering=-0.35)")
    print("  4: Backward straight (v=-1.0, steering=0.0)")
    print("  5: Forward + sharp right (v=1.0, steering=0.61)")
    print("  6: Forward + sharp left (v=1.0, steering=-0.61)")
    print("\nOther:")
    print("  SPACE: Emergency stop")
    print("  Q:     Quit")
    print("\nNote: Actions are [linear_velocity, steering_angle] (raw numbers).")
    print("      ClassicalCarActionCfg converts them to wheel commands.")
    print("=" * 80)


def keyboard_listener_thread():
    """Background thread to listen for keyboard input."""
    global keyboard_buffer, stop_keyboard_thread

    import select
    import termios
    import tty

    # Set terminal to raw mode
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while not stop_keyboard_thread:
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                with keyboard_lock:
                    keyboard_buffer.append(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def get_keyboard_input():
    """Get keyboard input from buffer (non-blocking)."""
    global keyboard_buffer

    with keyboard_lock:
        if keyboard_buffer:
            return keyboard_buffer.pop(0)
    return None


def interactive_mode(env):
    """Interactive keyboard control mode."""
    global stop_keyboard_thread

    print_instructions()

    # Start keyboard listener thread
    stop_keyboard_thread = False
    kb_thread = threading.Thread(target=keyboard_listener_thread, daemon=True)
    kb_thread.start()

    # Current velocity commands
    v = 0.0  # Linear velocity (m/s)
    steering = 0.0  # Steering angle (radians)

    # Control parameters
    v_step = 0.2  # m/s
    steering_step = 0.2

    # Reset environment
    env.reset()

    print("\nStarting interactive mode...")
    print(f"Current: v={v:.3f}, steering={steering:.3f}")
    print("Press keys to control the robot...")

    # Simulate environment
    step_count = 0
    while simulation_app.is_running():
        start_time = time.time()

        # Get keyboard input
        key = get_keyboard_input()

        if key:
            # Velocity controls
            if key == "w" or key == "W":
                v = v + v_step
                print(f"Increase v: action=[{v:.3f}, {steering:.3f}]")
            elif key == "s" or key == "S":
                v = v - v_step
                print(f"Decrease v: action=[{v:.3f}, {steering:.3f}]")
            elif key == "a" or key == "A":
                steering = steering + steering_step
                print(f"Turn left: action=[{v:.3f}, {steering:.3f}]")
            elif key == "d" or key == "D":
                steering = steering - steering_step
                print(f"Turn right: action=[{v:.3f}, {steering:.3f}]")
            elif key == "x" or key == "X":
                v, steering = 0.0, 0.0
                print(f"Stop: action=[{v:.3f}, {steering:.3f}]")

            # Preset commands
            elif key == "1":
                v, steering = 1.0, 0.0
                print(f"Preset 1 - Forward straight: action=[{v:.3f}, {steering:.3f}]")
            elif key == "2":
                v, steering = 1.0, 20.0 * torch.pi / 180
                print(f"Preset 2 - Forward + right: action=[{v:.3f}, {steering:.3f}]")
            elif key == "3":
                v, steering = 1.0, -20.0 * torch.pi / 180
                print(f"Preset 3 - Forward + left: action=[{v:.3f}, {steering:.3f}]")
            elif key == "4":
                v, steering = -1.0, 0.0
                print(f"Preset 4 - Backward: action=[{v:.3f}, {steering:.3f}]")
            elif key == "5":
                v, steering = 1.0, 35.0 * torch.pi / 180
                print(f"Preset 5 - Sharp right: action=[{v:.3f}, {steering:.3f}]")
            elif key == "6":
                v, steering = 1.0, -35.0 * torch.pi / 180
                print(f"Preset 6 - Sharp left: action=[{v:.3f}, {steering:.3f}]")

            # Emergency stop
            elif key == " ":
                v, steering = 0.0, 0.0
                print(f"EMERGENCY STOP: action=[{v:.3f}, {steering:.3f}]")

            # Quit
            elif key == "q" or key == "Q":
                print("Quitting...")
                break

        # Create action tensor - [linear_velocity, steering_angle]
        # The ClassicalCarActionCfg will process these
        actions = torch.tensor([[v, steering]], device=env.unwrapped.device, dtype=torch.float32)

        # Apply actions
        with torch.inference_mode():
            env.step(actions)

        step_count += 1

        # Print status every 100 steps
        if step_count % 100 == 0:
            print(f"Step {step_count}: action=[{v:.3f}, {steering:.3f}]")

        # Small delay for real-time control
        sleep_time = 0.05 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Stop keyboard thread
    stop_keyboard_thread = True
    kb_thread.join(timeout=1.0)


def sweep_mode(env):
    """Automated sweep through different velocity and steering commands."""
    print("\n" + "=" * 80)
    print("CONTROLLER TEST - AUTOMATED SWEEP MODE")
    print("=" * 80)

    # Test cases: (v, steering_deg, duration_steps, description)
    test_cases = [
        (0.0, 0.0, 50, "Stop"),
        (1.0, 0.0, 100, "Forward straight (v=1.0)"),
        (2.0, 0.0, 100, "Forward fast (v=2.0)"),
        (-1.0, 0.0, 100, "Backward straight (v=-1.0)"),
        (-2.0, 0.0, 100, "Backward fast (v=-2.0)"),
        (1.0, 10.0, 150, "Forward + slight right (10°)"),
        (1.0, -10.0, 150, "Forward + slight left (-10°)"),
        (1.0, 25.0, 150, "Forward + medium right (25°)"),
        (1.0, -25.0, 150, "Forward + medium left (-25°)"),
        (1.0, 40.0, 150, "Forward + sharp right (40°)"),
        (1.0, -40.0, 150, "Forward + sharp left (-40°)"),
        (-1.0, 20.0, 150, "Backward + right (20°)"),
        (-1.0, -20.0, 150, "Backward + left (-20°)"),
    ]

    # Reset environment
    env.reset()

    print(f"\nRunning {len(test_cases)} test cases...")
    print("Actions are [linear_velocity, steering_angle] for ClassicalCarActionCfg.\n")

    for i, (v, steering_deg, duration, description) in enumerate(test_cases):
        steering_rad = steering_deg * torch.pi / 180

        print(f"\n[Test {i + 1}/{len(test_cases)}] {description}")
        print(f"  Velocity: {v:.2f} m/s")
        print(f"  Steering: {steering_deg:.1f}° ({steering_rad:.3f} rad)")
        print(f"  Duration: {duration} steps ({duration * 0.05:.1f} seconds)")

        # Create action tensor - [linear_velocity, steering_angle]
        actions = torch.tensor([[v, steering_rad]], device=env.unwrapped.device, dtype=torch.float32)

        # Run for specified duration
        for _ in range(duration):
            with torch.inference_mode():
                env.step(actions)

            # Small delay for visualization
            time.sleep(0.02)

        # Brief pause between tests
        print("  ✓ Complete")
        time.sleep(0.5)

    print("\n" + "=" * 80)
    print("All test cases completed!")
    print("=" * 80)


def main():
    """Test controller with varying v and w values."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info
    print(f"\n[INFO]: Task: {args_cli.task}")
    print(f"[INFO]: Num envs: {env.unwrapped.num_envs}")
    print(f"[INFO]: Action space: {env.action_space}")
    print(f"[INFO]: Observation space: {env.observation_space}")

    # Run selected mode
    if args_cli.mode == "interactive":
        interactive_mode(env)
    else:
        sweep_mode(env)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
