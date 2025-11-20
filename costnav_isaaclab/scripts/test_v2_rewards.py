#!/usr/bin/env python3
"""
Quick test script to verify reward functions don't produce NaN/Inf values.
Run this before starting full training to catch issues early.
"""

from isaaclab.app import AppLauncher

app = AppLauncher()

import os
import sys

import torch

# Add the source directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "source/costnav_isaaclab"))


def test_reward_functions():
    """Test that reward functions handle edge cases without NaN/Inf."""

    print("=" * 80)
    print("Testing V2 Reward Functions for NaN/Inf Safety")
    print("=" * 80)

    # Import the reward functions
    from costnav_isaaclab.tasks.manager_based.costnav_isaaclab_v2_NavRL.mdp import rewards

    # Create mock environment data
    num_envs = 64
    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"\nDevice: {device}")
    print(f"Num envs: {num_envs}")

    # Test 1: position_command_error_tanh
    print("\n" + "-" * 80)
    print("Test 1: position_command_error_tanh")
    print("-" * 80)

    # Create mock env with command
    class MockEnv:
        def __init__(self):
            self.num_envs = num_envs
            self.device = device
            self.command_manager = MockCommandManager()

    class MockCommandManager:
        def get_command(self, name):
            # Return [x, y, z, heading] - shape (num_envs, 4)
            return torch.randn(num_envs, 4, device=device)

    env = MockEnv()

    # Test with various std values
    for std in [0.1, 1.0, 5.0, 10.0]:
        reward = rewards.position_command_error_tanh(env, std=std, command_name="pose_command")
        has_nan = torch.isnan(reward).any()
        has_inf = torch.isinf(reward).any()
        print(f"  std={std:5.1f}: min={reward.min():.4f}, max={reward.max():.4f}, NaN={has_nan}, Inf={has_inf}")
        assert not has_nan, f"NaN detected with std={std}"
        assert not has_inf, f"Inf detected with std={std}"

    # Test 2: moving_towards_goal_reward
    print("\n" + "-" * 80)
    print("Test 2: moving_towards_goal_reward")
    print("-" * 80)

    class MockEnvWithRobot:
        def __init__(self):
            self.num_envs = num_envs
            self.device = device
            self.command_manager = MockCommandManagerWithMetrics()
            self.scene = {"robot": MockRobot()}
            self.episode_length_buf = torch.randint(0, 100, (num_envs,), device=device)

    class MockCommandManagerWithMetrics:
        def get_command(self, name):
            return torch.randn(num_envs, 4, device=device)

        def get_term(self, name):
            return MockCommandTerm()

    class MockCommandTerm:
        def __init__(self):
            self.metrics = {"error_pos_2d": torch.rand(num_envs, device=device) * 10}

    class MockRobot:
        def __init__(self):
            self.data = MockRobotData()

    class MockRobotData:
        def __init__(self):
            # Linear velocity in base frame
            self.root_lin_vel_b = torch.randn(num_envs, 3, device=device)

    env = MockEnvWithRobot()

    # Test with various scenarios
    scenarios = [
        ("Random velocities", None),
        ("Zero velocities", torch.zeros(num_envs, 3, device=device)),
        ("High velocities", torch.randn(num_envs, 3, device=device) * 10),
    ]

    for scenario_name, vel_override in scenarios:
        if vel_override is not None:
            env.scene["robot"].data.root_lin_vel_b = vel_override

        reward = rewards.moving_towards_goal_reward(env, command_name="pose_command")
        has_nan = torch.isnan(reward).any()
        has_inf = torch.isinf(reward).any()
        print(f"  {scenario_name:20s}: min={reward.min():7.4f}, max={reward.max():7.4f}, NaN={has_nan}, Inf={has_inf}")
        assert not has_nan, f"NaN detected in {scenario_name}"
        assert not has_inf, f"Inf detected in {scenario_name}"

    # Test 3: target_vel_reward
    print("\n" + "-" * 80)
    print("Test 3: target_vel_reward")
    print("-" * 80)

    # Test with various distances to target
    test_cases = [
        ("Very close (0.01m)", torch.randn(num_envs, 4, device=device) * 0.01),
        ("Close (0.1m)", torch.randn(num_envs, 4, device=device) * 0.1),
        ("Medium (1m)", torch.randn(num_envs, 4, device=device) * 1.0),
        ("Far (10m)", torch.randn(num_envs, 4, device=device) * 10.0),
    ]

    for case_name, command in test_cases:
        env.command_manager.get_command = lambda name: command

        reward = rewards.target_vel_reward(env, command_name="pose_command")
        has_nan = torch.isnan(reward).any()
        has_inf = torch.isinf(reward).any()
        print(f"  {case_name:20s}: min={reward.min():7.4f}, max={reward.max():7.4f}, NaN={has_nan}, Inf={has_inf}")
        assert not has_nan, f"NaN detected in {case_name}"
        assert not has_inf, f"Inf detected in {case_name}"

    # Test 4: Edge case - exactly at target
    print("\n" + "-" * 80)
    print("Test 4: Edge Cases")
    print("-" * 80)

    # Exactly at target (distance = 0)
    zero_command = torch.zeros(num_envs, 4, device=device)
    env.command_manager.get_command = lambda name: zero_command

    reward = rewards.target_vel_reward(env, command_name="pose_command")
    has_nan = torch.isnan(reward).any()
    has_inf = torch.isinf(reward).any()
    print(f"  At target (dist=0):     min={reward.min():7.4f}, max={reward.max():7.4f}, NaN={has_nan}, Inf={has_inf}")
    assert not has_nan, "NaN detected when at target"
    assert not has_inf, "Inf detected when at target"

    print("\n" + "=" * 80)
    print("✅ All tests passed! Reward functions are safe from NaN/Inf.")
    print("=" * 80)
    print("\nYou can now run training with confidence:")
    print("  python scripts/rl_games/train.py --task Template-Costnav-Isaaclab-v2-NavRL")
    print()


if __name__ == "__main__":
    test_reward_functions()
