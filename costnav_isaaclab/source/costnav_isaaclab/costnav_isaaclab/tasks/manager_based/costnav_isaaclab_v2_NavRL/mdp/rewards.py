# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)


def position_command_error_tanh(env: ManagerBasedRLEnv, std: float, command_name: str) -> torch.Tensor:
    """Reward position tracking with tanh kernel.

    This function computes a reward based on the distance to the goal position.
    The reward is 1.0 when at the goal and decreases with distance using a tanh kernel.

    Args:
        env: The environment instance.
        std: Standard deviation for the tanh kernel (controls reward decay rate).
        command_name: Name of the command term containing the goal position.

    Returns:
        Reward tensor of shape (num_envs,).
    """
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :2]  # Only use x, y position (2D navigation)
    distance = torch.norm(des_pos_b, dim=1)

    # Compute reward with safety checks
    reward = (1 - torch.tanh(distance / std)).float()

    # Replace NaN/Inf with zeros
    reward = torch.where(torch.isfinite(reward), reward, torch.zeros_like(reward))

    return reward


def heading_command_error_abs(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize tracking orientation error.

    Args:
        env: The environment instance.
        command_name: Name of the command term containing the heading.

    Returns:
        Absolute heading error tensor of shape (num_envs,).
    """
    command = env.command_manager.get_command(command_name)
    heading_b = command[:, 3]
    return heading_b.abs()


def moving_towards_goal_reward(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Reward for moving towards the goal.

    This function rewards the robot for making progress towards the goal by checking
    the velocity component in the direction of the goal.

    Args:
        env: The environment instance.
        command_name: Name of the command term.

    Returns:
        Reward tensor of shape (num_envs,).
    """
    # Get the command manager to access previous distance
    command_term = env.command_manager.get_term(command_name)

    # Get current distance to goal
    command = env.command_manager.get_command(command_name)
    target_pos = command[:, :2]  # x, y position in base frame
    current_distance = torch.norm(target_pos, dim=1)

    # Compute movement towards goal (negative means moving closer)
    # We need to track previous distance - use metrics from command term
    if hasattr(command_term, "metrics") and "error_pos_2d" in command_term.metrics:
        # Use the stored distance metric if available
        # Reward is positive when distance decreases
        asset = env.scene["robot"]
        vel = asset.data.root_lin_vel_b[:, 0:2]

        # Compute velocity component towards goal
        distance_to_target = current_distance.clamp_min(1e-6)
        vel_direction = target_pos / distance_to_target.unsqueeze(-1)
        movement_reward = (vel * vel_direction).sum(-1)

        # Only give reward after initial steps to avoid instability
        reward = movement_reward * (env.episode_length_buf >= 10).float()

        # Clamp to prevent extreme values
        reward = torch.clamp(reward, -10.0, 10.0)

        # Replace NaN/Inf with zeros
        reward = torch.where(torch.isfinite(reward), reward, torch.zeros_like(reward))

        return reward
    else:
        # Fallback: return zero if metrics not available
        return torch.zeros(env.num_envs, device=env.device)


def target_vel_reward(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Reward for maintaining velocity towards the target.

    This function rewards the robot for moving with velocity aligned towards the goal.

    Args:
        env: The environment instance.
        command_name: Name of the command term containing the target position.

    Returns:
        Reward tensor of shape (num_envs,).
    """
    command = env.command_manager.get_command(command_name)
    target_pos = command[:, :2]
    distance_to_target_pos = torch.linalg.norm(target_pos, dim=1, keepdim=True)

    asset = env.scene["robot"]
    vel = asset.data.root_lin_vel_b[:, 0:2]

    # Avoid division by very small numbers - use a larger epsilon
    # When very close to target (< 0.1m), don't give velocity reward
    distance_threshold = 0.1
    safe_distance = distance_to_target_pos.clamp_min(distance_threshold)

    # Compute velocity direction only when far enough from target
    vel_direction = target_pos / safe_distance
    reward_vel = (vel * vel_direction).sum(-1)

    # Zero out reward when too close to target to avoid numerical issues
    mask = (distance_to_target_pos.squeeze(-1) > distance_threshold).float()
    reward_vel = reward_vel * mask

    # Clamp to prevent extreme values
    reward_vel = torch.clamp(reward_vel, -10.0, 10.0)

    # Replace NaN/Inf with zeros
    reward_vel = torch.where(torch.isfinite(reward_vel), reward_vel, torch.zeros_like(reward_vel))

    return reward_vel


def distance_to_goal_progress(env: ManagerBasedRLEnv, command_name: str, slack_penalty: float = 0.01) -> torch.Tensor:
    """Reward based on change in distance to goal (progress reward).

    This implements the reward function:
        r_t = D(P_t, g) - D(P_{t+1}, g) - λ

    where D(·, ·) is the distance to goal, and λ is a slack penalty to encourage
    efficient paths. Positive reward is given when the robot moves closer to the goal.

    Args:
        env: The environment instance.
        command_name: Name of the command term containing the goal position.
        slack_penalty: Constant slack penalty (λ) to encourage efficient paths.

    Returns:
        Reward tensor of shape (num_envs,).
    """
    # Get the command term to access previous distance
    command_term = env.command_manager.get_term(command_name)

    # Get current distance to goal
    command = env.command_manager.get_command(command_name)
    target_pos = command[:, :2]  # x, y position in base frame
    current_distance = torch.norm(target_pos, dim=1)

    # Initialize previous distance buffer if it doesn't exist
    if not hasattr(command_term, "_prev_distance"):
        command_term._prev_distance = torch.zeros(env.num_envs, device=env.device)
        command_term._prev_distance[:] = current_distance

    # Detect environments that just reset (episode_length_buf <= 1)
    # This prevents large reward spikes when goal changes after reset
    # episode_length_buf is incremented before reward computation, so:
    # - After reset: episode_length_buf is set to 0, then incremented to 1
    # - We reset _prev_distance on the first step (episode_length_buf == 1)
    if hasattr(env, "episode_length_buf"):
        reset_mask = env.episode_length_buf <= 1
        if reset_mask.any():
            command_term._prev_distance[reset_mask] = current_distance[reset_mask]

    # Compute progress: previous_distance - current_distance
    # Positive when moving closer to goal, negative when moving away
    progress = command_term._prev_distance - current_distance

    # Apply slack penalty
    reward = progress - slack_penalty

    # Update previous distance for next step
    command_term._prev_distance[:] = current_distance

    return reward


def print_rewards(env: ManagerBasedRLEnv, print_every_n_steps: int = 1) -> torch.Tensor:
    """Print all reward components for debugging.

    This function prints detailed reward information for the first environment
    at every N steps. It returns zero reward (doesn't affect training).

    Args:
        env: The environment instance.
        print_every_n_steps: Print frequency (default: every step).

    Returns:
        Zero tensor (this reward doesn't contribute to training).
    """
    try:
        # Only print for first environment to avoid spam
        env_id = 0

        # Check if we should print this step
        step = env.common_step_counter
        if step % print_every_n_steps != 0:
            return torch.zeros(env.num_envs, device=env.device)

        # Get reward manager to access individual reward terms
        reward_manager = env.reward_manager

        # Print header
        print("\n" + "=" * 80)
        print(f"STEP {step} | Episode Step: {env.episode_length_buf[env_id].item()}")
        print("=" * 80)

        # Print observation (goal command)
        if hasattr(env, "observation_manager"):
            print("\nOBSERVATION (Goal):")
            # Try to get the pose_command observation
            try:
                obs_dict = env.observation_manager.compute()
                if "policy" in obs_dict:
                    # For concatenated observations, we need to parse it
                    # The pose_command is the first 2 values (x, y) - normalized to max 5.0m
                    obs = obs_dict["policy"][env_id]
                    goal_obs = obs[:2].cpu().numpy()
                    goal_distance_obs = torch.norm(obs[:2]).item()
                    print(
                        f"  Goal (normalized, max=5.0m): [{goal_obs[0]:.3f}, {goal_obs[1]:.3f}], distance={goal_distance_obs:.3f}m"
                    )
            except Exception as e:
                print(f"  Could not extract observation: {e}")

        # Print action
        if hasattr(env, "action_manager"):
            print("\nACTION:")
            try:
                # Get the last action applied from action_manager._action
                action = env.action_manager._action[env_id]
                print(f"  Action: {action.cpu().numpy()}")
            except Exception as e:
                print(f"  Could not extract action: {e}")

        # Print command information
        if hasattr(env, "command_manager") and len(env.command_manager.active_terms) > 0:
            print("\nCOMMAND INFO (in base frame):")
            for cmd_name in env.command_manager.active_terms:
                cmd = env.command_manager.get_command(cmd_name)[env_id]
                if cmd.shape[0] >= 2:
                    distance = torch.norm(cmd[:2]).item()
                    # cmd format: [x, y, z, heading]
                    print(f"  {cmd_name}:")
                    print(f"    Position (x, y): [{cmd[0].item():.3f}, {cmd[1].item():.3f}], distance={distance:.3f}m")
                    if cmd.shape[0] >= 4:
                        print(f"    Heading: {cmd[3].item():.3f} rad ({cmd[3].item() * 180 / 3.14159:.1f}°)")

        # Print each reward term
        print("\nREWARDS:")
        total_reward = 0.0
        for idx, term_name in enumerate(reward_manager.active_terms):
            # Get the reward value for this term (already weighted and scaled by dt)
            term_value = reward_manager._step_reward[env_id, idx].item()
            total_reward += term_value

            # Get the weight
            term_cfg = reward_manager._term_cfgs[idx]
            weight = term_cfg.weight

            print(f"  {term_name:30s}: {term_value:10.4f} (weight: {weight:8.1f})")

        print("-" * 80)
        print(f"  {'TOTAL REWARD':30s}: {total_reward:10.4f}")

        # Print termination status
        if hasattr(env, "termination_manager"):
            print("\nTERMINATION STATUS:")
            for term_name in env.termination_manager.active_terms:
                term_value = env.termination_manager.get_term(term_name)[env_id].item()
                if term_value > 0:
                    print(f"  {term_name}: TRIGGERED")

        print("=" * 80 + "\n")

    except Exception as e:
        print(f"[DEBUG] Error in print_rewards: {e}")
        import traceback

        traceback.print_exc()

    # Return zero (this reward doesn't contribute to training)
    return torch.zeros(env.num_envs, device=env.device)
