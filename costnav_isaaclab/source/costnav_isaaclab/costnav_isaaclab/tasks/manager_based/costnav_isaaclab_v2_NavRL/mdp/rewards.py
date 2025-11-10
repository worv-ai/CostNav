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


def joint_pos_target_l2(
    env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)


def position_command_error_tanh(
    env: ManagerBasedRLEnv, std: float, command_name: str
) -> torch.Tensor:
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


def distance_to_goal_progress(
    env: ManagerBasedRLEnv, command_name: str, slack_penalty: float = 0.01
) -> torch.Tensor:
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

    # Compute progress: previous_distance - current_distance
    # Positive when moving closer to goal, negative when moving away
    progress = command_term._prev_distance - current_distance

    # Apply slack penalty
    reward = progress - slack_penalty

    # Update previous distance for next step
    command_term._prev_distance[:] = current_distance

    # Reset previous distance for environments that just reset
    # This prevents large reward spikes on reset
    if hasattr(env, "episode_length_buf"):
        reset_mask = env.episode_length_buf == 0
        if reset_mask.any():
            command_term._prev_distance[reset_mask] = current_distance[reset_mask]
            reward[reset_mask] = -slack_penalty  # Only apply slack penalty on first step

    return reward
