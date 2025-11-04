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
    return (1 - torch.tanh(distance / std)).float()


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
    command = env.command_manager.get_command(command_name)
    movement_xy = command[:, -1:]  # Last dimension contains movement metric
    reward = movement_xy[:, 0]
    # Only give reward after initial steps to avoid instability
    return reward * (env.episode_length_buf >= 10).float()


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

    vel_direction = target_pos / distance_to_target_pos.clamp_min(1e-6)
    reward_vel = (vel * vel_direction).sum(-1)
    return reward_vel
