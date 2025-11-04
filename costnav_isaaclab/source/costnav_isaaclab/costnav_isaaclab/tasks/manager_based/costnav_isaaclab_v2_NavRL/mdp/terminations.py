# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom termination functions for the costnav_isaaclab_v2_NavRL environment."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def arrive(env: ManagerBasedRLEnv, threshold: float, command_name: str) -> torch.Tensor:
    """Terminate when the robot arrives at the goal position.
    
    This function checks if the robot is within a threshold distance of the goal
    and has been running for at least 10 steps to avoid premature termination.
    
    Args:
        env: The environment instance.
        threshold: Distance threshold for arrival (in meters).
        command_name: Name of the command term containing the goal position.
        
    Returns:
        Boolean tensor indicating which environments should terminate.
    """
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :2]  # Only use x, y position (2D navigation)
    distance = torch.norm(des_pos_b, dim=1)
    # Terminate if within threshold and episode has run for at least 10 steps
    return ((distance <= threshold).bool() * (env.episode_length_buf >= 10).bool()).bool()

