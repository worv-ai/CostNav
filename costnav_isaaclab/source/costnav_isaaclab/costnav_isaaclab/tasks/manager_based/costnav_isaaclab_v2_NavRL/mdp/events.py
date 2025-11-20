# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom event functions for the environment."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def reset_root_state_from_safe_positions(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    safe_positions: list[tuple[float, float, float]],
    velocity_range: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset the asset root state by sampling from pre-validated safe positions.

    This function samples robot spawn positions from a list of safe positions that have been
    validated using upward raycasting to ensure they are not inside buildings or obstacles.

    Args:
        env: The environment instance.
        env_ids: Environment IDs to reset.
        safe_positions: List of (x, y, z) tuples representing safe spawn positions.
        velocity_range: Dictionary of velocity ranges for x, y, z, roll, pitch, yaw.
        asset_cfg: Scene entity configuration for the asset to reset.
    """
    # Extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # Get number of environments to reset
    num_resets = len(env_ids)

    # Sample random indices from safe positions
    num_safe_positions = len(safe_positions)
    if num_safe_positions == 0:
        raise ValueError("No safe positions available for robot spawning!")

    # Sample positions from the safe positions list
    position_indices = torch.randint(0, num_safe_positions, (num_resets,), device=env.device)

    # Convert safe positions to tensor
    safe_positions_tensor = torch.tensor(safe_positions, dtype=torch.float32, device=env.device)

    # Get the sampled positions
    sampled_positions = safe_positions_tensor[position_indices]

    # Get default root state
    root_states = asset.data.default_root_state[env_ids].clone()

    # Set positions (x, y, z)
    root_states[:, 0:3] = sampled_positions

    # Sample random yaw orientation
    if "yaw" in velocity_range:
        yaw_range = velocity_range["yaw"]
        yaw = torch.empty(num_resets, device=env.device).uniform_(*yaw_range)
        # Convert yaw to quaternion (assuming roll=0, pitch=0)
        # q = [cos(yaw/2), 0, 0, sin(yaw/2)]
        root_states[:, 3] = torch.cos(yaw / 2)  # w
        root_states[:, 4] = 0.0  # x
        root_states[:, 5] = 0.0  # y
        root_states[:, 6] = torch.sin(yaw / 2)  # z

    # Sample velocities
    velocity_keys = ["x", "y", "z", "roll", "pitch", "yaw"]
    for i, key in enumerate(velocity_keys):
        if key in velocity_range:
            vel_range = velocity_range[key]
            root_states[:, 7 + i] = torch.empty(num_resets, device=env.device).uniform_(*vel_range)

    # Set into physics simulation
    asset.write_root_state_to_sim(root_states, env_ids=env_ids)


def print_episode_rewards(env: ManagerBasedEnv, env_ids: torch.Tensor):
    """Print cumulative episode rewards for each terminated environment.

    This function prints the total cumulative reward for each environment that is being reset,
    along with the termination reason (collision, arrival, or timeout).

    Args:
        env: The environment instance.
        env_ids: Environment IDs that are being reset (terminated).
    """
    if len(env_ids) == 0:
        return

    # Get cumulative rewards for each reward term
    reward_manager = env.reward_manager

    # Calculate total cumulative reward for each resetting environment
    total_rewards = torch.zeros(len(env_ids), device=env.device)
    for term_name, episode_sum in reward_manager._episode_sums.items():
        total_rewards += episode_sum[env_ids]

    # Get termination reasons
    termination_manager = env.termination_manager

    # Print rewards for each environment
    for idx, env_id in enumerate(env_ids):
        env_id_val = env_id.item()
        total_reward = total_rewards[idx].item()

        # Determine termination reason
        termination_reason = "unknown"
        if hasattr(termination_manager, "_term_dones"):
            for term_name, term_dones in termination_manager._term_dones.items():
                if term_dones[env_id]:
                    termination_reason = term_name
                    break

        # Print the episode summary
        print(f"[Episode End] Env {env_id_val:3d} | Total Reward: {total_reward:8.3f} | Reason: {termination_reason}")

        # Optionally print individual reward components
        # Uncomment the following lines if you want to see breakdown by reward term
        print("  Reward breakdown:")
        for term_name, episode_sum in reward_manager._episode_sums.items():
            term_reward = episode_sum[env_id].item()
            print(f"    {term_name:30s}: {term_reward:8.3f}")
