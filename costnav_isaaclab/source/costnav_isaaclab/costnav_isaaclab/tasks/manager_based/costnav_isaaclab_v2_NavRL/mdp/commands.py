# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom command generators for the environment."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
import torch
from isaaclab.assets import Articulation
from isaaclab.envs.mdp.commands import UniformPose2dCommand
from isaaclab.managers import CommandTermCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import GREEN_ARROW_X_MARKER_CFG
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class SafePositionPose2dCommand(UniformPose2dCommand):
    """Command generator that samples goal positions from pre-validated safe positions.

    This command generator samples 2D goal positions from a list of safe positions that have been
    validated using upward raycasting to ensure they are not inside buildings or obstacles.
    The heading command is either set to point towards the target or is sampled uniformly.
    """

    cfg: SafePositionPose2dCommandCfg
    """Configuration for the command generator."""

    def __init__(self, cfg: SafePositionPose2dCommandCfg, env: ManagerBasedEnv):
        """Initialize the command generator.

        Args:
            cfg: The configuration for the command generator.
            env: The environment instance.
        """
        # Initialize parent class
        super().__init__(cfg, env)

        # Convert safe positions to tensor
        if len(cfg.safe_positions) == 0:
            raise ValueError("No safe positions provided for goal sampling!")

        self.safe_positions = torch.tensor(
            cfg.safe_positions, dtype=torch.float32, device=self.device
        )
        self.num_safe_positions = len(cfg.safe_positions)

    def _resample_command(self, env_ids: Sequence[int]):
        """Resample goal positions from safe positions.

        Args:
            env_ids: Environment IDs for which to resample commands.
        """
        num_resets = len(env_ids)

        # Sample random indices from safe positions
        position_indices = torch.randint(
            0, self.num_safe_positions, (num_resets,), device=self.device
        )

        # Get the sampled positions
        sampled_positions = self.safe_positions[position_indices]

        # Set position commands (x, y, z)
        self.pos_command_w[env_ids, 0:3] = sampled_positions

        # Set heading command
        if self.cfg.simple_heading:
            # Compute heading to point towards the target
            target_vec = self.pos_command_w[env_ids] - self.robot.data.root_pos_w[env_ids, :3]
            target_direction = math_utils.wrap_to_pi(
                torch.atan2(target_vec[:, 1], target_vec[:, 0])
            )

            # Account for the current heading
            curr_to_target = torch.abs(
                math_utils.wrap_to_pi(target_direction - self.robot.data.heading_w[env_ids])
            )
            flipped_target_direction = math_utils.wrap_to_pi(target_direction + torch.pi)
            curr_to_flipped_target = torch.abs(
                math_utils.wrap_to_pi(flipped_target_direction - self.robot.data.heading_w[env_ids])
            )

            # set the heading command to the closest direction
            self.heading_command_w[env_ids] = torch.where(
                curr_to_target < curr_to_flipped_target,
                target_direction,
                flipped_target_direction,
            )
        else:
            # Random heading command
            r = torch.empty(num_resets, device=self.device)
            self.heading_command_w[env_ids] = r.uniform_(*self.cfg.ranges.heading)


@configclass
class SafePositionPose2dCommandCfg(CommandTermCfg):
    """Configuration for the safe position-based 2D-pose command generator."""

    class_type: type = SafePositionPose2dCommand

    asset_name: str = "robot"
    """Name of the asset in the environment for which the commands are generated."""

    simple_heading: bool = True
    """Whether to use simple heading or not.

    If True, the heading is in the direction of the target position.
    """

    safe_positions: list[tuple[float, float, float]] = []
    """List of (x, y, z) tuples representing safe goal positions.

    These positions should be pre-validated using upward raycasting to ensure
    they are not inside buildings or obstacles.
    """

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/pose_goal"
    )
    """The configuration for the goal pose visualization marker. Defaults to GREEN_ARROW_X_MARKER_CFG."""

    # Set the scale of the visualization markers to (0.2, 0.2, 0.8)
    goal_pose_visualizer_cfg.markers["arrow"].scale = (0.2, 0.2, 0.8)

    @configclass
    class Ranges:
        """Ranges for the heading command (only used if simple_heading=False)."""

        heading: tuple[float, float] = (-3.14159, 3.14159)
        """Heading range for the position command (in radians). Defaults to (-pi, pi)."""

    ranges: Ranges = Ranges()
    """Ranges for the heading command."""
