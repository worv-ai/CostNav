# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (Fan Yang, ETH Zurich)
# Licensed under the MIT License

"""Base Model class for IL baselines."""

from typing import Optional, Tuple

import torch
import torch.nn as nn


class BaseModel(nn.Module):
    """Base class for all IL baseline models.

    Args:
        context_size: Number of previous observations to use for context.
        len_traj_pred: Number of waypoints to predict in the future.
        learn_angle: Whether to predict the yaw angle of the robot.
    """

    def __init__(
        self,
        context_size: int = 5,
        len_traj_pred: Optional[int] = 5,
        learn_angle: Optional[bool] = True,
    ) -> None:
        super(BaseModel, self).__init__()
        self.context_size = context_size
        self.learn_angle = learn_angle
        self.len_trajectory_pred = len_traj_pred

        # Action parameters: (x, y) position or (x, y, cos_theta, sin_theta)
        if self.learn_angle:
            self.num_action_params = 4  # last two dims are cos and sin of the angle
        else:
            self.num_action_params = 2

    def flatten(self, z: torch.Tensor) -> torch.Tensor:
        """Flatten feature maps using adaptive average pooling."""
        z = nn.functional.adaptive_avg_pool2d(z, (1, 1))
        z = torch.flatten(z, 1)
        return z

    def forward(self, obs_img: torch.Tensor, goal_img: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Forward pass of the model.

        Args:
            obs_img: Batch of observations of shape [B, C*context_size, H, W].
            goal_img: Batch of goal images of shape [B, 3, H, W].

        Returns:
            dist_pred: Predicted distance to goal of shape [B, 1].
            action_pred: Predicted waypoints of shape [B, len_traj_pred, num_action_params].
        """
        raise NotImplementedError
