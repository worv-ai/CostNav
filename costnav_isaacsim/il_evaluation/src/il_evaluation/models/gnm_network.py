# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (https://github.com/InternRobotics/NavDP)
# Reference: third_party/NavDP/baselines/gnm/gnm_model.py
# Licensed under the MIT License

"""GNM (General Navigation Model) network architecture."""

from typing import Callable, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor
from torchvision.models._utils import _make_divisible
from torchvision.models.mobilenetv2 import InvertedResidual
from torchvision.ops.misc import ConvNormActivation

from .base_model import BaseModel


class MobileNetEncoder(nn.Module):
    """MobileNetV2 encoder that supports stacked multi-image inputs."""

    def __init__(
        self,
        num_images: int = 1,
        num_classes: int = 1000,
        width_mult: float = 1.0,
        inverted_residual_setting: Optional[List[List[int]]] = None,
        round_nearest: int = 8,
        block: Optional[Callable[..., nn.Module]] = None,
        norm_layer: Optional[Callable[..., nn.Module]] = None,
        dropout: float = 0.2,
    ) -> None:
        super().__init__()

        if block is None:
            block = InvertedResidual
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d

        input_channel = 32
        last_channel = 1280

        if inverted_residual_setting is None:
            inverted_residual_setting = [
                [1, 16, 1, 1],
                [6, 24, 2, 2],
                [6, 32, 3, 2],
                [6, 64, 4, 2],
                [6, 96, 3, 1],
                [6, 160, 3, 2],
                [6, 320, 1, 1],
            ]
        if len(inverted_residual_setting) == 0 or len(inverted_residual_setting[0]) != 4:
            raise ValueError(
                f"inverted_residual_setting should be non-empty and 4-element lists, got {inverted_residual_setting}"
            )

        input_channel = _make_divisible(input_channel * width_mult, round_nearest)
        self.last_channel = _make_divisible(last_channel * max(1.0, width_mult), round_nearest)
        features: List[nn.Module] = [
            ConvNormActivation(
                num_images * 3,
                input_channel,
                stride=2,
                norm_layer=norm_layer,
                activation_layer=nn.ReLU6,
            )
        ]

        for t, c, n, s in inverted_residual_setting:
            output_channel = _make_divisible(c * width_mult, round_nearest)
            for i in range(n):
                stride = s if i == 0 else 1
                features.append(
                    block(
                        input_channel,
                        output_channel,
                        stride,
                        expand_ratio=t,
                        norm_layer=norm_layer,
                    )
                )
                input_channel = output_channel

        features.append(
            ConvNormActivation(
                input_channel,
                self.last_channel,
                kernel_size=1,
                norm_layer=norm_layer,
                activation_layer=nn.ReLU6,
            )
        )
        self.features = nn.Sequential(*features)
        self.classifier = nn.Sequential(
            nn.Dropout(p=dropout),
            nn.Linear(self.last_channel, num_classes),
        )

        for module in self.modules():
            if isinstance(module, nn.Conv2d):
                nn.init.kaiming_normal_(module.weight, mode="fan_out")
                if module.bias is not None:
                    nn.init.zeros_(module.bias)
            elif isinstance(module, (nn.BatchNorm2d, nn.GroupNorm)):
                nn.init.ones_(module.weight)
                nn.init.zeros_(module.bias)
            elif isinstance(module, nn.Linear):
                nn.init.normal_(module.weight, 0, 0.01)
                nn.init.zeros_(module.bias)

    def _forward_impl(self, x: Tensor) -> Tensor:
        x = self.features(x)
        x = nn.functional.adaptive_avg_pool2d(x, (1, 1))
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x

    def forward(self, x: Tensor) -> Tensor:
        return self._forward_impl(x)


class GNM(BaseModel):
    """GNM architecture for image-goal navigation."""

    def __init__(
        self,
        context_size: int = 5,
        len_traj_pred: Optional[int] = 5,
        learn_angle: Optional[bool] = True,
        obs_encoding_size: Optional[int] = 1024,
        goal_encoding_size: Optional[int] = 1024,
    ) -> None:
        super().__init__(context_size, len_traj_pred, learn_angle)

        mobilenet = MobileNetEncoder(num_images=1 + self.context_size)
        self.obs_mobilenet = mobilenet.features
        self.obs_encoding_size = obs_encoding_size
        self.compress_observation = nn.Sequential(
            nn.Linear(mobilenet.last_channel, self.obs_encoding_size),
            nn.ReLU(),
        )

        stacked_mobilenet = MobileNetEncoder(num_images=2 + self.context_size)
        self.goal_mobilenet = stacked_mobilenet.features
        self.goal_encoding_size = goal_encoding_size
        self.compress_goal = nn.Sequential(
            nn.Linear(stacked_mobilenet.last_channel, 1024),
            nn.ReLU(),
            nn.Linear(1024, self.goal_encoding_size),
            nn.ReLU(),
        )

        self.linear_layers = nn.Sequential(
            nn.Linear(self.goal_encoding_size + self.obs_encoding_size, 256),
            nn.ReLU(),
            nn.Linear(256, 32),
            nn.ReLU(),
        )
        self.dist_predictor = nn.Sequential(nn.Linear(32, 1))
        self.action_predictor = nn.Sequential(nn.Linear(32, self.len_trajectory_pred * self.num_action_params))

    def forward(self, obs_img: torch.Tensor, goal_img: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        obs_encoding = self.obs_mobilenet(obs_img)
        obs_encoding = self.flatten(obs_encoding)
        obs_encoding = self.compress_observation(obs_encoding)

        obs_goal_input = torch.cat([obs_img, goal_img], dim=1)
        goal_encoding = self.goal_mobilenet(obs_goal_input)
        goal_encoding = self.flatten(goal_encoding)
        goal_encoding = self.compress_goal(goal_encoding)

        embedding = torch.cat([obs_encoding, goal_encoding], dim=1)
        embedding = self.linear_layers(embedding)
        dist_pred = self.dist_predictor(embedding)
        action_pred = self.action_predictor(embedding)

        action_pred = action_pred.reshape((action_pred.shape[0], self.len_trajectory_pred, self.num_action_params))
        action_pred[:, :, :2] = torch.cumsum(action_pred[:, :, :2], dim=1)
        if self.learn_angle:
            action_pred[:, :, 2:] = F.normalize(action_pred[:, :, 2:].clone(), dim=-1)
        return dist_pred, action_pred


class NoGoalGNM(GNM):
    """GNM variant for no-goal exploration."""

    def __init__(
        self,
        context_size: int = 5,
        len_traj_pred: Optional[int] = 5,
        learn_angle: Optional[bool] = True,
        obs_encoding_size: Optional[int] = 1024,
        goal_encoding_size: Optional[int] = 1024,
    ) -> None:
        super().__init__(
            context_size=context_size,
            len_traj_pred=len_traj_pred,
            learn_angle=learn_angle,
            obs_encoding_size=obs_encoding_size,
            goal_encoding_size=goal_encoding_size,
        )
        self.linear_layers = nn.Sequential(
            nn.Linear(self.obs_encoding_size, 256),
            nn.ReLU(),
            nn.Linear(256, 32),
            nn.ReLU(),
        )

    def forward(self, obs_img: torch.Tensor) -> torch.Tensor:
        obs_encoding = self.obs_mobilenet(obs_img)
        obs_encoding = self.flatten(obs_encoding)
        obs_encoding = self.compress_observation(obs_encoding)

        embedding = self.linear_layers(obs_encoding)
        action_pred = self.action_predictor(embedding)

        action_pred = action_pred.reshape((action_pred.shape[0], self.len_trajectory_pred, self.num_action_params))
        action_pred[:, :, :2] = torch.cumsum(action_pred[:, :, :2], dim=1)
        if self.learn_angle:
            action_pred[:, :, 2:] = F.normalize(action_pred[:, :, 2:].clone(), dim=-1)
        return action_pred


class GNMPolicy(nn.Module):
    """GNM policy wrapper for inference."""

    def __init__(self, config: dict):
        super().__init__()
        self.config = config
        self.model = GNM(
            context_size=config["context_size"],
            len_traj_pred=config["len_traj_pred"],
            learn_angle=config["learn_angle"],
            obs_encoding_size=config["obs_encoding_size"],
            goal_encoding_size=config["goal_encoding_size"],
        )

    def predict_imagegoal_distance_and_action(
        self, image: torch.Tensor, goal_image: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.model(image, goal_image)

    def predict_nogoal_distance_and_action(
        self, image: torch.Tensor, fake_goal_image: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.model(image, fake_goal_image)


class NoGoalGNMPolicy(nn.Module):
    """No-goal GNM policy wrapper."""

    def __init__(self, config: dict):
        super().__init__()
        self.config = config
        self.model = NoGoalGNM(
            context_size=config["context_size"],
            len_traj_pred=config["len_traj_pred"],
            learn_angle=config["learn_angle"],
            obs_encoding_size=config["obs_encoding_size"],
            goal_encoding_size=config["goal_encoding_size"],
        )

    def predict_nogoal_distance_and_action(self, image: torch.Tensor) -> torch.Tensor:
        return self.model(image)
