# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom observation functions for the costnav_isaaclab_v2_NavRL environment."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import Camera, RayCasterCamera, TiledCamera
from isaaclab.utils import math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


def advanced_generated_commands(
    env: ManagerBasedRLEnv, command_name: str, max_dim: int, normalize: bool
) -> torch.Tensor:
    """The generated command from command term in the command manager with the given name.

    This function retrieves commands and optionally normalizes them to a maximum distance.

    Args:
        env: The environment instance.
        command_name: Name of the command term to retrieve.
        max_dim: Maximum dimension to slice from the command tensor.
        normalize: Whether to normalize commands that exceed max distance.

    Returns:
        Command tensor of shape (num_envs, max_dim).
    """
    if not normalize:
        return env.command_manager.get_command(command_name)[..., :max_dim]
    else:
        max_ = 5.0
        command = env.command_manager.get_command(command_name)[..., :max_dim]
        dis = torch.norm(command, dim=-1, keepdim=False)
        mask = dis > max_
        if mask.sum() > 0:
            scale = max_ / dis[mask]
            command[mask] = scale.reshape(-1, 1) * command[mask]
        return command


def rgbd_processed(
    env: ManagerBasedEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("tiled_camera"),
    convert_perspective_to_orthogonal: bool = False,
    normalize: bool = True,
    flatten: bool = False,
) -> torch.Tensor:
    """Process RGB-D images from the camera sensor.

    This function retrieves RGB and depth images from the camera and combines them into
    a single tensor. The RGB images are normalized to [0, 1] and the depth images are
    clamped to [0, 20] meters and normalized.

    Args:
        env: The environment the cameras are placed within.
        sensor_cfg: The desired sensor to read from. Defaults to SceneEntityCfg("tiled_camera").
        convert_perspective_to_orthogonal: Whether to orthogonalize perspective depth images.
            This is used only when the data type is "distance_to_camera". Defaults to False.
        normalize: Whether to normalize the images. Defaults to True.
        flatten: Whether to flatten the image to 1D vector. Defaults to False.
            If True, returns shape (num_envs, 4*height*width).
            If False, returns shape (num_envs, 4, height, width).

    Returns:
        Combined RGB-D tensor. Shape depends on flatten parameter:
        - If flatten=False: (num_envs, 4, height, width) where channels are [R, G, B, D]
        - If flatten=True: (num_envs, 4*height*width) flattened vector
    """
    # extract the used quantities (to enable type-hinting)
    sensor: TiledCamera | Camera | RayCasterCamera = env.scene.sensors[sensor_cfg.name]

    # obtain the input image
    images = sensor.data.output["rgb"]
    depth = sensor.data.output["distance_to_camera"]

    # depth image conversion
    if convert_perspective_to_orthogonal:
        depth = math_utils.orthogonalize_perspective_depth(depth, sensor.data.intrinsic_matrices)

    # rgb/depth image normalization
    if normalize:
        images = images.float() / 255.0
        images = images.permute(0, 3, 1, 2)
        B, C, H, W = images.shape
        depth[depth == float("inf")] = 0
        depth = depth.reshape(B, 1, H, W).clamp(0.0, 20.0) / 20.0

    rgbd = torch.cat([images, depth], dim=1).clone()

    # Flatten if requested (for concatenation with vector observations)
    if flatten:
        rgbd = rgbd.reshape(rgbd.shape[0], -1)

    return rgbd
