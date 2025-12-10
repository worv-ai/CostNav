# Copyright (c) 2025, COCO Navigation Project.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom skrl network for mixed input (CNN + MLP) architectures.

This module provides custom skrl model classes that combine:
- CNN branch for processing visual observations (RGB-D camera images)
- MLP branch for processing vector observations (goal commands, robot state)
- Fusion layer that combines both branches for policy and value networks

Based on the IMPALA architecture with residual blocks, analogous to rl_games_network.py.
"""

import torch
import torch.nn as nn
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model

# =============================================================================
# CNN Building Blocks (matching rl_games_network.py)
# =============================================================================


class Conv2dAuto(nn.Conv2d):
    """Conv2d layer with automatic padding based on kernel size."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.padding = (self.kernel_size[0] // 2, self.kernel_size[1] // 2)


class ConvBlock(nn.Module):
    """Convolutional block with optional batch normalization."""

    def __init__(self, in_channels: int, out_channels: int, use_bn: bool = False):
        super().__init__()
        self.use_bn = use_bn
        self.conv = Conv2dAuto(
            in_channels=in_channels,
            out_channels=out_channels,
            kernel_size=3,
            stride=1,
            bias=not use_bn,
        )
        if use_bn:
            self.bn = nn.BatchNorm2d(out_channels)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv(x)
        if self.use_bn:
            x = self.bn(x)
        return x


class ResidualBlock(nn.Module):
    """Residual block with two conv layers and skip connection."""

    def __init__(self, channels: int, use_bn: bool = False, use_zero_init: bool = False):
        super().__init__()
        self.use_zero_init = use_zero_init
        if use_zero_init:
            self.alpha = nn.Parameter(torch.zeros(1))
        self.conv1 = ConvBlock(channels, channels, use_bn)
        self.conv2 = ConvBlock(channels, channels, use_bn)
        self.activate1 = nn.ReLU()
        self.activate2 = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        residual = x
        x = self.activate1(x)
        x = self.conv1(x)
        x = self.activate2(x)
        x = self.conv2(x)
        if self.use_zero_init:
            x = x * self.alpha + residual
        else:
            x = x + residual
        return x


class ImpalaSequential(nn.Module):
    """IMPALA-style convolutional block with max pooling and residual connections."""

    def __init__(self, in_channels: int, out_channels: int, use_bn: bool = False, use_zero_init: bool = False):
        super().__init__()
        self.conv = ConvBlock(in_channels, out_channels, use_bn)
        self.max_pool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.res_block1 = ResidualBlock(out_channels, use_bn=use_bn, use_zero_init=use_zero_init)
        self.res_block2 = ResidualBlock(out_channels, use_bn=use_bn, use_zero_init=use_zero_init)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv(x)
        x = self.max_pool(x)
        x = self.res_block1(x)
        x = self.res_block2(x)
        return x


def build_impala_cnn(input_channels: int, depths: list) -> nn.Sequential:
    """Build IMPALA-style CNN with residual blocks."""
    layers = []
    in_channels = input_channels
    for d in depths:
        layers.append(ImpalaSequential(in_channels, d))
        in_channels = d
    return nn.Sequential(*layers)


def calc_cnn_output_size(cnn: nn.Module, input_shape: tuple) -> int:
    """Calculate the flattened output size of a CNN given input shape."""
    with torch.no_grad():
        dummy_input = torch.zeros(1, *input_shape)
        output = cnn(dummy_input)
        return output.flatten(1).size(1)


# =============================================================================
# Network Configuration
# =============================================================================

# Default configuration matching rl_games_network.py
DEFAULT_VECTOR_OBS_SIZE = 8  # pose_command (2) + base_lin_vel (3) + base_ang_vel (3)
DEFAULT_CNN_INPUT_SHAPE = (4, 135, 240)  # [channels, height, width] for RGB-D
DEFAULT_CONV_DEPTHS = [16, 32, 32]  # IMPALA channel depths
DEFAULT_MLP_UNITS = [256, 256, 256]  # MLP hidden layer sizes


def _build_mlp(input_size: int, units: list, activation: type = nn.ELU) -> nn.Sequential:
    """Build an MLP with the given layer sizes and activation."""
    layers = []
    in_size = input_size
    for out_size in units:
        layers.append(nn.Linear(in_size, out_size))
        layers.append(activation())
        in_size = out_size
    return nn.Sequential(*layers)


# =============================================================================
# skrl Model Classes
# =============================================================================


class MixInputPolicy(GaussianMixin, Model):
    """Policy network for mixed input (CNN + MLP) observations.

    This model processes:
    - Vector observations (goal commands, robot state) through an MLP
    - Visual observations (RGB-D images) through an IMPALA CNN
    - Fuses both branches and outputs action mean and log_std for Gaussian policy
    """

    def __init__(
        self,
        observation_space,
        action_space,
        device,
        clip_actions: bool = False,
        clip_log_std: bool = True,
        min_log_std: float = -20.0,
        max_log_std: float = 2.0,
        initial_log_std: float = 0.0,
        vector_obs_size: int = DEFAULT_VECTOR_OBS_SIZE,
        cnn_input_shape: tuple = DEFAULT_CNN_INPUT_SHAPE,
        conv_depths: list = None,
        mlp_units: list = None,
    ):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(
            self,
            clip_actions=clip_actions,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            role="policy",
        )

        self.vector_obs_size = vector_obs_size
        self.cnn_input_shape = cnn_input_shape
        self.conv_depths = conv_depths if conv_depths is not None else DEFAULT_CONV_DEPTHS
        self.mlp_units = mlp_units if mlp_units is not None else DEFAULT_MLP_UNITS
        self.image_flat_size = cnn_input_shape[0] * cnn_input_shape[1] * cnn_input_shape[2]

        # Build CNN branch for visual observations
        self.cnn = build_impala_cnn(cnn_input_shape[0], self.conv_depths)
        cnn_out_size = calc_cnn_output_size(self.cnn, cnn_input_shape)

        # Build pre-MLP for vector observations
        self.pre_mlp = _build_mlp(vector_obs_size, self.mlp_units)
        pre_mlp_out_size = self.mlp_units[-1] if self.mlp_units else vector_obs_size

        # Build main MLP for fused features
        fused_size = cnn_out_size + pre_mlp_out_size
        self.main_mlp = _build_mlp(fused_size, self.mlp_units)
        main_mlp_out_size = self.mlp_units[-1] if self.mlp_units else fused_size

        # Activation for CNN output
        self.flatten_act = nn.ReLU()

        # Policy head: outputs action mean
        self.action_head = nn.Linear(main_mlp_out_size, self.num_actions)

        # Learnable log_std parameter
        self.log_std_parameter = nn.Parameter(torch.full((self.num_actions,), initial_log_std))

        # Initialize weights
        self._init_weights()

    def _init_weights(self):
        """Initialize network weights using orthogonal initialization."""
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=1.0)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)
            elif isinstance(module, nn.Conv2d):
                nn.init.orthogonal_(module.weight, gain=1.0)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def compute(self, inputs, role):
        """Compute action mean and log_std from observations.

        Args:
            inputs: Dictionary containing "states" with concatenated observations
            role: Role of the model (e.g., "policy")

        Returns:
            Tuple of (action_mean, log_std_parameter, {})
        """
        states = inputs["states"]

        # Split observations: [vector_obs, flattened_image]
        vector_obs = states[:, : self.vector_obs_size]
        image_flat = states[:, self.vector_obs_size :]

        # Reshape image to [batch, channels, height, width]
        image = image_flat.view(-1, *self.cnn_input_shape)

        # Process through CNN branch
        cnn_out = self.cnn(image)
        cnn_out = cnn_out.flatten(1)
        cnn_out = self.flatten_act(cnn_out)

        # Process through pre-MLP branch
        mlp_out = self.pre_mlp(vector_obs)

        # Fuse features
        fused = torch.cat([cnn_out, mlp_out], dim=1)

        # Process through main MLP
        features = self.main_mlp(fused)

        # Output action mean
        action_mean = self.action_head(features)

        return action_mean, self.log_std_parameter, {}


class MixInputValue(DeterministicMixin, Model):
    """Value network for mixed input (CNN + MLP) observations.

    This model processes:
    - Vector observations (goal commands, robot state) through an MLP
    - Visual observations (RGB-D images) through an IMPALA CNN
    - Fuses both branches and outputs a scalar value estimate
    """

    def __init__(
        self,
        observation_space,
        action_space,
        device,
        clip_actions: bool = False,
        vector_obs_size: int = DEFAULT_VECTOR_OBS_SIZE,
        cnn_input_shape: tuple = DEFAULT_CNN_INPUT_SHAPE,
        conv_depths: list = None,
        mlp_units: list = None,
    ):
        Model.__init__(self, observation_space, action_space, device)
        DeterministicMixin.__init__(self, clip_actions=clip_actions, role="value")

        self.vector_obs_size = vector_obs_size
        self.cnn_input_shape = cnn_input_shape
        self.conv_depths = conv_depths if conv_depths is not None else DEFAULT_CONV_DEPTHS
        self.mlp_units = mlp_units if mlp_units is not None else DEFAULT_MLP_UNITS
        self.image_flat_size = cnn_input_shape[0] * cnn_input_shape[1] * cnn_input_shape[2]

        # Build CNN branch for visual observations
        self.cnn = build_impala_cnn(cnn_input_shape[0], self.conv_depths)
        cnn_out_size = calc_cnn_output_size(self.cnn, cnn_input_shape)

        # Build pre-MLP for vector observations
        self.pre_mlp = _build_mlp(vector_obs_size, self.mlp_units)
        pre_mlp_out_size = self.mlp_units[-1] if self.mlp_units else vector_obs_size

        # Build main MLP for fused features
        fused_size = cnn_out_size + pre_mlp_out_size
        self.main_mlp = _build_mlp(fused_size, self.mlp_units)
        main_mlp_out_size = self.mlp_units[-1] if self.mlp_units else fused_size

        # Activation for CNN output
        self.flatten_act = nn.ReLU()

        # Value head: outputs scalar value
        self.value_head = nn.Linear(main_mlp_out_size, 1)

        # Initialize weights
        self._init_weights()

    def _init_weights(self):
        """Initialize network weights using orthogonal initialization."""
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=1.0)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)
            elif isinstance(module, nn.Conv2d):
                nn.init.orthogonal_(module.weight, gain=1.0)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def compute(self, inputs, role):
        """Compute value estimate from observations.

        Args:
            inputs: Dictionary containing "states" with concatenated observations
            role: Role of the model (e.g., "value")

        Returns:
            Tuple of (value, {})
        """
        states = inputs["states"]

        # Split observations: [vector_obs, flattened_image]
        vector_obs = states[:, : self.vector_obs_size]
        image_flat = states[:, self.vector_obs_size :]

        # Reshape image to [batch, channels, height, width]
        image = image_flat.view(-1, *self.cnn_input_shape)

        # Process through CNN branch
        cnn_out = self.cnn(image)
        cnn_out = cnn_out.flatten(1)
        cnn_out = self.flatten_act(cnn_out)

        # Process through pre-MLP branch
        mlp_out = self.pre_mlp(vector_obs)

        # Fuse features
        fused = torch.cat([cnn_out, mlp_out], dim=1)

        # Process through main MLP
        features = self.main_mlp(fused)

        # Output value
        value = self.value_head(features)

        return value, {}


# =============================================================================
# Factory function for creating models
# =============================================================================


def get_mix_input_models(
    observation_space,
    action_space,
    device,
    vector_obs_size: int = DEFAULT_VECTOR_OBS_SIZE,
    cnn_input_shape: tuple = DEFAULT_CNN_INPUT_SHAPE,
    conv_depths: list = None,
    mlp_units: list = None,
    clip_actions: bool = False,
    clip_log_std: bool = True,
    min_log_std: float = -20.0,
    max_log_std: float = 2.0,
    initial_log_std: float = 0.0,
):
    """Create policy and value models for mixed input observations.

    Args:
        observation_space: Observation space from the environment
        action_space: Action space from the environment
        device: Device to place the models on
        vector_obs_size: Size of vector observations
        cnn_input_shape: Shape of CNN input (channels, height, width)
        conv_depths: List of channel depths for IMPALA CNN
        mlp_units: List of hidden layer sizes for MLPs
        clip_actions: Whether to clip actions to action space
        clip_log_std: Whether to clip log_std
        min_log_std: Minimum log_std value
        max_log_std: Maximum log_std value
        initial_log_std: Initial log_std value

    Returns:
        Dictionary with "policy" and "value" models
    """
    policy = MixInputPolicy(
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        clip_actions=clip_actions,
        clip_log_std=clip_log_std,
        min_log_std=min_log_std,
        max_log_std=max_log_std,
        initial_log_std=initial_log_std,
        vector_obs_size=vector_obs_size,
        cnn_input_shape=cnn_input_shape,
        conv_depths=conv_depths,
        mlp_units=mlp_units,
    )

    value = MixInputValue(
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        clip_actions=False,
        vector_obs_size=vector_obs_size,
        cnn_input_shape=cnn_input_shape,
        conv_depths=conv_depths,
        mlp_units=mlp_units,
    )

    return {"policy": policy, "value": value}
