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
# skrl Shared Model Class
# =============================================================================


class MixInputSharedModel(GaussianMixin, DeterministicMixin, Model):
    """Shared model for mixed input (CNN + MLP) observations.

    This model implements a shared backbone architecture matching rl_games_network.py:
    - Shared CNN branch for processing visual observations (RGB-D images)
    - Shared MLP branch for processing vector observations (goal commands, robot state)
    - Shared fusion MLP that combines both branches
    - Separate policy head (mu, sigma) for action distribution
    - Separate value head for state value estimation

    The CNN and MLP backbone are shared between policy and value, reducing parameters
    and enabling efficient single forward-pass computation.
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
        fixed_sigma: bool = True,
        mu_activation: str = "None",
        sigma_activation: str = "None",
        value_activation: str = "None",
    ):
        # Initialize base Model class first (required by skrl)
        Model.__init__(self, observation_space, action_space, device)

        # Initialize both mixins with their respective roles
        GaussianMixin.__init__(
            self,
            clip_actions=clip_actions,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            role="policy",
        )
        DeterministicMixin.__init__(self, clip_actions=False, role="value")

        # Store configuration
        self.vector_obs_size = vector_obs_size
        self.cnn_input_shape = cnn_input_shape
        self.conv_depths = conv_depths if conv_depths is not None else DEFAULT_CONV_DEPTHS
        self.mlp_units = mlp_units if mlp_units is not None else DEFAULT_MLP_UNITS
        self.image_flat_size = cnn_input_shape[0] * cnn_input_shape[1] * cnn_input_shape[2]
        self.fixed_sigma = fixed_sigma

        # =================================================================
        # SHARED BACKBONE (matching rl_games_network.py shared architecture)
        # =================================================================

        # Shared CNN branch for visual observations
        self.cnn = build_impala_cnn(cnn_input_shape[0], self.conv_depths)
        cnn_out_size = calc_cnn_output_size(self.cnn, cnn_input_shape)

        # Shared pre-MLP for vector observations
        self.pre_mlp = _build_mlp(vector_obs_size, self.mlp_units)
        pre_mlp_out_size = self.mlp_units[-1] if self.mlp_units else vector_obs_size

        # Shared main MLP for fused features
        fused_size = cnn_out_size + pre_mlp_out_size
        self.main_mlp = _build_mlp(fused_size, self.mlp_units)
        main_mlp_out_size = self.mlp_units[-1] if self.mlp_units else fused_size

        # Shared activation for CNN output (matching rl_games)
        self.flatten_act = nn.ReLU()

        # =================================================================
        # POLICY HEAD (mu and sigma for Gaussian policy)
        # =================================================================

        self.mu = nn.Linear(main_mlp_out_size, self.num_actions)
        self.mu_act = self._get_activation(mu_activation)

        self.sigma_act = self._get_activation(sigma_activation)
        if fixed_sigma:
            # Learnable parameter (not state-dependent), matching rl_games
            self.sigma = nn.Parameter(
                torch.zeros(self.num_actions, requires_grad=True, dtype=torch.float32),
                requires_grad=True,
            )
        else:
            # State-dependent sigma
            self.sigma = nn.Linear(main_mlp_out_size, self.num_actions)

        # =================================================================
        # VALUE HEAD (scalar value estimate)
        # =================================================================

        self.value = nn.Linear(main_mlp_out_size, 1)
        self.value_act = self._get_activation(value_activation)

        # Cache for shared output (enables single forward-pass optimization)
        self._shared_output = None

        # Initialize weights (matching rl_games initialization)
        self._init_weights(initial_log_std)

    def _get_activation(self, name: str) -> nn.Module:
        """Get activation function by name (matching rl_games activations_factory)."""
        activations = {
            "None": nn.Identity(),
            "none": nn.Identity(),
            "relu": nn.ReLU(),
            "tanh": nn.Tanh(),
            "sigmoid": nn.Sigmoid(),
            "elu": nn.ELU(),
            "selu": nn.SELU(),
            "swish": nn.SiLU(),
            "gelu": nn.GELU(),
            "softplus": nn.Softplus(),
        }
        return activations.get(name, nn.Identity())

    def _init_weights(self, initial_log_std: float = 0.0):
        """Initialize network weights matching rl_games initialization.

        rl_games uses "default" initializer which means no explicit initialization,
        keeping PyTorch's default (Kaiming uniform for nn.Linear).

        - CNN layers: Kaiming normal initialization (explicitly set in rl_games)
        - MLP layers: Default (PyTorch Kaiming uniform) - no explicit init
        - mu layer: Default (PyTorch Kaiming uniform) - no explicit init
        - sigma: Constant initialization with initial_log_std (const_initializer with val: 0)
        - value layer: Default (PyTorch Kaiming uniform) - no explicit init
        """
        # Initialize CNN layers with Kaiming normal
        for module in self.modules():
            if isinstance(module, nn.Conv2d):
                nn.init.kaiming_normal_(module.weight, mode="fan_out", nonlinearity="relu")
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

        # Initialize sigma (const_initializer with val: 0)
        if self.fixed_sigma:
            nn.init.constant_(self.sigma, initial_log_std)
        else:
            nn.init.constant_(self.sigma.weight, initial_log_std)
            if self.sigma.bias is not None:
                nn.init.zeros_(self.sigma.bias)

    def _forward_shared(self, states: torch.Tensor) -> torch.Tensor:
        """Forward pass through shared backbone (CNN + pre_mlp + main_mlp).

        Args:
            states: Concatenated observations [vector_obs, flattened_image]

        Returns:
            Features from shared backbone
        """
        # Split observations: [vector_obs, flattened_image]
        vector_obs = states[:, : self.vector_obs_size]
        image_flat = states[:, self.vector_obs_size :]

        # Reshape image to [batch, channels, height, width]
        image = image_flat.view(-1, *self.cnn_input_shape)

        # Process through shared CNN branch
        cnn_out = self.cnn(image)
        cnn_out = cnn_out.flatten(1)
        cnn_out = self.flatten_act(cnn_out)

        # Process through shared pre-MLP branch
        mlp_out = self.pre_mlp(vector_obs)

        # Fuse features
        fused = torch.cat([cnn_out, mlp_out], dim=1)

        # Process through shared main MLP
        features = self.main_mlp(fused)

        return features

    def act(self, inputs, role):
        """Override act to disambiguate between policy and value roles.

        This is required for shared models in skrl.
        """
        if role == "policy":
            return GaussianMixin.act(self, inputs, role)
        elif role == "value":
            return DeterministicMixin.act(self, inputs, role)
        else:
            raise ValueError(f"Unknown role: {role}")

    def compute(self, inputs, role):
        """Compute output based on role (policy or value).

        For efficiency, the policy pass caches the shared backbone output,
        which is reused by the subsequent value pass (single forward-pass).

        Args:
            inputs: Dictionary containing "states" with concatenated observations
            role: Role of the model ("policy" or "value")

        Returns:
            For policy: Tuple of (action_mean, log_std, {})
            For value: Tuple of (value, {})
        """
        if role == "policy":
            # Compute and cache shared backbone output
            self._shared_output = self._forward_shared(inputs["states"])

            # Compute mu (action mean) with activation
            mu = self.mu_act(self.mu(self._shared_output))

            # Compute sigma with activation
            if self.fixed_sigma:
                sigma = self.sigma_act(self.sigma)
            else:
                sigma = self.sigma_act(self.sigma(self._shared_output))

            return mu, sigma, {}

        elif role == "value":
            # Use cached shared output if available (single forward-pass optimization)
            if self._shared_output is not None:
                features = self._shared_output
                self._shared_output = None  # Reset to prevent stale data
            else:
                # Fallback: compute shared backbone (needed for standalone value calls)
                features = self._forward_shared(inputs["states"])

            # Compute value with activation
            value = self.value_act(self.value(features))

            return value, {}

        else:
            raise ValueError(f"Unknown role: {role}")


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
    fixed_sigma: bool = True,
    mu_activation: str = "None",
    sigma_activation: str = "None",
    value_activation: str = "None",
):
    """Create shared policy and value model for mixed input observations.

    This function creates a single shared model (MixInputSharedModel) that is
    used for both policy and value, matching the rl_games_network.py architecture
    where CNN and MLP backbone are shared between policy and value heads.

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
        fixed_sigma: Whether sigma is fixed (state-independent) or learned per-state
        mu_activation: Activation function for mu (action mean) output
        sigma_activation: Activation function for sigma output
        value_activation: Activation function for value output

    Returns:
        Dictionary with "policy" and "value" pointing to the same shared model
    """
    # Create single shared model for both policy and value
    shared_model = MixInputSharedModel(
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
        fixed_sigma=fixed_sigma,
        mu_activation=mu_activation,
        sigma_activation=sigma_activation,
        value_activation=value_activation,
    )

    # Return same model instance for both keys (shared model pattern in skrl)
    return {"policy": shared_model, "value": shared_model}
