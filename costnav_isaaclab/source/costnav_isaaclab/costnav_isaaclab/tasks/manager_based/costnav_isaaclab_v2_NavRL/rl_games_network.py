# Copyright (c) 2025, COCO Navigation Project.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom network builder for mixed input (CNN + MLP) architectures for RL-Games.

This module provides a custom network architecture that combines:
- CNN branch for processing visual observations (RGB-D camera images)
- MLP branch for processing vector observations (goal commands, robot state)
- Fusion layer that combines both branches for policy and value networks

Based on the IMPALA architecture with residual blocks.
"""

import torch
import torch.nn as nn
from rl_games.algos_torch import torch_ext
from rl_games.algos_torch.d2rl import D2RLNet
from rl_games.common import object_factory
from rl_games.common.layers.recurrent import GRUWithDones, LSTMWithDones
from rl_games.common.layers.value import DefaultValue


def _create_initializer(func, **kwargs):
    """Helper to create weight initializers."""
    return lambda v: func(v, **kwargs)


class Conv2dAuto(nn.Conv2d):
    """Conv2d layer with automatic padding based on kernel size."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.padding = (self.kernel_size[0] // 2, self.kernel_size[1] // 2)


class ConvBlock(nn.Module):
    """Convolutional block with optional batch normalization."""

    def __init__(self, in_channels, out_channels, use_bn=False):
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

    def forward(self, x):
        x = self.conv(x)
        if self.use_bn:
            x = self.bn(x)
        return x


class ResidualBlock(nn.Module):
    """Residual block with two conv layers and skip connection."""

    def __init__(self, channels, activation="relu", use_bn=False, use_zero_init=False):
        super().__init__()
        self.use_zero_init = use_zero_init
        if use_zero_init:
            self.alpha = nn.Parameter(torch.zeros(1))
        self.activation = activation
        self.conv1 = ConvBlock(channels, channels, use_bn)
        self.conv2 = ConvBlock(channels, channels, use_bn)
        self.activate1 = nn.ReLU()
        self.activate2 = nn.ReLU()

    def forward(self, x):
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

    def __init__(self, in_channels, out_channels, activation="relu", use_bn=False, use_zero_init=False):
        super().__init__()
        self.conv = ConvBlock(in_channels, out_channels, use_bn)
        self.max_pool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.res_block1 = ResidualBlock(
            out_channels, activation=activation, use_bn=use_bn, use_zero_init=use_zero_init
        )
        self.res_block2 = ResidualBlock(
            out_channels, activation=activation, use_bn=use_bn, use_zero_init=use_zero_init
        )

    def forward(self, x):
        x = self.conv(x)
        x = self.max_pool(x)
        x = self.res_block1(x)
        x = self.res_block2(x)
        return x


class NetworkBuilder:
    """Base network builder class."""

    def __init__(self, **kwargs):
        pass

    def load(self, params):
        pass

    def build(self, name, **kwargs):
        pass

    def __call__(self, name, **kwargs):
        return self.build(name, **kwargs)

    class BaseNetwork(nn.Module):
        """Base network with common utilities for building networks."""

        def __init__(self, **kwargs):
            nn.Module.__init__(self, **kwargs)

            # Activation functions factory
            self.activations_factory = object_factory.ObjectFactory()
            self.activations_factory.register_builder("relu", lambda **kwargs: nn.ReLU(**kwargs))
            self.activations_factory.register_builder("tanh", lambda **kwargs: nn.Tanh(**kwargs))
            self.activations_factory.register_builder("sigmoid", lambda **kwargs: nn.Sigmoid(**kwargs))
            self.activations_factory.register_builder("elu", lambda **kwargs: nn.ELU(**kwargs))
            self.activations_factory.register_builder("selu", lambda **kwargs: nn.SELU(**kwargs))
            self.activations_factory.register_builder("swish", lambda **kwargs: nn.SiLU(**kwargs))
            self.activations_factory.register_builder("gelu", lambda **kwargs: nn.GELU(**kwargs))
            self.activations_factory.register_builder("softplus", lambda **kwargs: nn.Softplus(**kwargs))
            self.activations_factory.register_builder("None", lambda **kwargs: nn.Identity())

            # Weight initializers factory
            self.init_factory = object_factory.ObjectFactory()
            self.init_factory.register_builder(
                "const_initializer",
                lambda **kwargs: _create_initializer(nn.init.constant_, **kwargs),
            )
            self.init_factory.register_builder(
                "orthogonal_initializer",
                lambda **kwargs: _create_initializer(nn.init.orthogonal_, **kwargs),
            )
            self.init_factory.register_builder(
                "glorot_normal_initializer",
                lambda **kwargs: _create_initializer(nn.init.xavier_normal_, **kwargs),
            )
            self.init_factory.register_builder(
                "glorot_uniform_initializer",
                lambda **kwargs: _create_initializer(nn.init.xavier_uniform_, **kwargs),
            )
            self.init_factory.register_builder(
                "variance_scaling_initializer",
                lambda **kwargs: _create_initializer(torch_ext.variance_scaling_initializer, **kwargs),
            )
            self.init_factory.register_builder(
                "random_uniform_initializer",
                lambda **kwargs: _create_initializer(nn.init.uniform_, **kwargs),
            )
            self.init_factory.register_builder(
                "kaiming_normal",
                lambda **kwargs: _create_initializer(nn.init.kaiming_normal_, **kwargs),
            )
            self.init_factory.register_builder(
                "orthogonal", lambda **kwargs: _create_initializer(nn.init.orthogonal_, **kwargs)
            )
            self.init_factory.register_builder("default", lambda **kwargs: nn.Identity())

        def is_separate_critic(self):
            return False

        def get_value_layer(self):
            return self.value

        def is_rnn(self):
            return False

        def get_default_rnn_state(self):
            return None

        def _build_rnn(self, name, input, units, layers):
            """Build RNN layer."""
            if name == "identity":
                return torch_ext.IdentityRNN(input, units)
            if name == "lstm":
                return LSTMWithDones(input_size=input, hidden_size=units, num_layers=layers)
            if name == "gru":
                return GRUWithDones(input_size=input, hidden_size=units, num_layers=layers)

        def _build_sequential_mlp(
            self,
            input_size,
            units,
            activation,
            dense_func,
            norm_only_first_layer=False,
            norm_func_name=None,
        ):
            """Build sequential MLP."""
            in_size = input_size
            layers = []
            need_norm = True
            for unit in units:
                layers.append(dense_func(in_size, unit))
                layers.append(self.activations_factory.create(activation))

                if not need_norm:
                    continue
                if norm_only_first_layer and norm_func_name is not None:
                    need_norm = False
                if norm_func_name == "layer_norm":
                    layers.append(torch.nn.LayerNorm(unit))
                elif norm_func_name == "batch_norm":
                    layers.append(torch.nn.BatchNorm1d(unit))
                in_size = unit

            return nn.Sequential(*layers)

        def _build_mlp(
            self,
            input_size,
            units,
            activation,
            dense_func,
            norm_only_first_layer=False,
            norm_func_name=None,
            d2rl=False,
        ):
            """Build MLP with optional D2RL architecture."""
            if d2rl:
                act_layers = [self.activations_factory.create(activation) for i in range(len(units))]
                return D2RLNet(input_size, units, act_layers, norm_func_name)
            else:
                return self._build_sequential_mlp(input_size, units, activation, dense_func, norm_func_name=None)

        def _build_value_layer(self, input_size, output_size, value_type="legacy"):
            """Build value layer."""
            if value_type == "legacy":
                return torch.nn.Linear(input_size, output_size)
            if value_type == "default":
                return DefaultValue(input_size, output_size)
            raise ValueError('value type is not "default" or "legacy"')


class MixInputNetworkBuilder(NetworkBuilder):
    """Network builder for mixed input architectures (CNN + MLP)."""

    def __init__(self, **kwargs):
        NetworkBuilder.__init__(self)

    def load(self, params):
        self.params = params

    class Network(NetworkBuilder.BaseNetwork):
        """Mixed input network combining CNN and MLP branches."""

        def _calc_input_size(self, input_shape, cnn_layers=None, extra_shape=None):
            """Calculate the flattened size after CNN layers."""
            if cnn_layers is None:
                assert len(input_shape) == 1
                return input_shape[0] + extra_shape[0] if extra_shape is not None else input_shape[0]
            else:
                cnn_out = nn.Sequential(*cnn_layers)(torch.rand(1, *(input_shape))).flatten(1).data.size(1)
                return cnn_out + extra_shape[0] if extra_shape is not None else cnn_out

        def __init__(self, params, **kwargs):
            """Initialize the mixed input network."""
            self.actions_num = actions_num = kwargs.pop("actions_num")
            input_shape = kwargs.pop("input_shape")
            if type(input_shape) is dict:
                input_shape = input_shape["observation"]
            self.num_seqs = num_seqs = kwargs.pop("num_seqs", 1)
            self.value_size = kwargs.pop("value_size", 1)

            NetworkBuilder.BaseNetwork.__init__(self)
            self.load(params)

            # Calculate sizes for splitting concatenated observations
            # Total obs: [pose_command (2), base_lin_vel (3), base_ang_vel (3), rgb_flattened (4*135*240)]
            self.vector_obs_size = 8  # pose_command (2) + base_lin_vel (3) + base_ang_vel (3)
            cnn_input_shape = self.cnn_input_shape  # [4, 135, 240]
            self.image_flat_size = cnn_input_shape[0] * cnn_input_shape[1] * cnn_input_shape[2]  # 72900

            # Build CNN branch for visual observations
            self.cnn = self._build_impala(cnn_input_shape, self.conv_depths) if len(cnn_input_shape) == 3 else None

            # Build pre-MLP for vector observations (goal commands, velocities, etc.)
            # This processes ONLY the vector observations (pose_command + velocities), not the full concatenated obs
            pre_mlp_args = {
                "input_size": self.vector_obs_size,  # Only vector obs size (8: 2 + 3 + 3)
                "units": self.units,
                "activation": self.activation,
                "norm_func_name": self.normalization,
                "dense_func": torch.nn.Linear,
            }
            self.pre_mlp = self._build_mlp(**pre_mlp_args)

            # Calculate combined input size after CNN and pre-MLP
            # CNN output size + pre-MLP output size (last layer of units)
            cnn_out_size = self._calc_input_size(cnn_input_shape, self.cnn, extra_shape=None)
            mlp_out_size = self.units[-1] if len(self.units) > 0 else self.vector_obs_size
            mlp_input_shape = cnn_out_size + mlp_out_size
            in_mlp_shape = mlp_input_shape

            if len(self.units) == 0:
                out_size = mlp_input_shape
            else:
                out_size = self.units[-1]

            # Build RNN if specified
            if self.has_rnn:
                if not self.is_rnn_before_mlp:
                    rnn_in_size = out_size
                    out_size = self.rnn_units
                else:
                    rnn_in_size = in_mlp_shape
                    in_mlp_shape = self.rnn_units
                if self.require_rewards:
                    rnn_in_size += 1
                if self.require_last_actions:
                    rnn_in_size += actions_num
                self.rnn = self._build_rnn(self.rnn_name, rnn_in_size, self.rnn_units, self.rnn_layers)

            # Build main MLP for fused features
            mlp_args = {
                "input_size": in_mlp_shape,
                "units": self.units,
                "activation": self.activation,
                "norm_func_name": self.normalization,
                "dense_func": torch.nn.Linear,
            }
            self.mlp = self._build_mlp(**mlp_args)

            # Build value head
            self.value = self._build_value_layer(out_size, self.value_size)
            self.value_act = self.activations_factory.create(self.value_activation)
            self.flatten_act = self.activations_factory.create(self.activation)

            # Build policy head (continuous actions)
            if self.is_continuous:
                self.mu = torch.nn.Linear(out_size, actions_num)
                self.mu_act = self.activations_factory.create(self.space_config["mu_activation"])
                mu_init = self.init_factory.create(**self.space_config["mu_init"])
                self.sigma_act = self.activations_factory.create(self.space_config["sigma_activation"])
                sigma_init = self.init_factory.create(**self.space_config["sigma_init"])

                if self.fixed_sigma:
                    self.sigma = nn.Parameter(
                        torch.zeros(actions_num, requires_grad=True, dtype=torch.float32),
                        requires_grad=True,
                    )
                else:
                    self.sigma = torch.nn.Linear(out_size, actions_num)

            # Initialize weights
            mlp_init = self.init_factory.create(**self.initializer)

            # Initialize CNN layers
            for m in self.modules():
                if isinstance(m, nn.Conv2d):
                    nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")

            # Initialize MLP layers
            for m in self.mlp:
                if isinstance(m, nn.Linear):
                    mlp_init(m.weight)

            # Initialize policy and value heads
            if self.is_continuous:
                mu_init(self.mu.weight)
                if self.fixed_sigma:
                    sigma_init(self.sigma)
                else:
                    sigma_init(self.sigma.weight)
            mlp_init(self.value.weight)

        def forward(self, obs_dict):
            """Forward pass through the network."""
            # Extract observations
            # obs_dict["obs"] is a concatenated tensor: [pose_command (2), rgb_flattened (4*135*240)]
            # Total size: 2 + 72900 = 72902

            obs = obs_dict["obs"]

            if not isinstance(obs, torch.Tensor):
                raise TypeError(f"Expected obs to be a tensor, got {type(obs)}")

            # Split the concatenated observation tensor using stored sizes
            # obs_policy contains: [pose_command (2), base_lin_vel (3), base_ang_vel (3)]
            obs_policy = obs[:, : self.vector_obs_size]  # [batch, 8]
            sensor_obs_flat = obs[:, self.vector_obs_size :]  # [batch, 72900]

            # Reshape flattened RGB-D back to image format
            # Original shape: [batch, 4, 135, 240]
            batch_size = sensor_obs_flat.shape[0]
            cnn_shape = self.cnn_input_shape
            sensor_obs = sensor_obs_flat.reshape(batch_size, cnn_shape[0], cnn_shape[1], cnn_shape[2])

            # Permute sensor observations if needed (already in NCHW format, so skip)
            # Note: permute_input should be False in config since rgbd_processed returns NCHW
            if self.permute_input:
                sensor_obs = sensor_obs.permute((0, 3, 1, 2))

            dones = obs_dict.get("dones", None)
            bptt_len = obs_dict.get("bptt_len", 0)
            states = obs_dict.get("rnn_states", None)

            # Extract additional inputs for RNN if needed
            reward = obs_dict.get("reward", None) if self.require_rewards else None
            last_action = obs_dict.get("last_action", None) if self.require_last_actions else None

            # Process visual observations through CNN
            sensor_out = sensor_obs
            out = self.cnn(sensor_out)
            out = out.flatten(1)
            out = self.flatten_act(out)

            # Concatenate CNN features with processed vector observations
            out = torch.cat([out, self.pre_mlp(obs_policy)], dim=1)

            # Process through RNN if specified
            if self.has_rnn:
                seq_length = obs_dict.get("seq_length", 1)

                if not self.is_rnn_before_mlp:
                    out = self.mlp(out)

                obs_list = [out]
                if self.require_rewards and reward is not None:
                    obs_list.append(reward.unsqueeze(1))
                if self.require_last_actions and last_action is not None:
                    obs_list.append(last_action)
                out = torch.cat(obs_list, dim=1)

                batch_size = out.size()[0]
                num_seqs = batch_size // seq_length
                out = out.reshape(num_seqs, seq_length, -1)

                if len(states) == 1:
                    states = states[0]

                out = out.transpose(0, 1)
                if dones is not None:
                    dones = dones.reshape(num_seqs, seq_length, -1)
                    dones = dones.transpose(0, 1)
                out, states = self.rnn(out, states, dones, bptt_len)
                out = out.transpose(0, 1)
                out = out.contiguous().reshape(out.size()[0] * out.size()[1], -1)

                if self.rnn_ln:
                    out = self.layer_norm(out)
                if self.is_rnn_before_mlp:
                    out = self.mlp(out)
                if type(states) is not tuple:
                    states = (states,)
            else:
                out = self.mlp(out)

            # Compute value
            value = self.value_act(self.value(out))

            # Compute policy output (continuous actions)
            if self.is_continuous:
                mu = self.mu_act(self.mu(out))
                if self.fixed_sigma:
                    sigma = self.sigma_act(self.sigma)
                else:
                    sigma = self.sigma_act(self.sigma(out))
                return mu, mu * 0 + sigma, value, states

        def load(self, params):
            """Load network parameters from config."""
            self.separate = False
            self.units = params["mlp"]["units"]
            self.activation = params["mlp"]["activation"]
            self.initializer = params["mlp"]["initializer"]
            self.is_discrete = "discrete" in params["space"]
            self.is_continuous = "continuous" in params["space"]
            self.is_multi_discrete = "multi_discrete" in params["space"]
            self.value_activation = params.get("value_activation", "None")
            self.normalization = params.get("normalization", None)

            if self.is_continuous:
                self.space_config = params["space"]["continuous"]
                self.fixed_sigma = self.space_config["fixed_sigma"]
            elif self.is_discrete:
                self.space_config = params["space"]["discrete"]
            elif self.is_multi_discrete:
                self.space_config = params["space"]["multi_discrete"]

            self.has_rnn = "rnn" in params
            if self.has_rnn:
                self.rnn_units = params["rnn"]["units"]
                self.rnn_layers = params["rnn"]["layers"]
                self.rnn_name = params["rnn"]["name"]
                self.is_rnn_before_mlp = params["rnn"].get("before_mlp", False)
                self.rnn_ln = params["rnn"].get("layer_norm", False)

            self.has_cnn = True
            self.permute_input = params["cnn"].get("permute_input", True)
            self.conv_depths = params["cnn"]["conv_depths"]
            self.cnn_input_shape = params["cnn"].get("input_shape", [3, 128, 128])
            self.require_rewards = params.get("require_rewards")
            self.require_last_actions = params.get("require_last_actions")

        def _build_impala(self, input_shape, depths):
            """Build IMPALA-style CNN with residual blocks."""
            in_channels = input_shape[0]
            layers = nn.ModuleList()
            for d in depths:
                layers.append(ImpalaSequential(in_channels, d))
                in_channels = d
            return nn.Sequential(*layers)

        def is_separate_critic(self):
            return False

        def is_rnn(self):
            return self.has_rnn

        def get_default_rnn_state(self):
            num_layers = self.rnn_layers
            if self.rnn_name == "lstm":
                return (
                    torch.zeros((num_layers, self.num_seqs, self.rnn_units)),
                    torch.zeros((num_layers, self.num_seqs, self.rnn_units)),
                )
            else:
                return (torch.zeros((num_layers, self.num_seqs, self.rnn_units)),)

        def get_aux_loss(self):
            """Return auxiliary loss (none for this network)."""
            return None

    def build(self, name, **kwargs):
        """Build the network."""
        net = MixInputNetworkBuilder.Network(self.params, **kwargs)
        return net


def register_mix_input_network():
    """Register the mix_input_actor_critic network with RL-Games."""
    try:
        from rl_games.algos_torch import model_builder

        # Register the mix_input_actor_critic network
        model_builder.register_network("mix_input_actor_critic", lambda **kwargs: MixInputNetworkBuilder())
        print("[INFO] Successfully registered mix_input_actor_critic network builder")
        return True
    except ImportError as e:
        print(f"[WARNING] Could not register mix_input_actor_critic network: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Unexpected error while registering mix_input_actor_critic network: {e}")
        return False
