# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (https://github.com/InternRobotics/NavDP)
# Reference: third_party/NavDP/baselines/vint/vint_network.py
# Licensed under the MIT License

"""ViNT (Visual Navigation Transformer) Network Architecture."""

import math
from typing import Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from efficientnet_pytorch import EfficientNet

from .base_model import BaseModel


class PositionalEncoding(nn.Module):
    """Sinusoidal positional encoding for transformer."""

    def __init__(self, d_model: int, max_seq_len: int = 6):
        super().__init__()

        # Compute positional encoding once
        pos_enc = torch.zeros(max_seq_len, d_model)
        pos = torch.arange(0, max_seq_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        pos_enc[:, 0::2] = torch.sin(pos * div_term)
        pos_enc[:, 1::2] = torch.cos(pos * div_term)
        pos_enc = pos_enc.unsqueeze(0)

        # Register as buffer (not a parameter)
        self.register_buffer("pos_enc", pos_enc)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + self.pos_enc[:, : x.size(1), :]


class MultiLayerDecoder(nn.Module):
    """Transformer-based decoder with MLP output layers."""

    def __init__(
        self,
        embed_dim: int = 512,
        seq_len: int = 6,
        output_layers: list = [256, 128, 64],
        nhead: int = 8,
        num_layers: int = 8,
        ff_dim_factor: int = 4,
    ):
        super(MultiLayerDecoder, self).__init__()
        self.positional_encoding = PositionalEncoding(embed_dim, max_seq_len=seq_len)

        # Transformer encoder layer
        self.sa_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim,
            nhead=nhead,
            dim_feedforward=ff_dim_factor * embed_dim,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.sa_decoder = nn.TransformerEncoder(self.sa_layer, num_layers=num_layers)

        # Output MLP layers
        self.output_layers = nn.ModuleList([nn.Linear(seq_len * embed_dim, embed_dim)])
        self.output_layers.append(nn.Linear(embed_dim, output_layers[0]))
        for i in range(len(output_layers) - 1):
            self.output_layers.append(nn.Linear(output_layers[i], output_layers[i + 1]))

    def forward(
        self,
        x: torch.Tensor,
        src_key_padding_mask: Optional[torch.Tensor] = None,
        avg_mask: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        if self.positional_encoding:
            x = self.positional_encoding(x)

        if src_key_padding_mask is not None and avg_mask is not None:
            x = self.sa_decoder(x, src_key_padding_mask=src_key_padding_mask)
            x = x * avg_mask
        else:
            x = self.sa_decoder(x)

        # Flatten and pass through MLP: [batch_size, seq_len, embed_dim] -> [batch_size, output_dim]
        x = x.reshape(x.shape[0], -1)
        for layer in self.output_layers:
            x = layer(x)
            x = F.relu(x)
        return x


class ViNT(BaseModel):
    """ViNT: Visual Navigation Transformer.

    Uses a Transformer-based architecture to encode visual observations and goals
    using an EfficientNet CNN, and predicts temporal distance and normalized actions.
    """

    def __init__(
        self,
        context_size: int = 5,
        len_traj_pred: Optional[int] = 5,
        learn_angle: Optional[bool] = True,
        obs_encoder: Optional[str] = "efficientnet-b0",
        obs_encoding_size: Optional[int] = 512,
        late_fusion: Optional[bool] = False,
        mha_num_attention_heads: Optional[int] = 2,
        mha_num_attention_layers: Optional[int] = 2,
        mha_ff_dim_factor: Optional[int] = 4,
    ) -> None:
        super(ViNT, self).__init__(context_size, len_traj_pred, learn_angle)
        self.obs_encoding_size = obs_encoding_size
        self.goal_encoding_size = obs_encoding_size
        self.late_fusion = late_fusion

        # Observation encoder
        if obs_encoder.split("-")[0] == "efficientnet":
            self.obs_encoder = EfficientNet.from_name(obs_encoder, in_channels=3)
            self.num_obs_features = self.obs_encoder._fc.in_features
            # Goal encoder (with or without late fusion)
            if self.late_fusion:
                self.goal_encoder = EfficientNet.from_name("efficientnet-b0", in_channels=3)
            else:
                self.goal_encoder = EfficientNet.from_name("efficientnet-b0", in_channels=6)
            self.num_goal_features = self.goal_encoder._fc.in_features
        else:
            raise NotImplementedError(f"Encoder {obs_encoder} not supported")

        # Compression layers
        if self.num_obs_features != self.obs_encoding_size:
            self.compress_obs_enc = nn.Linear(self.num_obs_features, self.obs_encoding_size)
        else:
            self.compress_obs_enc = nn.Identity()

        if self.num_goal_features != self.goal_encoding_size:
            self.compress_goal_enc = nn.Linear(self.num_goal_features, self.goal_encoding_size)
        else:
            self.compress_goal_enc = nn.Identity()

        # Transformer decoder
        self.decoder = MultiLayerDecoder(
            embed_dim=self.obs_encoding_size,
            seq_len=self.context_size + 2,  # context + current + goal
            output_layers=[256, 128, 64, 32],
            nhead=mha_num_attention_heads,
            num_layers=mha_num_attention_layers,
            ff_dim_factor=mha_ff_dim_factor,
        )

        # Output heads
        self.dist_predictor = nn.Sequential(nn.Linear(32, 1))
        self.action_predictor = nn.Sequential(nn.Linear(32, self.len_trajectory_pred * self.num_action_params))

    def forward(
        self,
        obs_img: torch.Tensor,
        goal_img: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """Forward pass.

        Args:
            obs_img: Observation images [B, 3*(context_size+1), H, W].
            goal_img: Goal image [B, 3, H, W].

        Returns:
            dist_pred: Distance prediction [B, 1].
            action_pred: Action prediction [B, len_traj_pred, num_action_params].
        """
        # Encode goal (with or without late fusion)
        if self.late_fusion:
            goal_encoding = self.goal_encoder.extract_features(goal_img)
        else:
            obsgoal_img = torch.cat([obs_img[:, 3 * self.context_size :, :, :], goal_img], dim=1)
            goal_encoding = self.goal_encoder.extract_features(obsgoal_img)

        goal_encoding = self.goal_encoder._avg_pooling(goal_encoding)
        if self.goal_encoder._global_params.include_top:
            goal_encoding = goal_encoding.flatten(start_dim=1)
            goal_encoding = self.goal_encoder._dropout(goal_encoding)
        goal_encoding = self.compress_goal_enc(goal_encoding)
        if len(goal_encoding.shape) == 2:
            goal_encoding = goal_encoding.unsqueeze(1)
        # [B, 1, goal_encoding_size]

        # Split observations into context frames
        obs_img = torch.split(obs_img, 3, dim=1)
        obs_img = torch.concat(obs_img, dim=0)
        # [B*(context_size+1), 3, H, W]

        # Encode observations
        obs_encoding = self.obs_encoder.extract_features(obs_img)
        obs_encoding = self.obs_encoder._avg_pooling(obs_encoding)
        if self.obs_encoder._global_params.include_top:
            obs_encoding = obs_encoding.flatten(start_dim=1)
            obs_encoding = self.obs_encoder._dropout(obs_encoding)
        obs_encoding = self.compress_obs_enc(obs_encoding)
        # [B*(context_size+1), obs_encoding_size]

        # Reshape to [B, context_size+1, obs_encoding_size]
        obs_encoding = obs_encoding.reshape((self.context_size + 1, -1, self.obs_encoding_size))
        obs_encoding = torch.transpose(obs_encoding, 0, 1)

        # Concatenate goal encoding
        tokens = torch.cat((obs_encoding, goal_encoding), dim=1)
        final_repr = self.decoder(tokens)
        # [B, 32]

        # Predictions
        dist_pred = self.dist_predictor(final_repr)
        action_pred = self.action_predictor(final_repr)

        # Reshape actions
        action_pred = action_pred.reshape((action_pred.shape[0], self.len_trajectory_pred, self.num_action_params))
        # Convert position deltas to cumulative waypoints
        action_pred[:, :, :2] = torch.cumsum(action_pred[:, :, :2], dim=1)
        # Normalize angle prediction
        if self.learn_angle:
            action_pred[:, :, 2:] = F.normalize(action_pred[:, :, 2:].clone(), dim=-1)

        return dist_pred, action_pred


class NoGoalViNT(ViNT):
    """ViNT variant for exploration without goal conditioning."""

    def __init__(self, **kwargs) -> None:
        super(NoGoalViNT, self).__init__(**kwargs)
        # Decoder without goal token
        self.decoder = MultiLayerDecoder(
            embed_dim=self.obs_encoding_size,
            seq_len=self.context_size + 1,  # No goal
            output_layers=[256, 128, 64, 32],
            nhead=kwargs.get("mha_num_attention_heads", 2),
            num_layers=kwargs.get("mha_num_attention_layers", 2),
            ff_dim_factor=kwargs.get("mha_ff_dim_factor", 4),
        )

    def forward(self, obs_img: torch.Tensor) -> torch.Tensor:
        """Forward pass without goal."""
        obs_img = torch.split(obs_img, 3, dim=1)
        obs_img = torch.concat(obs_img, dim=0)

        obs_encoding = self.obs_encoder.extract_features(obs_img)
        obs_encoding = self.obs_encoder._avg_pooling(obs_encoding)
        if self.obs_encoder._global_params.include_top:
            obs_encoding = obs_encoding.flatten(start_dim=1)
            obs_encoding = self.obs_encoder._dropout(obs_encoding)
        obs_encoding = self.compress_obs_enc(obs_encoding)

        obs_encoding = obs_encoding.reshape((self.context_size + 1, -1, self.obs_encoding_size))
        obs_encoding = torch.transpose(obs_encoding, 0, 1)

        final_repr = self.decoder(obs_encoding)
        action_pred = self.action_predictor(final_repr)

        action_pred = action_pred.reshape((action_pred.shape[0], self.len_trajectory_pred, self.num_action_params))
        action_pred[:, :, :2] = torch.cumsum(action_pred[:, :, :2], dim=1)
        if self.learn_angle:
            action_pred[:, :, 2:] = F.normalize(action_pred[:, :, 2:].clone(), dim=-1)

        return action_pred


class ViNTPolicy(nn.Module):
    """ViNT Policy wrapper for inference."""

    def __init__(self, config: dict):
        super(ViNTPolicy, self).__init__()
        self.config = config
        self.model = ViNT(
            context_size=config["context_size"],
            len_traj_pred=config["len_traj_pred"],
            learn_angle=config["learn_angle"],
            obs_encoder=config["obs_encoder"],
            obs_encoding_size=config["obs_encoding_size"],
            late_fusion=config["late_fusion"],
            mha_num_attention_heads=config["mha_num_attention_heads"],
            mha_num_attention_layers=config["mha_num_attention_layers"],
            mha_ff_dim_factor=config["mha_ff_dim_factor"],
        )

    def predict_imagegoal_distance_and_action(
        self, image: torch.Tensor, goal_image: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.model(image, goal_image)

    def predict_nogoal_distance_and_action(
        self, image: torch.Tensor, fake_goal_image: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.model(image, fake_goal_image)


class NoGoalViNTPolicy(nn.Module):
    """NoGoal ViNT Policy wrapper for exploration."""

    def __init__(self, config: dict):
        super(NoGoalViNTPolicy, self).__init__()
        self.config = config
        self.model = NoGoalViNT(
            context_size=config["context_size"],
            len_traj_pred=config["len_traj_pred"],
            learn_angle=config["learn_angle"],
            obs_encoder=config["obs_encoder"],
            obs_encoding_size=config["obs_encoding_size"],
            late_fusion=config["late_fusion"],
            mha_num_attention_heads=config["mha_num_attention_heads"],
            mha_num_attention_layers=config["mha_num_attention_layers"],
            mha_ff_dim_factor=config["mha_ff_dim_factor"],
        )

    def predict_nogoal_action(self, image: torch.Tensor) -> torch.Tensor:
        return self.model(image)
