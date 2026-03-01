#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""NavDP Agent for local (non-server) inference in ROS2.

This agent loads the InternNav NavDP model and provides point/image/pixel-goal
inference using RGB-D observations with a temporal memory queue.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import torch


def _find_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "third_party" / "InternNav").exists():
            return parent
    raise RuntimeError("Could not locate repo root (third_party/InternNav not found)")


def _ensure_internnav_on_path() -> None:
    repo_root = _find_repo_root()
    internnav_dir = repo_root / "third_party" / "InternNav"
    if str(internnav_dir) not in sys.path:
        sys.path.insert(0, str(internnav_dir))


class NavDPAgent:
    """NavDP Agent for point/image/pixel-goal navigation.

    Args:
        checkpoint: Path to NavDP checkpoint (state_dict).
        image_size: Input image size (square).
        memory_size: Temporal memory size.
        predict_size: Number of waypoints to predict.
        temporal_depth: Transformer depth.
        heads: Attention heads.
        token_dim: Token dimension.
        channels: Input channels.
        dropout: Dropout.
        device: Torch device (e.g., "cuda:0" or "cpu").
        sample_num: Number of diffusion samples.
        depth_scale: Scale for uint16 depth to meters (default 10000.0).
    """

    def __init__(
        self,
        checkpoint: str,
        depth_anything_checkpoint: Optional[str] = None,
        depth_anything_encoder: Optional[str] = None,
        image_size: int = 224,
        memory_size: int = 8,
        predict_size: int = 24,
        temporal_depth: int = 16,
        heads: int = 8,
        token_dim: int = 384,
        channels: int = 3,
        dropout: float = 0.1,
        device: str = "cuda:0",
        sample_num: int = 32,
        depth_scale: float = 10000.0,
    ):
        _ensure_internnav_on_path()

        from internnav.model.basemodel.navdp.navdp_policy import NavDPModelConfig, NavDPNet

        self.device = torch.device(device)
        self.image_size = int(image_size)
        self.memory_size = int(memory_size)
        self.predict_size = int(predict_size)
        self.temporal_depth = int(temporal_depth)
        self.heads = int(heads)
        self.token_dim = int(token_dim)
        self.channels = int(channels)
        self.dropout = float(dropout)
        self.sample_num = int(sample_num)
        self.depth_scale = float(depth_scale)

        # Minimal model_cfg needed by NavDPNet
        model_cfg = {
            "model": {"policy_name": "navdp_Policy", "state_encoder": None},
            "il": {
                "image_size": self.image_size,
                "memory_size": self.memory_size,
                "predict_size": self.predict_size,
                "pixel_channel": 4,
                "temporal_depth": self.temporal_depth,
                "heads": self.heads,
                "token_dim": self.token_dim,
                "channels": self.channels,
                "dropout": self.dropout,
                "scratch": False,
                "finetune": False,
            },
            "local_rank": 0,
        }
        if depth_anything_checkpoint:
            model_cfg["il"]["depth_anything_checkpoint"] = str(depth_anything_checkpoint)
        # NOTE:
        # `depth_anything_encoder` in eval is primarily for the online RGB->depth estimator
        # (navdp_policy_node). The trained NavDP checkpoint may have been trained with a
        # different backbone variant (commonly vits). Forcing the InternNav model encoder
        # here can break checkpoint loading with large state_dict size mismatches.
        #
        # Keep the NavDP model architecture at InternNav default unless we introduce a
        # separate explicit "model backbone encoder" argument.

        config = NavDPModelConfig(model_cfg=model_cfg)
        self.model = NavDPNet(config)
        if checkpoint:
            state_dict = torch.load(checkpoint, map_location=self.device, weights_only=False)
            if isinstance(state_dict, dict) and "state_dict" in state_dict:
                state_dict = state_dict["state_dict"]
            incompatible_keys = self.model.load_state_dict(state_dict, strict=False)
            if incompatible_keys.missing_keys or incompatible_keys.unexpected_keys:
                print(
                    "Incompatible keys: "
                    f"missing={incompatible_keys.missing_keys}, "
                    f"unexpected={incompatible_keys.unexpected_keys}"
                )
        self.model.to(self.device)
        self.model.eval()

        # Keep encoder device fields in sync
        for attr in ("rgbd_encoder", "pixel_encoder", "image_encoder"):
            if hasattr(self.model, attr):
                setattr(getattr(self.model, attr), "device", self.device)

        self.reset(batch_size=1)

    def reset(self, batch_size: int = 1) -> None:
        self.batch_size = int(batch_size)
        self.memory_queue: List[List[np.ndarray]] = [[] for _ in range(self.batch_size)]

    def _process_image(self, image: np.ndarray) -> np.ndarray:
        """Resize and normalize RGB image to model input size."""
        if image is None:
            raise ValueError("image is None")
        h, w = image.shape[:2]
        prop = self.image_size / max(h, w)
        resize_image = cv2.resize(image, (-1, -1), fx=prop, fy=prop)
        pad_width = max((self.image_size - resize_image.shape[1]) // 2, 0)
        pad_height = max((self.image_size - resize_image.shape[0]) // 2, 0)
        pad_image = np.pad(
            resize_image,
            ((pad_height, pad_height), (pad_width, pad_width), (0, 0)),
            mode="constant",
            constant_values=0,
        )
        resize_image = cv2.resize(pad_image, (self.image_size, self.image_size))
        resize_image = resize_image.astype(np.float32) / 255.0
        return resize_image

    def _process_depth(self, depth: np.ndarray) -> np.ndarray:
        """Resize and normalize depth to model input size."""
        if depth is None:
            raise ValueError("depth is None")
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / self.depth_scale
        else:
            depth = depth.astype(np.float32)
        h, w = depth.shape[:2]
        prop = self.image_size / max(h, w)
        resize_depth = cv2.resize(depth, (-1, -1), fx=prop, fy=prop)
        pad_width = max((self.image_size - resize_depth.shape[1]) // 2, 0)
        pad_height = max((self.image_size - resize_depth.shape[0]) // 2, 0)
        pad_depth = np.pad(
            resize_depth,
            ((pad_height, pad_height), (pad_width, pad_width)),
            mode="constant",
            constant_values=0,
        )
        pad_depth[pad_depth > 5.0] = 0
        pad_depth[pad_depth < 0.1] = 0
        resize_depth = cv2.resize(pad_depth, (self.image_size, self.image_size))
        return resize_depth[:, :, np.newaxis]

    def _update_memory(self, idx: int, processed_image: np.ndarray) -> np.ndarray:
        """Update memory queue and return stacked memory tensor."""
        queue = self.memory_queue[idx]
        if len(queue) < self.memory_size:
            queue.append(processed_image)
        else:
            queue.pop(0)
            queue.append(processed_image)

        input_image = np.array(queue)
        if input_image.shape[0] < self.memory_size:
            pad = np.zeros(
                (self.memory_size - input_image.shape[0], self.image_size, self.image_size, 3), dtype=np.float32
            )
            input_image = np.concatenate((pad, input_image), axis=0)
        return input_image

    def _to_tensor(self, array: np.ndarray) -> torch.Tensor:
        """Move numpy batch arrays onto the model device."""
        return torch.as_tensor(array, dtype=torch.float32, device=self.device)

    def _process_pixel_goal(self, image: np.ndarray, pixel_xy: Tuple[float, float]) -> np.ndarray:
        """Create pixel-goal mask aligned with processed image."""
        h, w = image.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        x = int(round(pixel_xy[0]))
        y = int(round(pixel_xy[1]))
        box = 10
        if 0 <= x < w and 0 <= y < h:
            x0 = max(x - box, 0)
            x1 = min(x + box, w - 1)
            y0 = max(y - box, 0)
            y1 = min(y + box, h - 1)
            mask[y0 : y1 + 1, x0 : x1 + 1] = 255

        prop = self.image_size / max(h, w)
        resize_mask = cv2.resize(mask, (-1, -1), fx=prop, fy=prop, interpolation=cv2.INTER_NEAREST)
        pad_width = max((self.image_size - resize_mask.shape[1]) // 2, 0)
        pad_height = max((self.image_size - resize_mask.shape[0]) // 2, 0)
        pad_mask = np.pad(
            resize_mask,
            ((pad_height, pad_height), (pad_width, pad_width)),
            mode="constant",
            constant_values=0,
        )
        mask_img = cv2.resize(pad_mask, (self.image_size, self.image_size), interpolation=cv2.INTER_NEAREST)
        mask_img = mask_img.astype(np.float32) / 255.0
        return mask_img[:, :, None]

    def _sample_with_goal_embed(self, goal_embed: torch.Tensor, rgbd_embed: torch.Tensor) -> torch.Tensor:
        noisy_action = torch.randn(
            (self.sample_num * goal_embed.shape[0], self.predict_size, 3), device=self.device
        )
        naction = noisy_action
        self.model.noise_scheduler.set_timesteps(self.model.noise_scheduler.config.num_train_timesteps)
        for k in self.model.noise_scheduler.timesteps[:]:
            noise_pred = self.model.predict_noise(naction, k.to(self.device).unsqueeze(0), goal_embed, rgbd_embed)
            naction = self.model.noise_scheduler.step(model_output=noise_pred, timestep=k, sample=naction).prev_sample
        critic_values = self.model.predict_critic(naction, rgbd_embed)
        positive_traj = torch.cumsum(naction / 4.0, dim=1)[(-critic_values).argsort()[0:8]]
        return positive_traj

    @torch.no_grad()
    def step_pointgoal(
        self, goal_xyz: np.ndarray, image: np.ndarray, depth: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Predict trajectory for point-goal navigation.

        Args:
            goal_xyz: Goal in robot frame [3] (x, y, theta).
            image: RGB image (H, W, 3) uint8.
            depth: Depth image (H, W) or (H, W, 1).

        Returns:
            Tuple of (best_trajectory, all_trajectories)
        """
        proc_image = self._process_image(image)
        proc_depth = self._process_depth(depth)

        mem_image = self._update_memory(0, proc_image)
        input_images = self._to_tensor(mem_image[None, ...])  # [B, T, H, W, 3]
        input_depths = self._to_tensor(proc_depth[None, ...])  # [B, H, W, 1]
        goal = np.asarray(goal_xyz, dtype=np.float32)[None, ...]  # [B, 3]

        negative_traj, positive_traj = self.model.predict_pointgoal_batch_action_vel(
            goal, input_images, input_depths, sample_num=self.sample_num
        )

        # positive_traj shape: [K, predict_size, 3] for B=1
        best_traj = positive_traj[0].detach().cpu().numpy()
        all_traj = positive_traj.detach().cpu().numpy()
        return best_traj, all_traj

    @torch.no_grad()
    def step_imagegoal(
        self, goal_image: np.ndarray, image: np.ndarray, depth: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Predict trajectory for image-goal navigation."""
        proc_image = self._process_image(image)
        proc_depth = self._process_depth(depth)
        proc_goal = self._process_image(goal_image)

        mem_image = self._update_memory(0, proc_image)
        input_images = self._to_tensor(mem_image[None, ...])  # [B, T, H, W, 3]
        input_depths = self._to_tensor(proc_depth[None, ...])  # [B, H, W, 1]

        rgbd_embed = self.model.rgbd_encoder(input_images, input_depths)
        image_goal = self._to_tensor(np.concatenate((proc_goal, proc_image), axis=-1)[None, ...])  # [B, H, W, 6]
        image_embed = self.model.image_encoder(image_goal).unsqueeze(1)

        positive_traj = self._sample_with_goal_embed(image_embed, rgbd_embed)
        best_traj = positive_traj[0].detach().cpu().numpy()
        all_traj = positive_traj.detach().cpu().numpy()
        return best_traj, all_traj

    @torch.no_grad()
    def step_pixelgoal(
        self, pixel_xy: Tuple[float, float], image: np.ndarray, depth: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Predict trajectory for pixel-goal navigation."""
        proc_image = self._process_image(image)
        proc_depth = self._process_depth(depth)
        pixel_mask = self._process_pixel_goal(image, pixel_xy)

        mem_image = self._update_memory(0, proc_image)
        input_images = self._to_tensor(mem_image[None, ...])
        input_depths = self._to_tensor(proc_depth[None, ...])

        rgbd_embed = self.model.rgbd_encoder(input_images, input_depths)
        pixel_goal = self._to_tensor(np.concatenate((proc_image, pixel_mask), axis=-1)[None, ...])  # [B, H, W, 4]
        pixel_embed = self.model.pixel_encoder(pixel_goal).unsqueeze(1)

        positive_traj = self._sample_with_goal_embed(pixel_embed, rgbd_embed)
        best_traj = positive_traj[0].detach().cpu().numpy()
        all_traj = positive_traj.detach().cpu().numpy()
        return best_traj, all_traj

    @torch.no_grad()
    def step_nogoal(self, image: np.ndarray, depth: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Predict trajectory for no-goal navigation."""
        proc_image = self._process_image(image)
        proc_depth = self._process_depth(depth)

        mem_image = self._update_memory(0, proc_image)
        input_images = self._to_tensor(mem_image[None, ...])
        input_depths = self._to_tensor(proc_depth[None, ...])

        negative_traj, positive_traj = self.model.predict_nogoal_batch_action_vel(
            input_images, input_depths, sample_num=self.sample_num
        )
        best_traj = positive_traj[0].detach().cpu().numpy()
        all_traj = positive_traj.detach().cpu().numpy()
        return best_traj, all_traj
