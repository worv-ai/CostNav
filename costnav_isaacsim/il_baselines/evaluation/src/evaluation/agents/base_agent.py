# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (Fan Yang, ETH Zurich)
# Licensed under the MIT License

"""Base Agent class for IL baselines."""

from typing import List

import numpy as np
import torch
import yaml
from PIL import Image


class BaseAgent:
    """Base class for all IL baseline agents.

    Handles common functionality like memory queue management, image preprocessing,
    and trajectory visualization.

    Args:
        model_config_path: Path to model configuration YAML file.
        robot_config_path: Path to robot configuration YAML file.
        device: PyTorch device to use for inference.
    """

    def __init__(
        self,
        model_config_path: str,
        robot_config_path: str,
        device: str = "cuda:0",
    ):
        self.device = device
        self.model_config_path = model_config_path

        # Load robot config
        with open(robot_config_path, "r") as f:
            robot_config = yaml.safe_load(f)
        self.MAX_V = robot_config["max_v"]
        self.MAX_W = robot_config["max_w"]
        self.RATE = robot_config["frame_rate"]
        self.DT = 1 / self.RATE
        self.EPS = 1e-8

        # Load model config
        with open(model_config_path, "r") as f:
            self.cfg = yaml.safe_load(f)
        self.memory_size = self.cfg["context_size"]
        self.image_size = self.cfg["image_size"]  # [width, height]
        self.normalize = self.cfg.get("normalize", True)

        # Memory queue for temporal context (initialized in reset)
        self.batch_size = 1
        self.memory_queue: List[List[Image.Image]] = [[]]

    def process_image(self, image: np.ndarray) -> Image.Image:
        """Convert numpy array to PIL Image."""
        return Image.fromarray(image)

    def reset(self, batch_size: int = 1):
        """Reset agent state for new episode(s).

        Args:
            batch_size: Number of parallel environments.
        """
        self.batch_size = batch_size
        self.memory_queue = [[] for _ in range(batch_size)]

    def reset_env(self, i: int):
        """Reset memory for a specific environment index."""
        self.memory_queue[i] = []

    def callback_obs(self, imgs: List[np.ndarray]):
        """Update memory queue with new observations.

        Args:
            imgs: List of images (one per batch element).
        """
        for i in range(len(self.memory_queue)):
            if len(self.memory_queue[i]) < self.memory_size + 1:
                # Initialize with repeated first frame
                self.memory_queue[i] = [self.process_image(imgs[i])] * (self.memory_size + 1)
            else:
                # FIFO queue: remove oldest, add newest
                self.memory_queue[i].pop(0)
                self.memory_queue[i].append(self.process_image(imgs[i]))

    def step_imagegoal(self, goal_image: np.ndarray, image: np.ndarray) -> tuple[torch.Tensor, torch.Tensor]:
        """Generate trajectory for image goal navigation.

        Args:
            goal_image: Target goal image.
            image: Current observation image.

        Returns:
            Tuple of (waypoints, trajectory).
        """
        raise NotImplementedError

    def step_nogoal(self, image: np.ndarray) -> tuple[torch.Tensor, torch.Tensor]:
        """Generate trajectory for exploration (no goal).

        Args:
            image: Current observation image.

        Returns:
            Tuple of (waypoints, trajectory).
        """
        raise NotImplementedError
