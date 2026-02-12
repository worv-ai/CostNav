# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (https://github.com/InternRobotics/NavDP)
# Reference: third_party/NavDP/baselines/vint/vint_agent.py
# Licensed under the MIT License

"""ViNT Agent for navigation."""

from typing import List, Tuple

import numpy as np
import torch
import torchvision.transforms.functional as TF
from PIL import Image as PILImage
from torchvision import transforms

from il_evaluation.agents.base_agent import BaseAgent
from il_evaluation.models.traj_opt import TrajOpt
from il_evaluation.models.vint_network import NoGoalViNTPolicy, ViNTPolicy

IMAGE_ASPECT_RATIO = 4 / 3


def to_numpy(tensor: torch.Tensor) -> np.ndarray:
    """Convert torch tensor to numpy array."""
    return tensor.detach().cpu().numpy()


def from_numpy(array: np.ndarray) -> torch.Tensor:
    """Convert numpy array to torch tensor."""
    return torch.from_numpy(array).float()


def transform_images(
    pil_imgs: List[PILImage.Image],
    image_size: List[int],
    center_crop: bool = False,
) -> torch.Tensor:
    """Transform PIL images to normalized torch tensor.

    Args:
        pil_imgs: List of PIL images.
        image_size: Target size [width, height].
        center_crop: Whether to center crop to maintain aspect ratio.

    Returns:
        Tensor of shape [1, N*3, H, W].
    """
    transform_type = transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    if not isinstance(pil_imgs, list):
        pil_imgs = [pil_imgs]

    transf_imgs = []
    for pil_img in pil_imgs:
        w, h = pil_img.size
        if center_crop:
            if w > h:
                pil_img = TF.center_crop(pil_img, (h, int(h * IMAGE_ASPECT_RATIO)))
            else:
                pil_img = TF.center_crop(pil_img, (int(w / IMAGE_ASPECT_RATIO), w))
        pil_img = pil_img.resize(image_size)
        transf_img = transform_type(pil_img)
        transf_img = torch.unsqueeze(transf_img, 0)
        transf_imgs.append(transf_img)

    return torch.cat(transf_imgs, dim=1)


class ViNTAgent(BaseAgent):
    """ViNT Agent for image-goal and no-goal navigation.

    Args:
        model_path: Path to trained ViNT model weights.
        model_config_path: Path to model configuration YAML.
        robot_config_path: Path to robot configuration YAML.
        device: PyTorch device for inference.
    """

    def __init__(
        self,
        model_path: str,
        model_config_path: str,
        robot_config_path: str,
        device: str = "cuda:0",
    ):
        super().__init__(model_config_path, robot_config_path, device)

        self.model_path = model_path

        # Initialize ViNT model
        self.vint_policy = ViNTPolicy(self.cfg)
        self.vint_policy.to(self.device)
        # Load checkpoint - handle different checkpoint formats
        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        if isinstance(checkpoint, dict) and "model" in checkpoint:
            # Training checkpoint - "model" may be state_dict or full model object
            model_data = checkpoint["model"]
            if hasattr(model_data, "state_dict"):
                self.vint_policy.model.load_state_dict(model_data.state_dict(), strict=True)
            else:
                self.vint_policy.model.load_state_dict(model_data, strict=True)
        elif hasattr(checkpoint, "state_dict"):
            # Full model object from vint_train
            self.vint_policy.model.load_state_dict(checkpoint.state_dict(), strict=True)
        else:
            # Already a state_dict
            self.vint_policy.model.load_state_dict(checkpoint, strict=True)
        self.vint_policy.eval()

        # Trajectory optimizer for smooth paths
        self.traj_generate = TrajOpt()

    def step_imagegoal(
        self, goal_images: List[np.ndarray], images: List[np.ndarray]
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Generate trajectory towards goal image.

        Args:
            goal_images: List of goal images (one per batch).
            images: List of current observation images (one per batch).

        Returns:
            Tuple of (waypoints, trajectory, distances).
                waypoints: [B, num_waypoints, 3] (x, y, theta).
                trajectory: [B, num_points, 3] smoothed trajectory.
                distances: [B] predicted distance to goal.
        """
        with torch.no_grad():
            self.callback_obs(images)

            # Transform goal images
            goal_image = [
                transform_images(PILImage.fromarray(g_img), self.image_size, center_crop=False).to(self.device)
                for g_img in goal_images
            ]
            goal_image = torch.concat(goal_image, dim=0)

            # Transform observation context
            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            # Inference
            distances, waypoints = self.vint_policy.predict_imagegoal_distance_and_action(input_image, goal_image)

            # Apply normalization if configured
            if self.normalize:
                waypoints[:, :, :2] *= self.MAX_V / self.RATE

            # Stop if far from goal (distance > 7.0)
            stop_mask = (distances > 7.0).unsqueeze(1).float()
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(waypoints[:, :, 0:3], step=0.1) * stop_mask

            return waypoints[:, :, 0:3], trajectory, distances

    def step_topomap(
        self,
        images: List[np.ndarray],
        subgoal_images: List[PILImage.Image],
        close_threshold: float = 3.0,
    ) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray, int]:
        """Run batched inference against a window of topomap subgoal images.

        This implements the core localization + subgoal selection loop from
        ViNT's ``navigate.py`` (non-NoMaD branch).  For each subgoal image in
        the provided window the model predicts a temporal distance and a set of
        waypoints.  The closest node is identified via ``argmin`` of the
        predicted distances, and the next subgoal is selected based on whether
        the closest node distance is below *close_threshold*.

        Args:
            images: List with a single current observation image ``[np.ndarray]``.
            subgoal_images: Window of PIL images from the topomap
                (e.g. ``topomap[start:end+1]``).
            close_threshold: Temporal distance below which the agent advances
                to the next node in the window (default: 3.0).

        Returns:
            Tuple of ``(waypoints, trajectory, distances, chosen_subgoal_idx)``.
                waypoints: ``[N, num_waypoints, 3]`` for each subgoal.
                trajectory: ``[1, num_points, 3]`` smoothed trajectory for the
                    chosen subgoal.
                distances: ``numpy [N]`` predicted distances to each subgoal.
                chosen_subgoal_idx: Index into *subgoal_images* of the chosen
                    subgoal (0-based within the window).
        """
        with torch.no_grad():
            self.callback_obs(images)

            # Transform observation context (single batch element)
            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device)
                for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)  # [1, C*(ctx+1), H, W]

            # Transform each subgoal image
            goal_tensors = [
                transform_images(sg_img, self.image_size, center_crop=False).to(self.device)
                for sg_img in subgoal_images
            ]
            goal_batch = torch.concat(goal_tensors, dim=0)  # [N, 3, H, W]

            # Repeat observation for each subgoal so batch dims match
            obs_batch = input_image.repeat(len(subgoal_images), 1, 1, 1)  # [N, C*(ctx+1), H, W]

            # Batched inference
            distances, waypoints = self.vint_policy.predict_imagegoal_distance_and_action(
                obs_batch, goal_batch
            )

            # Apply normalization
            if self.normalize:
                waypoints[:, :, :2] *= self.MAX_V / self.RATE

            distances_np = to_numpy(distances.flatten())  # [N]

            # Localize: find closest node in the window
            min_dist_idx = int(np.argmin(distances_np))

            # Select subgoal: advance if close enough
            if distances_np[min_dist_idx] > close_threshold:
                chosen_idx = min_dist_idx
            else:
                chosen_idx = min(min_dist_idx + 1, len(subgoal_images) - 1)

            # Generate smooth trajectory for the chosen subgoal only
            chosen_wp = waypoints[chosen_idx : chosen_idx + 1, :, 0:3]
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(chosen_wp, step=0.1)

            return waypoints[:, :, 0:3], trajectory, distances_np, chosen_idx

    def step_nogoal(self, images: List[np.ndarray]) -> Tuple[torch.Tensor, torch.Tensor]:
        """Generate trajectory for exploration without goal.

        Args:
            images: List of current observation images (one per batch).

        Returns:
            Tuple of (waypoints, trajectory).
        """
        with torch.no_grad():
            self.callback_obs(images)

            # Create fake goal (random noise - will be ignored by model)
            fake_goal = torch.randn((len(images), 3, self.image_size[1], self.image_size[0])).to(self.device)

            # Transform observation context
            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            # Inference with fake goal
            distances, waypoints = self.vint_policy.predict_nogoal_distance_and_action(input_image, fake_goal)

            # Apply normalization if configured
            if self.normalize:
                waypoints[:, :, :2] *= self.MAX_V / self.RATE

            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(waypoints[:, :, 0:3], step=0.1)

            return waypoints[:, :, 0:3], trajectory


class NoGoalViNTAgent(BaseAgent):
    """ViNT Agent specifically for exploration without goal.

    Uses a model architecture that doesn't require goal conditioning.
    """

    def __init__(
        self,
        model_path: str,
        model_config_path: str,
        robot_config_path: str,
        device: str = "cuda:0",
    ):
        super().__init__(model_config_path, robot_config_path, device)

        self.model_path = model_path

        # Initialize NoGoal ViNT model
        self.vint_policy = NoGoalViNTPolicy(self.cfg)
        self.vint_policy.to(self.device)
        # Load checkpoint - handle different checkpoint formats
        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        if isinstance(checkpoint, dict) and "model" in checkpoint:
            # Training checkpoint - "model" may be state_dict or full model object
            model_data = checkpoint["model"]
            if hasattr(model_data, "state_dict"):
                self.vint_policy.model.load_state_dict(model_data.state_dict(), strict=True)
            else:
                self.vint_policy.model.load_state_dict(model_data, strict=True)
        elif hasattr(checkpoint, "state_dict"):
            # Full model object from vint_train
            self.vint_policy.model.load_state_dict(checkpoint.state_dict(), strict=True)
        else:
            # Already a state_dict
            self.vint_policy.model.load_state_dict(checkpoint, strict=True)
        self.vint_policy.eval()

        # Trajectory optimizer
        self.traj_generate = TrajOpt()

    def step_nogoal(self, images: List[np.ndarray]) -> Tuple[torch.Tensor, torch.Tensor]:
        """Generate trajectory for exploration."""
        with torch.no_grad():
            self.callback_obs(images)

            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            waypoints = self.vint_policy.predict_nogoal_action(input_image)

            if self.normalize:
                waypoints[:, :, :2] *= self.MAX_V / self.RATE

            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(waypoints[:, :, 0:3], step=0.1)

            return waypoints[:, :, 0:3], trajectory
