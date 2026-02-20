# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (https://github.com/InternRobotics/NavDP)
# Reference: third_party/NavDP/baselines/gnm/gnm_agent.py
# Licensed under the MIT License

"""GNM Agent for navigation."""

from typing import List, Tuple

import numpy as np
import torch
from PIL import Image as PILImage

from il_evaluation.agents.base_agent import BaseAgent
from il_evaluation.agents.vint_agent import to_numpy, transform_images
from il_evaluation.models.gnm_network import GNMPolicy, NoGoalGNMPolicy
from il_evaluation.models.traj_opt import TrajOpt


class GNMAgent(BaseAgent):
    """GNM Agent for image-goal and no-goal navigation."""

    def __init__(
        self,
        model_path: str,
        model_config_path: str,
        device: str = "cuda:0",
    ):
        super().__init__(model_config_path, device)

        self.model_path = model_path

        self.gnm_policy = GNMPolicy(self.cfg)
        self.gnm_policy.to(self.device)

        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        if isinstance(checkpoint, dict) and "model" in checkpoint:
            model_data = checkpoint["model"]
            if hasattr(model_data, "state_dict"):
                self.gnm_policy.model.load_state_dict(model_data.state_dict(), strict=True)
            else:
                self.gnm_policy.model.load_state_dict(model_data, strict=True)
        elif hasattr(checkpoint, "state_dict"):
            self.gnm_policy.model.load_state_dict(checkpoint.state_dict(), strict=True)
        else:
            self.gnm_policy.model.load_state_dict(checkpoint, strict=True)
        self.gnm_policy.eval()

        self.traj_generate = TrajOpt()

    def step_imagegoal(
        self, goal_images: List[np.ndarray], images: List[np.ndarray]
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        with torch.no_grad():
            self.callback_obs(images)

            goal_image = [
                transform_images(PILImage.fromarray(g_img), self.image_size, center_crop=False).to(self.device)
                for g_img in goal_images
            ]
            goal_image = torch.concat(goal_image, dim=0)

            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            distances, waypoints = self.gnm_policy.predict_imagegoal_distance_and_action(input_image, goal_image)

            if self.normalize:
                waypoints[:, :, :2] *= self.denorm_scale

            stop_mask = (distances > 7.0).unsqueeze(1).float()
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(waypoints[:, :, 0:3], step=0.1) * stop_mask

            return waypoints[:, :, 0:3], trajectory, distances

    def step_topomap(
        self,
        images: List[np.ndarray],
        subgoal_images: List[PILImage.Image],
        close_threshold: float = 3.0,
    ) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray, int]:
        with torch.no_grad():
            self.callback_obs(images)

            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            goal_tensors = [
                transform_images(sg_img, self.image_size, center_crop=False).to(self.device) for sg_img in subgoal_images
            ]
            goal_batch = torch.concat(goal_tensors, dim=0)

            obs_batch = input_image.repeat(len(subgoal_images), 1, 1, 1)
            distances, waypoints = self.gnm_policy.predict_imagegoal_distance_and_action(obs_batch, goal_batch)

            if self.normalize:
                waypoints[:, :, :2] *= self.denorm_scale

            distances_np = to_numpy(distances.flatten())

            min_dist_idx = int(np.argmin(distances_np))
            if distances_np[min_dist_idx] > close_threshold:
                chosen_idx = min_dist_idx
            else:
                chosen_idx = min(min_dist_idx + 1, len(subgoal_images) - 1)

            chosen_wp = waypoints[chosen_idx : chosen_idx + 1, :, 0:3]
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(chosen_wp, step=0.1)

            return waypoints[:, :, 0:3], trajectory, distances_np, chosen_idx

    def step_nogoal(self, images: List[np.ndarray]) -> Tuple[torch.Tensor, torch.Tensor]:
        with torch.no_grad():
            self.callback_obs(images)

            fake_goal = torch.randn((len(images), 3, self.image_size[1], self.image_size[0])).to(self.device)

            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            distances, waypoints = self.gnm_policy.predict_nogoal_distance_and_action(input_image, fake_goal)

            if self.normalize:
                waypoints[:, :, :2] *= self.denorm_scale

            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(waypoints[:, :, 0:3], step=0.1)
            return waypoints[:, :, 0:3], trajectory


class NoGoalGNMAgent(BaseAgent):
    """GNM Agent specifically for exploration without goal conditioning."""

    def __init__(
        self,
        model_path: str,
        model_config_path: str,
        device: str = "cuda:0",
    ):
        super().__init__(model_config_path, device)

        self.model_path = model_path

        self.gnm_policy = NoGoalGNMPolicy(self.cfg)
        self.gnm_policy.to(self.device)

        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        if isinstance(checkpoint, dict) and "model" in checkpoint:
            model_data = checkpoint["model"]
            if hasattr(model_data, "state_dict"):
                self.gnm_policy.model.load_state_dict(model_data.state_dict(), strict=True)
            else:
                self.gnm_policy.model.load_state_dict(model_data, strict=True)
        elif hasattr(checkpoint, "state_dict"):
            self.gnm_policy.model.load_state_dict(checkpoint.state_dict(), strict=True)
        else:
            self.gnm_policy.model.load_state_dict(checkpoint, strict=True)
        self.gnm_policy.eval()

        self.traj_generate = TrajOpt()

    def step_nogoal(self, images: List[np.ndarray]) -> Tuple[torch.Tensor, torch.Tensor]:
        with torch.no_grad():
            self.callback_obs(images)

            input_image = [
                transform_images(imgs, self.image_size, center_crop=False).to(self.device) for imgs in self.memory_queue
            ]
            input_image = torch.concat(input_image, dim=0)

            waypoints = self.gnm_policy.predict_nogoal_distance_and_action(input_image)

            if self.normalize:
                waypoints[:, :, :2] *= self.denorm_scale

            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(waypoints[:, :, 0:3], step=0.1)
            return waypoints[:, :, 0:3], trajectory
