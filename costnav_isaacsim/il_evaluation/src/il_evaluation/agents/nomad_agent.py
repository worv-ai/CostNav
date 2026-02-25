# Copyright (c) 2026 CostNav Authors
# Adapted from NavDP Framework (https://github.com/InternRobotics/NavDP)
# Reference: third_party/visualnav-transformer/deployment/src/navigate.py
# Licensed under the MIT License

"""NoMaD Agent for diffusion-based navigation."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import torch
from PIL import Image as PILImage
from torchvision import transforms

from il_evaluation.agents.base_agent import BaseAgent
from il_evaluation.models.traj_opt import TrajOpt


def _ensure_diffusion_policy_on_path() -> None:
    """Add third_party/diffusion_policy to sys.path if available."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "third_party" / "diffusion_policy"
        if candidate.exists():
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
            return


_ensure_diffusion_policy_on_path()

# Local imports that depend on diffusion_policy and diffusers.
try:
    from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
except Exception as exc:  # pragma: no cover - surfaced at runtime
    raise ImportError(
        "diffusers is required for NoMaD evaluation. "
        "Please install it in the ROS2 container (rebuild the image if needed)."
    ) from exc

try:
    from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
except Exception as exc:  # pragma: no cover - surfaced at runtime
    raise ImportError(
        "diffusion_policy is required for NoMaD evaluation. "
        "Ensure third_party/diffusion_policy is present and importable."
    ) from exc

from vint_train.models.nomad.nomad import DenseNetwork, NoMaD
from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn


def _transform_images(pil_imgs: List[PILImage.Image], image_size: List[int]) -> torch.Tensor:
    """Transform PIL images to normalized torch tensor.

    Returns tensor of shape [1, N*3, H, W].
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
        pil_img = pil_img.resize(image_size)
        transf_img = transform_type(pil_img)
        transf_img = torch.unsqueeze(transf_img, 0)
        transf_imgs.append(transf_img)
    return torch.cat(transf_imgs, dim=1)


class NoMaDAgent(BaseAgent):
    """NoMaD Agent for topomap / image-goal navigation.

    Args:
        model_path: Path to trained NoMaD model weights.
        model_config_path: Path to model configuration YAML.
        device: PyTorch device for inference.
    """

    def __init__(
        self,
        model_path: str,
        model_config_path: str,
        device: str = "cuda:0",
    ):
        super().__init__(model_config_path, device)

        self.model_path = model_path

        cfg = self.cfg
        self.model_type = cfg.get("model_type", "nomad")
        self.vision_encoder = cfg.get("vision_encoder", "nomad_vint")
        self.encoding_size = cfg.get("encoding_size", 256)
        self.context_size = cfg.get("context_size", 3)
        self.len_traj_pred = cfg.get("len_traj_pred", 8)
        self.learn_angle = cfg.get("learn_angle", False)
        self.obs_encoder = cfg.get("obs_encoder", "efficientnet-b0")
        self.num_diffusion_iters = cfg.get("num_diffusion_iters", 10)
        self.num_samples = cfg.get("num_samples", 8)
        self.beta_schedule = cfg.get("beta_schedule", "squaredcos_cap_v2")
        self.down_dims = cfg.get("down_dims", [64, 128, 256])
        self.cond_predict_scale = cfg.get("cond_predict_scale", False)
        self.mha_num_attention_heads = cfg.get("mha_num_attention_heads", 4)
        self.mha_num_attention_layers = cfg.get("mha_num_attention_layers", 4)
        self.mha_ff_dim_factor = cfg.get("mha_ff_dim_factor", 4)
        self.patch_size = cfg.get("patch_size", 8)

        self.action_stats = self._load_action_stats(cfg)

        self._init_model()

        # Trajectory optimizer for smooth paths
        self.traj_generate = TrajOpt()

    def _load_action_stats(self, cfg: Dict) -> Dict[str, np.ndarray]:
        """Load action stats used to unnormalize diffusion outputs."""
        if "action_stats" in cfg:
            stats = cfg["action_stats"]
            return {
                "min": np.array(stats["min"], dtype=np.float32),
                "max": np.array(stats["max"], dtype=np.float32),
            }

        # Fallback: use data_config.yaml from visualnav-transformer if present
        data_cfg_path = (
            Path(__file__).resolve().parents[5]
            / "third_party"
            / "visualnav-transformer"
            / "train"
            / "vint_train"
            / "data"
            / "data_config.yaml"
        )
        if data_cfg_path.exists():
            import yaml

            with open(data_cfg_path, "r") as f:
                data_cfg = yaml.safe_load(f)
            stats = data_cfg.get("action_stats", None)
            if stats is not None:
                return {
                    "min": np.array(stats["min"], dtype=np.float32),
                    "max": np.array(stats["max"], dtype=np.float32),
                }

        # Safe default (should be overridden with correct dataset stats)
        return {
            "min": np.array([-2.5, -4.0], dtype=np.float32),
            "max": np.array([5.0, 4.0], dtype=np.float32),
        }

    def _init_model(self) -> None:
        """Initialize NoMaD model and diffusion scheduler."""
        if self.vision_encoder == "nomad_vint":
            vision_encoder = NoMaD_ViNT(
                obs_encoding_size=self.encoding_size,
                context_size=self.context_size,
                obs_encoder=self.obs_encoder,
                mha_num_attention_heads=self.mha_num_attention_heads,
                mha_num_attention_layers=self.mha_num_attention_layers,
                mha_ff_dim_factor=self.mha_ff_dim_factor,
            )
            vision_encoder = replace_bn_with_gn(vision_encoder)
        elif self.vision_encoder == "vit":
            from vint_train.models.vint.vit import ViT

            vision_encoder = ViT(
                obs_encoding_size=self.encoding_size,
                context_size=self.context_size,
                image_size=self.image_size,
                patch_size=self.patch_size,
                mha_num_attention_heads=self.mha_num_attention_heads,
                mha_num_attention_layers=self.mha_num_attention_layers,
            )
            vision_encoder = replace_bn_with_gn(vision_encoder)
        else:
            raise ValueError(f"Vision encoder {self.vision_encoder} not supported")

        noise_pred_net = ConditionalUnet1D(
            input_dim=2,
            global_cond_dim=self.encoding_size,
            down_dims=self.down_dims,
            cond_predict_scale=self.cond_predict_scale,
        )
        dist_pred_net = DenseNetwork(embedding_dim=self.encoding_size)

        self.model = NoMaD(
            vision_encoder=vision_encoder,
            noise_pred_net=noise_pred_net,
            dist_pred_net=dist_pred_net,
        )

        checkpoint = torch.load(self.model_path, map_location=self.device, weights_only=False)
        state_dict = None
        if isinstance(checkpoint, dict):
            if "model" in checkpoint and hasattr(checkpoint["model"], "state_dict"):
                state_dict = checkpoint["model"].state_dict()
            elif "state_dict" in checkpoint:
                state_dict = checkpoint["state_dict"]
            elif "model" in checkpoint and isinstance(checkpoint["model"], dict):
                state_dict = checkpoint["model"]
            else:
                state_dict = checkpoint
        else:
            state_dict = checkpoint
        self.model.load_state_dict(state_dict, strict=False)

        self.model.to(self.device)
        self.model.eval()

        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.num_diffusion_iters,
            beta_schedule=self.beta_schedule,
            clip_sample=True,
            prediction_type="epsilon",
        )
        # Move scheduler buffers to the same device as the model if supported.
        if hasattr(self.noise_scheduler, "to"):
            self.noise_scheduler = self.noise_scheduler.to(self.device)

    # ------------------------------------------------------------------
    # Action sampling helpers
    # ------------------------------------------------------------------

    def _unnormalize_actions(self, ndeltas: np.ndarray) -> np.ndarray:
        """Convert normalized deltas to metric actions and cumulative waypoints."""
        stats_min = self.action_stats["min"].reshape(1, 1, -1)
        stats_max = self.action_stats["max"].reshape(1, 1, -1)

        ndata = (ndeltas + 1.0) / 2.0
        data = ndata * (stats_max - stats_min) + stats_min
        actions = np.cumsum(data, axis=1)
        return actions

    def _get_action(self, diffusion_output: torch.Tensor) -> torch.Tensor:
        ndeltas = diffusion_output.reshape(diffusion_output.shape[0], -1, 2)
        ndeltas_np = ndeltas.detach().cpu().numpy()
        actions = self._unnormalize_actions(ndeltas_np)
        return torch.from_numpy(actions).to(diffusion_output.device)

    def _sample_actions(self, obs_cond: torch.Tensor) -> torch.Tensor:
        """Sample action trajectories from the diffusion policy."""
        if obs_cond.dim() == 2:
            obs_cond = obs_cond.repeat(self.num_samples, 1)
        elif obs_cond.dim() == 3:
            obs_cond = obs_cond.repeat(self.num_samples, 1, 1)
        else:
            raise ValueError(f"Unexpected obs_cond shape: {obs_cond.shape}")

        naction = torch.randn((self.num_samples, self.len_traj_pred, 2), device=self.device)

        try:
            self.noise_scheduler.set_timesteps(self.num_diffusion_iters, device=self.device)
        except TypeError:
            # Older diffusers versions don't accept device in set_timesteps.
            self.noise_scheduler.set_timesteps(self.num_diffusion_iters)
        for t in self.noise_scheduler.timesteps:
            timestep = t.to(self.device) if torch.is_tensor(t) else t
            noise_pred = self.model(
                "noise_pred_net",
                sample=naction,
                timestep=timestep,
                global_cond=obs_cond,
            )
            naction = self.noise_scheduler.step(
                model_output=noise_pred,
                timestep=timestep,
                sample=naction,
            ).prev_sample

        actions = self._get_action(naction)
        return actions

    # ------------------------------------------------------------------
    # Public inference APIs (matching ViNTAgent style)
    # ------------------------------------------------------------------

    def step_imagegoal(
        self, goal_images: List[np.ndarray], images: List[np.ndarray]
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Generate trajectory towards a single goal image."""
        with torch.no_grad():
            self.callback_obs(images)

            goal_image = [
                _transform_images(PILImage.fromarray(g_img), self.image_size).to(self.device) for g_img in goal_images
            ]
            goal_image = torch.concat(goal_image, dim=0)

            input_image = [_transform_images(imgs, self.image_size).to(self.device) for imgs in self.memory_queue]
            input_image = torch.concat(input_image, dim=0)

            goal_mask = torch.zeros((goal_image.shape[0],), dtype=torch.long, device=self.device)
            obsgoal_cond = self.model(
                "vision_encoder",
                obs_img=input_image,
                goal_img=goal_image,
                input_goal_mask=goal_mask,
            )
            distances = self.model("dist_pred_net", obsgoal_cond=obsgoal_cond).flatten()

            actions = self._sample_actions(obsgoal_cond)
            chosen = actions[0:1]
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(chosen, step=0.1)
            return chosen, trajectory, distances

    def step_nogoal(self, images: List[np.ndarray]) -> Tuple[torch.Tensor, torch.Tensor]:
        """Fallback exploration mode without a goal.

        This uses the current observation as a dummy goal with goal masking
        enabled to encourage exploration-style sampling.
        """
        with torch.no_grad():
            self.callback_obs(images)
            input_image = [_transform_images(imgs, self.image_size).to(self.device) for imgs in self.memory_queue]
            input_image = torch.concat(input_image, dim=0)

            dummy_goal = input_image[:, -3:, :, :]
            goal_mask = torch.ones((dummy_goal.shape[0],), dtype=torch.long, device=self.device)
            obs_cond = self.model(
                "vision_encoder",
                obs_img=input_image,
                goal_img=dummy_goal,
                input_goal_mask=goal_mask,
            )
            actions = self._sample_actions(obs_cond)
            chosen = actions[0:1]
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(chosen, step=0.1)
            return chosen, trajectory

    def step_topomap(
        self,
        images: List[np.ndarray],
        subgoal_images: List[PILImage.Image],
        close_threshold: float = 3.0,
    ) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray, int]:
        """Run batched inference over a topomap window."""
        with torch.no_grad():
            self.callback_obs(images)

            input_image = [_transform_images(imgs, self.image_size).to(self.device) for imgs in self.memory_queue]
            input_image = torch.concat(input_image, dim=0)  # [1, C*(ctx+1), H, W]

            goal_tensors = [_transform_images(sg_img, self.image_size).to(self.device) for sg_img in subgoal_images]
            goal_batch = torch.concat(goal_tensors, dim=0)

            obs_batch = input_image.repeat(len(subgoal_images), 1, 1, 1)
            goal_mask = torch.zeros((goal_batch.shape[0],), dtype=torch.long, device=self.device)
            obsgoal_cond = self.model(
                "vision_encoder",
                obs_img=obs_batch,
                goal_img=goal_batch,
                input_goal_mask=goal_mask,
            )
            dists = self.model("dist_pred_net", obsgoal_cond=obsgoal_cond)
            dists_np = dists.detach().cpu().numpy().flatten()

            min_idx = int(np.argmin(dists_np))
            sg_idx = min(min_idx + int(dists_np[min_idx] < close_threshold), len(obsgoal_cond) - 1)

            obs_cond = obsgoal_cond[sg_idx].unsqueeze(0)
            actions = self._sample_actions(obs_cond)
            chosen = actions[0:1]
            trajectory = self.traj_generate.TrajGeneratorFromPFreeRot(chosen, step=0.1)

            return chosen, trajectory, dists_np, sg_idx
