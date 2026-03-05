#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""DepthAnything wrapper for eval-time RGB->depth inference."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def _find_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "third_party" / "InternNav").exists():
            return parent
    raise RuntimeError("Could not locate repo root (third_party/InternNav not found)")


class DepthAnythingEstimator:
    """Lightweight DepthAnything runtime wrapper for eval-time RGB->depth inference."""

    def __init__(
        self,
        checkpoint: str,
        encoder: str = "vitb",
        input_size: int = 518,
        max_depth: float = 20.0,
        device: str = "cuda:0",
    ) -> None:
        import cv2
        import torch

        checkpoint_path = Path(checkpoint).expanduser()
        if not checkpoint_path.exists():
            raise FileNotFoundError(f"DepthAnything checkpoint not found: {checkpoint_path}")

        repo_root = _find_repo_root()
        internnav_dir = repo_root / "third_party" / "InternNav"
        depth_anything_root = internnav_dir / "internnav" / "model" / "encoder" / "depth_anything"
        if str(depth_anything_root) not in sys.path:
            sys.path.insert(0, str(depth_anything_root))
        try:
            from depth_anything_v2.dpt import DepthAnythingV2
        except Exception:
            if str(internnav_dir) not in sys.path:
                sys.path.insert(0, str(internnav_dir))
            from internnav.model.encoder.depth_anything.depth_anything_v2.dpt import DepthAnythingV2

        model_configs = {
            "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
            "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
            "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
            "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
        }
        if encoder not in model_configs:
            raise ValueError(f"Unsupported depth_anything encoder: {encoder}")

        self.cv2 = cv2
        self.input_size = int(input_size)

        model = DepthAnythingV2(max_depth=float(max_depth), **model_configs[encoder])
        state_dict = torch.load(str(checkpoint_path), map_location="cpu")
        model.load_state_dict(state_dict, strict=False)
        model.eval()

        requested = str(device)
        if requested.startswith("cuda") and torch.cuda.is_available():
            model = model.to(requested)
        elif requested.startswith("mps") and hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            model = model.to("mps")
        elif torch.cuda.is_available():
            model = model.to("cuda")
        else:
            model = model.to("cpu")
        self.model = model

    def infer(self, rgb_image: np.ndarray) -> np.ndarray:
        # DepthAnything expects BGR uint8 image (OpenCV convention).
        bgr = self.cv2.cvtColor(rgb_image, self.cv2.COLOR_RGB2BGR)
        depth = self.model.infer_image(bgr, input_size=self.input_size).astype(np.float32)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth[depth < 0.0] = 0.0
        return depth
