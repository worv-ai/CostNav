#!/usr/bin/env python
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

try:
    import yaml
except Exception as exc:  # pragma: no cover - runtime guard
    raise SystemExit(f"ERROR: PyYAML is required. Install deps in costnav_isaacsim/il_training. ({exc})")


def _repo_paths() -> tuple[Path, Path, Path]:
    this_file = Path(__file__).resolve()
    costnav_root = this_file.parents[2]  # costnav_isaacsim/
    repo_root = costnav_root.parent
    internnav_dir = repo_root / "third_party" / "InternNav"
    return repo_root, costnav_root, internnav_dir


def _ensure_sys_path(internnav_dir: Path) -> None:
    diff_policy_dir = internnav_dir / "src" / "diffusion-policy"
    for p in (internnav_dir, diff_policy_dir):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"ERROR: config must be a YAML mapping, got {type(data).__name__}")
    return data


def _make_abs(path_str: str, base_dir: Path) -> str:
    path = Path(os.path.expanduser(path_str))
    if not path.is_absolute():
        path = base_dir / path
    return str(path)


def _resolve_paths(cfg: dict, base_dir: Path) -> None:
    top_keys = {"output_dir", "tensorboard_dir", "checkpoint_folder", "log_dir"}
    il_keys = {
        "dataset_navdp",
        "root_dir",
        "lerobot_features_dir",
        "dataset_r2r_root_dir",
        "dataset_3dgs_root_dir",
        "dataset_grutopia10_root_dir",
        "ckpt_to_load",
        "depth_anything_checkpoint",
        "traj_names_path",
    }

    for key in top_keys:
        if isinstance(cfg.get(key), str):
            cfg[key] = _make_abs(cfg[key], base_dir)

    il_cfg = cfg.get("il")
    if isinstance(il_cfg, dict):
        for key in il_keys:
            if isinstance(il_cfg.get(key), str):
                il_cfg[key] = _make_abs(il_cfg[key], base_dir)


def _apply_overrides(obj, overrides: dict) -> None:
    for key, value in overrides.items():
        if isinstance(value, dict):
            current = getattr(obj, key, None)
            if current is None:
                setattr(obj, key, value)
            else:
                _apply_overrides(current, value)
        else:
            setattr(obj, key, value)


def _clone_cfg(cfg):
    if hasattr(cfg, "model_copy"):  # pydantic v2
        return cfg.model_copy(deep=True)
    return cfg.copy(deep=True)


def _derive_gpu_ids() -> list[int]:
    visible = os.environ.get("CUDA_VISIBLE_DEVICES", "").strip()
    if not visible:
        return [0]
    ids = [v.strip() for v in visible.split(",") if v.strip() != ""]
    return list(range(len(ids))) if ids else [0]


def main() -> None:
    parser = argparse.ArgumentParser(description="CostNav wrapper for InternNav NavDP training")
    repo_root, costnav_root, internnav_dir = _repo_paths()
    default_cfg = costnav_root / "il_training" / "training" / "configs" / "navdp_costnav.yaml"
    parser.add_argument("--config", default=str(default_cfg), help="Path to navdp_costnav.yaml")
    parser.add_argument("--name", default=None, help="Override experiment name")
    args = parser.parse_args()

    if not internnav_dir.exists():
        raise SystemExit(f"ERROR: InternNav not found at {internnav_dir}")

    _ensure_sys_path(internnav_dir)

    cfg_path = Path(args.config).expanduser()
    if not cfg_path.is_absolute():
        cwd_candidate = (Path.cwd() / cfg_path).resolve()
        repo_candidate = (costnav_root / cfg_path).resolve()
        cfg_path = cwd_candidate if cwd_candidate.exists() else repo_candidate
    if not cfg_path.exists():
        raise SystemExit(f"ERROR: config not found: {cfg_path}")

    cfg = _load_yaml(cfg_path)
    _resolve_paths(cfg, base_dir=costnav_root)

    # Set the local CUDA device before importing InternNav modules. Some imports touch
    # CUDA and would otherwise initialize on the default GPU (cuda:0) in every rank.
    local_rank_env = os.environ.get("LOCAL_RANK")
    if local_rank_env is not None:
        try:
            import torch

            local_rank = int(local_rank_env)
            if torch.cuda.is_available() and 0 <= local_rank < torch.cuda.device_count():
                torch.cuda.set_device(local_rank)
                print(f"[bootstrap] LOCAL_RANK={local_rank}: set CUDA device before model import")
        except Exception as exc:
            print(f"[bootstrap] WARNING: failed to set CUDA device early: {exc}")

    from scripts.train.base_train import train as train_module
    from scripts.train.base_train.configs import navdp_exp_cfg
    from internnav.model import get_config, get_policy

    exp_cfg = _clone_cfg(navdp_exp_cfg)

    _apply_overrides(exp_cfg, cfg)
    if args.name:
        exp_cfg.name = args.name

    exp_cfg.model_name = "navdp"

    if not getattr(exp_cfg, "torch_gpu_ids", None):
        exp_cfg.torch_gpu_ids = _derive_gpu_ids()
    if not getattr(exp_cfg, "torch_gpu_id", None):
        exp_cfg.torch_gpu_id = exp_cfg.torch_gpu_ids[0]

    # Match InternNav train.py expectations
    exp_cfg.num_gpus = len(exp_cfg.torch_gpu_ids)
    exp_cfg.world_size = exp_cfg.num_gpus

    import torch

    available_gpus = torch.cuda.device_count() if torch.cuda.is_available() else 1
    assert exp_cfg.num_gpus <= available_gpus, (
        f"Requested GPUs ({exp_cfg.num_gpus}) > available GPUs ({available_gpus})"
    )
    assert exp_cfg.num_gpus > 0, "Number of GPUs must be greater than 0"

    model_class, model_config_class = get_policy("NavDP_Policy"), get_config("NavDP_Policy")
    train_module.main(exp_cfg, model_class, model_config_class)


if __name__ == "__main__":
    main()
