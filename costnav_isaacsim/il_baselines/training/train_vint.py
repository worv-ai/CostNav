"""
MIT License

Copyright (c) 2023 Dhruv Shah, Ajay Sridhar, Nitish Dashora, Kyle Stachowicz, Kevin Black, Noriaki Hirose, Sergey Levine

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This code is adapted from https://github.com/robodhruv/visualnav-transformer
"""

import argparse
import os
import random
import re
import time
from pathlib import Path

import numpy as np
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
import wandb
import yaml
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from dotenv import load_dotenv
from torch.optim import Adam, AdamW
from torch.utils.data import ConcatDataset, DataLoader
from torchvision import transforms
from warmup_scheduler import GradualWarmupScheduler

"""
IMPORT YOUR MODEL HERE
"""
from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
from vint_train.data.vint_dataset import ViNT_Dataset
from vint_train.models.gnm.gnm import GNM
from vint_train.models.nomad.nomad import DenseNetwork, NoMaD
from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn
from vint_train.models.vint.vint import ViNT
from vint_train.models.vint.vit import ViT
from vint_train.training.train_eval_loop import (
    load_model,
    train_eval_loop,
    train_eval_loop_nomad,
)

# Load .env file - required for PROJECT_ROOT
load_dotenv()

# Load PROJECT_ROOT from environment (must be set in .env)
if "PROJECT_ROOT" not in os.environ:
    raise EnvironmentError(
        "PROJECT_ROOT environment variable is not set. "
        "Please set PROJECT_ROOT in your .env file at the project root. "
        "Example: PROJECT_ROOT=/path/to/CostNav"
    )
PROJECT_ROOT = Path(os.environ["PROJECT_ROOT"])


def resolve_env_variables(config: dict, project_root: str) -> dict:
    """Recursively resolve ${VAR} placeholders in config with environment variables.

    Args:
        config: Configuration dictionary with potential ${VAR} placeholders.
        project_root: Project root path to use for ${PROJECT_ROOT} resolution.

    Returns:
        Config dictionary with all placeholders resolved.
    """
    env_pattern = re.compile(r"\$\{(\w+)\}")

    def resolve_value(value):
        if isinstance(value, str):

            def replace_match(match):
                var_name = match.group(1)
                if var_name == "PROJECT_ROOT":
                    return project_root
                elif var_name == "USERNAME":
                    # Get USERNAME from environment, fallback to USER if not set
                    return os.environ.get("USERNAME", os.environ.get("USER", ""))
                return os.environ.get(var_name, "")

            return env_pattern.sub(replace_match, value)
        elif isinstance(value, dict):
            return {k: resolve_value(v) for k, v in value.items()}
        elif isinstance(value, list):
            return [resolve_value(item) for item in value]
        else:
            return value

    return resolve_value(config)


def update_data_config_yaml(config):
    """Update the data_config.yaml file with dataset-specific parameters.

    This function updates the third_party/visualnav-transformer/train/vint_train/data/data_config.yaml
    file with metric_waypoint_spacing and other parameters for each dataset specified in the config.

    Args:
        config: Training configuration dictionary containing dataset parameters.
    """
    data_config_path = (
        PROJECT_ROOT / "third_party" / "visualnav-transformer" / "train" / "vint_train" / "data" / "data_config.yaml"
    )

    # Load existing data_config.yaml
    with open(data_config_path, "r") as f:
        data_config = yaml.safe_load(f)

    # Update data_config with parameters from training config
    updated = False
    for dataset_name in config["datasets"]:
        dataset_config = config["datasets"][dataset_name]

        # Check if metric_waypoint_spacing is specified in the training config
        if "metric_waypoint_spacing" in dataset_config:
            # Create or update the dataset entry in data_config.yaml
            if dataset_name not in data_config:
                data_config[dataset_name] = {}

            data_config[dataset_name]["metric_waypoint_spacing"] = dataset_config["metric_waypoint_spacing"]
            updated = True
            print(
                f"Updated data_config.yaml: {dataset_name}.metric_waypoint_spacing = {dataset_config['metric_waypoint_spacing']}"
            )

    # Write back to data_config.yaml if any updates were made
    if updated:
        with open(data_config_path, "w") as f:
            yaml.dump(data_config, f, default_flow_style=False, sort_keys=False)
        print(f"Successfully updated {data_config_path}")


def _generate_data_splits(
    data_folder: str,
    train_dir: str,
    test_dir: str,
    split_ratio: float = 0.8,
    seed: int = 42,
) -> None:
    """Auto-generate train/test split files (traj_names.txt).

    Scans *data_folder* for sub-directories that contain ``traj_data.pkl``
    and writes the trajectory names into ``train_dir/traj_names.txt`` and
    ``test_dir/traj_names.txt``.  The split is deterministic for a given
    *seed*.
    """
    folder_names = sorted(
        f
        for f in os.listdir(data_folder)
        if os.path.isdir(os.path.join(data_folder, f))
        and os.path.exists(os.path.join(data_folder, f, "traj_data.pkl"))
    )
    if not folder_names:
        raise FileNotFoundError(f"No trajectory folders (with traj_data.pkl) found in {data_folder}")

    rng = random.Random(seed)
    rng.shuffle(folder_names)

    split_idx = int(split_ratio * len(folder_names))
    train_names = folder_names[:split_idx]
    test_names = folder_names[split_idx:]

    for dir_path, names in [(train_dir, train_names), (test_dir, test_names)]:
        if not dir_path:
            continue
        os.makedirs(dir_path, exist_ok=True)
        with open(os.path.join(dir_path, "traj_names.txt"), "w") as f:
            f.write("\n".join(names) + "\n")
        print(f"  Wrote {len(names)} trajectories to {dir_path}/traj_names.txt")


def main(config):
    assert config["distance"]["min_dist_cat"] < config["distance"]["max_dist_cat"]
    assert config["action"]["min_dist_cat"] < config["action"]["max_dist_cat"]

    # Update data_config.yaml with dataset parameters from training config
    update_data_config_yaml(config)

    if torch.cuda.is_available():
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
        if "gpu_ids" not in config:
            config["gpu_ids"] = [0]
        elif isinstance(config["gpu_ids"], int):
            config["gpu_ids"] = [config["gpu_ids"]]
        os.environ["CUDA_VISIBLE_DEVICES"] = ",".join([str(x) for x in config["gpu_ids"]])
        print("Using cuda devices:", os.environ["CUDA_VISIBLE_DEVICES"])
    else:
        print("Using cpu")

    first_gpu_id = config["gpu_ids"][0]
    device = torch.device(f"cuda:{first_gpu_id}" if torch.cuda.is_available() else "cpu")

    if "seed" in config:
        np.random.seed(config["seed"])
        torch.manual_seed(config["seed"])
        cudnn.deterministic = True

    cudnn.benchmark = True  # good if input sizes don't vary
    transform = [
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ]
    transform = transforms.Compose(transform)

    # Load the data
    train_dataset = []
    test_dataloaders = {}

    if "context_type" not in config:
        config["context_type"] = "temporal"

    if "clip_goals" not in config:
        config["clip_goals"] = False

    for dataset_name in config["datasets"]:
        data_config = config["datasets"][dataset_name]

        # Auto-generate train/test splits if they don't exist
        for split_key in ["train", "test"]:
            if split_key in data_config:
                split_dir = data_config[split_key]
                traj_names_file = os.path.join(split_dir, "traj_names.txt")
                if not os.path.exists(traj_names_file):
                    print(f"Split file not found: {traj_names_file}")
                    print(f"Auto-generating train/test splits from {data_config['data_folder']}...")
                    _generate_data_splits(
                        data_folder=data_config["data_folder"],
                        train_dir=data_config.get("train", ""),
                        test_dir=data_config.get("test", ""),
                        split_ratio=data_config.get("split_ratio", 0.8),
                        seed=config.get("seed", 42),
                    )
                    break  # both train and test are generated together
        if "negative_mining" not in data_config:
            data_config["negative_mining"] = True
        if "goals_per_obs" not in data_config:
            data_config["goals_per_obs"] = 1
        if "end_slack" not in data_config:
            data_config["end_slack"] = 0
        if "waypoint_spacing" not in data_config:
            data_config["waypoint_spacing"] = 1

        for data_split_type in ["train", "test"]:
            if data_split_type in data_config:
                dataset = ViNT_Dataset(
                    data_folder=data_config["data_folder"],
                    data_split_folder=data_config[data_split_type],
                    dataset_name=dataset_name,
                    image_size=config["image_size"],
                    waypoint_spacing=data_config["waypoint_spacing"],
                    min_dist_cat=config["distance"]["min_dist_cat"],
                    max_dist_cat=config["distance"]["max_dist_cat"],
                    min_action_distance=config["action"]["min_dist_cat"],
                    max_action_distance=config["action"]["max_dist_cat"],
                    negative_mining=data_config["negative_mining"],
                    len_traj_pred=config["len_traj_pred"],
                    learn_angle=config["learn_angle"],
                    context_size=config["context_size"],
                    context_type=config["context_type"],
                    end_slack=data_config["end_slack"],
                    goals_per_obs=data_config["goals_per_obs"],
                    normalize=config["normalize"],
                    goal_type=config["goal_type"],
                )
                if data_split_type == "train":
                    train_dataset.append(dataset)
                else:
                    dataset_type = f"{dataset_name}_{data_split_type}"
                    if dataset_type not in test_dataloaders:
                        test_dataloaders[dataset_type] = {}
                    test_dataloaders[dataset_type] = dataset

    # combine all the datasets from different robots
    train_dataset = ConcatDataset(train_dataset)

    train_loader = DataLoader(
        train_dataset,
        batch_size=config["batch_size"],
        shuffle=True,
        num_workers=config["num_workers"],
        drop_last=False,
        persistent_workers=config["num_workers"] > 0,
    )

    if "eval_batch_size" not in config:
        config["eval_batch_size"] = config["batch_size"]

    for dataset_type, dataset in test_dataloaders.items():
        test_dataloaders[dataset_type] = DataLoader(
            dataset,
            batch_size=config["eval_batch_size"],
            shuffle=True,
            num_workers=0,
            drop_last=False,
        )

    # Create the model
    if config["model_type"] == "gnm":
        model = GNM(
            config["context_size"],
            config["len_traj_pred"],
            config["learn_angle"],
            config["obs_encoding_size"],
            config["goal_encoding_size"],
        )
    elif config["model_type"] == "vint":
        model = ViNT(
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
    elif config["model_type"] == "nomad":
        if config["vision_encoder"] == "nomad_vint":
            vision_encoder = NoMaD_ViNT(
                obs_encoding_size=config["encoding_size"],
                context_size=config["context_size"],
                mha_num_attention_heads=config["mha_num_attention_heads"],
                mha_num_attention_layers=config["mha_num_attention_layers"],
                mha_ff_dim_factor=config["mha_ff_dim_factor"],
            )
            vision_encoder = replace_bn_with_gn(vision_encoder)
        elif config["vision_encoder"] == "vib":
            raise NotImplementedError("ViB vision encoder is not implemented yet")
        elif config["vision_encoder"] == "vit":
            vision_encoder = ViT(
                obs_encoding_size=config["encoding_size"],
                context_size=config["context_size"],
                image_size=config["image_size"],
                patch_size=config["patch_size"],
                mha_num_attention_heads=config["mha_num_attention_heads"],
                mha_num_attention_layers=config["mha_num_attention_layers"],
            )
            vision_encoder = replace_bn_with_gn(vision_encoder)
        else:
            raise ValueError(f"Vision encoder {config['vision_encoder']} not supported")

        noise_pred_net = ConditionalUnet1D(
            input_dim=2,
            global_cond_dim=config["encoding_size"],
            down_dims=config["down_dims"],
            cond_predict_scale=config["cond_predict_scale"],
        )
        dist_pred_network = DenseNetwork(embedding_dim=config["encoding_size"])

        model = NoMaD(
            vision_encoder=vision_encoder,
            noise_pred_net=noise_pred_net,
            dist_pred_net=dist_pred_network,
        )

        noise_scheduler = DDPMScheduler(
            num_train_timesteps=config["num_diffusion_iters"],
            beta_schedule="squaredcos_cap_v2",
            clip_sample=True,
            prediction_type="epsilon",
        )
    else:
        raise ValueError(f"Model {config['model']} not supported")

    if config["clipping"]:
        print("Clipping gradients to", config["max_norm"])
        for p in model.parameters():
            if not p.requires_grad:
                continue
            p.register_hook(lambda grad: torch.clamp(grad, -1 * config["max_norm"], config["max_norm"]))

    lr = float(config["lr"])
    config["optimizer"] = config["optimizer"].lower()
    if config["optimizer"] == "adam":
        optimizer = Adam(model.parameters(), lr=lr, betas=(0.9, 0.98))
    elif config["optimizer"] == "adamw":
        optimizer = AdamW(model.parameters(), lr=lr)
    elif config["optimizer"] == "sgd":
        optimizer = torch.optim.SGD(model.parameters(), lr=lr, momentum=0.9)
    else:
        raise ValueError(f"Optimizer {config['optimizer']} not supported")

    scheduler = None
    if config["scheduler"] is not None:
        config["scheduler"] = config["scheduler"].lower()
        if config["scheduler"] == "cosine":
            print("Using cosine annealing with T_max", config["epochs"])
            scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=config["epochs"])
        elif config["scheduler"] == "cyclic":
            print("Using cyclic LR with cycle", config["cyclic_period"])
            scheduler = torch.optim.lr_scheduler.CyclicLR(
                optimizer,
                base_lr=lr / 10.0,
                max_lr=lr,
                step_size_up=config["cyclic_period"] // 2,
                cycle_momentum=False,
            )
        elif config["scheduler"] == "plateau":
            print("Using ReduceLROnPlateau")
            scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                optimizer,
                factor=config["plateau_factor"],
                patience=config["plateau_patience"],
                verbose=True,
            )
        else:
            raise ValueError(f"Scheduler {config['scheduler']} not supported")

        if config["warmup"]:
            print("Using warmup scheduler")
            scheduler = GradualWarmupScheduler(
                optimizer,
                multiplier=1,
                total_epoch=config["warmup_epochs"],
                after_scheduler=scheduler,
            )

    current_epoch = 0
    if "pretrained_checkpoint" in config:
        # Load from a pretrained checkpoint file (for fine-tuning from scratch)
        checkpoint_path = config["pretrained_checkpoint"]
        print("Loading pretrained model from ", checkpoint_path)
        pretrained_checkpoint = torch.load(checkpoint_path, weights_only=False)
        load_model(model, config["model_type"], pretrained_checkpoint)
        # Don't restore epoch - start from 0 for fine-tuning
    elif "load_run" in config:
        load_project_folder = os.path.join("logs", config["load_run"])
        print("Loading model from ", load_project_folder)
        latest_path = os.path.join(load_project_folder, "latest.pth")
        latest_checkpoint = torch.load(latest_path, weights_only=False)
        load_model(model, config["model_type"], latest_checkpoint)
        if "epoch" in latest_checkpoint:
            current_epoch = latest_checkpoint["epoch"] + 1

    # Multi-GPU
    if len(config["gpu_ids"]) > 1:
        model = nn.DataParallel(model, device_ids=config["gpu_ids"])
    model = model.to(device)

    if "load_run" in config:  # load optimizer and scheduler after data parallel
        if "optimizer" in latest_checkpoint:
            optimizer.load_state_dict(latest_checkpoint["optimizer"].state_dict())
        if scheduler is not None and "scheduler" in latest_checkpoint:
            scheduler.load_state_dict(latest_checkpoint["scheduler"].state_dict())

    if config["model_type"] == "vint" or config["model_type"] == "gnm":
        train_eval_loop(
            train_model=config["train"],
            model=model,
            optimizer=optimizer,
            scheduler=scheduler,
            dataloader=train_loader,
            test_dataloaders=test_dataloaders,
            transform=transform,
            epochs=config["epochs"],
            device=device,
            project_folder=config["project_folder"],
            normalized=config["normalize"],
            print_log_freq=config["print_log_freq"],
            image_log_freq=config["image_log_freq"],
            num_images_log=config["num_images_log"],
            current_epoch=current_epoch,
            learn_angle=config["learn_angle"],
            alpha=config["alpha"],
            use_wandb=config["use_wandb"],
            eval_fraction=config["eval_fraction"],
        )
    else:
        train_eval_loop_nomad(
            train_model=config["train"],
            model=model,
            optimizer=optimizer,
            lr_scheduler=scheduler,
            noise_scheduler=noise_scheduler,
            train_loader=train_loader,
            test_dataloaders=test_dataloaders,
            transform=transform,
            goal_mask_prob=config["goal_mask_prob"],
            epochs=config["epochs"],
            device=device,
            project_folder=config["project_folder"],
            print_log_freq=config["print_log_freq"],
            wandb_log_freq=config["wandb_log_freq"],
            image_log_freq=config["image_log_freq"],
            num_images_log=config["num_images_log"],
            current_epoch=current_epoch,
            alpha=float(config["alpha"]),
            use_wandb=config["use_wandb"],
            eval_fraction=config["eval_fraction"],
            eval_freq=config["eval_freq"],
        )

    print("FINISHED TRAINING")


def resolve_relative_paths(config, config_dir):
    """Resolve relative paths in the config relative to the config file directory."""
    # Resolve dataset paths
    if "datasets" in config:
        for dataset_name, dataset_config in config["datasets"].items():
            if "data_folder" in dataset_config:
                data_folder = dataset_config["data_folder"]
                if not os.path.isabs(data_folder):
                    dataset_config["data_folder"] = os.path.normpath(os.path.join(config_dir, data_folder))
            if "train" in dataset_config:
                train_path = dataset_config["train"]
                if not os.path.isabs(train_path):
                    dataset_config["train"] = os.path.normpath(os.path.join(config_dir, train_path))
            if "test" in dataset_config:
                test_path = dataset_config["test"]
                if not os.path.isabs(test_path):
                    dataset_config["test"] = os.path.normpath(os.path.join(config_dir, test_path))

    # Resolve pretrained checkpoint path
    if "pretrained_checkpoint" in config:
        checkpoint_path = config["pretrained_checkpoint"]
        if not os.path.isabs(checkpoint_path):
            config["pretrained_checkpoint"] = os.path.normpath(os.path.join(config_dir, checkpoint_path))

    return config


if __name__ == "__main__":
    torch.multiprocessing.set_start_method("spawn")

    parser = argparse.ArgumentParser(description="Visual Navigation Transformer")

    # project setup
    parser.add_argument(
        "--config",
        "-c",
        default=str(
            PROJECT_ROOT
            / "costnav_isaacsim"
            / "il_baselines"
            / "training"
            / "visualnav_transformer"
            / "configs"
            / "vint_costnav.yaml"
        ),
        type=str,
        help="Path to the config file",
    )
    args = parser.parse_args()

    # Load defaults config
    defaults_path = str(
        PROJECT_ROOT
        / "costnav_isaacsim"
        / "il_baselines"
        / "training"
        / "visualnav_transformer"
        / "configs"
        / "defaults.yaml"
    )
    with open(defaults_path, "r") as f:
        default_config = yaml.safe_load(f)

    config = default_config

    # Load user config
    config_path = os.path.abspath(args.config)
    config_dir = os.path.dirname(config_path)
    with open(config_path, "r") as f:
        user_config = yaml.safe_load(f)

    config.update(user_config)

    # Resolve ${PROJECT_ROOT} and other environment variables in config
    config = resolve_env_variables(config, str(PROJECT_ROOT))

    # Resolve relative paths in the config relative to the config file location
    config = resolve_relative_paths(config, config_dir)

    config["run_name"] += "_" + time.strftime("%Y_%m_%d_%H_%M_%S")

    # Create project folder using configurable log_dir
    # If log_dir is relative, make it relative to PROJECT_ROOT
    log_dir = Path(config.get("log_dir", "logs"))
    if not log_dir.is_absolute():
        log_dir = PROJECT_ROOT / log_dir

    config["project_folder"] = str(log_dir / config["project_name"] / config["run_name"])
    os.makedirs(
        config["project_folder"],  # should error if dir already exists to avoid overwriting and old project
    )

    if config["use_wandb"]:
        wandb.login()
        wandb.init(
            project=config["project_name"],
            dir=str(log_dir),  # Set wandb directory to log_dir
            settings=wandb.Settings(start_method="fork"),
            entity=config.get("wandb_entity", os.environ.get("WANDB_ENTITY", "gnmv2")),
        )
        wandb.save(args.config, policy="now")  # save the config file
        wandb.run.name = config["run_name"]
        # update the wandb args with the training configurations
        if wandb.run:
            wandb.config.update(config)

    print(config)
    main(config)
