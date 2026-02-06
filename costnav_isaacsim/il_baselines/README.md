# CostNav Imitation Learning Baselines

This module implements imitation learning baselines for CostNav sidewalk robot navigation.

## Directory Structure

```
il_baselines/
├── pyproject.toml          # uv project root (data_processing + training)
├── uv.lock
├── README.md               # This file
├── data_processing/        # ROS2 bag → ViNT format conversion
├── training/               # Model training scripts and configs
├── scripts/                # SLURM sbatch scripts
└── evaluation/             # Separate package with its own pyproject.toml
```

### Runtime environments

| Component | Runtime | Python | Install |
| --- | --- | --- | --- |
| `data_processing/`, `training/` | SLURM cluster or bare-metal | ≥3.11 | `uv sync` from `il_baselines/` |
| `evaluation/` | ROS2 Docker container (ROS2 Jazzy) | 3.10 | `uv sync` inside container (own `pyproject.toml`) |

`evaluation/` is **independently installable** with its own `pyproject.toml` and a separate
dependency set (numpy<2 for cv_bridge, casadi, ROS2 bindings). It is excluded from the main
uv project build. You do not need the training/data-processing environment to run evaluation,
and vice versa.

## Environment Setup (uv)

### Prerequisites

- [uv](https://docs.astral.sh/uv/) (`curl -LsSf https://astral.sh/uv/install.sh | sh`)
- CUDA 12.4+ capable GPU and drivers

### Install

```bash
cd CostNav/costnav_isaacsim
uv sync          # creates .venv, installs all deps (PyTorch CUDA 12.4, etc.)
```

This installs `il_baselines` as an editable package so all imports
(`from il_baselines.data_processing.converters...`, `from il_baselines.training...`) work.

### Slurm

Before submitting jobs, configure the YAML config files with your paths:

- `data_processing/configs/processing_config.yaml` — set `input_dir` and `output_dir` for ROS Bag → MediaRef
- `data_processing/configs/vint_processing_config.yaml` — set `input_dir` and `output_dir` for MediaRef → ViNT
- `training/visualnav_transformer/configs/vint_costnav.yaml` — set dataset paths and `log_dir`

```bash
cd costnav_isaacsim/il_baselines/scripts/
sbatch process_data.sbatch
sbatch train_vint.sbatch
```

The sbatch scripts use `uv run` — no manual venv activation needed.

### pyproject.toml overview

The `pyproject.toml` at `costnav_isaacsim/` defines:

| Section               | Purpose                                                                                                          |
| --------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `[project]`           | `costnav-il-baselines`, Python ≥3.11, all runtime deps                                                           |
| `[tool.uv.sources]`   | PyTorch CUDA index, `vint_train` (editable local), `warmup-scheduler` (git), `diffusion_policy` (editable local) |
| `[tool.hatch.build]`  | Exposes `il_baselines` package (excludes `evaluation/`)                                                          |
| `[dependency-groups]` | `dev` group for pytest, ruff, ipython                                                                            |

### Key dependency decisions

| Dep                                  | Source                                                          | Notes                              |
| ------------------------------------ | --------------------------------------------------------------- | ---------------------------------- |
| `torch`, `torchvision`, `torchaudio` | PyTorch CUDA 12.4 index via `[tool.uv]`                         | Replaces conda `pytorch` channel   |
| `vint_train`                         | `third_party/visualnav-transformer/train` (editable)            | Local path source                  |
| `diffusion_policy`                   | `third_party/visualnav-transformer/diffusion_policy` (editable) | Provides `ConditionalUnet1D`       |
| `warmup-scheduler`                   | git+https://github.com/ildoonet/pytorch-gradual-warmup-lr.git   | No PyPI release                    |
| `diffusers`                          | PyPI                                                            | `DDPMScheduler` for NoMaD training |

### Why evaluation is separate

| Concern    | data_processing + training | evaluation                          |
| ---------- | -------------------------- | ----------------------------------- |
| Runtime    | Slurm / bare-metal         | Docker + ROS2 Humble                |
| Python     | 3.11                       | 3.10 (ROS2 constraint)              |
| numpy      | ≥2.0 OK                    | <2 (cv_bridge)                      |
| Extra deps | ray, wandb, hydra          | rclpy, cv_bridge, casadi            |
| Install    | `uv sync`                  | `pip install -e .` inside container |

Evaluation has its own `pyproject.toml` and lives in a separate folder
(`costnav_isaacsim/il_baselines/evaluation/`) to decouple the Docker/ROS2 lifecycle.

## Data Format

### Input: ROS2 MCAP Bags

CostNav rosbag files (`.mcap`) contain the following key topics:

| Topic                                 | Type                            | Description                      |
| ------------------------------------- | ------------------------------- | -------------------------------- |
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/msg/Image`         | Front camera images (RGB, ~30Hz) |
| `/chassis/odom`                       | `nav_msgs/msg/Odometry`         | Odometry data                    |
| `/cmd_vel`                            | `geometry_msgs/msg/Twist`       | Velocity commands                |
| `/chassis/imu`                        | `sensor_msgs/msg/Imu`           | IMU data                         |
| `/front_3d_lidar/lidar_points`        | `sensor_msgs/msg/PointCloud2`   | 3D LiDAR                         |
| `/scan`                               | `sensor_msgs/msg/LaserScan`     | 2D LiDAR scan                    |
| `/goal_pose`                          | `geometry_msgs/msg/PoseStamped` | Navigation goal                  |

### Output: MediaRef Format

```
dataset_name/
└── bagfile_name/
    ├── metadata.json            # Dataset metadata
    ├── bagfile_name.media/      # Extracted video files
    │   ├── front_stereo_camera_left_image_raw.mp4
    │   └── ...
    └── bagfile_name.mcap        # Processed trajectory data
```

## Usage

All commands below assume you have run `uv sync` from `costnav_isaacsim/`.

### Data Conversion

#### Option 1: Using Configuration Files (Recommended)

The easiest way to run the data processing pipeline is to configure paths in the YAML config files. This allows you to specify all paths and parameters once and reuse them.

**1. Edit the configuration files:**

Edit `data_processing/configs/processing_config.yaml` for Step 1 (ROS Bag → MediaRef):
Edit `data_processing/configs/vint_processing_config.yaml` for Step 2 (MediaRef → ViNT):

**2. Run the processing pipeline:**

```bash
# From CostNav/costnav_isaacsim/il_baselines/
# Step 1: ROS Bag → MediaRef
uv run python -m il_baselines.data_processing.converters.ray_batch_convert \
    --config data_processing/configs/processing_config.yaml

# Step 2: MediaRef → ViNT
uv run python -m il_baselines.data_processing.process_data.process_mediaref_bags \
    --config data_processing/configs/vint_processing_config.yaml
```

#### Option 2: SLURM Job Submission

For running data processing on a SLURM cluster, use the provided sbatch script. This is for **job allocation only** and will submit the processing pipeline to the cluster's job queue.

**1. Edit the configuration files:**

Edit `data_processing/configs/processing_config.yaml` for Step 1 (ROS Bag → MediaRef):
Edit `data_processing/configs/vint_processing_config.yaml` for Step 2 (MediaRef → ViNT):

**2. Submit the job to the cluster:**

```bash
cd costnav_isaacsim/il_baselines/scripts/
sbatch process_data.sbatch
```

The sbatch script will automatically read paths from the config files and execute both processing steps (ROS Bag → MediaRef → ViNT) as a batch job.

**Note:** This does not run the processing immediately like Options 1 and 3. Instead, it allocates resources and queues the job on the SLURM cluster.

#### Option 3: Command-Line Arguments Override

You can override config file paths with command-line arguments:

**Step 1: ROS Bag → MediaRef Format** (parallel batch conversion)

```bash
# From CostNav/costnav_isaacsim/
uv run python -m il_baselines.data_processing.converters.ray_batch_convert \
    --input_dir ../data/sample_rosbags/ \
    --output_dir ../data/mediaref/ \
    --config data_processing/configs/processing_config.yaml \
    --num_workers 8
```

Command-line arguments take precedence over config file settings. `ray_batch_convert.py` extracts image topics from ROS2 bags and encodes them as MP4 videos with MediaRef pointers. This provides significant storage reduction while keeping odometry/cmd data in MCAP format. Supports skipping already-processed bags via `_SUCCESS` markers.

**Step 2: MediaRef → ViNT Training Format**

```bash
uv run python -m il_baselines.data_processing.process_data.process_mediaref_bags \
    --input-dir ../data/mediaref/ \
    --output-dir ../data/vint_format/ \
    --config data_processing/configs/vint_processing_config.yaml \
    --sample-rate 4.0
```

Command-line arguments take precedence over config file settings. `process_mediaref_bags.py` reads MediaRef bags (video + MCAP), synchronizes images with odometry at a specified rate, filters out backward-moving segments, and outputs ViNT-format trajectory folders.

**Output Format:**

```
data/outputs/
├── recording_20260102_014822_0/    # Trajectory folder
│   ├── 0.jpg                       # Frame at t=0
│   ├── 1.jpg                       # Frame at t=0.25s (at 4Hz)
│   ├── 2.jpg                       # Frame at t=0.5s
│   ├── ...
│   └── traj_data.pkl               # Trajectory metadata
├── recording_20260102_015826_0/
│   ├── 0.jpg
│   ├── ...
│   └── traj_data.pkl
└── ...
```

The `traj_data.pkl` file contains:

- `position`: `np.array` of shape `(N, 2)` - x, y positions in meters
- `yaw`: `np.array` of shape `(N,)` - yaw angles in radians

**Single bag conversion (alternative):**

```bash
uv run python -m il_baselines.data_processing.converters.rosbag_to_mediaref \
    --input ../data/sample_rosbags/recording_20260109_061808 \
    --output ../data/processed/
```

### Training

#### Environment Setup

Before running training, you must configure the `.env` file at the project root with `PROJECT_ROOT`:

```bash
# In CostNav/.env, ensure PROJECT_ROOT is set:
PROJECT_ROOT=/path/to/CostNav
```

The training script loads environment variables from `.env` automatically using `python-dotenv`.

#### Download Pretrained Checkpoints

Download pretrained model checkpoints from the [visualnav-transformer checkpoints](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing) and place them in the `checkpoints/` directory.

**Option 1: Manual Download**

1. Visit the [Google Drive folder](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing)
2. Download the desired checkpoint files (e.g., `vint.pth`)
3. Place them in `checkpoints/` at the repository root

**Option 2: Using gdown (Recommended)**

```bash
# Create checkpoints directory at repository root
mkdir -p checkpoints
cd checkpoints

# Download ViNT checkpoint (replace FILE_ID with actual Google Drive file ID)
# You can get the file ID from the Google Drive share link
uv run gdown <FILE_ID>

# Or download the entire folder
uv run gdown --folder https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg
```

The expected checkpoints directory structure:

```
checkpoints/
├── vint.pth      # ViNT pretrained weights
├── nomad.pth     # NoMaD pretrained weights (optional)
└── gnm.pth       # GNM pretrained weights (optional)
```

#### Fine-tuning ViNT on CostNav Data

##### Option 1: Local Run

```bash
# From CostNav/costnav_isaacsim/
uv run python -m il_baselines.training.train_vint \
    --config il_baselines/training/visualnav_transformer/configs/vint_costnav.yaml
```

##### Option 2: SLURM Job Submission

**1. Edit the configuration file:**

Edit `training/visualnav_transformer/configs/vint_costnav.yaml` — set dataset paths and `log_dir`.

**2. Submit the job to the cluster:**

```bash
cd costnav_isaacsim/il_baselines/scripts/
sbatch train_vint.sbatch
```

The sbatch script will automatically read paths from the config file and run training as a batch job.

#### Configuration Options

The config file `vint_costnav.yaml` supports the following key options:

| Parameter               | Default                | Description                           |
| ----------------------- | ---------------------- | ------------------------------------- |
| `pretrained_checkpoint` | `checkpoints/vint.pth` | Path to pretrained weights            |
| `batch_size`            | 64                     | Training batch size                   |
| `epochs`                | 50                     | Number of training epochs             |
| `lr`                    | 1e-4                   | Learning rate (lower for fine-tuning) |
| `use_wandb`             | True                   | Enable Weights & Biases logging       |
| `wandb_entity`          | maumai                 | W&B team/entity name                  |
| `context_size`          | 5                      | Number of context frames              |
| `len_traj_pred`         | 5                      | Trajectory prediction length          |

To disable W&B logging, set `use_wandb: False` in the config or set:

```bash
export WANDB_MODE=disabled
```

Checkpoints are saved to `logs/vint-costnav/`.

## Evaluation

Evaluation runs in Docker with ROS2 and is maintained in a separate folder with its own `pyproject.toml`:

See [`evaluation/README.md`](evaluation/README.md) for documentation on running trained IL policies in Isaac Sim.