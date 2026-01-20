# CostNav Imitation Learning Baselines

This module implements imitation learning baselines for CostNav sidewalk robot navigation.

## Directory Structure

```
il_baselines/
├── README.md                     # This file
├── __init__.py
├── environment.yml               # Conda environment specification
├── data_processing/              # Data conversion pipeline
│   ├── __init__.py
│   ├── configs/
│   │   ├── processing_config.yaml
│   │   └── vint_processing_config.yaml
│   ├── converters/               # Format conversion scripts
│   │   ├── __init__.py
│   │   ├── rosbag_to_mediaref.py     # ROS2 mcaps → MediaRef
│   │   └── ray_batch_convert.py      # Batch conversion with Ray
│   ├── process_data/             # Data processing scripts
│   │   ├── __init__.py
│   │   └── process_mediaref_bags.py  # MediaRef → ViNT format
│   └── tests/
│       ├── __init__.py
│       ├── test_rosbag_to_mediaref.py
│       ├── test_ray_batch_convert.py
│       └── test_process_mediaref_bags.py
├── training/                     # Training framework
│   ├── __init__.py
│   ├── train_vint.py             # Main training script
│   └── visualnav_transformer/    # ViNT/NoMaD/GNM
│       ├── checkpoints/          # Pretrained model weights (download manually)
│       │   ├── vint.pth
│       │   ├── nomad.pth
│       │   └── gnm.pth
│       └── configs/
│           ├── defaults.yaml     # Default training config
│           └── vint_costnav.yaml # CostNav fine-tuning config
└── evaluation/                   # Evaluation & deployment
    └── __init__.py
```

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

### Data Conversion

**Step 1: ROS Bag → MediaRef Format** (parallel batch conversion)

```bash
# Batch convert ROS bags to MediaRef format using Ray
python costnav_isaacsim/il_baselines/data_processing/converters/ray_batch_convert.py \
    --input_dir data/sample_rosbags/ \
    --output_dir data/mediaref/ \
    --config configs/processing_config.yaml \
    --num_workers 8
```

`ray_batch_convert.py` extracts image topics from ROS2 bags and encodes them as MP4 videos with MediaRef pointers. This provides significant storage reduction while keeping odometry/cmd data in MCAP format. Supports skipping already-processed bags via `_SUCCESS` markers.

**Step 2: MediaRef → ViNT Training Format**

```bash
# Convert MediaRef bags to ViNT training format
python costnav_isaacsim/il_baselines/data_processing/process_data/process_mediaref_bags.py \
    --input-dir data/mediaref/ \
    --output-dir data/vint_format/ \
    --config configs/vint_processing_config.yaml \
    --sample-rate 4.0
```

`process_mediaref_bags.py` reads MediaRef bags (video + MCAP), synchronizes images with odometry at a specified rate, filters out backward-moving segments, and outputs ViNT-format trajectory folders.

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
python costnav_isaacsim/il_baselines/data_processing/converters/rosbag_to_mediaref.py \
    --input data/sample_rosbags/recording_20260109_061808 \
    --output data/processed/
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
3. Place them in `costnav_isaacsim/il_baselines/training/visualnav_transformer/checkpoints/`

**Option 2: Using gdown (Recommended)**

```bash
# Install gdown
pip install gdown

# Create checkpoints directory
mkdir -p costnav_isaacsim/il_baselines/training/visualnav_transformer/checkpoints
cd costnav_isaacsim/il_baselines/training/visualnav_transformer/checkpoints

# Download ViNT checkpoint (replace FILE_ID with actual Google Drive file ID)
# You can get the file ID from the Google Drive share link
gdown <FILE_ID>

# Or download the entire folder
gdown --folder https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg
```

The expected checkpoints directory structure:

```
costnav_isaacsim/il_baselines/training/visualnav_transformer/checkpoints/
├── vint.pth      # ViNT pretrained weights
├── nomad.pth     # NoMaD pretrained weights (optional)
└── gnm.pth       # GNM pretrained weights (optional)
```

#### Fine-tuning ViNT on CostNav Data

**Using the standalone training script (recommended)**

```bash
# Run fine-tuning from CostNav root directory
python costnav_isaacsim/il_baselines/training/train_vint.py \
    --config costnav_isaacsim/il_baselines/training/visualnav_transformer/configs/vint_costnav.yaml
```

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

## Installation

```bash
# Create conda environment from environment.yml
cd CostNav
conda env create -f costnav_isaacsim/il_baselines/environment.yml
conda activate costnav_il

# Initialize only the required submodules for IL baselines
git submodule update --init third_party/visualnav-transformer
git submodule update --init third_party/diffusion_policy

# Install visualnav-transformer (ViNT/NoMaD/GNM training)
pip install -e third_party/visualnav-transformer/train/

# Install diffusion_policy (with workaround for upstream bug)
# Note: The official diffusion_policy repository is missing __init__.py
# We need to add it manually to make the package importable
echo '# Diffusion Policy Package' > third_party/diffusion_policy/diffusion_policy/__init__.py
cd third_party/diffusion_policy && pip install -e . && cd ../..

# Configure environment variables
# Create a .env file at the project root with:
echo "PROJECT_ROOT=$(pwd)" > .env
```

### Known Issues

**diffusion_policy Missing `__init__.py`**

The official [diffusion_policy repository](https://github.com/real-stanford/diffusion_policy) is missing the `__init__.py` file in the `diffusion_policy/` directory, which prevents Python from recognizing it as a package. This causes `find_packages()` in `setup.py` to return an empty list.

**Workaround:** We add a minimal `__init__.py` file before installation (see installation steps above). This is a necessary fix for a broken upstream dependency. Consider reporting this issue to the upstream repository.

## Dependencies

- Python 3.10+
- PyTorch 2.x with CUDA 12.x
- rosbags (ROS bag reading library)
- mediaref (lightweight media references)
- ray (distributed processing)
- av (video encoding)
- opencv-python

## Sample Data

Sample rosbag files are stored at:

- `data/sample_rosbags/recording_20260109_061808/` (~202MB, ~8.5s duration)
- `data/sample_rosbags/recording_20260108_062956/` (~240MB)

These contain front camera images, odometry, velocity commands, and navigation goals.
