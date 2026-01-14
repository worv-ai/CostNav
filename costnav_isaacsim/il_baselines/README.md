# CostNav Imitation Learning Baselines

This module implements imitation learning baselines for CostNav sidewalk robot navigation.

## Directory Structure

```
il_baselines/
├── README.md                     # This file
├── __init__.py
├── data_processing/              # Data conversion pipeline
│   ├── __init__.py
│   ├── converters/               # Format conversion scripts
│   │   ├── __init__.py
│   │   └── rosbag_to_mediaref.py # ROS2 mcaps → MediaRef
│   └── configs/
│       └── processing_config.yaml
├── training/                     # Training framework
│   ├── __init__.py
│   └── visualnav_transformer/    # ViNT/NoMaD/GNM
│       ├── Dockerfile
│       ├── docker-compose.yml
│       └── configs/
│           ├── vint_costnav.yaml
│           ├── nomad_costnav.yaml
│           └── gnm_costnav.yaml
└── evaluation/                   # Evaluation & deployment
    └── __init__.py
```

## Data Format

### Input: ROS2 MCAP Bags

CostNav rosbag files (`.mcap`) contain the following key topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/msg/Image` | Front camera images (RGB, ~30Hz) |
| `/chassis/odom` | `nav_msgs/msg/Odometry` | Odometry data |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands |
| `/chassis/imu` | `sensor_msgs/msg/Imu` | IMU data |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR scan |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Navigation goal |

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

```bash
# Convert single rosbag to MediaRef
python costnav_isaacsim/il_baselines/data_processing/converters/rosbag_to_mediaref.py \
    --input data/sample_rosbags/recording_20260109_061808 \
    --output data/processed/

# Batch conversion with Ray
python costnav_isaacsim/il_baselines/data_processing/converters/ray_batch_rosbag_to_mediaref.py \
    --bag-dir data/sample_rosbags/ \
    --output-root data/processed/
```

### Training (Docker)

```bash
cd costnav_isaacsim/il_baselines/training/visualnav_transformer/
docker-compose up vint-train
```

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
