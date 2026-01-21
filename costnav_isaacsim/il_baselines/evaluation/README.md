# CostNav IL Baselines Evaluation

ROS2 package for evaluating Imitation Learning (IL) baselines in Isaac Sim using the CostNav infrastructure.

## Supported Baselines

- **ViNT** (Visual Navigation Transformer) - ✅ Implemented
- **NoMaD** (No Map Diffusion) - 🔄 Planned
- **GNM** (General Navigation Model) - 🔄 Planned

## Installation

### Prerequisites

- ROS2 Humble or later
- Python 3.10+
- PyTorch 2.0+
- CUDA-capable GPU

### Build the Package

```bash
# From the CostNav workspace root
cd /path/to/CostNav
colcon build --packages-select costnav_il_baselines
source install/setup.bash
```

### Python Dependencies

```bash
# Install Python dependencies (if not using conda environment)
pip install torch torchvision efficientnet_pytorch opencv-python pyyaml pillow
```

## Usage

### Launch ViNT Policy Node

```bash
# Using launch file
ros2 launch costnav_il_baselines vint_policy.launch.py \
    checkpoint:=/path/to/vint_model.pth

# Or run directly with all options
ros2 run costnav_il_baselines vint_policy_node \
    --ros-args \
    -p checkpoint:=/path/to/vint_model.pth \
    -p model_config:=/path/to/vint_eval.yaml \
    -p robot_config:=/path/to/robot_carter.yaml \
    -p inference_rate:=10.0 \
    -p use_imagegoal:=false
```

### Integration with Isaac Sim

1. **Start Isaac Sim** with ROS2 bridge enabled:
   ```bash
   make run-isaacsim
   ```

2. **Start Teleop Node** (in a new terminal):
   ```bash
   make teleop
   ```

3. **Start ViNT Policy Node** (in a new terminal):
   ```bash
   ros2 launch costnav_il_baselines vint_policy.launch.py \
       checkpoint:=/path/to/vint_model.pth
   ```

4. **Enable Model Control** in the teleop node:
   - Press the RT (Right Trigger) button on the joystick to switch to model control
   - The ViNT policy will now control the robot via `/cmd_vel_model`

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image` | Camera image input |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/goal_image` | `sensor_msgs/Image` | Goal image (ImageGoal mode) |
| `/vint_enable` | `std_msgs/Bool` | Enable/disable policy |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_model` | `geometry_msgs/Twist` | Velocity commands |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `checkpoint` | string | (required) | Path to trained model weights |
| `model_config` | string | `configs/vint_eval.yaml` | Path to model config |
| `robot_config` | string | `configs/robot_carter.yaml` | Path to robot config |
| `inference_rate` | float | 10.0 | Inference frequency (Hz) |
| `image_topic` | string | `/front_stereo_camera/left/image_raw` | Camera topic |
| `use_imagegoal` | bool | false | Use image goal navigation |
| `device` | string | `cuda:0` | PyTorch device |

## Package Structure

```
evaluation/
├── agents/           # Policy inference agents
│   ├── base_agent.py
│   └── vint_agent.py
├── models/           # Neural network architectures
│   ├── base_model.py
│   ├── traj_opt.py
│   └── vint_network.py
├── nodes/            # ROS2 nodes
│   └── vint_policy_node.py
├── configs/          # Configuration files
│   ├── vint_eval.yaml
│   └── robot_carter.yaml
└── launch/           # Launch files
    └── vint_policy.launch.py
```

## Training Models

See `costnav_isaacsim/il_baselines/training/` for training scripts and configs.
See `docs/imitation_learning_baselines.md` for detailed documentation.
