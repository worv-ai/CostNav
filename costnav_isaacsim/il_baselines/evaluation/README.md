# CostNav IL Baselines Evaluation

ROS2 package for evaluating Imitation Learning (IL) baselines in Isaac Sim using the CostNav infrastructure.

## References

This evaluation framework is adapted from the [NavDP](https://github.com/InternRobotics/NavDP) benchmark:

> **NavDP: Learning Sim-to-Real Navigation Diffusion Policy with Privileged Information Guidance**
> Wenzhe Cai, Jiaqi Peng, Yuqiang Yang, Yujian Zhang, Meng Wei, Hanqing Wang, Yilun Chen, Tai Wang, Jiangmiao Pang
> Shanghai AI Laboratory, Tsinghua University, Zhejiang University, The University of Hong Kong
> [Paper](https://arxiv.org/abs/2505.08712) | [GitHub](https://github.com/InternRobotics/NavDP) | [Project Page](https://wzcai99.github.io/navigation-diffusion-policy.github.io/)

The baseline implementations (ViNT, NoMaD, GNM) are derived from:
- NavDP baselines: `third_party/NavDP/baselines/`
- Original [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) repository

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

There are two main approaches for testing ViNT evaluation:

### Approach 1: Docker-Based Automated Testing (Recommended)

This approach runs the complete stack (Isaac Sim + ViNT policy) in Docker containers.

1. **Start the ViNT stack** (in terminal 1):

   ```bash
   MODEL_CHECKPOINT=/path/to/vint_model.pth make run-vint
   ```

   This starts both Isaac Sim and the ViNT policy node together.

2. **Run automated evaluation** (in terminal 2):

   ```bash
   # Run evaluation with default settings (3 missions, 169s timeout)
   make run-eval-vint

   # Or customize the evaluation parameters
   make run-eval-vint TIMEOUT=120 NUM_MISSIONS=10
   ```

   This will automatically run consecutive missions and generate evaluation logs in `./logs/vint_evaluation_<timestamp>.log`.

3. **Manually trigger individual missions** (alternative to automated evaluation):
   ```bash
   make start-mission
   ```

**Note:** The `MODEL_CHECKPOINT` environment variable must point to your trained ViNT model weights (`.pth` file).

### Approach 2: Development/Manual Testing

This approach is useful for development and debugging within the devcontainer.

1. **Start Isaac Sim and Teleop** (in terminal 1):

   ```bash
   make run-teleop
   ```

   This starts both Isaac Sim and the teleop node together.

2. **Launch ViNT Policy Node from devcontainer** (in terminal 2):

   ```bash
   # Inside the devcontainer
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

3. **Enable Model Control** in the teleop node:
   - Press the RT (Right Trigger) button on the joystick to switch to model control
   - The ViNT policy will now control the robot via `/cmd_vel_model`

4. **Trigger missions manually**:
   ```bash
   make start-mission
   ```

## Topics

### Subscribed Topics

| Topic                                 | Type                | Description                 |
| ------------------------------------- | ------------------- | --------------------------- |
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image` | Camera image input          |
| `/odom`                               | `nav_msgs/Odometry` | Robot odometry              |
| `/goal_image`                         | `sensor_msgs/Image` | Goal image (ImageGoal mode) |
| `/vint_enable`                        | `std_msgs/Bool`     | Enable/disable policy       |

### Published Topics

| Topic            | Type                  | Description       |
| ---------------- | --------------------- | ----------------- |
| `/cmd_vel_model` | `geometry_msgs/Twist` | Velocity commands |

## Parameters

| Parameter        | Type   | Default                               | Description                   |
| ---------------- | ------ | ------------------------------------- | ----------------------------- |
| `checkpoint`     | string | (required)                            | Path to trained model weights |
| `model_config`   | string | `configs/vint_eval.yaml`              | Path to model config          |
| `robot_config`   | string | `configs/robot_carter.yaml`           | Path to robot config          |
| `inference_rate` | float  | 10.0                                  | Inference frequency (Hz)      |
| `image_topic`    | string | `/front_stereo_camera/left/image_raw` | Camera topic                  |
| `use_imagegoal`  | bool   | false                                 | Use image goal navigation     |
| `device`         | string | `cuda:0`                              | PyTorch device                |

## Evaluation

### Automated Evaluation Workflow

The evaluation system runs consecutive missions and collects comprehensive metrics including:

- Success rate (goal reached within timeout)
- Traveled distance and elapsed time
- Average velocity and mechanical power
- Contact counts and impulses
- Property damage metrics
- Delta-v (collision severity) metrics
- Injury cost estimates

**Running Automated Evaluation:**

```bash
# Terminal 1: Start the ViNT stack
MODEL_CHECKPOINT=/path/to/vint_model.pth make run-vint

# Terminal 2: Run evaluation
make run-eval-vint TIMEOUT=169 NUM_MISSIONS=10
```

**Evaluation Parameters:**

- `TIMEOUT`: Maximum time per mission in seconds (default: 169s)
- `NUM_MISSIONS`: Number of consecutive missions to run (default: 3)

**Output:**

- Logs are saved to `./logs/vint_evaluation_<timestamp>.log`
- Real-time progress is displayed in the terminal
- Summary statistics are generated at the end

**Interactive Controls:**

- Press `→` (right arrow) during a mission to skip it

### Manual Mission Triggering

For development and debugging, you can manually trigger individual missions:

```bash
# After starting the ViNT stack (make run-vint)
make start-mission
```

This calls the `/start_mission` ROS2 service to begin a new navigation task.

### Evaluation Metrics

The evaluation system tracks the following metrics per mission:

**Navigation Metrics:**

- Mission result (SUCCESS_SLA, FAILURE_TIMEOUT, FAILURE_PHYSICALASSISTANCE, FAILURE_FOODSPOILED)
- Distance to goal (m)
- Traveled distance (m)
- Elapsed time (s)
- Average velocity (m/s)
- Average mechanical power (kW)

**Safety Metrics:**

- Contact count (total collisions)
- Total impulse (N·s)
- Property damage by type (fire hydrant, traffic light, street lamp, bollard, building)
- Delta-v count and average (collision severity in m/s and mph)
- Total injury cost estimate

**Food Delivery Metrics** (if enabled):

- Food pieces (initial → final)
- Food loss fraction
- Food spoiled status

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

## Citation

If you use this evaluation framework, please cite NavDP:

```bibtex
@misc{navdp,
    title = {NavDP: Learning Sim-to-Real Navigation Diffusion Policy with Privileged Information Guidance},
    author = {Wenzhe Cai, Jiaqi Peng, Yuqiang Yang, Yujian Zhang, Meng Wei, Hanqing Wang, Yilun Chen, Tai Wang and Jiangmiao Pang},
    year = {2025},
    booktitle = {arXiv},
}
```
