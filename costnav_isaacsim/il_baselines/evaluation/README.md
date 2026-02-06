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

### Install the Package

```bash
# From the CostNav workspace root (install as editable package)
cd costnav_isaacsim/il_baselines/evaluation
pip install -e .

# Or using uv (recommended)
uv pip install -e .
```

Dependencies are automatically installed from `pyproject.toml`.

## Usage

There are two main approaches for testing ViNT evaluation:

### Approach 1: Docker-Based Automated Testing (Recommended)

This approach runs the complete stack (Isaac Sim + ViNT policy) in Docker containers.

**Prerequisites:**
Download the pretrained model weights from Google Drive or train your model and place it to `checkpoints/`
See [Download Pretrained Checkpoints](../README.md#download-pretrained-checkpoints) for more information.

**Note:** The `MODEL_CHECKPOINT` environment variable must point to your trained ViNT model weights (`.pth` file).

1. **Start the ViNT stack** (in terminal 1):

   ```bash
   MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint
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

### Approach 2: Development/Manual Testing

This approach is useful for development and debugging within the devcontainer.

1. **Start Isaac Sim** (in terminal 1):

   ```bash
   make run-isaac-sim
   ```

2. **Launch ViNT Policy Node using VSCode Debugger** (recommended):

   Use the VSCode launch configurations in `.vscode/launch.json`:
   - **"Python: ViNT Policy Node (ROS2)"** - Runs ViNT inference node with debugger
   - **"Python: Trajectory Follower Node (ROS2)"** - Runs MPC trajectory follower with debugger

   Press `F5` or use the Run and Debug panel to start with breakpoints enabled.

3. **Trigger missions manually**:
   ```bash
   make start-mission
   ```

## Topics

### ViNT Policy Node

| Direction | Topic                                 | Type                | Description                                  |
| --------- | ------------------------------------- | ------------------- | -------------------------------------------- |
| Subscribe | `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image` | Camera image input                           |
| Subscribe | `/goal_image`                         | `sensor_msgs/Image` | Goal image (ImageGoal mode, transient local) |
| Subscribe | `/vint_enable`                        | `std_msgs/Bool`     | Enable/disable policy execution              |
| Publish   | `/vint_trajectory`                    | `nav_msgs/Path`     | Predicted trajectory (5 waypoints)           |
| Service   | `/reset_agent`                        | `std_srvs/Trigger`  | Reset agent memory for new mission           |

### Trajectory Follower Node

| Direction | Topic                         | Type                  | Description                                    |
| --------- | ----------------------------- | --------------------- | ---------------------------------------------- |
| Subscribe | `/vint_trajectory`            | `nav_msgs/Path`       | Trajectory from ViNT policy node               |
| Subscribe | `/chassis/odom`               | `nav_msgs/Odometry`   | Robot odometry (configurable via robot config) |
| Subscribe | `/trajectory_follower_enable` | `std_msgs/Bool`       | Enable/disable trajectory following            |
| Publish   | `/cmd_vel`                    | `geometry_msgs/Twist` | Velocity commands to robot                     |

## Parameters

### ViNT Policy Node

| Parameter          | Type   | Default                               | Description                   |
| ------------------ | ------ | ------------------------------------- | ----------------------------- |
| `--checkpoint`     | string | (required)                            | Path to trained model weights |
| `--model_config`   | string | `configs/vint_eval.yaml`              | Path to model config          |
| `--robot_config`   | string | `configs/robot_carter.yaml`           | Path to robot config          |
| `--inference_rate` | float  | 10.0                                  | Inference frequency (Hz)      |
| `--image_topic`    | string | `/front_stereo_camera/left/image_raw` | Camera topic                  |
| `--use_imagegoal`  | flag   | false                                 | Use image goal navigation     |
| `--device`         | string | `cuda:0`                              | PyTorch device                |

### Trajectory Follower Node

| Parameter              | Type   | Default                     | Description                      |
| ---------------------- | ------ | --------------------------- | -------------------------------- |
| `--robot_config`       | string | `configs/robot_carter.yaml` | Path to robot config             |
| `--control_rate`       | float  | 20.0                        | Control loop frequency (Hz)      |
| `--max_linear_vel`     | float  | 0.5                         | Maximum linear velocity (m/s)    |
| `--max_angular_vel`    | float  | 0.5                         | Maximum angular velocity (rad/s) |
| `--trajectory_timeout` | float  | 0.5                         | Trajectory timeout (s)           |

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
MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint

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
├── pyproject.toml        # Package configuration (pip install -e .)
├── configs/              # Configuration files
│   ├── vint_eval.yaml            # Model architecture config
│   ├── robot_carter.yaml         # Nova Carter robot config
│   └── robot_segway.yaml         # Segway E1 robot config
└── src/evaluation/       # Python package source
    ├── agents/           # Policy inference agents
    │   ├── base_agent.py
    │   └── vint_agent.py
    ├── models/           # Neural network architectures
    │   ├── base_model.py
    │   ├── traj_opt.py
    │   └── vint_network.py
    ├── nodes/            # ROS2 nodes (entry points)
    │   ├── vint_policy_node.py       # ViNT inference (~10Hz)
    │   └── trajectory_follower_node.py  # MPC controller (~20Hz)
    ├── utils/            # Utility functions
    └── tests/            # Unit tests
```

## MPC Controller

The trajectory follower node uses a **Model Predictive Control (MPC)** controller based on the [NavDP](https://github.com/InternRobotics/NavDP) implementation. The controller tracks trajectories published by the ViNT policy node.

### Controller Parameters

| Parameter   | Default | Description                             |
| ----------- | ------- | --------------------------------------- |
| `N`         | 15      | MPC horizon length                      |
| `desired_v` | 0.5     | Desired velocity for reference spacing  |
| `v_max`     | 0.5     | Maximum linear velocity (m/s)           |
| `w_max`     | 0.5     | Maximum angular velocity (rad/s)        |
| `ref_gap`   | 3       | Gap between reference points in horizon |
| `dt`        | 0.1     | Time step for dynamics (s)              |

### Cost Matrices

The MPC uses the following cost matrices:

- **State cost Q**: `diag([10.0, 10.0, 0.0])` - penalizes x, y error, ignores heading
- **Control cost R**: `diag([0.02, 0.15])` - penalizes velocity (v) and angular velocity (w)

### Unicycle Dynamics

The controller uses unicycle dynamics:

```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = w
```

### Solver

- **Solver**: IPOPT via CasADi
- **Max iterations**: 100
- **Tolerance**: 1e-8
- **Warm start**: Uses previous solution for faster convergence

### Trajectory Processing

1. **Density interpolation**: Trajectory is made 50× denser via linear interpolation
2. **Nearest point finding**: Controller finds nearest point on trajectory to current pose
3. **Reference selection**: Reference points are sampled along the trajectory at `desired_v * ref_gap * dt` spacing
4. **Output**: Uses second control output (index 1) for smoother response

## Configuration Reference

### vint_eval.yaml (Model Architecture)

```yaml
# Model architecture parameters
context_size: 5 # Number of past frames for temporal context
len_traj_pred: 5 # Number of waypoints to predict
learn_angle: true # Predict heading angles
obs_encoder: "efficientnet-b0" # Image encoder backbone
obs_encoding_size: 512 # Encoding dimension
late_fusion: false # Early fusion of obs+goal
mha_num_attention_heads: 4 # Transformer attention heads
mha_num_attention_layers: 4 # Transformer layers
mha_ff_dim_factor: 4 # Feedforward dimension factor
image_size: [85, 64] # [width, height] - matches training
normalize: true # Normalize actions by max velocity
```

### robot_segway.yaml (Robot Parameters)

```yaml
# Velocity limits
max_v: 0.8 # Maximum linear velocity (m/s)
max_w: 0.8 # Maximum angular velocity (rad/s)

# Timing - IMPORTANT: must match training config
frame_rate: 3 # Training frame rate (Hz) - used for waypoint scaling
```

## Troubleshooting

### Common Issues

| Issue                          | Cause                                  | Solution                                                        |
| ------------------------------ | -------------------------------------- | --------------------------------------------------------------- |
| **Model checkpoint not found** | Invalid path or missing file           | Verify `checkpoint` parameter points to valid `.pth` file       |
| **CUDA out of memory**         | GPU memory exhausted                   | Reduce `context_size` or use smaller batch during inference     |
| **No trajectory published**    | Camera topic not connected             | Check `/front_stereo_camera/left/image_raw` topic is publishing |
| **Robot not moving**           | Trajectory follower not receiving data | Verify `/vint_trajectory` and `/chassis/odom` topics            |
| **MPC solver fails**           | Invalid trajectory or constraints      | Check trajectory waypoints are valid; try increasing `dt`       |
| **Path not visible in RViz**   | Transform or QoS mismatch              | Set Fixed Frame to `base_link`; increase Transform Tolerance    |
| **Trajectory jumps**           | Memory queue not reset on new mission  | Call `/reset_agent` service when starting new mission           |

### Debug Commands

```bash
# Check ViNT policy node status
ros2 topic echo /vint_trajectory --once

# Check trajectory follower status
ros2 topic echo /cmd_vel --once

# Verify camera images are received
ros2 topic hz /front_stereo_camera/left/image_raw

# Check odometry
ros2 topic echo /chassis/odom --once

# Reset agent memory (for new mission)
ros2 service call /reset_agent std_srvs/srv/Trigger {}

# Enable/disable policy execution
ros2 topic pub /vint_enable std_msgs/msg/Bool "data: true"
```

### Performance Tuning

- **Inference rate**: Default 10Hz. Lower for slower GPUs, higher for more responsive control
- **Control rate**: Default 20Hz. Higher rates = smoother motion, more CPU load
- **MPC horizon**: Default N=15. Longer = better planning, slower computation
- **Trajectory density**: 50× interpolation by default. Higher = smoother tracking

## Training Models

See `costnav_isaacsim/il_baselines/training/` for training scripts and configs.
See `docs/imitation_learning_baselines.md` for detailed documentation.

## Related Documentation

- [IL Baselines Overview](../README.md) - Parent module documentation
- [Main CostNav README](../../README.md) - CostNav Isaac Sim documentation
- [IL Design Document](../../../docs/imitation_learning_baselines.md) - Detailed IL architecture

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
