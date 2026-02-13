# CostNav IL Evaluation

ROS2 package for evaluating Imitation Learning (IL) baselines in Isaac Sim using the CostNav infrastructure.

## Runtime Environment

This package **only runs inside Docker** (`Dockerfile.ros_torch`).
It depends on ROS2 system packages (`rclpy`, `cv_bridge`) and PyTorch with CUDA,
which are not installable via `uv` on a bare-metal host.

|             | Docker (only supported path)                                         |
| ----------- | -------------------------------------------------------------------- |
| **Image**   | `Dockerfile.ros_torch` (ROS2 Jazzy + PyTorch)                        |
| **Install** | `uv pip install -e .` inside the container                           |
| **Run**     | `make run-vint` (docker-compose)                                     |
| **Debug**   | VSCode devcontainer (`ros2-vint-dev` service)                        |
| **Test**    | `docker build -f Dockerfile.ros_torch .` → `pytest` inside container |

For IL training (data processing + model training), which uses `uv` on bare-metal,
see [`../il_training/`](../il_training/README.md).

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

### Install (inside Docker container)

```bash
# Dependencies are installed automatically when building the image:
docker build -f Dockerfile.ros_torch -t costnav-ros2-vint .

# For development inside the container:
cd /workspace/costnav_isaacsim/il_evaluation
uv pip install --system --break-system-packages -e .
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

| Parameter        | Type   | Default                | Description                                               |
| ---------------- | ------ | ---------------------- | --------------------------------------------------------- |
| `--checkpoint`   | string | (required)             | Path to trained model weights                             |
| `--model_config` | string | (required)             | Path to model config YAML (inference + navigation params) |
| `--robot_config` | string | (required)             | Path to robot config YAML (topics)                        |
| `--use_topomap`  | flag   | false                  | Enable topomap navigation (docker-compose injected)       |
| `--topomap_dir`  | string | `/tmp/costnav_topomap` | Topomap image directory (docker-compose injected)         |
| `--log_level`    | string | `info`                 | Log level (`debug`, `info`, `warn`, `error`, `fatal`)     |

All other parameters (`inference_rate`, `device`, `use_imagegoal`, `visualize_goal_image`, `topomap_goal_node`, `topomap_radius`, `topomap_close_threshold`) are read from `vint_eval.yaml`. Topic names (`image`, `goal_image`) are read from the robot config YAML.

### Trajectory Follower Node

| Parameter        | Type   | Default    | Description                                                     |
| ---------------- | ------ | ---------- | --------------------------------------------------------------- |
| `--robot_config` | string | (required) | Path to robot config YAML (topics + trajectory_follower params) |
| `--log_level`    | string | `info`     | Log level (`debug`, `info`, `warn`, `error`, `fatal`)           |

All control parameters (`control_rate`, `max_linear_vel`, `max_angular_vel`, `trajectory_timeout`) are read from the `trajectory_follower` section of the robot config YAML.

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
il_evaluation/
├── pyproject.toml            # Package configuration (uv pip install -e .)
├── configs/                  # Configuration files
│   ├── vint_eval.yaml                # Model architecture config
│   ├── robot_carter.yaml             # Nova Carter robot config
│   └── robot_segway.yaml             # Segway E1 robot config
└── src/il_evaluation/        # Python package source
    ├── agents/               # Policy inference agents
    │   ├── base_agent.py
    │   └── vint_agent.py
    ├── models/               # Neural network architectures
    │   ├── base_model.py
    │   ├── traj_opt.py
    │   └── vint_network.py
    ├── nodes/                # ROS2 nodes (entry points)
    │   ├── vint_policy_node.py       # ViNT inference (~4Hz)
    │   └── trajectory_follower_node.py  # MPC controller (~20Hz)
    ├── utils/                # Utility functions
    └── tests/                # Unit tests
```

## Trajectory Follower

The system uses a **two-node architecture** that decouples perception from control:

```
Camera ──► ViNT Policy Node (4 Hz) ──► /vint_trajectory ──► Trajectory Follower Node (20 Hz) ──► /cmd_vel ──► Robot
                                                                     ▲
                                                          /odom ─────┘
```

- **ViNT policy node** runs inference at **4 Hz**, predicts 5 waypoints, generates a smooth trajectory via cubic spline, and publishes it on `/vint_trajectory` as a `nav_msgs/Path`.
- **Trajectory follower node** subscribes to that trajectory and runs an MPC control loop at **20 Hz**, reading the robot's current pose from `/odom` and publishing velocity commands to `/cmd_vel`.

### Control Flow

The trajectory follower node has three callbacks:

| Callback              | Trigger                | What it does                                                                       |
| --------------------- | ---------------------- | ---------------------------------------------------------------------------------- |
| `trajectory_callback` | New `/vint_trajectory` | Converts Path → numpy, transforms to world frame if needed, creates or updates MPC |
| `odom_callback`       | New `/odom`            | Stores latest robot pose (used by `control_callback`)                              |
| `control_callback`    | Timer at 20 Hz         | Reads robot state, calls `mpc.solve(x0)`, publishes `cmd_vel`                      |

The control timer fires independently of trajectory arrival. If no valid trajectory exists or it has timed out (default 0.5 s), the callback publishes zero velocity (stop).

### Coordinate Frame Handling

ViNT predicts waypoints in the **robot-local frame** (`base_link`). MPC needs a **world-frame** trajectory to track against a moving robot. When a trajectory arrives in `base_link`:

1. Read the current robot pose `(x, y, quaternion)` from odometry
2. Build a **full 3D rotation matrix** from the quaternion (not yaw-only, so it's correct on slopes)
3. For each local waypoint `(dx, dy)`:
   - Extend to 3D: `(dx, dy, 0)`
   - Rotate: `rotated = R @ [dx, dy, 0]`
   - Translate: `world_x = robot_x + rotated[0]`, `world_y = robot_y + rotated[1]`

Once in world frame, the trajectory is stored and tracked until the next update or timeout.

### Enable / Disable

Publishing `false` on `/trajectory_follower_enable`:

- Immediately publishes a zero-velocity `Twist` (stops the robot)
- Calls `mpc.reset()` to clear the warm-start state so the next enable starts fresh

### MPC Controller

The trajectory follower uses a **Model Predictive Control (MPC)** controller based on the [NavDP](https://github.com/InternRobotics/NavDP) implementation.

#### MPC Reuse (Warm Start)

The CasADi optimization problem is built **once** on the first trajectory and reused for all subsequent updates. When a new trajectory arrives (~4 Hz), only the reference trajectory (`ref_traj`) is replaced — the solver structure, constraints, and warm-start state (`last_opt_u_controls`, `last_opt_x_states`) are preserved. This avoids the expensive `_setup_mpc()` rebuild and lets IPOPT converge faster from the previous solution.

```python
# First trajectory — build the CasADi problem once
if self.mpc_controller is None:
    self.mpc_controller = MPCController(trajectory=world_trajectory, ...)
else:
    # Reuse existing MPC: update ref trajectory, keep warm start
    self.mpc_controller.ref_traj = self.mpc_controller._make_ref_denser(world_trajectory)
```

#### Controller Parameters

| Parameter   | Default | Description                             |
| ----------- | ------- | --------------------------------------- |
| `N`         | 15      | MPC horizon length                      |
| `desired_v` | 0.5     | Desired velocity for reference spacing  |
| `v_max`     | 2.0     | Maximum linear velocity (m/s)           |
| `w_max`     | 0.5     | Maximum angular velocity (rad/s)        |
| `ref_gap`   | 3       | Gap between reference points in horizon |
| `dt`        | 0.1     | Time step for dynamics (s)              |

#### Cost Matrices

The MPC uses the following cost matrices:

- **State cost Q**: `diag([10.0, 10.0, 0.0])` - penalizes x, y error, ignores heading
- **Control cost R**: `diag([0.02, 0.15])` - penalizes velocity (v) and angular velocity (w)

#### Unicycle Dynamics

The controller uses unicycle dynamics:

```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = w
```

#### Solver

- **Solver**: IPOPT via CasADi
- **Max iterations**: 100
- **Tolerance**: 1e-8
- **Warm start**: Uses previous solution for faster convergence (preserved across trajectory updates)

#### Trajectory Processing

1. **Density interpolation**: Trajectory is made 50× denser via linear interpolation
2. **Nearest point finding**: Controller finds nearest point on trajectory to current pose
3. **Reference selection**: Reference points are sampled along the trajectory at `desired_v * ref_gap * dt` spacing
4. **Output**: Uses second control output (index 1) for smoother response

## Waypoint Normalization

ViNT outputs **normalized** waypoints during training and expects the same normalization to be reversed at evaluation time. Getting this wrong causes the robot to over- or under-shoot.

### Training (normalization)

```python
# In training data processing:
actions[:, :2] /= metric_waypoint_spacing * waypoint_spacing   # = 0.25 * 1 = 0.25
```

### Evaluation (denormalization)

```python
# In vint_agent.py:
waypoints[:, :, :2] *= self.denorm_scale  # = metric_waypoint_spacing * waypoint_spacing = 0.25
```

The denormalization scale (`denorm_scale`) is computed in `base_agent.py` directly from the model config:

```python
denorm_scale = metric_waypoint_spacing * waypoint_spacing  # 0.25 * 1 = 0.25 m
```

The source-of-truth training parameters (`waypoint_spacing`, `metric_waypoint_spacing`) live in `vint_eval.yaml` and must match the training config `vint_costnav.yaml`. Velocity limits (`max_linear_vel`, `max_angular_vel`) are **not** involved in denormalization — they are only used by the trajectory follower for MPC control (configured in the robot config YAML under `trajectory_follower`).

### Inference Rate

The inference rate (`--inference_rate`, default **4.0 Hz**) must match the training data `sample_rate` (4.0 Hz from `vint_processing_config.yaml`). This ensures the temporal context window (past `context_size=5` frames) spans the same time interval as during training. Running at a different rate (e.g. 10 Hz) fills the memory queue with near-identical frames, degrading temporal context quality.

## Configuration Reference

### vint_eval.yaml (Model + Inference Config)

```yaml
# Model architecture parameters
context_size: 5
len_traj_pred: 5
learn_angle: true
obs_encoder: "efficientnet-b0"
image_size: [85, 64] # [width, height] - matches training

# Action normalization (must match training config vint_costnav.yaml)
normalize: true
waypoint_spacing: 1
metric_waypoint_spacing: 0.25

# Inference parameters
inference_rate: 4.0 # Hz, must match training sample_rate
device: "cuda:0"

# Navigation mode
use_imagegoal: false
visualize_goal_image: false

# Topomap parameters (used when --use_topomap is passed on CLI)
topomap_goal_node: -1
topomap_radius: 4
topomap_close_threshold: 3.0
```

### robot_carter.yaml / robot_segway.yaml (Robot Parameters)

```yaml
# Topics
topics:
  odom: /chassis/odom
  image: /front_stereo_camera/left/image_raw
  goal_image: /goal_image

# Trajectory follower parameters
trajectory_follower:
  control_rate: 20.0
  max_linear_vel: 2.0
  max_angular_vel: 0.5
  trajectory_timeout: 0.5
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

- **Inference rate**: Default 4 Hz (must match training `sample_rate`). Do not change unless retraining with a different rate
- **Control rate**: Default 20 Hz. Higher rates = smoother motion, more CPU load
- **MPC horizon**: Default N=15. Longer = better planning, slower computation
- **Trajectory density**: 50× interpolation by default. Higher = smoother tracking
- **MPC warm start**: The CasADi problem is built once and reused — no tuning needed. Warm start is cleared on mission reset

## Training Models

See `costnav_isaacsim/il_training/` for training scripts and configs.
See `docs/imitation_learning_baselines.md` for detailed documentation.

## Related Documentation

- [IL Training](../il_training/README.md) - Data processing and model training
- [CostNav Isaac Sim README](../README.md) - Parent module documentation
- [Main CostNav README](../../README.md) - Project root documentation
- [IL Design Document](../../docs/imitation_learning_baselines.md) - Detailed IL architecture

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
