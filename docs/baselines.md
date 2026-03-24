# :compass: Navigation Baselines

CostNav provides both **learning-based** (imitation learning) and **rule-based** (Nav2) navigation baselines for benchmarking sidewalk robot policies.

---

## :brain: Imitation Learning Baselines

### Supported Algorithms

| Algorithm | Type | Framework | Goal Support |
|-----------|------|-----------|-------------|
| **NavDP** | Diffusion Policy | NavDP | Point, Image, Pixel, No-Goal, Mixed |
| **ViNT** | Visual Transformer | visualnav-transformer | Image, No-Goal |
| **NoMaD** | Diffusion Model | visualnav-transformer | Image, No-Goal |
| **GNM** | General Navigation | visualnav-transformer | Image, No-Goal |
| **CANVAS** | Sketch-based | canvas | Point Goal |

### Architecture

All IL baselines share a **two-node ROS2 architecture**:

- **Policy Node** (~10 Hz): Runs model inference, publishes trajectory
- **Trajectory Follower Node** (~20 Hz): MPC controller tracking the trajectory

```
Isaac Sim Container              IL Container (ROS2 Jazzy)
┌────────────────────┐          ┌────────────────────────────────┐
│ Physics sim        │          │ policy_node (~10Hz)            │
│ Robot + ROS2 Bridge│◄────────►│ trajectory_follower_node (~20Hz)│
└────────────────────┘          └────────────────────────────────┘
         ROS2 Topics (via ROS_DOMAIN_ID)
```

**Key ROS2 Topics:**

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image` | Isaac Sim → Policy | Camera images |
| `/chassis/odom` | `nav_msgs/Odometry` | Isaac Sim → Policy | Robot odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Policy → Isaac Sim | Velocity commands |
| `/model_trajectory` | `nav_msgs/Path` | Internal | Policy → Trajectory Follower |

### Running IL Baselines

```bash
# Build shared ROS2 + PyTorch image (first time only)
make build-ros2-torch

# Run a specific baseline
make run-vint
make run-nomad
make run-gnm
make run-navdp
make run-canvas
```

**Navigation mode override** — switch between `image_goal` and `topomap`:

```bash
# Image-goal mode
GOAL_TYPE=image_goal MODEL_CHECKPOINT=checkpoints/nomad.pth make run-nomad

# Topomap mode
GOAL_TYPE=topomap MODEL_CHECKPOINT=checkpoints/nomad.pth make run-nomad
```

### Automated Evaluation

```bash
# Run evaluation with default settings
make run-eval-vint
make run-eval-gnm
make run-eval-nomad
make run-eval-navdp
make run-eval-canvas

# Custom parameters
make run-eval-vint TIMEOUT=120 NUM_MISSIONS=10
```

Logs saved to `./logs/<baseline>_evaluation_<timestamp>.log`.

### Data Collection & Training

#### Data Collection

Teleoperation data is collected via:

```bash
make teleop
```

#### Data Processing Pipeline

```
ROS2 Bags → MediaRef → Processing → ViNT Format
```

```bash
cd CostNav/costnav_isaacsim

# Install dependencies
uv sync

# Step 1: Convert ROS bags to MediaRef
uv run python -m il_training.data_processing.converters.ray_batch_convert \
    --config data_processing/configs/processing_config.yaml

# Step 2: Convert to ViNT format
uv run python -m il_training.data_processing.process_data.process_mediaref_bags \
    --config data_processing/configs/vint_processing_config.yaml
```

#### Training

Download pretrained checkpoints first:

```bash
make download-baseline-checkpoints-hf
```

Train models:

```bash
cd CostNav/costnav_isaacsim

# Train ViNT
uv run python -m il_training.training.train_vint \
    --config il_training/training/visualnav_transformer/configs/vint_costnav.yaml

# Train NoMaD
uv run python -m il_training.training.train_vint \
    --config il_training/training/visualnav_transformer/configs/nomad_costnav.yaml
```

For SLURM cluster training:

```bash
cd costnav_isaacsim/il_training/scripts/
sbatch train_vint.sbatch
sbatch train_nomad.sbatch
```

### NavDP HTTP Server Mode

NavDP also supports a **decoupled HTTP-based architecture** for evaluation, where policies run on separate GPUs:

```bash
# Start policy server
cd third_party/NavDP/baselines/navdp
python navdp_server.py --port 8888 --checkpoint ./checkpoints/model.ckpt

# Run evaluation
cd third_party/NavDP
python eval_pointgoal_wheeled.py \
    --scene_dir ./asset_scenes/cluttered_easy \
    --num_envs 4 \
    --port 8888
```

**API Endpoints:**

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/navigator_reset` | POST | Initialize agent with camera intrinsics and batch size |
| `/pointgoal_step` | POST | Point goal navigation step |
| `/imagegoal_step` | POST | Image goal navigation step |
| `/nogoal_step` | POST | Exploration step (no goal) |

### Baseline Comparison

| Feature | ViNT | NoMaD | GNM | NavDP |
|---------|------|-------|-----|-------|
| **Architecture** | Transformer | Diffusion | CNN | Diffusion + Critic |
| **Goal Support** | Image, NoGoal | Image, NoGoal | Image, NoGoal | Point, Image, Pixel |
| **Trajectory Length** | 8 waypoints | 8 waypoints | 5 waypoints | 24 waypoints |
| **Context Frames** | 5 | 5 | 5 | 8 |

### Heading Alignment

IL baselines (ViNT, GNM, NoMaD, NavDP) default to `ALIGN_HEADING=True`. This aligns the robot's initial heading with the first topomap waypoint direction before navigation begins. IL models are trained on forward-moving demonstration trajectories, so they cannot handle arbitrary initial headings — the model receives out-of-distribution observations and produces poor actions.

| Method | `ALIGN_HEADING` default | Reason |
|--------|------------------------|--------|
| Nav2 | `False` | Classical planner handles arbitrary headings |
| Canvas | `False` | Learned policy handles arbitrary headings |
| ViNT | **`True`** | IL model requires aligned initial heading |
| GNM | **`True`** | IL model requires aligned initial heading |
| NoMaD | **`True`** | IL model requires aligned initial heading |
| NavDP | **`True`** | IL model requires aligned initial heading |

```bash
# Disable for testing
ALIGN_HEADING=False MODEL_CHECKPOINT=checkpoints/baseline-vint.pth make run-vint
```

---

## :robot: Nav2 Rule-Based Baseline

CostNav integrates the **ROS2 Navigation Stack 2** as a rule-based baseline for comparison with learning-based approaches.

### Supported Robots

| Robot | Command |
|-------|---------|
| Nova Carter | `SIM_ROBOT=nova_carter make run-nav2` |
| Segway E1 | `SIM_ROBOT=segway_e1 make run-nav2` |

### Features

- Full Nav2 integration with Isaac Sim
- Mission orchestration with automated start/goal sampling from NavMesh
- Two localization modes: AMCL (realistic) and ground truth (benchmarking)
- Cost model integration: energy, distance, time, collision, and food spoilage tracking

### Localization Modes

```bash
# AMCL mode (default — realistic)
SIM_ROBOT=nova_carter make run-nav2

# Ground truth mode (benchmarking)
SIM_ROBOT=nova_carter AMCL=False make run-nav2
```

---

## :bar_chart: Evaluation Metrics

All baselines are evaluated using CostNav's unified economic metrics:

| Metric | Description |
|--------|-------------|
| **SLA Compliance** | Percentage of deliveries within time budget |
| **Operating Margin** | Revenue - Operating Costs |
| **Break-even Time** | Time to recover capital investment |
| **Cost per Mission** | Average cost per navigation task |

**Safety Metrics:**

- Contact count (total collisions)
- Total impulse (N·s)
- Property damage by type (fire hydrant, traffic light, street lamp, bollard, building)
- Delta-v count and average injury cost estimate

**Food Delivery Metrics:**

- Food pieces (initial → final)
- Food loss fraction

---

## :link: References

### Research Papers

1. **GNM**: Shah et al., "GNM: A General Navigation Model to Drive Any Robot", ICRA 2023
2. **ViNT**: Shah et al., "ViNT: A Foundation Model for Visual Navigation", CoRL 2023
3. **NoMaD**: Sridhar et al., "NoMaD: Goal Masking Diffusion Policies for Navigation and Exploration", 2023
4. **NavDP**: Cai et al., "NavDP: Learning Sim-to-Real Navigation Diffusion Policy", 2025
5. **CANVAS**: Choi et al., "CANVAS: Commonsense-Aware Navigation System", ICRA 2025

### Code

- [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) — ViNT, NoMaD, GNM
- [NavDP](https://github.com/OpenRobotLab/NavDP) — Navigation diffusion policy
- [MediaRef](https://github.com/open-world-agents/MediaRef) — Lightweight media references
