# Navigation Imitation Learning Baselines

> Comprehensive guide for implementing imitation learning baselines for sidewalk robot navigation in CostNav.

This document describes the design and implementation of imitation learning (IL) baselines for the CostNav navigation benchmark, enabling comparison between learning-from-demonstrations approaches and rule-based (Nav2) baselines.

---

## Overview

### Motivation

While CostNav currently supports:

- **Rule-based navigation** via Nav2 stack (`costnav_isaacsim/`)

Imitation learning offers complementary advantages:

- **Sample efficiency**: Learn from expert demonstrations without reward engineering
- **Natural behavior**: Capture human-like navigation patterns
- **Cross-embodiment transfer**: Leverage pre-trained models like ViNT/NoMaD

### Supported Algorithms

| Algorithm | Type               | Framework             | Description                                           |
| --------- | ------------------ | --------------------- | ----------------------------------------------------- |
| **ViNT**  | Transformer        | visualnav-transformer | Vision transformer for navigation                     |
| **NoMaD** | Diffusion          | visualnav-transformer | Goal-masked diffusion policies                        |
| **GNM**   | General Navigation | visualnav-transformer | General navigation model                              |
| **NavDP** | Diffusion          | NavDP                 | Sim-to-real navigation diffusion with privileged info |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                 CostNav Navigation IL Pipeline                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐            │
│  │ Data          │   │ Data          │   │ Training      │            │
│  │ Collection    │──►│ Processing    │──►│ Framework     │            │
│  │               │   │               │   │               │            │
│  │ • Teleop      │   │ • MediaRef    │   │ • ViNT/NoMaD  │            │
│  │ • Nav2 Expert │   │ • Filtering   │   │ • GNM         │            │
│  │ • ROS bags    │   │ • Augmentation│   │ • Diffusion   │            │
│  └───────────────┘   └───────────────┘   └───────┬───────┘            │
│                                                   │                    │
│                                                   ▼                    │
│                      ┌───────────────────────────────────────┐        │
│                      │ Evaluation & Deployment               │        │
│                      │                                       │        │
│                      │ • Isaac Sim closed-loop testing       │        │
│                      │ • Nav2 baseline comparison            │        │
│                      │ • CostNav economic metrics            │        │
│                      └───────────────────────────────────────┘        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
CostNav/
├── costnav_isaacsim/
│   └── il_training/                        # IL Training (data processing & model training)
│       ├── data_processing/                 # ROS2 bag → ViNT format conversion
│       ├── training/                        # Model training scripts and configs
├── il_evaluation/                       # IL Evaluation (inference & ROS2 nodes)
│
├── Dockerfile.ros_torch                     # IL evaluation Docker image (ROS2 + PyTorch)
└── third_party/
    └── visualnav-transformer/               # ViNT/NoMaD/GNM training code (submodule)
```

See [il_training/README.md](../costnav_isaacsim/il_training/README.md) for detailed directory structure.

---

## Data Collection

Teleoperation is done via:

```bash
make teleop
```

---

## Data Processing

### Environment Setup

Data processing uses **uv** for dependency management:

```bash
cd CostNav/costnav_isaacsim
uv sync  # Creates .venv with all dependencies
```

### Pipeline Overview

```
ROS2 Bags → MediaRef → Processing → ViNT Format
```

### MediaRef: Lightweight Media References

We use [MediaRef](https://github.com/open-world-agents/MediaRef) for efficient media handling:

- **Lazy loading**: References are lightweight (~40 bytes) until media is accessed
- **Video frame extraction**: Direct timestamp-based access to video frames
- **Batch decoding**: Optimized video frame loading (4.9× faster throughput)

```python
from mediaref import MediaRef, batch_decode

# Create lightweight references (no I/O until accessed)
ref = MediaRef(uri="episode_01.mp4", pts_ns=1_000_000_000)  # Frame at 1.0s

# Load when needed
rgb = ref.to_ndarray()  # Returns (H, W, 3) RGB array

# Batch decode for efficient video loading
refs = [MediaRef(uri="video.mp4", pts_ns=int(i*1e9)) for i in range(10)]
frames = batch_decode(refs)  # Opens video once, reuses handle
```

### Step 1: Convert ROS Bags to MediaRef

```bash
# From CostNav/costnav_isaacsim/il_training/
uv run python -m il_training.data_processing.converters.ray_batch_convert \
    --config data_processing/configs/processing_config.yaml
```

This creates for each bag:

- `bagfile_name.mcap` - Trajectory data with MediaRef pointers
- `bagfile_name.media/` - Extracted video files (MP4)

### Step 2: Convert to ViNT Format

```bash
# From CostNav/costnav_isaacsim/il_training/
uv run python -m il_training.data_processing.process_data.process_mediaref_bags \
    --config data_processing/configs/vint_processing_config.yaml
```

### SLURM Batch Processing

For large-scale processing on a cluster:

```bash
cd costnav_isaacsim/il_training/scripts/
sbatch process_data.sbatch
```

See [il_training/README.md](../costnav_isaacsim/il_training/README.md) for detailed configuration options.

---

### Dataset Format (Primary)

```
dataset_name/
└── bagfile_name/                    # Folder for each bagfile
    ├── metadata.json                # Dataset metadata
    └── bagfile_name.mcap          # Trajectory data
```

### MediaRef Format (Secondary)

```
dataset_name/
└── bagfile_name/                    # Folder for each bagfile
    ├── metadata.json                # Dataset metadata
    ├── bagfile_name.media/          # Source video files
    │   ├── front_camera_compressed.mp4 # Topic name
    │   ├── left_camera_compressed.mp4
    │   ├── right_camera_compressed.mp4
    │   └── ...
    └── bagfile_name.mcap          # Trajectory data
```

---

## Training Framework

### uv-based Training Environment

Training uses **uv** for dependency management with PyTorch CUDA 12.4+ support:

```bash
cd CostNav/costnav_isaacsim
uv sync  # Installs PyTorch, visualnav-transformer, and all dependencies
```

#### Environment Setup

**Prerequisites:**

- [uv](https://docs.astral.sh/uv/) package manager
- CUDA 12.4+ capable GPU and drivers

**Configuration:**

Before training, configure the `.env` file at the project root:

```bash
# In CostNav/.env
PROJECT_ROOT=/path/to/CostNav
```

#### Download Pretrained Checkpoints

Download pretrained models from [visualnav-transformer checkpoints](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing):

```bash
# Create checkpoints directory at repository root
mkdir -p checkpoints
cd checkpoints

# Download using gdown
uv run gdown --folder https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg
```

Expected structure:

```
checkpoints/
├── vint.pth      # ViNT pretrained weights
├── nomad.pth     # NoMaD pretrained weights
└── gnm.pth       # GNM pretrained weights
```

#### Training ViNT

```bash
# From CostNav/costnav_isaacsim/
uv run python -m il_training.training.train_vint \
    --config il_training/training/visualnav_transformer/configs/vint_costnav.yaml
```

#### SLURM Training

For cluster training:

```bash
cd costnav_isaacsim/il_training/scripts/
sbatch train_vint.sbatch
```

The sbatch script uses `uv run` — no manual venv activation needed.

---

## Evaluation

### Evaluation Metrics

#### CostNav Economic Metrics

| Metric               | Description                                 |
| -------------------- | ------------------------------------------- |
| **SLA Compliance**   | Percentage of deliveries within time budget |
| **Operating Margin** | Revenue - Operating Costs                   |
| **Break-even Time**  | Time to recover capital investment          |
| **Cost per Mission** | Average cost per navigation task            |

---

## Implementation Roadmap

### Phase 1: Data Processing Pipeline ✅

- [x] ~~Set up Docker-based visualnav-transformer environment~~ (Skipped - using **uv** for faster reproduction)
- [x] Implement `ray_batch_convert.py` converter (ROS bags → MediaRef with parallel processing)
- [x] Add `process_mediaref_bags.py` for ViNT format conversion

### Phase 2: Training Framework ✅

- [x] Create CostNav-specific training configs for ViNT/NoMaD/GNM
  - Config: `costnav_isaacsim/il_training/training/visualnav_transformer/configs/vint_costnav.yaml`
  - Training script: `costnav_isaacsim/il_training/training/train_vint.py`
- [x] Implement pre-trained model fine-tuning pipeline
- [x] Test training with collected sidewalk navigation data ✅ **Training successful!**

### Phase 3: Evaluation & Comparison 🔄 (In Progress)

**Current Status:**

- ✅ ViNT model (trained on Nova Carter data) runs successfully in Isaac Sim
- ✅ Two-node architecture implemented (policy node + trajectory follower)
- ✅ Topological graph navigation implemented (NavMesh-to-Topomap pipeline)

**Completed Tasks:**

- [x] Setup uv environment + Slurm setup
- [x] **(Critical)** Implement topological graph navigation
  - NavMesh-based `TopomapGenerator` generates ViNT-compatible topomaps from Isaac Sim shortest paths
  - `ViNTPolicyNode` supports `topomap` goal mode with batched subgoal inference, localization, and goal tracking
  - `VintAgent.step_topomap()` runs batched distance prediction against a sliding window of subgoal images
  - Online topomap generation: NavMesh query → waypoint interpolation → virtual camera capture → sequential PNGs
  - Configurable via `mission_config.yaml` (`topomap:` key) and `vint_eval.yaml` (radius, threshold)
  - Enabled via `make run-vint` or `make run-isaac-sim TOPOMAP=True`
  - See [TOPOMAP_PIPELINE.md](../costnav_isaacsim/costnav_isaacsim/TOPOMAP_PIPELINE.md) for full details

**Remaining Tasks:**

- [ ] Train and compare with Segway E1 data (2h, 4h, 6h training runs)

---

## Implementation Details

### Strategy Overview

The key objective is **not** to implement NavDP in isolation. Instead, we first implement **ViNT (Visual Navigation Transformer)** using a ROS2-native architecture for open-source usability.

**Why this approach?**

1. **ROS2 native communication**: Uses standard ROS2 topics instead of HTTP for better open-source compatibility and integration with existing robotics stacks
2. **Unified evaluation framework**: All baselines share the same ROS2 interface, enabling fair comparison
3. **Two-node architecture**: Decoupled policy inference (~10Hz) + MPC trajectory follower (~20Hz) as separate ROS2 nodes
4. **Easy baseline addition**: Once ViNT works, adding NoMaD, GNM, and NavDP follows the same pattern

### Communication Architecture

CostNav uses **ROS2** as its communication layer. The ViNT container runs two nodes that communicate with Isaac Sim:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                 CostNav IL Evaluation Architecture                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Isaac Sim Container              ViNT Container (ROS2 Jazzy)           │
│  ┌────────────────────┐          ┌────────────────────────────────┐    │
│  │ launch.py          │          │ vint_policy_node (~10Hz)       │    │
│  │ - Physics sim      │          │ trajectory_follower_node (~20Hz)│    │
│  │ - Nova Carter      │          └────────────────────────────────┘    │
│  │ - ROS2 Bridge      │                       │                        │
│  └────────────────────┘                       │                        │
│           │                                   │                        │
│           │  ROS2 Topics (via ROS_DOMAIN_ID)  │                        │
│           └───────────────────────────────────┘                        │
│                                                                         │
│  Isaac Sim → ViNT:     /front_stereo_camera/left/image_raw             │
│                        /chassis/odom                                    │
│                        /goal_image                                      │
│                                                                         │
│  ViNT → Isaac Sim:     /cmd_vel                                        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key ROS2 Topics (between containers):**

| Topic                                 | Type                  | Direction        | Description                     |
| :------------------------------------ | :-------------------- | :--------------- | :------------------------------ |
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image`   | Isaac Sim → Policy | Camera images for policy        |
| `/chassis/odom`                       | `nav_msgs/Odometry`   | Isaac Sim → Policy | Robot odometry for MPC          |
| `/goal_image`                         | `sensor_msgs/Image`   | Isaac Sim → Policy | Goal image (ImageGoal mode)     |
| `/cmd_vel`                            | `geometry_msgs/Twist` | Policy → Isaac Sim | Velocity commands to robot      |
| `/model_enable`                        | `std_msgs/Bool`       | Isaac Sim → Policy | Enable/disable policy execution |
| `/model_trajectory`                    | `nav_msgs/Path`       | Internal (Policy)  | Policy → Trajectory Follower    |

All IL baselines share the same `/model_*` topic names so the trajectory follower and RViz configs remain model-agnostic.

### ROS2 Node Interface (Abstract)

The IL evaluation framework uses a two-node architecture that can be extended for other models:

**1. Policy Node** - Runs model inference and publishes trajectory

```python
class BasePolicyNode(Node):
    """Abstract base class for IL policy nodes."""

    def __init__(self, agent: BaseAgent, inference_rate: float = 10.0):
        # Subscribers
        self.create_subscription(Image, '/front_stereo_camera/left/image_raw', ...)
        self.create_subscription(Image, '/goal_image', ...)  # ImageGoal mode
        self.create_subscription(Bool, '/model_enable', ...)

        # Publishers
        self.trajectory_pub = self.create_publisher(Path, '/model_trajectory', 10)

        # Inference timer
        self.create_timer(1.0 / inference_rate, self.inference_callback)

    def inference_callback(self):
        """Run policy inference and publish trajectory."""
        trajectory = self.agent.step(self.current_image, self.goal_image)
        self.trajectory_pub.publish(self.trajectory_to_path(trajectory))
```

**2. Trajectory Follower Node** - MPC controller for trajectory tracking

```python
class TrajectoryFollowerNode(Node):
    """MPC-based trajectory follower for IL policies."""

    def __init__(self, control_rate: float = 20.0):
        # Subscribers
        self.create_subscription(Path, '/<model>_trajectory', ...)
        self.create_subscription(Odometry, '/chassis/odom', ...)
        self.create_subscription(Bool, '/trajectory_follower_enable', ...)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer
        self.create_timer(1.0 / control_rate, self.control_callback)

    def control_callback(self):
        """Run MPC and publish velocity command."""
        v, w = self.mpc_controller.step(self.current_pose)
        self.cmd_vel_pub.publish(self.create_twist(v, w))
```

See [evaluation/README.md](../costnav_isaacsim/il_evaluation/README.md) for implementation details.

---

### Phase 4: ViNT Implementation (First Baseline) 🔄

ViNT (Visual Navigation Transformer) is being implemented as the reference baseline using the NavDP framework.

**Current Status:**

- ✅ Training completed with sidewalk navigation data
- 🔄 Evaluation integration in progress (using NavDP framework)

#### 4.1 ViNT Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      ViNT Policy Architecture                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐            │
│  │ Visual        │   │ Goal Image    │   │ Transformer   │            │
│  │ Encoder       │   │ Encoder       │   │ Decoder       │            │
│  │               │   │               │   │               │            │
│  │ • EfficientNet│   │ • EfficientNet│   │ • Attention   │            │
│  │ • Context (5) │   │ • Shared      │   │ • 8 waypoints │            │
│  │ • 224x224     │   │ • Weights     │   │ • Distance    │            │
│  └───────────────┘   └───────────────┘   └───────┬───────┘            │
│          │                   │                   │                    │
│          └───────────────────┴───────────────────┘                    │
│                              │                                         │
│                              ▼                                         │
│                    ┌───────────────────┐                              │
│                    │ Trajectory Output │                              │
│                    │ (8 waypoints)     │                              │
│                    └───────────────────┘                              │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

#### 4.2 ViNT Integration Tasks

**Training (Completed):**

- [x] Create CostNav-specific ViNT config (`vint_costnav.yaml`)
- [x] Process sidewalk navigation data to ViNT format
- [x] Train ViNT model with collected data

**Evaluation (Implemented ✅):**

- [x] Port ViNT agent from `third_party/NavDP/baselines/vint/vint_agent.py`
  - Location: `costnav_isaacsim/il_evaluation/src/il_evaluation/agents/vint_agent.py`
- [x] Create ROS2 policy node that:
  - Subscribes to `/front_stereo_camera/left/image_raw` (sensor_msgs/Image)
  - Subscribes to `/goal_image` for ImageGoal mode
  - Publishes `/model_trajectory` (nav_msgs/Path)
  - Location: `costnav_isaacsim/il_evaluation/src/il_evaluation/nodes/vint_policy_node.py`
- [x] Implement MPC trajectory follower node
  - Subscribes to `/model_trajectory` and `/chassis/odom`
  - Publishes `/cmd_vel` directly
  - Location: `costnav_isaacsim/il_evaluation/src/il_evaluation/nodes/trajectory_follower_node.py`

#### 4.3 CostNav ROS2 Integration ✅

**Running ViNT in Isaac Sim:**

```bash
# Build shared ROS2 + PyTorch image for IL baselines (first time only)
make build-ros2-torch

# Start all containers (Isaac Sim + ViNT)
make run-vint
```

This launches:

- Isaac Sim with Nova Carter robot and ROS2 bridge
- ViNT policy node (`vint_policy_node`)
- Shared trajectory follower service (`ros2-trajectory-follower`) running `trajectory_follower_node`

See [evaluation/README.md](../costnav_isaacsim/il_evaluation/README.md) for detailed usage and configuration options.

---

### Phase 5: Additional Baselines

Baselines integrated into the same two-node architecture (see [ROS2 Node Interface](#ros2-node-interface-abstract) above):

- [x] **NoMaD** - Diffusion-based navigation policy (ROS2 policy node + trajectory follower; `make run-nomad`)
- [x] **GNM** - General Navigation Model (ROS2 policy node + trajectory follower; `make run-gnm`)
- [ ] **NavDP** - HTTP-based baseline (planned)

**Navigation mode override:** use `GOAL_TYPE` to switch between `image_goal` and `topomap` when running a baseline.
This auto-syncs `GOAL_IMAGE` and `IL_TOPOMAP` unless you explicitly override them.

```bash
# Image-goal mode
GOAL_TYPE=image_goal MODEL_CHECKPOINT=checkpoints/nomad.pth make run-nomad

# Topomap mode
GOAL_TYPE=topomap MODEL_CHECKPOINT=checkpoints/nomad.pth make run-nomad
```

---

### Baseline Comparison

| Feature               | ViNT          | NoMaD         | GNM           | NavDP               |
| --------------------- | ------------- | ------------- | ------------- | ------------------- |
| **Architecture**      | Transformer   | Diffusion     | CNN           | Diffusion + Critic  |
| **Goal Support**      | Image, NoGoal | Image, NoGoal | Image, NoGoal | Point, Image, Pixel |
| **Trajectory Length** | 8 waypoints   | 8 waypoints   | 5 waypoints   | 24 waypoints        |
| **Context Frames**    | 5             | 5             | 5             | 8                   |
| **Implementation**    | ✅ Implemented | ✅ Implemented | ✅ Implemented | Planned             |

### References

- [ViNT: A Foundation Model for Visual Navigation](https://arxiv.org/abs/2306.14846)
- [NoMaD: Goal Masked Diffusion Policies](https://arxiv.org/abs/2310.07896)
- [GNM: A General Navigation Model](https://arxiv.org/abs/2210.03370)
- [NavDP Paper](https://arxiv.org/abs/2505.08712)
- [NavDP GitHub](https://github.com/OpenRobotLab/NavDP)
- See also: [NavDP Baseline Integration](navdp.md)

---

## References

### Research Papers

1. **GNM**: Shah et al., "GNM: A General Navigation Model to Drive Any Robot", ICRA 2023
2. **ViNT**: Shah et al., "ViNT: A Foundation Model for Visual Navigation", CoRL 2023
3. **NoMaD**: Sridhar et al., "NoMaD: Goal Masking Diffusion Policies for Navigation and Exploration", 2023

### Code References

- [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) - ViNT, NoMaD, GNM implementations
- [MediaRef](https://github.com/open-world-agents/MediaRef) - Lightweight media references

### Related CostNav Documentation

- [CostNav Architecture](architecture.md)
- [Cost Model](cost_model.md)
- [Training Guide](training_guide.md)
- [Nav2 Implementation](nav2/nav2_implementation_plan.md)

---
