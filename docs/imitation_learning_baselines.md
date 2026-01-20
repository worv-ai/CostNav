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
│   ├── ...                                  # Existing IsaacSim modules
│   │
│   └── il_baselines/                        # Navigation Imitation Learning Module
│       ├── README.md                        # Module documentation
│       ├── __init__.py
│       ├── Dockerfile                       # Reproduces visualnav-transformer
│       ├── docker-compose.yml
│       ├── training/                        # Training Framework (via Docker)
│       │   ├── __init__.py
│       │   ├── visualnav_transformer/       # ViNT/NoMaD/GNM
│       │   │   └── configs/
│       │   │       ├── vint_costnav.yaml
│       │   │       ├── nomad_costnav.yaml
│       │   │       └── gnm_costnav.yaml
│       │   ├── process_data/
│       │   │   ├── rosbag_to_mediaref.py    # ROS2 mcaps → MediaRef
│       │   │   └── mediaref_to_vint.py      # MediaRef → ViNT format
│       │   ├── data_split.py
│       │   └── configs/
│       │       └── processing_config.yaml
│       │
│       ├── evaluation/                      # Evaluation & Deployment
│       │   └── __init__.py
```

---

## Data Collection

Teleoperation is done via:

```bash
make teleop
```

---

## Data Processing

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
python costnav_isaacsim/il_baselines/data_processing/converters/rosbag_to_mediaref.py \
    --input_dir rosbags/ \
    --output_dir datasets/processed/sidewalk_teleop/
```

This creates:

- `trajectories.mcap` - Trajectory data with MediaRef columns
- `trajectories.media/` - Source video files referenced by MediaRef

### Step 2: Convert to ViNT Format

```bash
python costnav_isaacsim/il_baselines/data_processing/converters/mediaref_to_vint.py \
    --input datasets/processed/sidewalk_augmented/ \
    --output datasets/vint_format/sidewalk/ \
    --config costnav_isaacsim/il_baselines/data_processing/configs/processing_config.yaml \
    --train_ratio 0.8 \
    --val_ratio 0.1 \
    --test_ratio 0.1
```

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

### Docker-based visualnav-transformer

We use Docker to reproduce the [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) environment for training ViNT, NoMaD, and GNM models.

#### Docker Setup

```bash
# Build the Docker image
cd costnav_isaacsim/il_baselines/training/visualnav_transformer/
docker-compose build
```

#### Docker Compose Configuration

```yaml
# costnav_isaacsim/il_baselines/training/visualnav_transformer/docker-compose.yml
version: "3.8"
services:
  vint-train:
    build:
      context: .
      dockerfile: Dockerfile
    volumes:
      - datasets-path:/data:ro
      - outputs-path:/outputs
    command: python train/train_vint.py
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

  nomad-train:
    extends: vint-train
    command: python train/train_nomad.py

  gnm-train:
    extends: vint-train
    command: python train/train_gnm.py
```

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

- [x] ~~Set up Docker-based visualnav-transformer environment~~ (Skipped - using conda for faster reproduction)
- [x] Implement `rosbag_to_mediaref.py` converter (ROS bags → MediaRef)
- [x] Add `mediaref_to_vint.py` for ViNT format conversion

### Phase 2: Training Framework ✅

- [x] Create CostNav-specific training configs for ViNT/NoMaD/GNM
  - Config: `costnav_isaacsim/il_baselines/training/visualnav_transformer/configs/vint_costnav.yaml`
  - Training script: `costnav_isaacsim/il_baselines/training/train_vint.py`
- [x] Implement pre-trained model fine-tuning pipeline
- [x] Test training with collected sidewalk navigation data ✅ **Training successful!**

### Phase 3: Evaluation & Comparison 🔄 (In Progress)

- [ ] Create Isaac Sim policy evaluation script (using NavDP framework)
- [ ] Integrate CostNav economic metrics
- [ ] Build Nav2 baseline comparison pipeline
- [ ] Create visualization and reporting tools

---

## NavDP Framework-Based Implementation Plan

### Strategy Overview

The key objective is **not** to implement NavDP in isolation. Instead, we first implement **ViNT (Visual Navigation Transformer)** using the NavDP baseline framework's communication architecture and simulation interface.

**Why this approach?**

1. **Leverage proven infrastructure**: NavDP's HTTP-based decoupled architecture is battle-tested for simulator communication
2. **Unified evaluation framework**: All baselines share the same API endpoints, enabling fair comparison
3. **Dual-threaded execution**: Async planning (~10Hz) + control loop (simulation rate) pattern works well
4. **Easy baseline addition**: Once ViNT works, adding NoMaD, GNM, and NavDP follows the same pattern

### Communication Architecture Options

CostNav uses **ROS2** as its communication layer. We have two options for integrating IL baselines:

#### Option A: ROS2 Native (Recommended for CostNav)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                 CostNav ROS2 Communication Architecture                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌───────────────┐      ROS2 Topics     ┌───────────────┐              │
│  │ Isaac Sim     │◄────────────────────►│ IL Policy     │              │
│  │ + Teleop Node │                      │ ROS2 Node     │              │
│  └───────────────┘                      └───────┬───────┘              │
│         │                                       │                      │
│         │ /odom, /image, /depth                 │ Model Inference      │
│         │                                       ▼                      │
│         │                               ┌───────────────┐              │
│         │◄──── /cmd_vel_model ──────────│ ViNT / NoMaD  │              │
│         │                               │ / GNM Agent   │              │
│         ▼                               └───────────────┘              │
│  ┌───────────────┐                                                     │
│  │ Robot Control │ ← Teleop node forwards /cmd_vel_model to /cmd_vel   │
│  └───────────────┘   when model_input_switch is enabled                │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key ROS2 Topics (already available in CostNav):**

| Topic              | Type                        | Direction | Description                       |
| ------------------ | --------------------------- | --------- | --------------------------------- |
| `/odom`            | `nav_msgs/Odometry`         | Subscribe | Robot odometry (pose + velocity)  |
| `/front_*/image_*` | `sensor_msgs/Image`         | Subscribe | Camera images                     |
| `/cmd_vel_model`   | `geometry_msgs/Twist`       | Publish   | Model velocity commands           |
| `/is_model`        | `std_msgs/Bool`             | (teleop)  | Indicates model control is active |
| `/goal_pose`       | `geometry_msgs/PoseStamped` | Subscribe | Navigation goal (ImageGoal mode)  |

#### Option B: HTTP-based (NavDP Framework Style)

For compatibility with NavDP's existing baselines, we can also use HTTP:

```
Isaac Sim (ROS2) ←→ ROS2-HTTP Bridge ←→ Flask Policy Server
```

### ROS2 Policy Node Interface

```python
class ILPolicyNode(Node):
    """ROS2 node for IL policy inference."""

    def __init__(self, agent: BaseAgent):
        super().__init__('il_policy_node')
        self.agent = agent

        # Subscribers
        self.create_subscription(Image, '/front_stereo_camera/left/image_raw', self.image_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)  # For ImageGoal

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_model', 10)

        # Inference timer (~10Hz)
        self.create_timer(0.1, self.inference_callback)

    def inference_callback(self):
        """Run policy inference and publish velocity command."""
        trajectory = self.agent.step_imagegoal(self.goal_image, self.current_image, self.depth)
        cmd_vel = self.trajectory_to_cmd_vel(trajectory)
        self.cmd_vel_pub.publish(cmd_vel)
```

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

**Evaluation (Remaining - using ROS2 + NavDP agent pattern):**

- [ ] Port ViNT agent from `third_party/NavDP/baselines/vint/vint_agent.py`
- [ ] Create ROS2 policy node (`vint_ros2_node.py`) that:
  - Subscribes to `/front_stereo_camera/left/image_raw` (sensor_msgs/Image)
  - Subscribes to `/odom` (nav_msgs/Odometry)
  - Subscribes to `/goal_pose` or `/goal_image` for ImageGoal mode
  - Publishes `/cmd_vel_model` (geometry_msgs/Twist)
- [ ] Implement trajectory → cmd_vel conversion (from ViNT waypoints to Twist)
- [ ] Add model switch integration with teleop node (`/is_model` topic)
- [ ] Test with CostNav mission manager for automated evaluation

#### 4.3 ViNT Model Configuration

```yaml
# configs/vint_costnav.yaml
model:
  image_size: 224
  context_size: 5 # Temporal context frames
  waypoint_spacing: 0.25 # Waypoint interval (meters)
  num_images: 1 # Number of cameras
  len_traj_pred: 8 # Trajectory prediction length
  learn_angle: true # Predict heading angles

encoder:
  type: efficientnet_b0
  pretrained: true
  obs_encoding_size: 1024

inference:
  checkpoint: checkpoints/vint/vint.pth
  device: cuda:0
```

#### 4.4 ViNT ROS2 Node Implementation

```python
# costnav_isaacsim/il_baselines/evaluation/vint_ros2_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import torch
from vint_agent import ViNT_Agent

class ViNTROS2Node(Node):
    def __init__(self):
        super().__init__('vint_policy_node')

        # Load ViNT model
        checkpoint = self.declare_parameter('checkpoint', '').value
        self.agent = ViNT_Agent(checkpoint=checkpoint)
        self.agent.reset(batch_size=1)
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image, '/front_stereo_camera/left/image_raw',
                                 self.image_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_model', 10)

        # Inference timer (~10Hz)
        self.create_timer(0.1, self.inference_callback)
        self.current_image = None
        self.current_odom = None

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def odom_callback(self, msg):
        self.current_odom = msg

    def inference_callback(self):
        if self.current_image is None:
            return

        # Run ViNT inference
        trajectory = self.agent.step_nogoal(self.current_image, depth=None)

        # Convert first waypoint to velocity command
        cmd_vel = self.trajectory_to_twist(trajectory[0])
        self.cmd_vel_pub.publish(cmd_vel)

    def trajectory_to_twist(self, waypoint):
        """Convert waypoint (dx, dy, dtheta) to Twist message."""
        twist = Twist()
        # Simple proportional control (can be replaced with MPC)
        twist.linear.x = min(waypoint[0] * 2.0, 1.0)  # Forward velocity
        twist.angular.z = waypoint[2] * 1.5           # Angular velocity
        return twist
```

#### 4.5 CostNav ROS2 Integration

- [ ] Create ROS2 launch file for ViNT policy node
- [ ] Integrate with existing teleop node (`/cmd_vel_model` → `/cmd_vel`)
- [ ] Add mission manager support for automated IL evaluation
- [ ] Implement trajectory → velocity conversion (pure pursuit or MPC)

**Integration with Existing CostNav Infrastructure:**

```bash
# Terminal 1: Isaac Sim with ROS2 bridge
make run-isaacsim

# Terminal 2: ViNT Policy Node (new)
ros2 run costnav_il_baselines vint_policy_node --ros-args \
    --param checkpoint:=/path/to/vint_costnav.pth

# Terminal 3: Teleop node (enable model input)
make teleop  # Press button to switch to model control
```

---

### Phase 5: Additional Baselines

After ViNT is working, add other baselines using the same infrastructure:

#### 5.1 NoMaD (Diffusion Model)

- [ ] Port NoMaD agent from NavDP framework
- [ ] Implement `NoMaD_Agent` with diffusion-based action generation
- [ ] Support ImageGoal and NoGoal navigation modes

#### 5.2 GNM (General Navigation Model)

- [ ] Port GNM agent from NavDP framework
- [ ] Implement distance prediction + temporal distance head
- [ ] Support ImageGoal and NoGoal navigation modes

#### 5.3 NavDP (Full Implementation)

- [ ] Port NavDP agent with multi-goal support
- [ ] Set up DepthAnything V2 + DINOv2 encoders
- [ ] Implement diffusion policy with critic head
- [ ] Support PointGoal, ImageGoal, PixelGoal, and NoGoal modes

---

### Baseline Comparison

| Feature               | ViNT          | NoMaD         | GNM           | NavDP               |
| --------------------- | ------------- | ------------- | ------------- | ------------------- |
| **Architecture**      | Transformer   | Diffusion     | CNN           | Diffusion + Critic  |
| **Goal Support**      | Image, NoGoal | Image, NoGoal | Image, NoGoal | Point, Image, Pixel |
| **Trajectory Length** | 8 waypoints   | 8 waypoints   | 5 waypoints   | 24 waypoints        |
| **Context Frames**    | 5             | 5             | 5             | 8                   |
| **Implementation**    | **Phase 4**   | Phase 5       | Phase 5       | Phase 5             |

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
