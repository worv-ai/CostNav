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

| Algorithm            | Type               | Framework             | Description                       |
| -------------------- | ------------------ | --------------------- | --------------------------------- |
| **ViNT**             | Transformer        | visualnav-transformer | Vision transformer for navigation |
| **NoMaD**            | Diffusion          | visualnav-transformer | Goal-masked diffusion policies    |
| **GNM**              | General Navigation | visualnav-transformer | General navigation model          |
| **Diffusion Policy** | Diffusion          | diffusion_policy      | General diffusion imitation       |

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

### Phase 1: Data Processing Pipeline

- [ ] Set up Docker-based visualnav-transformer environment
- [ ] Implement `rosbag_to_mediaref.py` converter (ROS bags → MediaRef)
- [ ] Add `mediaref_to_vint.py` for ViNT format conversion

### Phase 2: Training Framework

- [ ] Create CostNav-specific training configs for ViNT/NoMaD/GNM
- [ ] Implement pre-trained model fine-tuning pipeline
- [ ] Test training with collected sidewalk navigation data

### Phase 3: Evaluation & Comparison

- [ ] Create Isaac Sim policy evaluation script
- [ ] Integrate CostNav economic metrics
- [ ] Build Nav2 baseline comparison pipeline
- [ ] Create visualization and reporting tools

---

## References

### Research Papers

1. **GNM**: Shah et al., "GNM: A General Navigation Model to Drive Any Robot", ICRA 2023
2. **ViNT**: Shah et al., "ViNT: A Foundation Model for Visual Navigation", CoRL 2023
3. **NoMaD**: Sridhar et al., "NoMaD: Goal Masking Diffusion Policies for Navigation and Exploration", 2023
4. **Diffusion Policy**: Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion", RSS 2023

### Code References

- [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) - ViNT, NoMaD, GNM implementations
- [MediaRef](https://github.com/open-world-agents/MediaRef) - Lightweight media references
- [Diffusion Policy](https://github.com/real-stanford/diffusion_policy) - General diffusion imitation

### Related CostNav Documentation

- [CostNav Architecture](architecture.md)
- [Cost Model](cost_model.md)
- [Training Guide](training_guide.md)
- [Nav2 Implementation](nav2/nav2_implementation_plan.md)

---
