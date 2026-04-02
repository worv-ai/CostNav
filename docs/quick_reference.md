# :zap: Quick Reference

Quick commands for common tasks in CostNav.

---

## :package: Installation

### Prerequisites

- Linux host PC (Ubuntu 24.04 preferred)
- NVIDIA GPU with recent graphics drivers
- Docker with NVIDIA container toolkit

### Setup

```bash
git clone https://github.com/worv-ai/CostNav.git
cd CostNav
make fetch-third-party
```

### Configure Environment

1. Copy `.env.example` to `.env`
2. Set `NGC_PASS` — create an API key at [NGC](https://org.ngc.nvidia.com/setup/api-keys)
3. Set `PROJECT_ROOT` as the absolute path of cloned `CostNav`
4. Set `HF_TOKEN` — create a token at [HuggingFace](https://huggingface.co/settings/tokens) (required for asset download)

### Build

```bash
make build-ros2
make build-isaac-sim
```

### Download Assets

```bash
make download-assets-hf            # requires HF_TOKEN
make download-baseline-checkpoints-hf  # download pretrained IL models
make start-nucleus
# make stop-nucleus  # when done
```

Teleop dataset: [maum-ai/CostNav-Teleop-Dataset](https://huggingface.co/datasets/maum-ai/CostNav-Teleop-Dataset)

---

## :rocket: Running Nav2 (Rule-Based Navigation)

```bash
make run-nav2
# Defaults: NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True TUNED=True AMCL=False

# Then run ONE of the following:
make start-mission    # single mission
make run-eval-nav2    # batch evaluation
```

This starts Isaac Sim with the Street Sidewalk environment and Segway E1 robot, along with the ROS2 Nav2 stack.

---

## :robot: Running IL Baselines

CostNav supports the following IL baselines, adapted from [NavDP](https://github.com/InternRobotics/NavDP):

| Baseline   | Architecture           | Supported Tasks              | Run Command | Eval Command |
|:-----------|:-----------------------|:-----------------------------|:------------|:-------------|
| **ViNT**   | Transformer            | ImageGoal, NoGoal            | `make run-vint` | `make run-eval-vint` |
| **NoMaD**  | Diffusion              | ImageGoal, NoGoal            | `make run-nomad` | `make run-eval-nomad` |
| **GNM**    | CNN                    | ImageGoal, NoGoal            | `make run-gnm` | `make run-eval-gnm` |
| **NavDP**  | Diffusion + Critic     | PointGoal, ImageGoal, NoGoal | `make run-navdp` | `make run-eval-navdp` |
| **Canvas** | Vision-Language Action | Sketch+Language Goal         | `make run-canvas` | `make run-eval-canvas` |

### Quick Start

```bash
# 1. Download pretrained checkpoints
make download-baseline-checkpoints-hf

# 2. Build docker image
make build-ros2-torch

# 3. Run (example: ViNT)
# Terminal 1: Start the ViNT stack
make run-vint

# Terminal 2: Run evaluation
make run-eval-vint
```

### :art: Canvas (VLA Learning-Based Navigation)

```bash
# 1. Build the Canvas Docker image
make build-canvas

# 2. Start Isaac Sim + Canvas agent (includes model worker)
make run-canvas
# Defaults: MODEL_PATH=./checkpoints/canvas-costnav MODEL_WORKER_URI=http://localhost:8200

# 3. Run evaluation
make run-eval-canvas
```

> **Tip:** To offload the model worker to a separate GPU server, see **[Baselines](baselines.md)**.

---

## :joystick: Running Teleop (Data Collection)

```bash
make run-teleop
# Defaults: NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True TOPOMAP=True

make run-rosbag       # start rosbag record
make start-mission    # start single mission
make stop-rosbag      # stop rosbag record when mission is completed
make run-eval-teleop  # run evaluation
```

> **Tip:** Press **Ctrl+C once** to stop teleop. The teardown runs automatically — do not press Ctrl+C again while containers are being cleaned up.

---

## :file_folder: Project Structure

```
CostNav/
├── costnav_isaacsim/
│   ├── costnav_isaacsim/          # Isaac Sim simulation & mission management
│   ├── canvas/                    # Canvas sketch-based navigation agent
│   ├── il_training/               # IL data processing + model training
│   ├── il_evaluation/             # IL inference + ROS2 policy nodes
│   ├── isaac_sim_teleop_ros2/     # ROS2 teleoperation package
│   └── nav2_params/               # Nav2 launch files & parameters
├── Dockerfile                     # Isaac Sim & Isaac Lab (multi-stage)
├── Dockerfile.ros                 # ROS2 Jazzy (teleop + nav2)
├── Dockerfile.ros_torch           # ROS2 Jazzy + PyTorch (IL evaluation)
└── docker-compose.yml
```

### Component Environments

| Component               | Runtime                | Notes                                          |
|:------------------------|:-----------------------|:-----------------------------------------------|
| `costnav_isaacsim`      | NVIDIA Isaac Sim 5.1.0 | Requires NGC + GPU                             |
| `canvas`                | ROS2 Jazzy + PyTorch   | GPU inference                                  |
| `il_training`           | Bare-metal / SLURM     | CPU-only for data processing; GPU for training |
| `il_evaluation`         | ROS2 Jazzy + PyTorch   | GPU inference                                  |
| `isaac_sim_teleop_ros2` | ROS2 Jazzy             | Joystick teleoperation                         |
| `nav2_params`           | ROS2 Jazzy             | Launch files only                              |

### Docker Compose Profiles

| Profile     | Services                           | Command              | Use Case                       |
|:------------|:-----------------------------------|:---------------------|:-------------------------------|
| `nav2`      | Isaac Sim + ROS2 Nav2              | `make run-nav2`      | Full navigation stack          |
| `isaac-sim` | Isaac Sim only                     | `make run-isaac-sim` | Simulation development         |
| `ros2`      | ROS2 Nav2 only                     | `make run-ros2`      | Nav2 tuning (requires sim)     |
| `teleop`    | Isaac Sim + Teleop                 | `make run-teleop`    | Manual driving (joystick)      |
| `vint`      | Isaac Sim + ViNT Policy + Follower | `make run-vint`      | ViNT IL baseline evaluation    |
| `canvas`    | Isaac Sim + RViz + Model Worker + Canvas Bridge | `make run-canvas` | Canvas VLA navigation     |

```bash
# Direct docker compose usage
docker compose --profile nav2 up
docker compose --profile nav2 down
```

---

## :rotating_light: Common Issues

### :x: Nucleus server won't start

??? solution "Solution"
    ```bash
    make stop-nucleus
    make start-nucleus
    ```

### :boom: "CUDA out of memory"

??? solution "Solution"
    Reduce `NUM_PEOPLE` or ensure no other GPU processes are running:
    ```bash
    make run-nav2 NUM_PEOPLE=5
    ```

### :warning: Docker containers not cleaning up

??? solution "Solution"
    ```bash
    docker compose down --remove-orphans
    ```

---

## :link: Useful Links

| Resource | Link |
|:---------|:-----|
| :fontawesome-brands-github: GitHub | [github.com/worv-ai/CostNav](https://github.com/worv-ai/CostNav) |
| :book: Documentation | [worv-ai.github.io/CostNav](https://worv-ai.github.io/CostNav) |
| :robot: Isaac Sim | [developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim) |
| :package: Sim Assets | [maum-ai/CostNav](https://huggingface.co/datasets/maum-ai/CostNav) |
| :floppy_disk: Teleop Dataset | [maum-ai/CostNav-Teleop-Dataset](https://huggingface.co/datasets/maum-ai/CostNav-Teleop-Dataset) |
| :brain: Baseline Models | [maum-ai/CostNav_baseline](https://huggingface.co/maum-ai/CostNav_baseline) |
