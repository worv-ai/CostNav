# Canvas-CostNav

Evaluation pipeline for CostNav using Docker Compose. This setup runs the **neural planner** and **cmd_vel_publisher** agents against a simulation (or real) environment, with a remote model worker serving inference on a GPU server.

## Architecture

```
+------------------+         +-------------------------+
|   GPU Server     |         |   Evaluation Host       |
|   (e.g. H100)    |         |   (Docker Compose)      |
|                  |  HTTP   |                         |
|  model_worker  <----------->  neural_planner         |
|  :MODEL_WORKER_  |         |  cmd_vel_publisher      |
|   PORT           |         |                         |
+------------------+         +-------+-----------------+
                                     |  ROS 2 topics
                                     v
                             +-------+-----------------+
                             |  Simulation / Real Robot|
                             |  (Isaac Sim or Segway)  |
                             +-------------------------+
```

## Prerequisites

- Docker & Docker Compose
- NVIDIA GPU + NVIDIA Container Toolkit
- ROS 2 Jazzy (bundled inside the Docker image)

## Quick Start

### 1. Build the Docker Image

```bash
cd docker
./build.sh
```

This builds `canvas:latest`. You can override the image name and tag:

```bash
IMAGE_NAME=my-registry/canvas IMAGE_TAG=v1.0 ./build.sh
```

### 2. Launch the Model Worker (GPU Server)

> **Note:** A dedicated GPU server is recommended for production, but running both the simulation and model worker on the same desktop works fine — though it will degrade model inference performance due to shared GPU resources. Verified on:
> - **Motherboard:** ASUS ROG STRIX B760-G GAMING WIFI
> - **CPU:** Intel Core i7-13700K (16 cores / 24 threads)
> - **RAM:** 64 GB
> - **OS:** Ubuntu 24.04
> - **GPU:** NVIDIA GeForce RTX 4070 Ti SUPER (16 GB VRAM)

1. Edit `apps/model_workers/.env.pub`:

   | Variable            | Example               | Description                     |
   | ------------------- | --------------------- | ------------------------------- |
   | `MODEL_PATH`        | `/path/to/checkpoint` | Path to model weights directory |
   | `MODEL_WORKER_PORT` | `8200`                | Port for the FastAPI server     |
   | `CANVAS_VERSION`    | `latest`              | Docker image tag                |

   > `CUDA_VISIBLE_DEVICES` is set automatically by the sbatch script from the SLURM GPU allocation.

2. Submit via SLURM (recommended):

   ```bash
   cd apps/model_workers
   sbatch model_worker.sbatch
   ```

   Check the SLURM output file for the assigned port and GPU device:

   ```bash
   cat slurm-<job_id>.out
   ```

   Alternatively, to run directly without SLURM:

   ```bash
   cd apps/model_workers
   cp .env.pub .env
   # Edit .env to set CUDA_VISIBLE_DEVICES manually
   docker compose up
   ```

### 3. Launch the Agent Services (Evaluation Host)

1. Copy the environment template and configure it:

   ```bash
   cd apps/agent
   cp .env.pub .env
   ```

2. Edit `.env`:

   | Variable           | Example            | Description                                     |
   | ------------------ | ------------------ | ----------------------------------------------- |
   | `MODEL_WORKER_URI` | `http://<ip>:8200` | URL of the model worker (IP:port of GPU server) |
   | `CANVAS_VERSION`   | `latest`           | Docker image tag                                |

3. Start the agent services:

   ```bash
   docker compose up
   ```

   This starts two ROS 2 nodes:
   - **neural_planner** - processes sensor data, queries model worker, publishes velocity predictions
   - **cmd_vel_publisher** - converts predictions to safe velocity commands for the robot

## Configuration

### Neural Planner

Config file: [`apps/agent/config/config.yml`](apps/agent/config/config.yml)

```yaml
odom_topic: /chassis/odom
rgb_front_topic: /front_stereo_camera/left/image_raw
rgb_image_type: Image # Image | CompressedImage
cmd_vel_msg_type: Twist # Twist | TwistStamped

rate: 20 # control loop frequency (Hz)

visualize: true # publish visualization topics
reached_distance: 0.7 # goal-reached threshold (m)
```

Additional parameters available in `NeuralPlannerConfig` (set in code, overridable via YAML):

| Parameter                   | Default              | Description                              |
| --------------------------- | -------------------- | ---------------------------------------- |
| `map_yaml`                  | `maps/sidewalk.yaml` | Path to the map YAML file                |
| `num_action`                | `20`                 | Number of predicted action steps         |
| `action_dim`                | `2`                  | Dimension per action (`[v, w]`)          |
| `stopped_timeout`           | `10.0`               | Seconds before declaring stopped         |
| `stopped_threshold`         | `0.5`                | Linear distance threshold (m)            |
| `stopped_angular_threshold` | `0.5`                | Angular distance threshold (rad)         |
| `signal_reached_distance`   | `2.0`                | Distance to start signaling approach (m) |

### Velocity Publisher

Config file: [`apps/agent/config/segway_e1.yml`](apps/agent/config/segway_e1.yml)

```yaml
vel_topic: /cmd_vel
cmd_vel_msg_type: "Twist" # Twist | TwistStamped

max_v: 15 # linear speed limit (m/s)
max_w: 10 # angular speed limit (rad/s)
v_gain: 1.0 # proportional gain for linear velocity
w_gain: 1.0 # proportional gain for angular velocity

zero_velocity_threshold_linear: 0.0
zero_velocity_threshold_angular: 0.0
```

Additional parameters available in `CMDVelPublisherConfig`:

| Parameter                     | Default | Description                            |
| ----------------------------- | ------- | -------------------------------------- |
| `rate`                        | `20`    | Publishing frequency (Hz)              |
| `vel_timeout`                 | `1.0`   | Seconds before zeroing stale velocity  |
| `enable_latency_compensation` | `true`  | Compensate for model inference latency |

### Maps

Place map files (image + YAML) in [`apps/agent/maps/`](apps/agent/maps/). The YAML file specifies the map image, resolution, and origin:

```yaml
image: sidewalk_orthographic.png
resolution: 0.05 # meters per pixel
origin: [-100.875, -100.875, 0.0] # [x, y, theta] in meters
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Customization

To use a different robot or config, update the Docker Compose command:

```yaml
command: /bin/bash -c "source /opt/ros/jazzy/setup.bash && python3 neural_planner.py --config config/<your_config.yml>"
```

## Project Structure

```
canvas-costnav/
├── docker/
│   ├── Dockerfile           # CUDA 12.8 + ROS 2 Jazzy + PyTorch 2.7.1
│   ├── build.sh             # Build script (canvas:latest)
│   └── ros_entrypoint.sh    # ROS 2 environment wrapper
├── apps/
│   ├── agent/               # Neural planner + velocity publisher
│   │   ├── docker-compose.yml
│   │   ├── .env.pub         # Environment template
│   │   ├── config/          # YAML configs
│   │   └── maps/            # Map images + metadata
│   └── model_workers/       # Model inference service
│       ├── docker-compose.yml
│       └── .env.pub         # Environment template
├── src/canvas/              # Python package
│   ├── agent/
│   │   ├── neural_planner/  # ROS 2 node: sensor → model → velocity
│   │   └── cmd_vel_publisher/ # ROS 2 node: velocity → safe commands
│   └── model_worker/        # FastAPI app: InternVL3 inference
└── pyproject.toml
```
