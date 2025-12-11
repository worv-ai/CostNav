# CostNav Isaac Sim

Isaac Sim integration for the CostNav project with Nav2 navigation support.

## Quick Start

### 1. Build Docker Images

```bash
# Build Isaac Sim image
make build-isaac-sim

# Build ROS2 workspace (required for Nav2)
# clean up build_ws if error
# sudo rm -rf third_party/IsaacSim-ros_workspaces/build_ws
make build-ros-ws

# Build ROS2 runtime image
make build-ros2
```

### 2. Run Nav2 Navigation

```bash
# Run both Isaac Sim and ROS2 Nav2 together (recommended)
make run-nav2
```

This starts:

- **Isaac Sim**: Street Sidewalk environment with Nova Carter robot
- **ROS2 Nav2**: Carter navigation with pre-configured parameters

### 3. Alternative: Run Services Separately

```bash
# Terminal 1: Isaac Sim only
make run-isaac-sim

# Terminal 2: ROS2 Nav2 only
make run-ros2
```

## Docker Compose Profiles

The project uses Docker Compose profiles to manage different service combinations:

| Profile     | Services              | Command              |
| ----------- | --------------------- | -------------------- |
| `nav2`      | Isaac Sim + ROS2 Nav2 | `make run-nav2`      |
| `isaac-sim` | Isaac Sim only        | `make run-isaac-sim` |
| `ros2`      | ROS2 Nav2 only        | `make run-ros2`      |

### Using Profiles Directly

```bash
# Run both services with nav2 profile
docker compose --profile nav2 up

# Run individual services
docker compose --profile isaac-sim up isaac-sim
docker compose --profile ros2 up ros2
```

## Files

| File           | Description                                                         |
| -------------- | ------------------------------------------------------------------- |
| `launch.py`    | Isaac Sim launcher with SimulationContext and proper physics timing |
| `nav2_params/` | Nav2 configuration files for Nova Carter navigation                 |

## Configuration

### Default Environment

```
omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd
```

### Nav2 Parameters

- `nav2_params/carter_navigation_params.yaml` - Nav2 configuration
- `nav2_params/carter_sidewalk.yaml` - Map configuration
- `nav2_params/carter_sidewalk.png` - Map image

## Requirements

- Docker with NVIDIA Container Toolkit
- NVIDIA GPU with driver 525.60.11+
- Access to Omniverse server at `10.50.2.21`
