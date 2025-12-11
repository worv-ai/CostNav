# CostNav Isaac Sim

> Isaac Sim integration for the CostNav project with Nav2 navigation support.

This module provides the **rule-based navigation baseline** for CostNav using NVIDIA Isaac Sim and ROS2 Nav2 stack. It enables direct comparison between learning-based (RL) and classical navigation approaches.

---

## Overview

### Purpose

The `costnav_isaacsim` module serves as:

1. **Rule-Based Baseline**: Industry-standard Nav2 navigation for benchmarking against RL policies
2. **Simulation Bridge**: Connects Isaac Sim physics with ROS2 navigation stack
3. **Multi-Container Architecture**: Separates simulation and navigation for modularity

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Docker Network (host)                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────┐    ┌─────────────────────────┐   │
│  │   Isaac Sim Container   │    │    ROS2 Nav2 Container  │   │
│  │                         │    │                         │   │
│  │  ┌───────────────────┐  │    │  ┌───────────────────┐  │   │
│  │  │    launch.py      │  │    │  │   Nav2 Stack      │  │   │
│  │  │  - Physics sim    │  │    │  │   - AMCL          │  │   │
│  │  │  - Nova Carter    │  │    │  │   - Planner       │  │   │
│  │  │  - Sensors        │  │    │  │   - Controller    │  │   │
│  │  └─────────┬─────────┘  │    │  │   - Costmaps      │  │   │
│  │            │            │    │  └─────────┬─────────┘  │   │
│  │  ┌─────────▼─────────┐  │    │            │            │   │
│  │  │  ROS2 Bridge      │◄─┼────┼────────────┘            │   │
│  │  │  Extension        │──┼────┼─► /cmd_vel              │   │
│  │  └───────────────────┘  │    │                         │   │
│  └─────────────────────────┘    └─────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Current Status

| Component              | Status         | Notes                              |
| ---------------------- | -------------- | ---------------------------------- |
| Nav2 Integration       | ✅ Complete    | Nova Carter navigates successfully |
| Docker Setup           | ✅ Complete    | Multi-container architecture       |
| Occupancy Map          | ✅ Complete    | Street_sidewalk environment        |
| Parameter Tuning       | ⏳ In Progress | Optimizing for performance         |
| Cost Model Integration | 📋 Planned     | Economic metrics tracking          |

> **See Also**: [Nav2 Implementation Plan](../docs/nav2/nav2_implementation_plan.md) for detailed roadmap.

---

## Directory Structure

```
costnav_isaacsim/
├── README.md                              # This file
├── launch.py                              # Isaac Sim launcher script
└── nav2_params/                           # Nav2 configuration files
    ├── carter_navigation_params.yaml      # Full Nav2 stack parameters
    ├── carter_sidewalk.yaml               # Map metadata (origin, resolution)
    └── carter_sidewalk.png                # Occupancy grid image
```

---

## Quick Start

### Prerequisites

- Docker with NVIDIA Container Toolkit
- NVIDIA GPU with driver 525.60.11+
- Access to Omniverse server at `10.50.2.21`
- X11 display forwarding configured

### 1. Build Docker Images

```bash
# From repository root
cd /path/to/CostNav

# Build Isaac Sim image
make build-isaac-sim

# Build ROS2 workspace (required for Nav2)
# Note: Clean up build_ws if errors occur
# sudo rm -rf third_party/IsaacSim-ros_workspaces/build_ws
make build-ros-ws

# Build ROS2 runtime image
make build-ros2
```

### 2. Run Nav2 Navigation (Recommended)

```bash
# Run both Isaac Sim and ROS2 Nav2 together
make run-nav2
```

This starts:

- **Isaac Sim**: Street Sidewalk environment with Nova Carter robot
- **ROS2 Nav2**: Carter navigation with pre-configured parameters

### 3. Alternative: Run Services Separately

```bash
# Terminal 1: Isaac Sim only
make run-isaac-sim

# Terminal 2: ROS2 Nav2 only (after Isaac Sim is ready)
make run-ros2
```

---

## Docker Compose Profiles

| Profile     | Services              | Command              | Use Case                           |
| ----------- | --------------------- | -------------------- | ---------------------------------- |
| `nav2`      | Isaac Sim + ROS2 Nav2 | `make run-nav2`      | Full navigation stack              |
| `isaac-sim` | Isaac Sim only        | `make run-isaac-sim` | Simulation development             |
| `ros2`      | ROS2 Nav2 only        | `make run-ros2`      | Nav2 tuning (requires running sim) |

### Using Profiles Directly

```bash
# Run both services with nav2 profile
docker compose --profile nav2 up

# Run individual services
docker compose --profile isaac-sim up isaac-sim
docker compose --profile ros2 up ros2

# Tear down
docker compose --profile nav2 down
```

---

## File Reference

### `launch.py` - Isaac Sim Launcher

Main entry point for Isaac Sim simulation with Nav2 support.

**Usage:**

```bash
python launch.py                                    # Default: GUI mode
python launch.py --headless                         # Headless mode (no GUI)
python launch.py --usd_path omniverse://path/to.usd # Custom USD scene
python launch.py --debug                            # Enable debug logging
```

**Command Line Arguments:**

| Argument         | Default                                                         | Description          |
| ---------------- | --------------------------------------------------------------- | -------------------- |
| `--usd_path`     | `omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd` | USD scene path       |
| `--headless`     | `false`                                                         | Run without GUI      |
| `--physics_dt`   | `1/60` (0.0167s)                                                | Physics timestep     |
| `--rendering_dt` | `1/30` (0.0333s)                                                | Rendering timestep   |
| `--debug`        | `false`                                                         | Enable debug logging |

**Key Features:**

- **ROS2 Bridge**: Enables `isaacsim.ros2.bridge` extension for Nav2 communication
- **Health Check**: Writes `/tmp/.isaac_sim_running` when ready (Docker healthcheck)
- **Warm-up Steps**: Runs 100 warm-up steps to stabilize physics timing
- **Throttled Loop**: Maintains consistent rendering framerate

### `nav2_params/` - Navigation Configuration

#### `carter_navigation_params.yaml`

Complete Nav2 stack configuration for Nova Carter robot.

**Key Components:**

| Component      | Plugin                             | Description                       |
| -------------- | ---------------------------------- | --------------------------------- |
| **AMCL**       | `nav2_amcl`                        | Adaptive Monte Carlo Localization |
| **Planner**    | `nav2_navfn_planner::NavfnPlanner` | Global path planning              |
| **Controller** | `dwb_core::DWBLocalPlanner`        | Dynamic Window Approach           |
| **Costmaps**   | `nav2_costmap_2d`                  | Local (6x6m) + Global costmaps    |
| **Behaviors**  | `nav2_behaviors`                   | Recovery: spin, backup, wait      |

**Important Parameters:**

```yaml
# Robot footprint (Nova Carter dimensions)
footprint: "[ [0.14, 0.25], [0.14, -0.25], [-0.607, -0.25], [-0.607, 0.25] ]"

# Velocity limits
max_vel_x: 0.8 # m/s forward
max_vel_theta: 0.7 # rad/s rotation

# Goal tolerances
xy_goal_tolerance: 0.25 # meters
yaw_goal_tolerance: 0.25 # radians

# Costmap layers
plugins: ["front_3d_lidar_layer", "inflation_layer", "static_map_layer"]
```

#### `carter_sidewalk.yaml`

Map metadata for the Street_sidewalk environment.

```yaml
image: carter_sidewalk.png # Occupancy grid image
resolution: 0.05 # 5cm per pixel
origin: [-100.875, -100.875, 0.0] # Map origin (meters)
occupied_thresh: 0.65 # Occupied cell threshold
free_thresh: 0.196 # Free cell threshold
```

#### `carter_sidewalk.png`

Occupancy grid image (grayscale):

- **White (255)**: Free space
- **Black (0)**: Occupied
- **Gray (205)**: Unknown

---

## ROS2 Topics

### Published by Isaac Sim (via ROS2 Bridge)

| Topic                          | Type                      | Description    |
| ------------------------------ | ------------------------- | -------------- |
| `/chassis/odom`                | `nav_msgs/Odometry`       | Robot odometry |
| `/tf`, `/tf_static`            | `tf2_msgs/TFMessage`      | Transform tree |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 3D LiDAR data  |

### Subscribed by Isaac Sim

| Topic      | Type                  | Description                 |
| ---------- | --------------------- | --------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from Nav2 |

### Nav2 Topics

| Topic                     | Type                        | Description     |
| ------------------------- | --------------------------- | --------------- |
| `/goal_pose`              | `geometry_msgs/PoseStamped` | Navigation goal |
| `/map`                    | `nav_msgs/OccupancyGrid`    | Static map      |
| `/local_costmap/costmap`  | `nav_msgs/OccupancyGrid`    | Local costmap   |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid`    | Global costmap  |
| `/plan`                   | `nav_msgs/Path`             | Global path     |

---

## Sending Navigation Goals

### Via RViz2

1. Launch the nav2 profile: `make run-nav2`
2. Open RViz2 in the ROS2 container
3. Click "2D Goal Pose" button
4. Click on the map to set goal position and orientation

### Via Command Line

```bash
# Inside ROS2 container
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Via Python (Action Client)

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()
navigator.waitUntilNav2Active()

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 5.0
goal_pose.pose.position.y = 3.0
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)
navigator.waitUntilTaskComplete()
```

---

## Troubleshooting

### Common Issues

| Issue                                        | Solution                                          |
| -------------------------------------------- | ------------------------------------------------- |
| Isaac Sim fails to start                     | Check GPU drivers, NVIDIA Container Toolkit       |
| ROS2 container starts before Isaac Sim ready | Use `make run-nav2` (has health check dependency) |
| No ROS2 topics visible                       | Verify `ROS_DOMAIN_ID` matches between containers |
| Map not loading                              | Check Omniverse server connectivity               |
| Robot not moving                             | Verify `/cmd_vel` is being published              |

### Debug Commands

```bash
# Check ROS2 topics
ros2 topic list
ros2 topic echo /chassis/odom

# Check TF tree
ros2 run tf2_tools view_frames

# Check Nav2 status
ros2 lifecycle list /bt_navigator
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState

# View logs
docker logs costnav-isaac-sim
docker logs costnav-ros2
```

---

## Related Documentation

- [Nav2 Implementation Plan](../docs/nav2/nav2_implementation_plan.md) - Detailed roadmap and architecture
- [Isaac Sim Launch Details](../docs/nav2/isaac_sim_launch.md) - Launch script documentation
- [Architecture Overview](../docs/architecture.md) - CostNav system architecture
- [Cost Model](../docs/cost_model.md) - Economic metrics for navigation evaluation

## External References

- [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html)
- [Nav2 Documentation](https://docs.nav2.org/)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/index.html)
