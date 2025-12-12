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
├── launch.py                              # Isaac Sim launcher script (main simulation)
├── launch_mission.py                      # Nav2 mission launcher (run after launch.py)
├── config/                                # Configuration files
│   ├── __init__.py                        # Config module exports
│   ├── config_loader.py                   # YAML config loader with MissionConfig dataclass
│   └── mission_config.yaml                # Default mission parameters
├── nav2_params/                           # Nav2 navigation stack configuration
│   ├── carter_navigation_params.yaml      # Full Nav2 stack parameters
│   ├── carter_sidewalk.yaml               # Map metadata (origin, resolution)
│   └── carter_sidewalk.png                # Occupancy grid image
└── nav2_mission/                          # Nav2 mission orchestration module
    ├── __init__.py                        # Package exports (conditional ROS2 imports)
    ├── navmesh_sampler.py                 # NavMesh-based position sampling
    ├── marker_publisher.py                # RViz marker visualization
    ├── mission_orchestrator.py            # Mission coordination
    ├── mission_runner.py                  # Background thread mission runner
    └── tests/                             # Unit tests
        ├── test_navmesh_sampler.py
        ├── test_marker_publisher.py
        └── test_mission_orchestrator.py
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

## Nav2 Mission Module

The `nav2_mission` module provides automated navigation mission orchestration with NavMesh-based position sampling and RViz visualization. It is **integrated directly into `launch.py`** for seamless operation.

### Container Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                Docker Network (host mode)                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────┐  ┌─────────────────────────┐  │
│  │    Isaac Sim Container      │  │   ROS2 Nav2 Container   │  │
│  │    (costnav-isaac-sim)      │  │   (costnav-ros2-nav2)   │  │
│  │                             │  │                         │  │
│  │  launch.py --mission        │  │   Nav2 Stack            │  │
│  │  ├─ Simulation loop         │  │   ├─ AMCL localization  │  │
│  │  ├─ NavMesh sampling        │  │   ├─ Path planning      │  │
│  │  ├─ Robot teleportation     │  │   └─ Navigation control │  │
│  │  ├─ /initialpose publish ───┼──┼─►                       │  │
│  │  ├─ /goal_pose publish ─────┼──┼─►                       │  │
│  │  └─ /markers publish ───────┼──┼─► RViz2 visualization   │  │
│  └─────────────────────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

**Important**: The `nav2_mission` module runs **inside the Isaac Sim container** because it:
1. Uses Isaac Sim's NavMesh API (`omni.anim.navigation.core`)
2. Teleports the robot using Isaac Sim's physics engine
3. Shares ROS2 topics with Nav2 via host networking

### Quick Start: Running Missions

#### Option 1: Integrated Launch (Recommended)

Run Isaac Sim with mission orchestration in a single command:

```bash
# From repository root
make run-nav2
```

Then modify the docker-compose command or use:

```bash
# Inside Isaac Sim container, uses config/mission_config.yaml by default
python /workspace/costnav_isaacsim/launch.py --mission

# Use custom config file
python /workspace/costnav_isaacsim/launch.py --mission --config /path/to/custom.yaml

# Override config values via CLI
python /workspace/costnav_isaacsim/launch.py --mission --mission-count 5 --min-distance 10
```

#### Option 2: Two-Step Launch

**Step 1: Start the Nav2 Stack**

```bash
# From repository root, start both Isaac Sim and ROS2 Nav2 containers
make run-nav2
```

**Step 2: Run Missions Separately**

```bash
# Open a new terminal and exec into the Isaac Sim container
docker exec -it costnav-isaac-sim /isaac-sim/python.sh \
    /workspace/costnav_isaacsim/launch_mission.py
```

### Configuration

Mission parameters are configured via YAML file at `config/mission_config.yaml`:

```yaml
# Mission execution settings
mission:
  count: 1          # Number of missions (0 = infinite)
  delay: 30.0       # Delay between missions (seconds)

# Distance constraints (meters)
distance:
  min: 5.0          # Minimum start-goal distance
  max: 50.0         # Maximum start-goal distance

# Nav2 integration
nav2:
  wait_time: 10.0   # Wait for Nav2 stack to initialize

# Robot teleportation
teleport:
  height_offset: 0.5
  robot_prim: "/World/Nova_Carter_ROS"

# RViz markers
markers:
  enabled: true
  topics:
    start: "/start_marker"
    goal: "/goal_marker"
    robot: "/robot_marker"
```

### Command Line Options

The integrated `launch.py` uses config file by default with CLI overrides:

```bash
# Use default config (config/mission_config.yaml)
python launch.py --mission

# Use custom config file
python launch.py --mission --config /path/to/custom.yaml

# Override specific config values
python launch.py --mission --mission-count 5 --min-distance 10.0

# Headless mode with missions
python launch.py --headless --mission
```

#### Simulation Options

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--usd_path` | (internal) | Path to USD scene file |
| `--headless` | false | Run without GUI |
| `--physics_dt` | 1/60 | Physics time step |
| `--rendering_dt` | 1/30 | Rendering time step |
| `--debug` | false | Enable debug logging |

#### Mission Options

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--mission` | false | Enable Nav2 mission orchestration |
| `--config` | config/mission_config.yaml | Path to mission config file |
| `--mission-count` | (from config) | Override: Number of missions |
| `--mission-delay` | (from config) | Override: Delay between missions |
| `--min-distance` | (from config) | Override: Minimum start-goal distance |
| `--max-distance` | (from config) | Override: Maximum start-goal distance |
| `--nav2-wait` | (from config) | Override: Nav2 wait time |

### Interactive Python Usage

For interactive testing inside the Isaac Sim container:

```bash
# Start Python shell inside Isaac Sim container
docker exec -it costnav-isaac-sim /isaac-sim/python.sh
```

```python
import rclpy
from nav2_mission import MissionOrchestrator, MissionConfig

rclpy.init()

# Configure mission
config = MissionConfig(min_distance=5.0, max_distance=50.0)
orchestrator = MissionOrchestrator(config=config)

# Run a navigation mission
success = orchestrator.run_mission()
print(f"Mission success: {success}")

orchestrator.destroy_node()
rclpy.shutdown()
```

### Features

- **NavMesh Position Sampling**: Sample valid start/goal positions from Isaac Sim's NavMesh
- **Distance Constraints**: Enforce minimum (5m) and maximum (100m) distance between positions
- **Path Validation**: Verify navigable paths exist between sampled positions
- **Robot Teleportation**: Teleport robot to start position in Isaac Sim
- **AMCL Initialization**: Publish initial pose for localization
- **RViz Markers**: Visualize start (green), goal (red), and robot (blue) positions
- **Integrated Launch**: Run missions alongside simulation with `--mission` flag

### RViz Marker Topics

| Topic          | Color | Description                    |
| -------------- | ----- | ------------------------------ |
| `/start_marker`| Green | Start position (ARROW marker)  |
| `/goal_marker` | Red   | Goal position (ARROW marker)   |
| `/robot_marker`| Blue  | Current robot position (10 Hz) |

### Viewing Markers in RViz2

1. Open RViz2 in the ROS2 container or host
2. Add "Marker" displays for each topic:
   - `/start_marker`
   - `/goal_marker`
   - `/robot_marker`
3. Set "Fixed Frame" to `map`

### Running Tests

Tests can be run on the host without ROS2/Isaac Sim (NavMesh-independent tests):

```bash
cd /path/to/CostNav
python3 -c "
from costnav_isaacsim.nav2_mission.navmesh_sampler import SampledPosition

# Test distance calculation
pos1 = SampledPosition(x=0, y=0, z=0)
pos2 = SampledPosition(x=3, y=4, z=0)
print(f'Distance: {pos1.distance_to(pos2)}')  # Should print 5.0
"
```

For full tests inside Isaac Sim container:

```bash
docker exec -it costnav-isaac-sim /isaac-sim/python.sh -m pytest \
    /workspace/costnav_isaacsim/nav2_mission/tests/ -v
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
