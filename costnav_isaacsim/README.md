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

| Component               | Status      | Notes                              |
| ----------------------- | ----------- | ---------------------------------- |
| Nav2 Integration        | ✅ Complete | Nova Carter navigates successfully |
| Docker Setup            | ✅ Complete | Multi-container architecture       |
| Occupancy Map           | ✅ Complete | Street_sidewalk environment        |
| Parameter Tuning        | ✅ Complete | Optimized for performance          |
| Cost Model Integration  | ✅ Complete | Economic metrics tracking          |
| **IL Baselines (ViNT)** | ✅ Complete | Training + evaluation pipeline     |

---

## Directory Structure

```
costnav_isaacsim/
├── costnav_isaacsim/              # CostNav Python package (pip install -e)
│   ├── launch.py                  # Thin entry point (delegates to launcher package)
│   ├── config/                    # YAML configuration files
│   └── src/costnav_isaacsim/      # Installable modules
│       ├── config/                # Config dataclasses & YAML loader
│       ├── launcher/              # Simulation launcher (refactored from launch.py)
│       │   ├── sim_launcher.py    # CostNavSimLauncher class
│       │   ├── extensions.py      # Isaac Sim extension management
│       │   └── event_handlers.py  # Stage/physics event handlers
│       ├── utils/                 # Shared utilities
│       │   ├── args.py            # CLI argument parsing
│       │   ├── config_cli.py      # Config loading & CLI overrides
│       │   ├── robot_config.py    # Robot constants & resolution
│       │   └── prim_utils.py      # USD prim manipulation helpers
│       ├── mission_manager/       # Mission orchestration & NavMesh sampling
│       └── people_manager.py      # Animated people (PeopleAPI)
├── nav2_params/                   # Nav2 navigation stack configuration
│   ├── maps/                      # Occupancy grid maps
│   ├── nova_carter/               # Nova Carter robot config
│   └── segway_e1/                 # Segway E1 robot config
├── il_baselines/                  # Imitation learning baselines
│   ├── data_processing/           # ROS bag to training data conversion
│   ├── evaluation/                # ViNT evaluation package (pip install -e)
│   └── training/                  # Model training scripts
└── isaac_sim_teleop_ros2/         # ROS2 teleop package
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

# Build ViNT image (for IL baseline evaluation)
make build-vint
```

### 2. Run Nav2 Navigation (Recommended)

```bash
# Run both Isaac Sim and ROS2 Nav2 together
make run-nav2

# Run with animated people walking in the scene (default: 20 people)
make run-nav2 NUM_PEOPLE=20
```

This starts:

- **Isaac Sim**: Street Sidewalk environment with Nova Carter robot
- **ROS2 Nav2**: Carter navigation with pre-configured parameters
- **People (optional)**: Animated people walking naturally in NavMesh areas

You can also trigger missions while nav2 is running:

```bash
make start-mission
```

### 3. Run Teleop (Joystick Control)

```bash
# Run Isaac Sim + teleop node (defaults to nova_carter)
make run-teleop

# Run with animated people walking in the scene (default: 20 people)
make run-teleop NUM_PEOPLE=20

# Or select robot explicitly (optionally with people)
make run-teleop SIM_ROBOT=nova_carter NUM_PEOPLE=20
make run-teleop SIM_ROBOT=segway_e1 NUM_PEOPLE=20
```

You can also trigger missions while teleop is running:

```bash
make start-mission
```

### 4. Alternative: Run Services Separately

```bash
# Terminal 1: Isaac Sim only
make run-isaac-sim

# Terminal 2: ROS2 Nav2 only (after Isaac Sim is ready)
make run-ros2
```

### 5. Run ViNT IL Baseline

Run the imitation learning baseline using ViNT (Visual Navigation Transformer):

Download the pretrained model weights from Google Drive or train your model and place it to `checkpoints/`
See [Download Pretrained Checkpoints](il_baselines/README.md#download-pretrained-checkpoints) for more information.

```bash
# Build ViNT Docker image (first time only)
make build-vint

# Run ViNT evaluation with Isaac Sim
# Optionally specify model checkpoint path
MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint

# Or run with default model
make run-vint
```

This starts:

- **Isaac Sim**: Street Sidewalk environment with Nova Carter robot
- **ViNT Policy Node**: Runs ViNT inference at ~10Hz, publishes trajectories
- **Trajectory Follower Node**: MPC controller at ~20Hz, publishes `/cmd_vel`

You can trigger missions while ViNT is running:

```bash
make start-mission
```

To run automated evaluation with metrics collection:

```bash
# In terminal 1: Start ViNT
make run-vint

# In terminal 2: Run evaluation (default: 20s timeout, 10 missions)
make run-eval-vint

# Or with custom parameters
make run-eval-vint TIMEOUT=30 NUM_MISSIONS=20
```

---

## Animated People (PeopleAPI Integration)

CostNav supports spawning animated people that walk naturally in NavMesh-enabled areas using NVIDIA's PeopleAPI extension. This creates a more realistic simulation environment for testing navigation in crowded scenarios.

### Features

- **Natural Walking**: People use NavMesh for pathfinding and walk to random destinations
- **Dynamic Avoidance**: People avoid each other and obstacles
- **Configurable Count**: Spawn any number of people via `NUM_PEOPLE` environment variable
- **Random Spawning**: People spawn at random valid NavMesh positions throughout the entire map
- **RANDOM_GOTO Behavior**: People continuously walk to random destinations across the map
- **Distributed Placement**: People are evenly distributed across all NavMesh-enabled areas

### Usage

Enable people by setting the `NUM_PEOPLE` environment variable:

```bash
# Run Nav2 with 20 people (default)
make run-nav2

# Run teleop with 20 people (default)
make run-teleop

# Run with custom number of people
make run-nav2 NUM_PEOPLE=10
make run-teleop NUM_PEOPLE=5
```

### Command Line Arguments

You can also pass the `--people` argument directly to `launch.py`:

```bash
python launch.py --people 5
```

### Requirements

- **NavMesh**: The USD scene must have a baked NavMesh for people to navigate
- **PeopleAPI Extension**: Automatically loaded from `third_party/PeopleAPI/exts`
- **Character Assets**: Loaded from Isaac Sim's built-in character library

### How It Works

1. **Initialization**: After simulation warmup, the PeopleManager initializes
2. **NavMesh Setup**: Checks if NavMesh is baked, if not, automatically creates NavMeshVolume and bakes it
3. **Random Position Sampling**: Uses `navmesh.query_random_point()` to sample positions from NavMesh (same as robot spawning)
4. **Minimum Distance Check**: Ensures people spawn at least 2 meters apart to avoid clustering
5. **Character Spawning**: Spawns characters at the sampled NavMesh positions
6. **Animation Setup**: Applies animation graphs and RANDOM_GOTO behavior scripts
7. **Natural Walking**: People continuously walk to random NavMesh destinations throughout the map

### Technical Details

- **Module**: `costnav_isaacsim.people_manager` (from `costnav_isaacsim/costnav_isaacsim/src/costnav_isaacsim/people_manager.py`)
- **Behavior**: `CharacterBehavior.RANDOM_GOTO` (random destination walking)
- **Character Root**: `/World/Characters`
- **Spawn Method**: `navmesh.query_random_point()` with unique random IDs (same as robot spawning)
- **Minimum Spacing**: 2.0 meters between spawned people
- **Position Validation**: Rejects positions too close to existing people
- **Max Attempts**: 20 attempts per person (num_people \* 20 total)
- **Dynamic Avoidance**: Enabled by default
- **NavMesh**: Automatically created and baked if not present

### Troubleshooting

| Issue                        | Solution                                           |
| ---------------------------- | -------------------------------------------------- |
| People not spawning          | Check that NavMesh is baked in the USD scene       |
| People not moving            | Verify NavMesh is enabled and baked correctly      |
| Slow initialization          | Character assets need to load; wait for completion |
| People walking through walls | NavMesh may not be properly configured             |

---

## Docker Compose Profiles

| Profile     | Services                           | Command              | Use Case                           |
| ----------- | ---------------------------------- | -------------------- | ---------------------------------- |
| `nav2`      | Isaac Sim + ROS2 Nav2              | `make run-nav2`      | Full navigation stack              |
| `isaac-sim` | Isaac Sim only                     | `make run-isaac-sim` | Simulation development             |
| `ros2`      | ROS2 Nav2 only                     | `make run-ros2`      | Nav2 tuning (requires running sim) |
| `teleop`    | Isaac Sim + Teleop                 | `make run-teleop`    | Manual driving (joystick)          |
| `vint`      | Isaac Sim + ViNT Policy + Follower | `make run-vint`      | ViNT IL baseline evaluation        |

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

### `costnav_isaacsim/launch.py` - Entry Point

Thin entry point that delegates to the `costnav_isaacsim.launcher` and `costnav_isaacsim.utils` packages. Parses CLI arguments, resolves robot/USD settings, loads mission config, creates a `CostNavSimLauncher`, and runs the simulation loop.

**Usage:**

```bash
python launch.py                                    # Default: GUI mode
python launch.py --headless                         # Headless mode (no GUI)
python launch.py --robot segway_e1                  # Select robot preset
python launch.py --usd_path omniverse://path/to.usd # Custom USD scene
python launch.py --debug                            # Enable debug logging
```

**Command Line Arguments:**

| Argument            | Default                      | Description                               |
| ------------------- | ---------------------------- | ----------------------------------------- |
| `--usd_path`        | (derived from `--robot`)     | USD scene path                            |
| `--robot`           | `nova_carter`                | Robot preset (`nova_carter`, `segway_e1`) |
| `--headless`        | `false`                      | Run without GUI                           |
| `--physics_dt`      | `1/120` (0.0083s)            | Physics timestep                          |
| `--rendering_dt`    | `1/30` (0.0333s)             | Rendering timestep                        |
| `--debug`           | `false`                      | Enable debug logging                      |
| `--people`          | `20`                         | Number of people to spawn                 |
| `--config`          | `config/mission_config.yaml` | Path to mission config file               |
| `--mission-timeout` | (from config)                | Override: Mission timeout                 |
| `--min-distance`    | (from config)                | Override: Minimum start-goal distance     |
| `--max-distance`    | (from config)                | Override: Maximum start-goal distance     |
| `--nav2-wait`       | (from config)                | Override: Nav2 wait time                  |

`--robot` defaults to `SIM_ROBOT` environment variable when set, otherwise `nova_carter`.
If the Segway prim is not detected automatically, set `ROBOT_PRIM_PATH` to the robot base prim.

### `launcher/` - Simulation Launcher Package

| File                | Description                                                                                                             |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `sim_launcher.py`   | `CostNavSimLauncher` class — SimulationApp creation, stage loading, simulation loop, mission/people manager integration |
| `extensions.py`     | `enable_isaac_sim_extensions()` — enables navigation, sensor, ROS2 bridge, and PeopleAPI extensions                     |
| `event_handlers.py` | `register_stage_open_handler()`, `handle_physics_reinit()` — stage reload and physics view invalidation recovery        |

### `utils/` - Shared Utilities Package

| File              | Description                                                                                                           |
| ----------------- | --------------------------------------------------------------------------------------------------------------------- |
| `args.py`         | `parse_args()` — all CLI argument definitions (no Isaac Sim dependency)                                               |
| `config_cli.py`   | `load_and_override_config()` — loads YAML config and applies CLI overrides                                            |
| `robot_config.py` | Robot constants (`DEFAULT_USD_PATHS`, `DEFAULT_ROBOT_PRIM_PATHS`, etc.), `resolve_robot_name()`, `resolve_usd_path()` |
| `prim_utils.py`   | USD prim helpers: `resolve_robot_prim_path()`, `find_robot_prim_by_tokens()`, `get/set_prim_world_translation()`      |

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

## Mission Manager Module

The `costnav_isaacsim.mission_manager` module provides automated navigation mission orchestration with NavMesh-based position sampling and RViz visualization. It uses a **state machine-based approach** integrated directly into the main simulation loop for proper synchronization.

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
│  │  launch.py                  │  │   Nav2 Stack            │  │
│  │  ├─ Simulation loop         │  │   ├─ AMCL localization  │  │
│  │  ├─ MissionManager.step()   │  │   ├─ Path planning      │  │
│  │  │  ├─ NavMesh sampling     │  │   └─ Navigation control │  │
│  │  │  ├─ Robot teleportation  │  │                         │  │
│  │  │  ├─ Physics settling     │  │                         │  │
│  │  │  └─ State transitions    │  │                         │  │
│  │  ├─ /initialpose publish ───┼──┼─►                       │  │
│  │  ├─ /goal_pose publish ─────┼──┼─►                       │  │
│  │  └─ /markers publish ───────┼──┼─► RViz2 visualization   │  │
│  └─────────────────────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

**Important**: The `costnav_isaacsim.mission_manager` module runs **inside the Isaac Sim container** because it:

1. Uses Isaac Sim's NavMesh API (`omni.anim.navigation.core`)
2. Teleports the robot using Isaac Sim's physics engine
3. Integrates with the main simulation loop for proper physics synchronization
4. Shares ROS2 topics with Nav2 via host networking

### Quick Start: Running Missions

Missions are triggered manually. Start the stack, then call `/start_mission`:

```bash
# From repository root
make run-nav2

# In another terminal
make start-mission
```

Each `make start-mission` runs a single mission. If you call it while a mission is running, the current mission is stopped and a new one starts.

You can also trigger the service directly:

```bash
ros2 service call /start_mission std_srvs/srv/Trigger {}
```

### Configuration

Mission parameters are configured via YAML file at `config/mission_config.yaml`:

```yaml
# Mission execution settings
mission:
  timeout: 3600.0 # Mission timeout (seconds)

# Distance constraints (meters)
distance:
  min: 5.0 # Minimum start-goal distance
  max: 50.0 # Maximum start-goal distance

# NavMesh sampling settings
sampling:
  max_attempts: 100 # Maximum attempts to sample valid positions
  validate_path: true # Validate path exists between start and goal

# Nav2 integration
nav2:
  wait_time: 10.0 # Wait for Nav2 stack to initialize (seconds)
  topics:
    initial_pose: "/initialpose"
    goal_pose: "/goal_pose"
    odom: "/odom"

# Robot teleportation
teleport:
  height_offset: 0.5 # Height offset for teleportation (meters)
  robot_prim: "/World/Nova_Carter_ROS" # Robot prim path in USD stage

# RViz markers
markers:
  enabled: true
  topics:
    start: "/start_marker"
    goal: "/goal_marker"
    robot: "/robot_marker"
  scale:
    arrow_length: 1.0
    arrow_width: 0.2
    arrow_height: 0.2
  robot_scale:
    length: 0.9
    width: 0.5
    height: 0.2
```

**Key Configuration Parameters:**

| Section    | Parameter       | Default                  | Description                            |
| ---------- | --------------- | ------------------------ | -------------------------------------- |
| `mission`  | `timeout`       | 3600.0                   | Mission timeout (seconds)              |
| `distance` | `min`           | 5.0                      | Minimum start-goal distance (meters)   |
| `distance` | `max`           | 50.0                     | Maximum start-goal distance (meters)   |
| `sampling` | `max_attempts`  | 100                      | Max attempts to sample valid positions |
| `sampling` | `validate_path` | true                     | Validate navigable path exists         |
| `nav2`     | `wait_time`     | 10.0                     | Wait for Nav2 initialization (seconds) |
| `teleport` | `height_offset` | 0.5                      | Teleportation height offset (meters)   |
| `teleport` | `robot_prim`    | `/World/Nova_Carter_ROS` | Robot prim path in USD                 |

### Command Line Options

`launch.py` always loads the mission config and waits for `/start_mission`:

```bash
# Use default config (config/mission_config.yaml)
python launch.py

# Use custom config file
python launch.py --config /path/to/custom.yaml

# Override specific config values
python launch.py --mission-timeout 600 --min-distance 10.0

# Headless mode
python launch.py --headless
```

#### Simulation Options

| Option           | Default    | Description            |
| ---------------- | ---------- | ---------------------- |
| `--usd_path`     | (internal) | Path to USD scene file |
| `--headless`     | false      | Run without GUI        |
| `--physics_dt`   | 1/120      | Physics time step      |
| `--rendering_dt` | 1/30       | Rendering time step    |
| `--debug`        | false      | Enable debug logging   |

#### Mission Options

| Option              | Default                    | Description                           |
| ------------------- | -------------------------- | ------------------------------------- |
| `--config`          | config/mission_config.yaml | Path to mission config file           |
| `--mission-timeout` | (from config)              | Override: Mission timeout             |
| `--min-distance`    | (from config)              | Override: Minimum start-goal distance |
| `--max-distance`    | (from config)              | Override: Maximum start-goal distance |
| `--nav2-wait`       | (from config)              | Override: Nav2 wait time              |

### Interactive Python Usage

For interactive testing inside the Isaac Sim container:

```bash
# Start Python shell inside Isaac Sim container
docker exec -it costnav-isaac-sim /isaac-sim/python.sh
```

**Basic Usage:**

```python
from costnav_isaacsim.mission_manager import MissionManager
from costnav_isaacsim.config import MissionConfig

# Load mission configuration
mission_config = MissionConfig(timeout=3600.0)

# Create mission manager (runs in main simulation loop)
manager = MissionManager(mission_config, simulation_context)

# Main simulation loop
while running:
    simulation_context.step(render=True)
    manager.step()  # Step mission manager after physics step
```

**Advanced Usage with MissionManagerConfig:**

```python
from costnav_isaacsim.mission_manager import MissionManager
from costnav_isaacsim.config import MissionConfig, MissionManagerConfig

# Load mission configuration
mission_config = MissionConfig(timeout=3600.0)

# Fine-tune runtime behavior with MissionManagerConfig
manager_config = MissionManagerConfig(
    min_distance=5.0,              # Minimum start-goal distance (meters)
    max_distance=100.0,            # Maximum start-goal distance (meters)
    initial_pose_delay=1.0,        # Delay after initial pose (seconds)
    goal_delay=0.5,                # Delay after goal pose (seconds)
    teleport_height=0.1,           # Teleportation height offset (meters)
    robot_prim_path="/World/Nova_Carter_ROS",  # Robot prim path
    teleport_settle_steps=5,       # Physics settling steps after teleport
)

# Create mission manager with custom config
manager = MissionManager(
    mission_config=mission_config,
    simulation_context=simulation_context,
    manager_config=manager_config,
)

# Main simulation loop
while running:
    simulation_context.step(render=True)
    manager.step()
```

### Architecture: State Machine-Based Execution

The MissionManager uses a state machine to ensure proper synchronization between Isaac Sim physics and Nav2:

**Mission States:**

1. **INIT**: Initialize ROS2 node, NavMesh sampler, and marker publisher
2. **WAITING_FOR_NAV2**: Wait for Nav2 stack to become ready
3. **WAITING_FOR_START**: Wait for `/start_mission` trigger
4. **READY**: Sample new start/goal positions from NavMesh
5. **TELEPORTING**: Teleport robot to start position in Isaac Sim
6. **SETTLING**: Wait for physics to settle after teleportation (5 steps)
7. **PUBLISHING_INITIAL_POSE**: Publish initial pose to `/initialpose` for AMCL
8. **PUBLISHING_GOAL**: Publish goal pose to `/goal_pose` for Nav2
9. **WAITING_FOR_COMPLETION**: Monitor navigation progress
10. **COMPLETED**: All missions finished

This state machine approach ensures:

- Physics simulation steps occur between teleportation and pose publishing
- Proper timing delays for AMCL initialization
- Clean separation of concerns for each mission phase

### Features

- **State Machine Execution**: Proper synchronization with simulation loop
- **NavMesh Position Sampling**: Sample valid start/goal positions from Isaac Sim's NavMesh
- **Distance Constraints**: Enforce minimum (5m) and maximum (50m) distance between positions
- **Path Validation**: Verify navigable paths exist between sampled positions
- **Robot Teleportation**: Teleport robot to start position in Isaac Sim with physics settling
- **AMCL Initialization**: Publish initial pose for localization
- **RViz Markers**: Visualize start (green), goal (red), and robot (blue) positions
- **Manual Trigger**: Start missions via `/start_mission` (e.g. `make start-mission`)
- **Auto-Setup**: Automatically configures Isaac Sim teleportation if robot_prim_path is provided

### RViz Marker Topics

The MissionManager publishes visualization markers for debugging and monitoring:

| Topic           | Type                        | Color           | Description                    |
| --------------- | --------------------------- | --------------- | ------------------------------ |
| `/start_marker` | `visualization_msgs/Marker` | Green (0, 1, 0) | Start position (ARROW marker)  |
| `/goal_marker`  | `visualization_msgs/Marker` | Red (1, 0, 0)   | Goal position (ARROW marker)   |
| `/robot_marker` | `visualization_msgs/Marker` | Blue (0, 0, 1)  | Current robot position (10 Hz) |

**Marker Properties:**

- **Type**: ARROW (0)
- **Frame**: `map`
- **Scale**: 1.0m length, 0.2m width, 0.2m height (configurable)
- **Lifetime**: Persistent (0 seconds)

### Viewing Markers in RViz2

1. Open RViz2 in the ROS2 container or host
2. Add "Marker" displays for each topic:
   - `/start_marker`
   - `/goal_marker`
   - `/robot_marker`
3. Set "Fixed Frame" to `map`
4. Markers will appear when missions are running

### Running Tests

The `costnav_isaacsim.mission_manager` module includes unit tests for all components.

**Test Coverage:**

- `test_navmesh_sampler.py`: NavMesh sampling and distance calculations
- `test_marker_publisher.py`: RViz marker publishing
- `test_mission_manager.py`: State machine and mission execution
- `test_config_loader.py`: Configuration loading and validation

**Run tests on the host** (NavMesh-independent tests):

```bash
cd /path/to/CostNav
python3 -c "
from costnav_isaacsim.mission_manager import SampledPosition

# Test distance calculation
pos1 = SampledPosition(x=0, y=0, z=0)
pos2 = SampledPosition(x=3, y=4, z=0)
print(f'Distance: {pos1.distance_to(pos2)}')  # Should print 5.0
"
```

**Run full tests inside Isaac Sim container:**

```bash
docker exec -it costnav-isaac-sim /isaac-sim/python.sh -m pytest \
    /workspace/costnav_isaacsim/costnav_isaacsim/tests/ -v
```

**Run specific test file:**

```bash
docker exec -it costnav-isaac-sim /isaac-sim/python.sh -m pytest \
    /workspace/costnav_isaacsim/costnav_isaacsim/tests/test_mission_manager.py -v
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

## IL Baselines (Imitation Learning)

CostNav includes an imitation learning (IL) baseline evaluation framework for comparing learned navigation policies against the rule-based Nav2 stack. The first implemented baseline is **ViNT (Visual Navigation Transformer)**.

### Quick Start: Run ViNT Evaluation

```bash
# Build the ViNT Docker image (first time only)
make build-vint

# Run ViNT policy evaluation with Isaac Sim
make run-vint
```

This starts:

- **Isaac Sim**: Street Sidewalk environment with Nova Carter robot
- **ViNT Policy Node**: Runs ViNT inference at ~10Hz, publishes trajectories
- **Trajectory Follower Node**: MPC controller at ~20Hz, publishes cmd_vel

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ViNT Evaluation Architecture                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Isaac Sim Container           ViNT Container (ROS2 Jazzy)          │
│  ┌────────────────────┐       ┌────────────────────────────────┐   │
│  │ launch.py          │       │ ViNT Policy Node (~10Hz)       │   │
│  │ - Physics sim      │       │ - Camera → ViNT inference      │   │
│  │ - Nova Carter      │       │ - Goal image navigation        │   │
│  │ - ROS2 Bridge      │       │ - Publishes /vint_trajectory   │   │
│  │                    │       └────────────┬───────────────────┘   │
│  │ /front_*/image ────┼──────►             │                       │
│  │                    │                    ▼                       │
│  │                    │       ┌────────────────────────────────┐   │
│  │                    │       │ Trajectory Follower (~20Hz)    │   │
│  │                    │       │ - MPC controller (CasADi)      │   │
│  │ /cmd_vel ◄─────────┼───────│ - Trajectory tracking          │   │
│  │                    │       │ - Publishes /cmd_vel           │   │
│  └────────────────────┘       └────────────────────────────────┘   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### ROS2 Topics (ViNT)

| Topic              | Type                | Direction | Description                        |
| ------------------ | ------------------- | --------- | ---------------------------------- |
| `/vint_trajectory` | `nav_msgs/Path`     | Publish   | Predicted trajectory (8 waypoints) |
| `/vint_enable`     | `std_msgs/Bool`     | Subscribe | Enable/disable policy execution    |
| `/goal_image`      | `sensor_msgs/Image` | Subscribe | Goal image for ImageGoal mode      |

### Makefile Targets (IL Baselines)

| Target          | Description                                       |
| --------------- | ------------------------------------------------- |
| `build-vint`    | Build ViNT Docker image with ROS2 Jazzy + PyTorch |
| `run-vint`      | Run Isaac Sim + ViNT policy + trajectory follower |
| `run-eval-vint` | Run automated evaluation with metrics collection  |

### Configuration Files

- **Model config**: `il_baselines/evaluation/configs/vint_eval.yaml`
- **Robot config**: `il_baselines/evaluation/configs/robot_segway.yaml`
- **Training config**: `il_baselines/training/visualnav_transformer/configs/vint_costnav.yaml`

> **See Also**: [IL Baselines Documentation](il_baselines/README.md) for detailed setup and usage.

---

## Related Documentation

- [Nav2 Implementation Plan](../docs/nav2/nav2_implementation_plan.md) - Detailed roadmap and architecture
- [Isaac Sim Launch Details](../docs/nav2/isaac_sim_launch.md) - Launch script documentation
- [Architecture Overview](../docs/architecture.md) - CostNav system architecture
- [Cost Model](../docs/cost_model.md) - Economic metrics for navigation evaluation
- [IL Baselines](il_baselines/README.md) - Imitation learning baselines documentation
- [IL Baselines Design](../docs/imitation_learning_baselines.md) - Detailed IL design document

## External References

- [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html)
- [Nav2 Documentation](https://docs.nav2.org/)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/index.html)
