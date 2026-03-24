# :joystick: Isaac Sim Integration Guide

The `costnav_isaacsim` module provides a **simulation bridge** between NVIDIA Isaac Sim and ROS2, a **rule-based Nav2 baseline**, **imitation learning baselines** (ViNT, NoMaD, GNM, NavDP, Canvas), and a **multi-container Docker architecture** that separates simulation from navigation for modularity.

---

## :white_check_mark: Prerequisites

- **Docker** with NVIDIA Container Toolkit
- **NVIDIA GPU** with driver **525.60.11+**
- **X11 display forwarding** configured on the host

---

## :hammer_and_wrench: Building Docker Images

```bash
# Isaac Sim container
make build-isaac-sim

# ROS2 Nav2 runtime container
make build-ros2

# ROS2 + PyTorch container (for IL baseline evaluation)
make build-ros2-torch
```

---

## :rocket: Running Navigation Stacks

### Nav2 (Rule-Based)

```bash
# Run Isaac Sim + ROS2 Nav2 together
make run-nav2

# With animated people in the scene
make run-nav2 NUM_PEOPLE=20

# Trigger a mission (in a second terminal)
make start-mission
```

This launches the Street Sidewalk environment with Nova Carter, the full Nav2 stack (AMCL, planner, controller, costmaps), and optionally animated pedestrians.

### IL Baselines (ViNT, NoMaD, GNM, NavDP, Canvas)

Download pretrained checkpoints first:

```bash
make download-baseline-checkpoints-hf
```

Run any baseline:

```bash
make run-vint
make run-nomad
make run-gnm
make run-navdp
```

Override the goal type or model checkpoint:

```bash
GOAL_TYPE=image_goal MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint
GOAL_TYPE=topomap MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint
```

Run automated evaluation with metrics collection:

```bash
make run-eval-vint
make run-eval-vint TIMEOUT=30 NUM_MISSIONS=20

make run-eval-nomad
make run-eval-gnm
make run-eval-navdp
```

**Canvas** (sketch-based navigation) requires a separate model worker:

```bash
# Build Canvas image (one-time)
make build-canvas

# Start Isaac Sim + Canvas agent
make run-canvas MODEL_WORKER_URI=http://<gpu-server>:<port>

# Automated evaluation
make run-eval-canvas TIMEOUT=241 NUM_MISSIONS=10
```

### Teleop (Joystick Control)

```bash
make run-teleop
make run-teleop SIM_ROBOT=nova_carter NUM_PEOPLE=20
make run-teleop SIM_ROBOT=segway_e1 NUM_PEOPLE=20
```

!!! tip
    Press ++ctrl+c++ once to stop teleop. The teardown runs automatically -- do not press it again while containers are being cleaned up.

### Running Services Separately

```bash
# Terminal 1: Isaac Sim only
make run-isaac-sim

# Terminal 2: ROS2 Nav2 only (after Isaac Sim is ready)
make run-ros2
```

---

## :whale: Docker Compose Profiles

| Profile     | Services                           | Command              | Use Case                       |
| ----------- | ---------------------------------- | -------------------- | ------------------------------ |
| `nav2`      | Isaac Sim + ROS2 Nav2              | `make run-nav2`      | Full navigation stack          |
| `isaac-sim` | Isaac Sim only                     | `make run-isaac-sim` | Simulation development         |
| `ros2`      | ROS2 Nav2 only                     | `make run-ros2`      | Nav2 tuning (requires sim)     |
| `teleop`    | Isaac Sim + Teleop                 | `make run-teleop`    | Manual driving (joystick)      |
| `vint`      | Isaac Sim + ViNT Policy + Follower | `make run-vint`      | ViNT IL baseline evaluation    |
| `canvas`    | Isaac Sim + RViz + Canvas Bridge   | `make run-canvas`    | Canvas sketch-based navigation |

You can also use profiles directly with Docker Compose:

```bash
docker compose --profile nav2 up
docker compose --profile nav2 down
```

---

## :people_holding_hands: Animated People (PeopleAPI)

CostNav supports spawning animated pedestrians using NVIDIA's PeopleAPI extension.

**Features:**

- Natural NavMesh-based walking with dynamic avoidance
- Configurable count via `NUM_PEOPLE` environment variable (default: 20)
- Random spawning distributed across all NavMesh-enabled areas

**Requirements:**

- The USD scene must have a baked NavMesh
- PeopleAPI extension (auto-loaded from `third_party/PeopleAPI/exts`)
- Isaac Sim built-in character assets

```bash
make run-nav2 NUM_PEOPLE=10
make run-teleop NUM_PEOPLE=5
```

---

## :dart: Mission Manager

The mission manager orchestrates automated navigation missions with NavMesh-based position sampling and RViz visualization.

### Triggering Missions

```bash
# Via make target
make start-mission

# Via ROS2 service
ros2 service call /start_mission std_srvs/srv/Trigger {}
```

Each call runs a single mission. Calling it while a mission is running restarts with a new mission.

### Configuration

Missions are configured via `config/mission_config.yaml`:

```yaml
mission:
  timeout: 3600.0          # Mission timeout (seconds)
distance:
  min: 5.0                 # Minimum start-goal distance (meters)
  max: 50.0                # Maximum start-goal distance (meters)
sampling:
  max_attempts: 100        # Max attempts to sample valid positions
  validate_path: true      # Validate navigable path exists
nav2:
  wait_time: 10.0          # Wait for Nav2 initialization (seconds)
```

Override via CLI:

```bash
python launch.py --mission-timeout 600 --min-distance 10.0 --max-distance 100.0
```

### State Machine

The manager steps through these states each simulation tick:

| #  | State                      | Description                                      |
| -- | -------------------------- | ------------------------------------------------ |
| 1  | `INIT`                     | Initialize ROS2 node, NavMesh sampler, markers   |
| 2  | `WAITING_FOR_NAV2`         | Wait for Nav2 stack readiness                    |
| 3  | `WAITING_FOR_START`        | Wait for `/start_mission` trigger                |
| 4  | `READY`                    | Sample start/goal positions from NavMesh         |
| 5  | `TELEPORTING`              | Teleport robot to start position                 |
| 6  | `SETTLING`                 | Wait for physics to settle (5 steps)             |
| 7  | `PUBLISHING_INITIAL_POSE`  | Publish initial pose to `/initialpose` for AMCL  |
| 8  | `PUBLISHING_GOAL`          | Publish goal pose to `/goal_pose` for Nav2       |
| 9  | `WAITING_FOR_COMPLETION`   | Monitor navigation progress                      |
| 10 | `COMPLETED`                | Mission finished                                 |

### RViz Markers

| Topic           | Color | Description                  |
| --------------- | ----- | ---------------------------- |
| `/start_marker` | Green | Start position (arrow)       |
| `/goal_marker`  | Red   | Goal position (arrow)        |

Set the RViz fixed frame to `map` and add Marker displays for each topic.

---

## :satellite: ROS2 Topics

### Published by Isaac Sim

| Topic                          | Type                      | Description    |
| ------------------------------ | ------------------------- | -------------- |
| `/chassis/odom`                | `nav_msgs/Odometry`       | Robot odometry |
| `/tf`, `/tf_static`            | `tf2_msgs/TFMessage`      | Transform tree |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 3D LiDAR data |

### Subscribed by Isaac Sim

| Topic      | Type                  | Description                 |
| ---------- | --------------------- | --------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands from Nav2 |

### Nav2 Topics

| Topic                     | Type                        | Description    |
| ------------------------- | --------------------------- | -------------- |
| `/goal_pose`              | `geometry_msgs/PoseStamped` | Navigation goal |
| `/map`                    | `nav_msgs/OccupancyGrid`   | Static map     |
| `/local_costmap/costmap`  | `nav_msgs/OccupancyGrid`   | Local costmap  |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid`   | Global costmap |
| `/plan`                   | `nav_msgs/Path`             | Global path    |

### ViNT-Specific Topics

| Topic               | Type                | Direction | Description                        |
| ------------------- | ------------------- | --------- | ---------------------------------- |
| `/model_trajectory` | `nav_msgs/Path`     | Publish   | Predicted trajectory (8 waypoints) |
| `/model_enable`     | `std_msgs/Bool`     | Subscribe | Enable/disable policy execution    |
| `/goal_image`       | `sensor_msgs/Image` | Subscribe | Goal image for ImageGoal mode      |

---

## :compass: Sending Navigation Goals

### Via RViz2

1. Launch the nav2 profile: `make run-nav2`
2. Open RViz2 in the ROS2 container
3. Click **2D Goal Pose** and click on the map to set the goal

### Via Command Line

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Via Python

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

## :gear: launch.py Reference

| Argument            | Default                      | Description                               |
| ------------------- | ---------------------------- | ----------------------------------------- |
| `--usd_path`        | (derived from `--robot`)     | USD scene path                            |
| `--robot`           | `nova_carter`                | Robot preset (`nova_carter`, `segway_e1`) |
| `--headless`        | `false`                      | Run without GUI                           |
| `--physics_dt`      | `1/120` (0.0083s)            | Physics timestep                          |
| `--rendering_dt`    | `1/30` (0.0333s)             | Rendering timestep                        |
| `--debug`           | `false`                      | Enable debug logging                      |
| `--people`          | `20`                         | Number of animated people to spawn        |
| `--config`          | `config/mission_config.yaml` | Path to mission config file               |
| `--mission-timeout` | (from config)                | Override mission timeout                  |
| `--min-distance`    | (from config)                | Override minimum start-goal distance      |
| `--max-distance`    | (from config)                | Override maximum start-goal distance      |
| `--nav2-wait`       | (from config)                | Override Nav2 wait time                   |

`--robot` defaults to the `SIM_ROBOT` environment variable when set.

---

## :wrench: Troubleshooting

### Common Issues

| Issue                                  | Solution                                                    |
| -------------------------------------- | ----------------------------------------------------------- |
| Isaac Sim fails to start               | Check GPU drivers and NVIDIA Container Toolkit installation |
| ROS2 container starts before sim ready | Use `make run-nav2` (has health check dependency)           |
| No ROS2 topics visible                 | Verify `ROS_DOMAIN_ID` matches between containers           |
| Map not loading                        | Check Omniverse server connectivity                         |
| Robot not moving                       | Verify `/cmd_vel` is being published                        |
| People not spawning                    | Check that NavMesh is baked in the USD scene                |
| People not moving                      | Verify NavMesh is enabled and baked correctly               |

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

# View container logs
docker logs costnav-isaac-sim
docker logs costnav-ros2
```
