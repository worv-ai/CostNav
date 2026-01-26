# Nav2 (ROS2 Navigation Stack 2) Implementation Plan

**Issue Reference:** [#5 - Support rule-based navigation with nav2](https://github.com/worv-ai/CostNav/issues/5)

**Status:** ✅ Nav2 Integration Complete | ✅ Mission Orchestration Complete | ✅ Parameter Tuning Complete | ⏳ Cost Model Integration In Progress
**Target Version:** CostNav v0.2.0
**Last Updated:** 2026-01-26

---

## Current Status

### ✅ Completed

- **Nav2 Integration with Nova Carter**: Full ROS2 Navigation Stack 2 integration complete
- **Nav2 Integration with Segway E1**: Dynamic robot selection via SIM_ROBOT environment variable
- **Docker Setup**: Multi-container architecture configured
- **ROS2 Bridge**: Communication between Isaac Sim and Nav2 established
- **Occupancy Map**: Generated and configured for Street_sidewalk environment
- **Basic Navigation**: Both Nova Carter and Segway E1 successfully navigate to goals
- **Mission Orchestration**: Automated start/goal sampling, robot teleportation, and RViz visualization
- **NavMesh Position Sampling**: Valid start/goal positions sampled from Isaac Sim's NavMesh
- **RViz Marker Visualization**: Start (green), goal (red), and robot (blue) markers
- **Robot-Specific Configuration**: Separate parameter files for Nova Carter and Segway E1
- **Parameter Tuning**: Nav2 parameters optimized for both Nova Carter and Segway E1

### ⏳ In Progress

1. **Cost Model Integration**: Track economic metrics for Nav2 navigation
   - ✅ Energy consumption tracking via odometry (merged to main)
   - ✅ Distance and time metrics logging (merged to main)
   - ✅ Collision impulse tracking and health monitoring (merged to main)
   - ✅ Food spoilage tracking for delivery missions (merged to main)
   - ⏳ Integration with existing cost model framework
   - ⏳ Benchmark comparison with RL baseline

### 📋 Future Work

- **Hybrid RL+Nav2**: Combine learning-based and rule-based approaches
- **Multi-Robot Navigation**: Extend to support multiple robots simultaneously

### ⚠️ Known Issues

1. **~~Segway E1 Spin-in-Place Problem~~** - ✅ RESOLVED
   - **Description**: The Segway E1 robot could not perform in-place rotation maneuvers
   - **Impact**: Affected navigation in tight spaces and goal orientation alignment
   - **Status**: ✅ Resolved by modifying Isaac Sim USD Action Graph robot controller configuration
   - **Root Cause Solution**: Changed the robot controller used in the USD Action Graph of Isaac Sim to properly support differential drive in-place rotation commands
   - **Navigation Enhancement**: Implemented `nav2_rotation_shim_controller::RotationShimController` wrapper around DWB controller as a complementary improvement for better in-place rotation behavior
   - **Details**: See "Parameter Tuning: RotationShimController" section below for the navigation enhancement configuration

---

## Parameter Tuning: RotationShimController

### Overview

To resolve the Segway E1 spin-in-place problem and improve navigation performance for both robots, we implemented the **Nav2 RotationShimController** as a wrapper around the DWB local planner. This controller enforces in-place rotation when the robot needs to make large heading corrections before following a path.

### What is RotationShimController?

The RotationShimController is a Nav2 controller plugin that:

1. **Monitors heading error** between the robot's current orientation and the path direction
2. **Triggers in-place rotation** when heading error exceeds a threshold (e.g., 45°)
3. **Hands off to primary controller** (DWB) once the robot is roughly aligned with the path
4. **Enables smooth transitions** between rotation and path-following modes

**Reference:** [Nav2 RotationShimController Documentation](https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html)

### Configuration Strategy

We use **consistent angular velocity limits** across all navigation phases:

| Phase                  | Controller             | Angular Velocity | Purpose                                        |
| ---------------------- | ---------------------- | ---------------- | ---------------------------------------------- |
| **In-Place Rotation**  | RotationShimController | 0.5 rad/s        | Controlled heading corrections when stationary |
| **Path Following**     | DWB Controller         | 0.5 rad/s        | Stable turning while moving forward            |
| **Recovery Behaviors** | Behavior Server        | 0.5 rad/s        | Controlled recovery spins                      |

### Parameter Configuration

#### 1. RotationShimController Parameters

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  angular_dist_threshold: 0.785 # 45° in radians - rotate in place if heading error > this
  forward_sampling_distance: 0.5 # meters - distance to look ahead on path
  rotate_to_heading_angular_vel: 0.5 # rad/s - rotation speed during in-place rotation
  max_angular_accel: 3.5 # rad/s² - max angular acceleration
  simulate_ahead_time: 2.0 # seconds - collision checking projection time
  rotate_to_goal_heading: false # Don't rotate to goal heading at the end
  closed_loop: false # Use commanded velocity instead of odometry (prevents premature deceleration)
```

**Note on `closed_loop: false` (Segway E1 only):**

- When `true` (default): Uses robot odometry velocity for acceleration calculations
- When `false`: Uses last commanded velocity for acceleration calculations
- Setting to `false` prevents the controller from decelerating too early when approaching the target heading
- This ensures the robot maintains minimum velocity (0.3 rad/s) during in-place rotation
- Only needed for Segway E1 due to its specific dynamics

#### 2. DWB Controller Parameters (Wrapped by RotationShim)

```yaml
primary_controller: "dwb_core::DWBLocalPlanner"
max_vel_theta: 0.5 # rad/s - Cap angular velocity
min_speed_theta: 0.3 # rad/s - Minimum rotation speed when rotating
```

#### 3. Velocity Smoother Parameters

```yaml
velocity_smoother:
  max_velocity: [2.0, 0.0, 0.5] # [vx, vy, vθ] - Cap angular velocity
  min_velocity: [-2.0, 0.0, -0.5]
```

#### 4. Behavior Server Parameters

```yaml
behavior_server:
  max_rotational_vel: 0.5 # rad/s - Cap angular velocity
  min_rotational_vel: 0.3
  rotational_acc_lim: 3.5
```

### Why This Configuration Works

1. **Controlled In-Place Rotations (0.5 rad/s)**
   - Slower speed prevents position loss during rotation
   - 180° turn takes ~6.3 seconds
   - Only active when heading error > 45° (angular_dist_threshold)
   - Prevents excessive speed that could cause localization drift

2. **Stable Forward Motion (0.5 rad/s)**
   - Consistent angular velocity limit across all navigation phases
   - Prevents excessive oscillation on curved paths
   - Maintains smooth trajectory tracking
   - Reduces wear on motors during normal navigation

3. **Velocity Smoother Compatibility**
   - Set to 0.5 rad/s to match controller limits
   - Ensures consistent velocity caps across the navigation stack
   - Smooths acceleration/deceleration for motor protection

4. **Consistent Recovery Behaviors**
   - Recovery spins use same 0.5 rad/s as in-place rotations
   - Controlled recovery from stuck situations
   - Consistent behavior across navigation modes

5. **Minimum Velocity Enforcement (0.3 rad/s)**
   - Prevents robot from getting stuck with very low angular velocities
   - `closed_loop: false` helps maintain this minimum during deceleration
   - Ensures reliable rotation completion

### Implementation Files

**Tuned Parameter Files:**

- `costnav_isaacsim/nav2_params/nova_carter/navigation_params_tuned_true.yaml`
- `costnav_isaacsim/nav2_params/segway_e1/navigation_params_tuned_true.yaml`

**Key Sections:**

- `controller_server.FollowPath` - RotationShimController configuration
- `controller_server.FollowPath.primary_controller` - DWB parameters
- `velocity_smoother` - Velocity limits
- `behavior_server` - Recovery behavior parameters

### Benefits

✅ **Segway E1 can now perform in-place rotations** - Resolved the spin-in-place problem
✅ **Controlled navigation** - Consistent 0.5 rad/s angular velocity across all phases
✅ **Stable path tracking** - Limited angular velocity prevents oscillation
✅ **Smooth transitions** - RotationShimController hands off cleanly to DWB
✅ **Prevents position loss** - Slower rotation (0.5 rad/s) maintains better localization
✅ **Reliable rotation completion** - Minimum 0.3 rad/s prevents getting stuck

---

## Executive Summary

This document outlines the Nav2 integration for CostNav, following NVIDIA's official [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html).

**Supported Robots:**

- **Nova Carter** (NVIDIA's reference platform) - ✅ Complete
- **Segway E1** (Delivery robot) - ✅ Complete

**Current Capabilities:**

- Rule-based navigation baseline for comparison with RL-based approaches
- Industry-standard Nav2 navigation stack
- Foundation for future hybrid RL+Nav2 approaches
- Ready for cost model evaluation integration

---

## Table of Contents

1. [Background & Motivation](#background--motivation)
2. [Reference Tutorial](#reference-tutorial)
3. [Architecture Overview](#architecture-overview)
4. [Implementation Plan (4 Weeks)](#implementation-plan-4-weeks)
5. [Technical Requirements](#technical-requirements)
6. [Success Metrics](#success-metrics)
7. [References](#references)
8. [Appendices](#appendices)

---

## Background & Motivation

### Current State

CostNav currently supports:

- **Learning-based navigation** using RL-Games, RSL-RL, SKRL, and Stable-Baselines3
- **Isaac Sim/Isaac Lab** simulation environment (v5.1.0 / v2.3.0)
- **✅ Nav2 Integration** - Complete and operational for both robots
- **Nova Carter robot** - NVIDIA's reference platform with full sensor suite
- **Segway E1 delivery robot** - Delivery robot for urban navigation
- **Dynamic robot selection** - SIM_ROBOT environment variable for switching between robots
- **Cost-driven evaluation** metrics (SLA compliance, profitability, break-even time)
- **Custom MDP components** for navigation (commands, observations, rewards, terminations)

### Completed Achievements

- ✅ Rule-based navigation baseline with Nav2
- ✅ Integration with industry-standard ROS2 navigation stack
- ✅ Classical planner implementation (NavFn, DWB, etc.)
- ✅ Multi-container Docker architecture
- ✅ ROS2 bridge for Isaac Sim communication

### Completed Tasks

1. ✅ **Start and Goal Sampling** - Complete (nav2_mission module)
2. ✅ **Segway E1 Robot Adaptation** - Complete (SIM_ROBOT environment variable with dynamic parameter selection)
3. ✅ **Parameter Tuning** - Complete (Optimized Nav2 parameters for both Nova Carter and Segway E1, including RotationShimController configuration)

### Remaining Tasks

1. **Cost Model Integration** - Track economic metrics for Nav2 navigation

---

## Reference Tutorial

**Primary Reference:** [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html)

This implementation follows NVIDIA's official tutorial which covers:

- ✅ Multi-container Docker setup (Isaac Sim + ROS2/Nav2) - **COMPLETE**
- ✅ ROS2 bridge configuration with `omni.isaac.ros2_bridge` - **COMPLETE**
- ✅ Occupancy map generation using Isaac Sim tools - **COMPLETE**
- ✅ Nav2 stack setup (AMCL, planners, controllers, behavior servers) - **COMPLETE**
- ✅ Robot description with `robot_state_publisher` - **COMPLETE**
- ✅ Navigation goal sending (RViz2, programmatic, ActionGraph) - **COMPLETE**

**Current Implementation:**

- **Primary Robot:** Nova Carter (NVIDIA's reference platform)
- **Status:** Fully operational Nav2 integration
- **Remaining Work:** Start/goal sampling and parameter tuning

**Current Capabilities:**

- ✅ **Multi-robot support** - Nova Carter and Segway E1 via SIM_ROBOT environment variable
- ✅ **Dynamic parameter selection** - Robot-specific navigation parameters and RViz configurations
- 📋 **Cost model tracking** for economic metrics (in progress)
- 📋 **Integration with existing CostNav benchmark framework** (in progress)
- 📋 **Comparison with RL-based navigation** (planned)

**Tutorial Sections to Follow:**

1. [Getting Started](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#getting-started)
2. [Nav2 Setup](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#nav2-setup)
3. [Occupancy Map Generation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#occupancy-map)
4. [Running Nav2](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#running-nav2)
5. [Sending Goals Programmatically](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#sending-goals-programmatically)

---

## Architecture Overview

### High-Level Design

```
┌─────────────────────────────────────────────────────────────────┐
│                        CostNav System                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐              ┌──────────────────┐        │
│  │  Learning-Based  │              │   Rule-Based     │        │
│  │   Navigation     │              │   Navigation     │        │
│  │   (Existing)     │              │   (Nav2) ✅      │        │
│  └────────┬─────────┘              └────────┬─────────┘        │
│           │                                  │                   │
│           │         ┌──────────────┐        │                   │
│           └────────►│ Unified Cost │◄───────┘                   │
│                     │    Model     │                            │
│                     └──────────────┘                            │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Isaac Sim / Isaac Lab Environment           │  │
│  │  ┌──────────────┐  ┌────────────┐  ┌────────────┐       │  │
│  │  │ Nova Carter  │  │   Sensors  │  │  Sidewalk  │       │  │
│  │  │  (Primary)   │  │  (RGB-D,   │  │    Map     │       │  │
│  │  │              │  │   LiDAR)   │  │            │       │  │
│  │  └──────────────┘  └────────────┘  └────────────┘       │  │
│  │  ┌──────────────┐                                        │  │
│  │  │ Segway E1    │  ✅ Supported                         │  │
│  │  │ (Delivery)   │                                        │  │
│  │  └──────────────┘                                        │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Nav2 Component Integration

![Nav2 Architecture](nav2_architecture.png)

**Key Nav2 Components to Integrate:**

1. **BT Navigator** - Behavior tree-based navigation orchestration
2. **Planner Server** - Global path planning (NavFn, Smac, Theta\*)
3. **Controller Server** - Local trajectory control (DWB, RPP, MPPI)
4. **Behavior Server** - Recovery behaviors (spin, backup, wait)
5. **Smoother Server** - Path smoothing for feasibility
6. **Costmap 2D** - Environmental representation with layers

---

## Robot Selection

CostNav supports multiple robots for Nav2 navigation. Use the `SIM_ROBOT` environment variable to select between supported robots.

### Supported Robots

| Robot       | Value         | Status      | Description                                        |
| ----------- | ------------- | ----------- | -------------------------------------------------- |
| Nova Carter | `nova_carter` | ✅ Complete | NVIDIA's reference platform with full sensor suite |
| Segway E1   | `segway_e1`   | ✅ Complete | Delivery robot for urban navigation                |

### Configuration Structure

Robot-specific configuration files are organized as follows:

```
nav2_params/
├── maps/                          # Shared map files for all robots
│   ├── sidewalk.yaml
│   ├── sidewalk.png
│   └── sidewalk_orthographic.png
├── nova_carter/                   # Nova Carter specific files
│   ├── navigation_params.yaml
│   ├── navigation.rviz
│   └── navigation_teleop.rviz
└── segway_e1/                     # Segway E1 specific files
    ├── navigation_params.yaml
    ├── navigation.rviz
    └── navigation_teleop.rviz
```

### Switching Between Robots

**Option 1: Environment Variable**

```bash
# Use Nova Carter (default)
make run-nav2

# Use Segway E1
SIM_ROBOT=segway_e1 make run-nav2
```

**Option 2: Set in `.env` file**

```bash
# Add to .env file
SIM_ROBOT=segway_e1
```

**Option 3: Docker Compose Override**

```bash
# Use Segway E1
docker compose --profile nav2 up -e SIM_ROBOT=segway_e1
```

The `SIM_ROBOT` variable automatically selects:

- Robot-specific navigation parameters (`/workspace/nav2_params/${SIM_ROBOT}/navigation_params.yaml`)
- Robot-specific RViz configuration (`/workspace/nav2_params/${SIM_ROBOT}/navigation.rviz`)
- Robot-specific teleop RViz configuration (`/workspace/nav2_params/${SIM_ROBOT}/navigation_teleop.rviz`)
- Shared map files (`/workspace/nav2_params/maps/sidewalk.yaml`)

---

## Running Nav2

### Quick Start

**Option 1: Run Stack (Manual Missions)**

```bash
# From repository root, start both containers with Nav2 profile
make run-nav2

# Trigger a mission manually
make start-mission
```

To customize mission parameters, edit `config/mission_config.yaml` or use:

```bash
# Inside Isaac Sim container
python /workspace/costnav_isaacsim/launch.py \
    --mission-timeout 600 \
    --min-distance 10.0 \
    --max-distance 30.0
```

**Option 2: Manual Launch (Step by Step)**

**1. Launch Isaac Sim with Nova Carter:**

```bash
cd /workspace/costnav_isaacsim
python launch.py
```

**2. In Docker Container, Launch Nav2:**

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /workspace/build_ws/install/local_setup.sh

# Launch Nav2 with Nova Carter
ros2 launch carter_navigation carter_navigation.launch.py \
    map:=/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.yaml \
    params_file:=/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml
```

**3. Mission Orchestration (Manual Trigger):**

Missions are triggered manually via `/start_mission`:

```bash
# Trigger a mission via Makefile
make start-mission

# Or call the service directly
ros2 service call /start_mission std_srvs/srv/Trigger {}
```

**Configuration Files:**

- **Map:** `/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.yaml`
- **Parameters:** `/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml`
- **Map Image:** `/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.png`
- **Mission Module:** `/workspace/costnav_isaacsim/nav2_mission/`

### Docker Volume Mounting

Ensure the following volumes are mounted in `docker-compose.yml`:

```yaml
volumes:
  - /workspace/costnav_isaacsim/nav2_params:/workspace/costnav_isaacsim/nav2_params:ro
```

---

## Implementation Plan (4 Weeks) - ✅ WEEKS 1-3 COMPLETE

Following the [official Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html), we are implementing Nav2 integration.

**Status:**

- ✅ Week 1: Docker Setup & ROS2 Bridge - Complete
- ✅ Week 2: Nova Carter Setup & Occupancy Map - Complete
- ✅ Week 3: Nav2 Stack Configuration & Mission Orchestration - Complete
- ✅ Week 4: Parameter Tuning - Complete
- ⏳ Week 5: Cost Model Integration - In progress (partially merged to main)

**Remaining Work:**

- Complete cost model integration with existing framework
- Benchmark comparison report: Nav2 vs RL baseline
- Documentation of cost metrics and evaluation methodology

---

### Week 1: Docker Setup & ROS2 Bridge - ✅ COMPLETE

**Objective:** Set up multi-container architecture and establish ROS2 communication

**Reference:** [Getting Started](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#getting-started) | [Nav2 Setup](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#nav2-setup)

**Tasks:**

- [x] Create `Dockerfile.ros` for ROS2/Nav2 container (based on `ros:jazzy-ros-base`)
- [x] Update `docker-compose.yml` with multi-container setup
- [x] Install Nav2 packages in ROS2 container
- [x] Configure shared Docker network and volumes
- [x] Enable `omni.isaac.ros2_bridge` extension in Isaac Sim
- [x] Verify ROS2 topics published from Isaac Sim (odom, tf, sensor data)
- [x] Test inter-container communication

**Deliverables:**

- ✅ `/workspace/Dockerfile.ros`
- ✅ `/workspace/docker-compose.yml`
- ✅ Working ROS2 bridge between containers

**Success Criteria:**

- ✅ `ros2 topic list` shows Isaac Sim topics from Nav2 container
- ✅ TF tree visualizes correctly

---

### Week 2: Nova Carter Setup & Occupancy Map - ✅ COMPLETE

**Objective:** Configure Nova Carter for Nav2 and generate occupancy map

**Reference:** [Occupancy Map](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#occupancy-map)

**Tasks:**

- [x] Configure Nova Carter robot in Isaac Sim
- [x] Configure `robot_state_publisher` for Nova Carter
- [x] Set up joint state publishing from Isaac Sim
- [x] Generate occupancy map using Isaac Sim Occupancy Map Generator
  - Configure bounds for Nova Carter sensor height
  - Export PNG + YAML in ROS format
- [x] Configure `map_server` in Nav2 container
- [x] Verify map visualization in RViz2

**Deliverables:**

- ✅ Nova Carter robot configuration
- ✅ Occupancy map (PNG + YAML) at `/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.*`
- ✅ ROS2 launch file for robot description

**Success Criteria:**

- ✅ Robot model displays in RViz2
- ✅ Occupancy map loads correctly

---

### Week 3: Nav2 Stack Configuration & Basic Navigation - ✅ COMPLETE

**Objective:** Configure Nav2 components, achieve basic navigation, and implement goal sampling with visualization

**Reference:** [Running Nav2](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#running-nav2) | [Sending Goals Programmatically](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#sending-goals-programmatically) | [Omniverse Navigation Mesh Extension](https://docs.omniverse.nvidia.com/extensions/latest/ext_navigation-mesh.html) | [ROS2 Marker Display Types](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html)

**Tasks:**

- [x] Configure Nav2 parameters for Nova Carter
- [x] Create ROS2 launch file for Nav2 stack
- [x] Test navigation with RViz2 "Nav2 Goal" button
- [x] Tune parameters for Nova Carter and Segway E1 kinematics
- [x] Implement position sampling using NavMesh
- [x] Ensure minimum distance threshold between start and goal
- [x] Implement robot teleportation and mission initiation
- [x] Configure RViz markers for start position, goal position, current robot position

**Deliverables:**

- ✅ Nav2 configuration files at `/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml`
- ✅ Nav2 launch file (`carter_navigation.launch.py`)
- ✅ RViz2 configuration
- ✅ Mission orchestration module at `/workspace/costnav_isaacsim/nav2_mission/`
- ✅ Configuration module at `/workspace/costnav_isaacsim/config/` with YAML-based settings
- ✅ Manual mission trigger via `/start_mission` with config loaded at startup
- ✅ Optimized parameters for both Nova Carter and Segway E1

**Success Criteria:**

- ✅ Robot navigates to goals successfully
- ✅ Avoids obstacles
- ✅ Recovery behaviors work
- ✅ Start/goal positions sampled from NavMesh with minimum distance threshold (5-50m configurable)
- ✅ Robot teleports to start and navigates to goal automatically
- ✅ RViz displays distinct markers for start (green), goal (red), and robot (blue) positions
- ✅ Parameters optimized for performance

#### Implementation: Nav2 Mission Module

The following components have been implemented in `/workspace/costnav_isaacsim/nav2_mission/`:

**1. NavMesh Sampler (`navmesh_sampler.py`)**

- Uses `omni.anim.navigation.core` extension for NavMesh queries
- `NavMeshSampler` class with methods:
  - `sample_random_position()` - Sample random point on NavMesh
  - `sample_start_goal_pair()` - Sample valid start/goal with distance constraints
  - `validate_position()` - Check if position is on NavMesh
  - `check_path_exists()` - Verify navigable path exists
- Configurable min/max distance thresholds (default: 5-50 meters)

**2. Marker Publisher (`marker_publisher.py`)**

- `MarkerPublisher` ROS2 node publishing to:
  - `/start_marker` - Green arrow for start position
  - `/goal_marker` - Red arrow for goal position
  - `/robot_marker` - Blue arrow for current robot position (real-time from `/odom`)
- Uses `visualization_msgs/Marker` with ARROW type
- TRANSIENT_LOCAL QoS for persistent marker visibility

**3. Mission Orchestrator (`mission_orchestrator.py`)**

- `MissionOrchestrator` ROS2 node that coordinates:
  - NavMesh-based position sampling
  - Robot teleportation via Isaac Sim API
  - Initial pose publication to `/initialpose`
  - Goal pose publication to `/goal_pose`
  - RViz marker updates
- `MissionConfig` dataclass for configuration
- `create_isaac_sim_teleport_callback()` helper for Isaac Sim integration

**4. Mission Runner (`mission_runner.py`)**

- `MissionRunner` class for background thread execution
- Manages mission lifecycle alongside simulation loop
- Graceful stop/cleanup with `_stop_event`
- Interruptible sleep between missions

**5. Configuration (`config/`)**

Mission parameters are configured via YAML file at `config/mission_config.yaml`:

```yaml
mission:
  timeout: 3600.0 # Mission timeout (seconds)

distance:
  min: 5.0 # Minimum start-goal distance (meters)
  max: 50.0 # Maximum start-goal distance (meters)

nav2:
  wait_time: 10.0 # Wait for Nav2 stack to initialize

teleport:
  robot_prim: "/World/Nova_Carter_ROS"
  height_offset: 0.5

markers:
  enabled: true
  topics:
    start: "/start_marker"
    goal: "/goal_marker"
    robot: "/robot_marker"
```

**6. Integrated Launch (`launch.py`)**

The mission module loads config at startup with CLI overrides:

```bash
# Use default config (config/mission_config.yaml)
python launch.py

# Use custom config file
python launch.py --config /path/to/custom.yaml

# Override config values via CLI
python launch.py --mission-timeout 600 --min-distance 10

# Available CLI overrides:
#   --config PATH       Path to config YAML file
#   --mission-timeout S Override: Mission timeout
#   --min-distance M    Override: Minimum start-goal distance
#   --max-distance M    Override: Maximum start-goal distance
#   --nav2-wait S       Override: Nav2 wait time
```

#### RViz Marker Topics

| Topic           | Color | Description                    |
| --------------- | ----- | ------------------------------ |
| `/start_marker` | Green | Start position (ARROW marker)  |
| `/goal_marker`  | Red   | Goal position (ARROW marker)   |
| `/robot_marker` | Blue  | Current robot position (10 Hz) |

---

### Week 4: Parameter Tuning - ✅ COMPLETE

**Objective:** Optimize Nav2 parameters for both Nova Carter and Segway E1 robots

**Tasks:**

- [x] Tune DWB controller parameters for Nova Carter
- [x] Tune DWB controller parameters for Segway E1
- [x] Optimize costmap parameters (inflation radius, obstacle layer)
- [x] Configure planner parameters for efficient path planning
- [x] Test and validate navigation performance for both robots
- [x] Create robot-specific parameter files

**Deliverables:**

- ✅ Optimized `nova_carter/navigation_params.yaml`
- ✅ Optimized `segway_e1/navigation_params.yaml`
- ✅ Robot-specific RViz configurations
- ✅ Validated navigation performance

**Success Criteria:**

- ✅ Both robots navigate successfully to goals
- ✅ Obstacle avoidance works reliably
- ✅ Recovery behaviors function properly
- ✅ Parameters documented and committed

**Known Issues:**

- ⚠️ Segway E1 cannot spin in place (see Known Issues section for details)

---

### Week 5: Cost Model Integration & Testing - ⏳ IN PROGRESS

**Objective:** Integrate cost tracking, run benchmark scenarios, and validate Nav2 against RL baseline

**Reference:** [Sending Goals Programmatically](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#sending-goals-programmatically) | [CostNav Cost Model](cost_model.md)

**Tasks:**

- [x] Basic Nav2 integration complete
- [x] Create programmatic goal sender (Python script)
- [x] Document usage and API
- [x] Implement energy consumption tracking (via odometry in mission_manager.py)
- [x] Implement distance and time metrics logging
- [x] Implement collision impulse tracking and health monitoring
- [x] Implement food spoilage tracking for delivery missions
- [ ] Create unified `nav2_cost_tracker.py` ROS2 node (consolidate existing metrics)
- [ ] Implement SLA compliance calculation based on distance/time metrics
- [ ] Create benchmark scenario runner with automated evaluation
- [ ] Generate comparison report: Nav2 vs RL

**Deliverables:**

- ✅ Python launch script (`launch.py`)
- ✅ Nav2 configuration files
- ✅ Documentation
- ✅ Mission manager with integrated cost tracking (`mission_manager.py`)
  - Energy consumption tracking via odometry
  - Distance and time metrics
  - Collision impulse and health monitoring
  - Food spoilage tracking
- ✅ Evaluation scripts (`eval_nav2.sh`, `eval_teleop.sh`)
- 📋 `nav2_cost_tracker.py` - Unified ROS2 node for real-time cost tracking (consolidation)
- 📋 `nav2_benchmark_runner.py` - Automated benchmark scenario executor
- 📋 `nav2_metrics_report.py` - Report generator with comparison analysis
- 📋 Benchmark results CSV/JSON files
- 📋 Nav2 vs RL comparison report (Markdown + visualizations)

**Success Criteria:**

- ✅ Nav2 runs successfully with Nova Carter and Segway E1
- ✅ Documentation published
- ✅ Energy consumption tracked via odometry integration
- ✅ Distance and time metrics logged for each mission
- ✅ Collision impulse and health monitoring implemented
- ✅ Food spoilage tracking for delivery missions
- 📋 Unified cost tracker node publishes real-time metrics to `/nav2/metrics` topic
- 📋 SLA compliance calculated for each navigation mission
- 📋 Benchmark scenarios complete with >95% automation
- 📋 Comparison report shows Nav2 vs RL performance delta
- 📋 Operating margin, break-even time, and SLA metrics comparable to RL baseline targets

#### Implementation Details: Cost Tracker Integration

**Current Implementation (Merged to Main):**

The cost tracking functionality is currently integrated into `mission_manager.py` with the following features:

**1. Energy Consumption Tracking**

- Implemented via odometry subscription in `_odom_callback()`
- Tracks distance traveled using odometry data
- Calculates mechanical power based on velocity and robot mass
- Logged in mission results for evaluation

**2. Distance and Time Metrics**

- `_traveled_distance`: Accumulated distance from odometry
- `_last_elapsed_time`: Mission duration tracking
- Logged via `eval_nav2.sh` and `eval_teleop.sh` scripts
- Provides data for SLA compliance calculation

**3. Collision Impulse and Health Monitoring**

- Contact sensor integration via `_on_contact_report()`
- Impulse-based health tracking system
- Contact count and total impulse logging
- Health degradation based on collision severity

**4. Food Spoilage Tracking**

- Food piece counting for delivery missions
- Spoilage detection based on mission duration
- Integration with Isaac Sim food assets
- Logged in mission results

**Future Consolidation: `nav2_cost_tracker.py` ROS2 Node**

To improve modularity, the following consolidation is planned:

- Extract cost tracking logic from `mission_manager.py` into dedicated node
- Subscribe to `/cmd_vel` for velocity-based energy estimation
- Subscribe to `/odom` for distance and path efficiency
- Subscribe to Nav2 action server feedback for navigation status
- Subscribe to `/local_costmap/costmap` for near-collision detection
- Publish unified metrics to `/nav2/metrics` topic
- Maintain compatibility with existing evaluation scripts

#### Implementation Details: Benchmark Runner

**1. Standard Test Scenarios**

- Short-range navigation (5-15m)
- Medium-range navigation (15-50m)
- Long-range navigation (50-100m)
- Obstacle-dense environments

**2. Benchmark Execution**

- Run N trials per scenario (e.g., N=100)
- Collect statistics: success rate, avg time, avg energy, SLA compliance
- Export metrics to CSV/JSON for analysis

**3. Comparison Report: Nav2 vs RL**

- Calculate key performance indicators:
  - Navigation success rate
  - Average time-to-goal
  - Energy efficiency (Wh/m)
  - SLA compliance rate
  - Recovery event frequency
- Compare against existing RL baseline metrics from CostNav
- Generate summary table and visualizations

---

## Technical Requirements

### Hardware Requirements

- **GPU:** NVIDIA GPU with 8GB+ VRAM (for Isaac Sim)
- **CPU:** 8+ cores recommended for Nav2 + Isaac Sim
- **RAM:** 32GB+ recommended
- **Storage:** 50GB+ for containers and maps

### Compatibility Matrix

| Component | Version       | Notes                 |
| --------- | ------------- | --------------------- |
| Isaac Sim | 5.1.0         | Current version       |
| Isaac Lab | 2.3.0         | Current version       |
| ROS2      | Humble/Iron   | LTS recommended       |
| Nav2      | 1.1.x / 1.2.x | Match ROS2 distro     |
| Python    | 3.10+         | Isaac Sim requirement |
| Ubuntu    | 22.04         | Container base        |

---

## Integration Points

### 1. Isaac Sim ↔ Nav2 Bridge (Multi-Container)

**Container Architecture:**

```
┌─────────────────────────────────────────────────────────────────┐
│                     Docker Network Bridge                        │
│                      (costnav_network)                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────┐  ┌────────────────────────────┐│
│  │   Isaac Sim Container      │  │   Nav2 Container           ││
│  │  (nvcr.io/nvidia/          │  │  (ros:humble-ros-base)     ││
│  │   isaac-sim:5.1.0)         │  │                            ││
│  │                            │  │                            ││
│  │  ┌──────────────────────┐  │  │  ┌──────────────────────┐ ││
│  │  │ Isaac Sim Simulation │  │  │  │  Nav2 Stack          │ ││
│  │  │ - Nova Carter        │  │  │  │  - BT Navigator      │ ││
│  │  │ - Sensors (RGB-D)    │  │  │  │  - Planner Server    │ ││
│  │  │ - Environment        │  │  │  │  - Controller Server │ ││
│  │  └──────────────────────┘  │  │  │  - Costmap 2D        │ ││
│  │           │                 │  │  │  - AMCL              │ ││
│  │  ┌────────▼──────────────┐ │  │  │  - Map Server        │ ││
│  │  │ ROS2 Bridge Extension │ │  │  └──────────────────────┘ ││
│  │  │ (omni.isaac.ros2_     │ │  │           │               ││
│  │  │  bridge)              │ │  │  ┌────────▼──────────────┐││
│  │  └───────────────────────┘ │  │  │ RViz2 Visualization  │││
│  │           │                 │  │  └──────────────────────┘││
│  └───────────┼─────────────────┘  └────────────┼─────────────┘│
│              │                                  │               │
│              └──────────────────────────────────┘               │
│                    ROS2 DDS Communication                       │
│              (Topics, Services, Actions, TF)                    │
└─────────────────────────────────────────────────────────────────┘
```

**Data Flow (Cross-Container):**

```
Isaac Sim Container                Nav2 Container
────────────────────────────────────────────────────────────────
Robot State        ──────────────► /odom (nav_msgs/Odometry)
Joint States       ──────────────► /joint_states (sensor_msgs/JointState)
                   ──────────────► /tf (tf2_msgs/TFMessage)
                   ──────────────► /tf_static (tf2_msgs/TFMessage)
RGB-D Camera       ──────────────► /camera/depth/points (PointCloud2)
                   ──────────────► /camera/rgb/image_raw (Image)
                   ──────────────► /camera/depth/image_raw (Image)
Lidar (optional)   ──────────────► /scan (LaserScan)
Contact Sensors    ──────────────► /bumper (sensor_msgs/Range)
Ground Truth       ──────────────► /ground_truth/pose (PoseStamped)

Nav2 Commands      ◄────────────── /cmd_vel (geometry_msgs/Twist)
Goal Poses         ◄────────────── /goal_pose (PoseStamped)
Initial Pose       ◄────────────── /initialpose (PoseWithCovarianceStamped)
```

**Implementation:**

**Isaac Sim Container:**

- Enable `omni.isaac.ros2_bridge` extension
- Configure ROS2 bridge to publish on Docker network
- Use Action Graphs for sensor data publishing
- Implement velocity command subscriber
- Set ROS_DOMAIN_ID environment variable
- Configure DDS discovery for cross-container communication

**Nav2 Container:**

- Source ROS2 environment on startup
- Launch Nav2 stack with proper parameters
- Run RViz2 for visualization
- Use same ROS_DOMAIN_ID as Isaac Sim
- Configure DDS middleware (CycloneDDS recommended)

### 2. Cost Model Integration

Track Nav2 navigation metrics for comparison with RL baseline:

```python
# nav2_cost_tracker.py - Subscribe to Nav2 topics and compute costs
nav2_metrics = {
    "energy_consumed": float,    # From /cmd_vel
    "time_to_goal": float,       # From action status
    "collision_count": int,      # From costmap
    "sla_compliance": bool,      # From timestamps
}
```

**Integration:** Create `nav2_cost_tracker.py` that subscribes to Nav2 topics and computes CostNav economic metrics.

---

## Testing Strategy

**Basic Testing Approach:**

1. **Unit Tests** - Test ROS2 bridge, costmap, cost tracker
2. **Integration Tests** - End-to-end navigation scenarios
3. **Benchmark Tests** - Compare Nav2 vs RL baseline

**Key Test Scenarios:**

- Basic navigation to goal
- Obstacle avoidance
- Delivery mission completion
- Cost metric validation

**Reference:** Use same test scenarios as RL evaluation for fair comparison.

---

## Success Metrics

### Functional Metrics

| Metric                  | Target | Measurement                           |
| ----------------------- | ------ | ------------------------------------- |
| Navigation Success Rate | >85%   | Goals reached / Total attempts        |
| Collision-Free Rate     | >90%   | Collision-free runs / Total runs      |
| SLA Compliance          | >40%   | Deliveries on-time / Total deliveries |

### Cost Model Metrics (Comparison with RL Baseline)

| Metric           | RL Baseline | Nav2 Target |
| ---------------- | ----------- | ----------- |
| Operating Margin | 46.5%       | >40%        |
| Break-Even Time  | 0.90 years  | <1.2 years  |
| SLA Compliance   | 43.0%       | >40%        |

**Goal:** Establish Nav2 as a competitive rule-based baseline for comparison with RL approaches.

---

## References

### Nav2 Documentation

- [Nav2 Official Docs](https://docs.nav2.org/)
- [Nav2 GitHub Repository](https://github.com/ros-planning/navigation2)
- [Nav2 Tutorials](https://docs.nav2.org/tutorials/index.html)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/index.html)

### Isaac Sim Integration

- [**Isaac Sim ROS2 Navigation Tutorial (Official)**](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html) - Primary reference for Nav2 integration
- [Isaac Sim ROS2 Bridge Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/index.html)
- [Isaac Sim Multi-Robot Navigation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_multi_navigation.html)
- [Isaac Sim ROS2 Joint States Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#nav2-with-nova-carter-with-robot-state-publisher-in-a-small-warehouse)
- [Isaac Sim Occupancy Map Generator](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#occupancy-map)

### Related Projects

- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Groot2](https://www.behaviortree.dev/groot/)
- [Robot Localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

### CostNav Internal

- [Architecture Documentation](architecture.md)
- [MDP Components](mdp_components.md)
- [Cost Model](cost_model.md)
- [Training Guide](training_guide.md)

---

## Document History

| Version | Date       | Author       | Changes                                                                                                                                                                                  |
| ------- | ---------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0.1     | 2025-11-21 | CostNav Team | Initial draft                                                                                                                                                                            |
| 0.2     | 2025-11-21 | CostNav Team | Updated with multi-container architecture and official Isaac Sim Nav2 tutorial references                                                                                                |
| 1.0     | 2025-12-10 | CostNav Team | Updated to reflect completed Nav2 integration with Nova Carter, removed network config, updated priorities                                                                               |
| 1.1     | 2025-12-12 | CostNav Team | Week 3 complete: Added nav2_mission module with NavMesh sampling, RViz markers, mission orchestration, integrated launch                                                                 |
| 1.2     | 2025-12-12 | CostNav Team | Refactored to use YAML config file (config/mission_config.yaml), added MissionRunner, separated config module                                                                            |
| 1.3     | 2026-01-22 | CostNav Team | Added Segway E1 robot support with SIM_ROBOT environment variable for dynamic robot selection, robot-specific parameter files, and shared map directory structure                        |
| 1.4     | 2026-01-23 | CostNav Team | Documented known issue: Segway E1 cannot spin in place, added troubleshooting section and investigation steps                                                                            |
| 1.5     | 2026-01-23 | CostNav Team | Updated status to reflect completed parameter tuning for both robots, reorganized implementation plan with Week 4 complete and Week 5 in progress                                        |
| 1.6     | 2026-01-26 | CostNav Team | Added RotationShimController documentation: resolved Segway E1 spin-in-place issue, documented angular velocity configuration strategy (1.2 rad/s in-place, 0.7 rad/s forward motion)    |
| 1.7     | 2026-01-26 | CostNav Team | Clarified Known Issues section: Distinguished between root cause solution (Isaac Sim USD Action Graph robot controller changes) and navigation enhancement (RotationShimController)      |
| 1.8     | 2026-01-26 | CostNav Team | Added `closed_loop: false` parameter to Segway E1 RotationShimController configuration to prevent premature deceleration and maintain minimum angular velocity during in-place rotation  |
| 1.9     | 2026-01-26 | CostNav Team | Reorganized task sections: moved completed tasks (Start/Goal Sampling, Segway E1 Adaptation, Parameter Tuning) to "Completed Tasks" section instead of removing them                     |
| 1.10    | 2026-01-26 | CostNav Team | Updated angular velocity parameters: reduced from 1.2 to 0.7 rad/s for all phases (rotation, path following, recovery), min_speed_theta reduced from 0.7 to 0.4 rad/s for better control |
| 1.11    | 2026-01-26 | CostNav Team | Further reduced angular velocity to 0.5 rad/s and minimum to 0.3 rad/s (Segway E1 only) to prevent position loss during fast rotation                                                    |
| 1.12    | 2026-01-26 | CostNav Team | Updated cost model integration status: documented merged features (energy tracking, distance/time metrics, collision monitoring, food spoilage), added RViz message filter queue issue   |

---

**End of Document**
