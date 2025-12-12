# Nav2 (ROS2 Navigation Stack 2) Implementation Plan

**Issue Reference:** [#5 - Support rule-based navigation with nav2](https://github.com/worv-ai/CostNav/issues/5)

**Status:** ✅ Nav2 Integration Complete | ✅ Mission Orchestration Complete | ⏳ Parameter Tuning In Progress
**Target Version:** CostNav v0.2.0
**Last Updated:** 2025-12-12

---

## Current Status

### ✅ Completed

- **Nav2 Integration with Nova Carter**: Full ROS2 Navigation Stack 2 integration complete
- **Docker Setup**: Multi-container architecture configured
- **ROS2 Bridge**: Communication between Isaac Sim and Nav2 established
- **Occupancy Map**: Generated and configured for Street_sidewalk environment
- **Basic Navigation**: Nova Carter successfully navigates to goals
- **Mission Orchestration**: Automated start/goal sampling, robot teleportation, and RViz visualization
- **NavMesh Position Sampling**: Valid start/goal positions sampled from Isaac Sim's NavMesh
- **RViz Marker Visualization**: Start (green), goal (red), and robot (blue) markers

### ⏳ In Progress

1. **Parameter Tuning**: Optimizing Nav2 parameters for Nova Carter performance
2. **Cost Model Integration**: Track economic metrics for Nav2 navigation

### 📋 Future Work

- **COCO Robot Integration**: Adapt Nav2 for COCO delivery robot (lower priority)
- **Hybrid RL+Nav2**: Combine learning-based and rule-based approaches

---

## Executive Summary

This document outlines the Nav2 integration for CostNav, following NVIDIA's official [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html).

**Primary Robot:** Nova Carter (NVIDIA's reference platform)
**Future Robot:** COCO delivery robot (planned)

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
- **✅ Nav2 Integration with Nova Carter** - Complete and operational
- **Nova Carter robot** (primary) - NVIDIA's reference platform with full sensor suite
- **COCO delivery robot** (future) - Custom delivery robot for urban navigation
- **Cost-driven evaluation** metrics (SLA compliance, profitability, break-even time)
- **Custom MDP components** for navigation (commands, observations, rewards, terminations)

### Completed Achievements

- ✅ Rule-based navigation baseline with Nav2
- ✅ Integration with industry-standard ROS2 navigation stack
- ✅ Classical planner implementation (NavFn, DWB, etc.)
- ✅ Multi-container Docker architecture
- ✅ ROS2 bridge for Isaac Sim communication

### Remaining Tasks

1. ~~**Start and Goal Sampling**~~ - ✅ Complete (nav2_mission module)
2. **Parameter Tuning** - Optimize Nav2 parameters for Nova Carter navigation performance
3. **Cost Model Integration** - Track economic metrics for Nav2 navigation
4. **COCO Robot Adaptation** - Extend Nav2 support to COCO delivery robot (future work)

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

**Future Adaptation:**

- Extend to **COCO delivery robot** (lower priority)
- Add **cost model tracking** for economic metrics
- Integrate with **existing CostNav benchmark framework**
- Enable **comparison with RL-based navigation**

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
│  │  │ COCO Robot   │  (Future Work)                        │  │
│  │  │  (Future)    │                                        │  │
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

## Running Nav2 with Nova Carter

### Quick Start

**Option 1: Run with Mission Orchestration (Recommended)**

```bash
# From repository root, start both containers with Nav2 profile
make run-nav2

# Isaac Sim will launch with mission orchestration enabled
# Missions will run automatically after Nav2 stack is ready
```

To customize mission parameters, modify the docker-compose command or use:

```bash
# Inside Isaac Sim container
python /workspace/costnav_isaacsim/launch.py --mission \
    --mission-count 5 \
    --mission-delay 60 \
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

**3. (Optional) Run Mission Orchestrator Separately:**

```bash
docker exec -it costnav-isaac-sim /isaac-sim/python.sh \
    /workspace/costnav_isaacsim/launch_mission.py --mission-count 5
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
- ⏳ Week 4: Cost Model Integration & Parameter Tuning - In progress

**Remaining Work:**

- Parameter tuning for optimal Nova Carter performance
- Cost model integration for economic metrics tracking

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
- [ ] ⏳ Tune parameters for Nova Carter kinematics (moved to Week 4)
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
- ✅ Integrated launch with `--mission` and `--config` flags in `launch.py`
- ⏳ Optimized parameters (moved to Week 4)

**Success Criteria:**

- ✅ Robot navigates to goals successfully
- ✅ Avoids obstacles
- ✅ Recovery behaviors work
- ✅ Start/goal positions sampled from NavMesh with minimum distance threshold (5-50m configurable)
- ✅ Robot teleports to start and navigates to goal automatically
- ✅ RViz displays distinct markers for start (green), goal (red), and robot (blue) positions
- ⏳ Parameters optimized for performance (moved to Week 4)

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
  count: 1          # Number of missions
  delay: 30.0       # Delay between missions (seconds)

distance:
  min: 5.0          # Minimum start-goal distance (meters)
  max: 50.0         # Maximum start-goal distance (meters)

nav2:
  wait_time: 10.0   # Wait for Nav2 stack to initialize

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

The mission module uses config file with CLI overrides:

```bash
# Use default config (config/mission_config.yaml)
python launch.py --mission

# Use custom config file
python launch.py --mission --config /path/to/custom.yaml

# Override config values via CLI
python launch.py --mission --mission-count 5 --min-distance 10

# Available CLI overrides:
#   --config PATH       Path to config YAML file
#   --mission-count N   Override: Number of missions
#   --mission-delay S   Override: Delay between missions
#   --min-distance M    Override: Minimum start-goal distance
#   --max-distance M    Override: Maximum start-goal distance
#   --nav2-wait S       Override: Nav2 wait time
```

#### RViz Marker Topics

| Topic          | Color | Description                    |
| -------------- | ----- | ------------------------------ |
| `/start_marker`| Green | Start position (ARROW marker)  |
| `/goal_marker` | Red   | Goal position (ARROW marker)   |
| `/robot_marker`| Blue  | Current robot position (10 Hz) |

---

### Week 4: Cost Model Integration & Testing

**Objective:** Integrate cost tracking, run benchmark scenarios, and validate Nav2 against RL baseline

**Reference:** [Sending Goals Programmatically](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#sending-goals-programmatically) | [CostNav Cost Model](cost_model.md)

**Tasks:**

- [x] Basic Nav2 integration complete
- [x] Create programmatic goal sender (Python script)
- [x] Document usage and API
- [ ] Create `nav2_cost_tracker.py` ROS2 node
- [ ] Implement energy consumption model
- [ ] Implement SLA compliance tracking
- [ ] Implement collision and recovery event tracking
- [ ] Create benchmark scenario runner
- [ ] Generate comparison report: Nav2 vs RL

**Deliverables:**

- ✅ Python launch script (`launch.py`)
- ✅ Nav2 configuration files
- ✅ Documentation
- 📋 `nav2_cost_tracker.py` - ROS2 node for real-time cost tracking
- 📋 `nav2_benchmark_runner.py` - Automated benchmark scenario executor
- 📋 `nav2_metrics_report.py` - Report generator with comparison analysis
- 📋 Benchmark results CSV/JSON files
- 📋 Nav2 vs RL comparison report (Markdown + visualizations)

**Success Criteria:**

- ✅ Nav2 runs successfully with Nova Carter
- ✅ Documentation published
- 📋 Cost tracker node publishes real-time metrics to `/nav2/metrics` topic
- 📋 Energy consumption tracked with <5% error vs ground truth
- 📋 SLA compliance calculated for each navigation mission
- 📋 Benchmark scenarios complete with >95% automation
- 📋 Comparison report shows Nav2 vs RL performance delta
- 📋 Operating margin, break-even time, and SLA metrics comparable to RL baseline targets

#### Implementation Details: Cost Tracker Node

**1. `nav2_cost_tracker.py` ROS2 Node**

- Subscribe to `/cmd_vel` to compute energy consumption from velocity commands
- Subscribe to `/odom` to track distance traveled and path efficiency
- Subscribe to Nav2 action server feedback for navigation status
- Subscribe to `/local_costmap/costmap` to detect near-collision events
- Implement timer-based tracking for time-to-goal metrics

**2. Energy Consumption Model**

- Calculate power draw: `P = k1*v + k2*ω + P_idle`
- Integrate power over time to get total energy (Wh)
- Use Nova Carter specifications for motor constants
- Log instantaneous and cumulative energy consumption

**3. SLA Compliance Tracking**

- Define delivery time thresholds based on distance (e.g., 0.5 m/s average speed)
- Track mission start time, goal reach time, and total duration
- Compare against SLA deadline to determine compliance
- Calculate SLA compliance rate across multiple missions

**4. Collision and Recovery Event Tracking**

- Monitor Nav2 behavior server for recovery triggers (spin, backup, wait)
- Count recovery events per mission
- Detect collision events from costmap proximity or contact sensors
- Log recovery success/failure rates

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
│  │  │ - COCO Robot         │  │  │  │  - BT Navigator      │ ││
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

| Version | Date       | Author       | Changes                                                                                                    |
| ------- | ---------- | ------------ | ---------------------------------------------------------------------------------------------------------- |
| 0.1     | 2025-11-21 | CostNav Team | Initial draft                                                                                              |
| 0.2     | 2025-11-21 | CostNav Team | Updated with multi-container architecture and official Isaac Sim Nav2 tutorial references                  |
| 1.0     | 2025-12-10 | CostNav Team | Updated to reflect completed Nav2 integration with Nova Carter, removed network config, updated priorities |
| 1.1     | 2025-12-12 | CostNav Team | Week 3 complete: Added nav2_mission module with NavMesh sampling, RViz markers, mission orchestration, integrated launch |
| 1.2     | 2025-12-12 | CostNav Team | Refactored to use YAML config file (config/mission_config.yaml), added MissionRunner, separated config module |

---

**End of Document**
