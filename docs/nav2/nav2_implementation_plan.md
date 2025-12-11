# Nav2 (ROS2 Navigation Stack 2) Implementation Plan

**Issue Reference:** [#5 - Support rule-based navigation with nav2](https://github.com/worv-ai/CostNav/issues/5)

**Status:** ✅ Nav2 Integration Complete | ⏳ Parameter Tuning In Progress
**Target Version:** CostNav v0.2.0
**Last Updated:** 2025-12-10

---

## Current Status

### ✅ Completed

- **Nav2 Integration with Nova Carter**: Full ROS2 Navigation Stack 2 integration complete
- **Docker Setup**: Multi-container architecture configured
- **ROS2 Bridge**: Communication between Isaac Sim and Nav2 established
- **Occupancy Map**: Generated and configured for Street_sidewalk environment
- **Basic Navigation**: Nova Carter successfully navigates to goals

### ⏳ In Progress

1. **Start and Goal Sampling**: Mission planning system for autonomous goal generation
2. **Parameter Tuning**: Optimizing Nav2 parameters for Nova Carter performance

### 📋 Future Work

- **COCO Robot Integration**: Adapt Nav2 for COCO delivery robot (lower priority)
- **Cost Model Integration**: Track economic metrics for Nav2 navigation
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

1. **Start and Goal Sampling** - Implement mission planning system for autonomous goal generation
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

**Configuration Files:**

- **Map:** `/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.yaml`
- **Parameters:** `/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml`
- **Map Image:** `/workspace/costnav_isaacsim/nav2_params/carter_sidewalk.png`

### Docker Volume Mounting

Ensure the following volumes are mounted in `docker-compose.yml`:

```yaml
volumes:
  - /workspace/costnav_isaacsim/nav2_params:/workspace/costnav_isaacsim/nav2_params:ro
```

---

## Implementation Plan (4 Weeks) - ⏳ IN PROGRESS

Following the [official Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html), we are implementing Nav2 integration.

**Status:**

- ✅ Week 1: Docker Setup & ROS2 Bridge - Complete
- ✅ Week 2: Nova Carter Setup & Occupancy Map - Complete
- ⏳ Week 3: Nav2 Stack Configuration & Basic Navigation - Parameter tuning in progress
- ⏳ Week 4: Cost Model Integration & Testing - In progress

**Remaining Work:**

- Parameter tuning for optimal Nova Carter performance
- Start and goal sampling system implementation
- Cost model integration (future)

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

### Week 3: Nav2 Stack Configuration & Basic Navigation -(⏳ Parameter Tuning In Progress)

**Objective:** Configure Nav2 components and achieve basic navigation

**Reference:** [Running Nav2](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#running-nav2)

**Tasks:**

- [x] Configure Nav2 parameters for Nova Carter
  - Costmap configuration (global + local)
  - AMCL localization parameters
  - Planner server (NavFn or Smac)
  - Controller server (DWB or RPP)
  - Behavior server (recovery behaviors)
- [x] Create ROS2 launch file for Nav2 stack
- [x] Test navigation with RViz2 "Nav2 Goal" button
- [ ] ⏳ **Tune parameters for Nova Carter kinematics** (In Progress)

**Deliverables:**

- ✅ Nav2 configuration files at `/workspace/costnav_isaacsim/nav2_params/carter_navigation_params.yaml`
- ✅ Nav2 launch file (`carter_navigation.launch.py`)
- ✅ RViz2 configuration
- ⏳ Optimized parameters (in progress)

**Success Criteria:**

- ✅ Robot navigates to goals successfully
- ✅ Avoids obstacles
- ✅ Recovery behaviors work
- ⏳ Parameters optimized for performance (in progress)

---

### Week 4: Cost Model Integration & Testing

**Objective:** Integrate cost tracking and validate against RL baseline

**Reference:** [Sending Goals Programmatically](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#sending-goals-programmatically)

**Tasks:**

- [x] Basic Nav2 integration complete
- [x] Create programmatic goal sender (Python script)
- [x] Document usage and API
- [ ] Implement cost model tracker for Nav2 navigation (⏳ Future work)
  - Track energy consumption
  - Track time to goal
  - Track SLA compliance
  - Track collision/recovery events
- [ ] Run benchmark scenarios (same as RL evaluation) (⏳ Future work)
- [ ] Compare Nav2 vs RL performance (⏳ Future work)

**Deliverables:**

- ✅ Python launch script (`launch.py`)
- ✅ Nav2 configuration files
- ✅ Documentation
- 📋 `nav2_cost_tracker.py` (future)
- 📋 Benchmark comparison report (future)

**Success Criteria:**

- ✅ Nav2 runs successfully with Nova Carter
- ✅ Documentation published
- ⏳ Cost metrics collection (future)
- ⏳ Comparison with RL baseline (future)

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

---

**End of Document**
