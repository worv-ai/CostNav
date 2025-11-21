# Nav2 (ROS2 Navigation Stack 2) Implementation Plan

**Issue Reference:** [#5 - Support rule-based navigation with nav2](https://github.com/worv-ai/CostNav/issues/5)

**Status:** Planning Phase  
**Target Version:** CostNav v0.2.0  
**Last Updated:** 2025-11-21

---

## Executive Summary

This document outlines a **simplified 1-month implementation plan** for integrating ROS2 Navigation Stack 2 (Nav2) into CostNav, following NVIDIA's official [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html).

**Scope:** Basic Nav2 integration with COCO robot for rule-based navigation baseline

**Timeline:** 4 weeks (1 month)

**Approach:** Adapt the official Nova Carter Nav2 example to CostNav's COCO delivery robot

This integration will enable:
- Rule-based navigation baseline for comparison with RL-based approaches
- Industry-standard Nav2 navigation stack
- Foundation for future hybrid RL+Nav2 approaches
- Cost model evaluation of rule-based navigation

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
- **COCO delivery robot** with RGB-D cameras and configurable sensors
- **Cost-driven evaluation** metrics (SLA compliance, profitability, break-even time)
- **Custom MDP components** for navigation (commands, observations, rewards, terminations)

### Gaps

- No rule-based navigation baseline for comparison
- Limited ability to define custom navigation constraints
- No integration with industry-standard ROS2 navigation stack
- Missing classical planner benchmarks (A*, DWA, TEB, etc.)

### Goals

1. **Enable Nav2 integration** within Isaac Sim environment
2. **Provide rule-based navigation API** for defining custom logic and constraints
3. **Maintain compatibility** with existing RL-based path planners
4. **Support hybrid approaches** combining learning and rule-based methods
5. **Preserve cost model evaluation** for fair comparison

---

## Reference Tutorial

**Primary Reference:** [Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html)

This implementation follows NVIDIA's official tutorial which covers:
- ✅ Multi-container Docker setup (Isaac Sim + ROS2/Nav2)
- ✅ ROS2 bridge configuration with `omni.isaac.ros2_bridge`
- ✅ Occupancy map generation using Isaac Sim tools
- ✅ Nav2 stack setup (AMCL, planners, controllers, behavior servers)
- ✅ Robot description with `robot_state_publisher`
- ✅ Navigation goal sending (RViz2, programmatic, ActionGraph)

**Our Adaptation:**
- Replace **Nova Carter** → **COCO delivery robot**
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
│  │   (Existing)     │              │   (New - Nav2)   │        │
│  └────────┬─────────┘              └────────┬─────────┘        │
│           │                                  │                   │
│           │         ┌──────────────┐        │                   │
│           └────────►│ Unified Cost │◄───────┘                   │
│                     │    Model     │                            │
│                     └──────────────┘                            │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Isaac Sim / Isaac Lab Environment           │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐         │  │
│  │  │ COCO Robot │  │   Sensors  │  │  Sidewalk  │         │  │
│  │  │            │  │  (RGB-D)   │  │    Map     │         │  │
│  │  └────────────┘  └────────────┘  └────────────┘         │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Nav2 Component Integration

![Nav2 Architecture](nav2_architecture.png)

**Key Nav2 Components to Integrate:**

1. **BT Navigator** - Behavior tree-based navigation orchestration
2. **Planner Server** - Global path planning (NavFn, Smac, Theta*)
3. **Controller Server** - Local trajectory control (DWB, RPP, MPPI)
4. **Behavior Server** - Recovery behaviors (spin, backup, wait)
5. **Smoother Server** - Path smoothing for feasibility
6. **Costmap 2D** - Environmental representation with layers

---

## Implementation Plan (4 Weeks)

Following the [official Isaac Sim ROS2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html), we will implement a basic Nav2 integration in 4 weeks.

---

### Week 1: Docker Setup & ROS2 Bridge

**Objective:** Set up multi-container architecture and establish ROS2 communication

**Reference:** [Getting Started](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#getting-started) | [Nav2 Setup](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#nav2-setup)

**Tasks:**
- [ ] Create `Dockerfile.nav2` for ROS2/Nav2 container (based on `ros:humble-ros-base`)
- [ ] Update `docker-compose.yml` with multi-container setup
- [ ] Install Nav2 packages in ROS2 container
- [ ] Configure shared Docker network and volumes
- [ ] Enable `omni.isaac.ros2_bridge` extension in Isaac Sim
- [ ] Verify ROS2 topics published from Isaac Sim (odom, tf, sensor data)
- [ ] Test inter-container communication

**Deliverables:**
- `Dockerfile.nav2`
- `docker-compose.nav2.yml`
- Working ROS2 bridge between containers

**Success Criteria:**
- `ros2 topic list` shows Isaac Sim topics from Nav2 container
- TF tree visualizes correctly

---

### Week 2: COCO Robot Setup & Occupancy Map

**Objective:** Adapt COCO robot for Nav2 and generate occupancy map

**Reference:** [Occupancy Map](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#occupancy-map)

**Tasks:**
- [ ] Create COCO robot URDF from USD (or adapt existing)
- [ ] Configure `robot_state_publisher` for COCO robot
- [ ] Set up joint state publishing from Isaac Sim
- [ ] Generate occupancy map using Isaac Sim Occupancy Map Generator
  - Configure bounds for COCO robot sensor height
  - Export PNG + YAML in ROS format
- [ ] Configure `map_server` in Nav2 container
- [ ] Verify map visualization in RViz2

**Deliverables:**
- COCO robot URDF
- Occupancy map (PNG + YAML)
- ROS2 launch file for robot description

**Success Criteria:**
- Robot model displays in RViz2
- Occupancy map loads correctly

---

### Week 3: Nav2 Stack Configuration & Basic Navigation

**Objective:** Configure Nav2 components and achieve basic navigation

**Reference:** [Running Nav2](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#running-nav2)

**Tasks:**
- [ ] Configure Nav2 parameters for COCO robot
  - Costmap configuration (global + local)
  - AMCL localization parameters
  - Planner server (NavFn or Smac)
  - Controller server (DWB or RPP)
  - Behavior server (recovery behaviors)
- [ ] Create ROS2 launch file for Nav2 stack
- [ ] Test navigation with RViz2 "Nav2 Goal" button
- [ ] Tune parameters for COCO robot kinematics

**Deliverables:**
- Nav2 configuration files (YAML)
- Nav2 launch file
- RViz2 configuration

**Success Criteria:**
- Robot navigates to goals successfully
- Avoids obstacles
- Recovery behaviors work

---

### Week 4: Cost Model Integration & Testing

**Objective:** Integrate cost tracking and validate against RL baseline

**Reference:** [Sending Goals Programmatically](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#sending-goals-programmatically)

**Tasks:**
- [ ] Implement cost model tracker for Nav2 navigation
  - Track energy consumption
  - Track time to goal
  - Track SLA compliance
  - Track collision/recovery events
- [ ] Create programmatic goal sender (Python script)
- [ ] Run benchmark scenarios (same as RL evaluation)
- [ ] Compare Nav2 vs RL performance
- [ ] Document results and findings

**Deliverables:**
- `nav2_cost_tracker.py`
- Goal sender script
- Benchmark comparison report
- Documentation

**Success Criteria:**
- Cost metrics collected for Nav2 navigation
- Comparison with RL baseline complete
- Documentation published

---

## Technical Requirements

### Software Dependencies

**Docker Container Architecture:**

```yaml
# Multi-container setup
containers:
  isaac_sim:
    base_image: nvcr.io/nvidia/isaac-sim:5.1.0
    purpose: Robot simulation and sensor data generation
    extensions:
      - omni.isaac.ros2_bridge
      - omni.isaac.sensor
      - omni.isaac.range_sensor
      - omni.isaac.wheeled_robots
      - omni.anim.navigation.core
      - omni.anim.navigation.bundle
    network: costnav_network

  nav2:
    base_image: ros:humble-ros-base  # or ros:iron-ros-base
    purpose: ROS2 Navigation Stack 2
    packages: [see below]
    network: costnav_network
    depends_on: isaac_sim
```

**ROS2 Packages (Nav2 Container):**
```yaml
ros2_packages:
  core:
    - ros-humble-navigation2
    - ros-humble-nav2-bringup
    - ros-humble-nav2-common
    - ros-humble-nav2-msgs

  planners:
    - ros-humble-nav2-navfn-planner
    - ros-humble-nav2-smac-planner
    - ros-humble-nav2-theta-star-planner

  controllers:
    - ros-humble-nav2-dwb-controller
    - ros-humble-nav2-regulated-pure-pursuit-controller
    - ros-humble-nav2-mppi-controller

  utilities:
    - ros-humble-nav2-costmap-2d
    - ros-humble-nav2-lifecycle-manager
    - ros-humble-nav2-map-server
    - ros-humble-nav2-amcl
    - ros-humble-robot-localization
    - ros-humble-pointcloud-to-laserscan  # For RGB-D to LaserScan conversion

  visualization:
    - ros-humble-rviz2
    - ros-humble-nav2-rviz-plugins
    - ros-humble-tf2-tools

  development:
    - ros-humble-rqt
    - ros-humble-rqt-common-plugins
    - groot2  # Behavior tree visualization (optional)
```

**Python Dependencies (Nav2 Container):**
```python
# Add to Nav2 container requirements.txt
rclpy>=3.3.0
nav2-simple-commander>=1.0.0
transforms3d>=0.4.1
pyyaml>=6.0
numpy>=1.24.0
```

**DDS Middleware (Both Containers):**
```yaml
# Recommended: CycloneDDS for better cross-container performance
dds_middleware:
  - ros-humble-rmw-cyclonedds-cpp

# Alternative: FastDDS (default)
# - ros-humble-rmw-fastrtps-cpp
```

### Hardware Requirements

- **GPU:** NVIDIA GPU with 8GB+ VRAM (for Isaac Sim)
- **CPU:** 8+ cores recommended for Nav2 + Isaac Sim
- **RAM:** 32GB+ recommended
- **Storage:** 50GB+ for containers and maps

### Compatibility Matrix

| Component | Version | Notes |
|-----------|---------|-------|
| Isaac Sim | 5.1.0 | Current version |
| Isaac Lab | 2.3.0 | Current version |
| ROS2 | Humble/Iron | LTS recommended |
| Nav2 | 1.1.x / 1.2.x | Match ROS2 distro |
| Python | 3.10+ | Isaac Sim requirement |
| Ubuntu | 22.04 | Container base |

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

**Network Configuration:**
```yaml
# docker-compose.yml network settings
networks:
  costnav_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

# Environment variables for both containers
environment:
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CYCLONEDDS_URI=/config/cyclonedds.xml
```

**DDS Configuration (cyclonedds.xml):**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

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

| Metric | Target | Measurement |
|--------|--------|-------------|
| Navigation Success Rate | >85% | Goals reached / Total attempts |
| Collision-Free Rate | >90% | Collision-free runs / Total runs |
| SLA Compliance | >40% | Deliveries on-time / Total deliveries |

### Cost Model Metrics (Comparison with RL Baseline)

| Metric | RL Baseline | Nav2 Target |
|--------|-------------|-------------|
| Operating Margin | 46.5% | >40% |
| Break-Even Time | 0.90 years | <1.2 years |
| SLA Compliance | 43.0% | >40% |

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

### Academic Papers
- Macenski, S., et al. (2020). "The Marathon 2: A Navigation System." IEEE/RSJ IROS.
- Macenski, S., et al. (2023). "From the Lab to the Real World: Practical Considerations for Deploying Nav2."
- CostNav Paper (forthcoming)

---

## Appendix A: File Structure

### New Files to Create

```
# Root directory
.
├── Dockerfile                                 # Existing Isaac Sim container
├── Dockerfile.nav2                            # NEW: ROS2/Nav2 container
├── docker-compose.yml                         # UPDATED: Multi-container setup
├── docker-compose.nav2.yml                    # NEW: Nav2-specific compose file
└── .env                                       # NEW: Environment variables

# Docker configuration
docker/
├── nav2/
│   ├── Dockerfile                             # Nav2 container definition
│   ├── entrypoint.sh                          # Container startup script
│   ├── requirements.txt                       # Python dependencies
│   └── cyclonedds.xml                         # DDS configuration
└── isaac-sim/
    └── ros2_bridge_config.yaml                # ROS2 bridge settings

# ROS2 workspace (for Nav2 container)
ros2_ws/
├── src/
│   ├── costnav_nav2/                          # Main Nav2 package
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── navigation.launch.py
│   │   │   ├── coco_navigation.launch.py
│   │   │   └── robot_description.launch.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── costmap_common.yaml
│   │   │   ├── global_costmap.yaml
│   │   │   ├── local_costmap.yaml
│   │   │   ├── planner_server.yaml
│   │   │   ├── controller_server.yaml
│   │   │   └── behavior_server.yaml
│   │   ├── maps/
│   │   │   ├── coco_warehouse.yaml
│   │   │   └── coco_warehouse.png
│   │   ├── urdf/
│   │   │   ├── coco_robot.urdf
│   │   │   └── coco_robot.xacro
│   │   ├── rviz/
│   │   │   └── nav2_default.rviz
│   │   └── behavior_trees/
│   │       ├── navigate_to_pose.xml
│   │       ├── navigate_through_poses.xml
│   │       └── cost_aware_navigation.xml
│   ├── costnav_nav2_plugins/                  # Custom Nav2 plugins
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── costnav_nav2_plugins/
│   │   │       ├── cost_aware_planner_selector.hpp
│   │   │       ├── sla_goal_checker.hpp
│   │   │       └── energy_efficient_controller.hpp
│   │   └── src/
│   │       ├── cost_aware_planner_selector.cpp
│   │       ├── sla_goal_checker.cpp
│   │       └── energy_efficient_controller.cpp
│   └── costnav_nav2_msgs/                     # Custom messages
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── msg/
│       │   ├── CostMetrics.msg
│       │   └── NavigationRule.msg
│       └── srv/
│           └── SetNavigationRule.srv
└── install/                                   # Built packages

# Isaac Lab integration
costnav_isaaclab/
├── source/costnav_isaaclab/costnav_isaaclab/
│   ├── tasks/manager_based/
│   │   └── costnav_isaaclab_v3_Nav2/          # New Nav2 task
│   │       ├── __init__.py
│   │       ├── nav2_env_cfg.py
│   │       ├── nav2_bridge.py
│   │       ├── nav2_cost_tracker.py
│   │       ├── action_graphs/
│   │       │   ├── ros2_publishers.py
│   │       │   └── ros2_subscribers.py
│   │       └── rules/
│   │           ├── rule_engine.py
│   │           ├── rule_schema.yaml
│   │           └── example_rules.yaml
│   └── utils/
│       ├── usd_to_occupancy_map.py
│       └── ros2_bridge_helpers.py
├── scripts/
│   ├── nav2/
│   │   ├── launch_multi_container.sh          # Start both containers
│   │   ├── test_nav2_integration.py
│   │   ├── benchmark_nav2_vs_rl.py
│   │   ├── send_navigation_goal.py
│   │   └── visualize_nav2.py
│   └── tools/
│       ├── convert_usd_to_map.py
│       ├── generate_nav2_config.py
│       └── validate_ros2_bridge.py
└── config/
    └── nav2/
        ├── network_config.yaml
        └── bridge_topics.yaml

# Documentation
docs/
├── nav2/
│   ├── nav2_implementation_plan.md            # This file
│   ├── nav2_architecture.png
│   ├── ros2_bridge_setup.md
│   ├── multi_container_setup.md               # NEW: Docker setup guide
│   ├── nav2_costmap_config.md
│   ├── nav2_planners_controllers.md
│   ├── nav2_behavior_trees.md
│   ├── rule_based_navigation.md
│   ├── nav2_testing_guide.md
│   └── nav2_benchmarks.md
└── diagrams/
    └── multi_container_architecture.png

# Tests
tests/
└── nav2/
    ├── test_ros2_bridge.py
    ├── test_multi_container_communication.py  # NEW
    ├── test_costmap_integration.py
    ├── test_planners.py
    ├── test_controllers.py
    ├── test_behavior_trees.py
    └── test_rule_engine.py
```

---

## Appendix B: Example Rule Specification

```yaml
# example_rules.yaml
rules:
  - name: "speed_limit_crosswalk"
    type: "speed_limit"
    condition:
      zone_type: "crosswalk"
      distance_threshold: 5.0  # meters
    action:
      max_velocity: 0.5  # m/s
      priority: high

  - name: "keepout_construction"
    type: "keepout_zone"
    condition:
      zone_id: "construction_area_1"
      time_range: ["08:00", "18:00"]
    action:
      cost_multiplier: 1000.0
      allow_traversal: false

  - name: "priority_delivery_zone"
    type: "priority_zone"
    condition:
      zone_type: "delivery_area"
      sla_remaining: "<5min"
    action:
      planner: "smac_hybrid"  # More accurate
      controller: "mppi"       # More aggressive
      cost_weight: 2.0

  - name: "energy_conservation"
    type: "cost_optimization"
    condition:
      battery_level: "<30%"
      distance_to_charger: ">100m"
    action:
      max_velocity: 0.8
      planner: "navfn"  # Faster planning
      prefer_flat_terrain: true

  - name: "maintenance_risk_avoidance"
    type: "risk_mitigation"
    condition:
      terrain_roughness: ">0.7"
      maintenance_cost_ratio: ">0.3"
    action:
      cost_multiplier: 1.5
      max_acceleration: 0.5
```

---

## Appendix C: Example Behavior Tree

```xml
<!-- cost_aware_navigation.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">

        <!-- Cost-aware planner selection -->
        <CostAwarePlannerSelector
          planner_ids="NavFn;SmacHybrid;ThetaStar"
          cost_threshold="100.0"
          sla_weight="0.4"
          energy_weight="0.3"
          maintenance_weight="0.3"/>

        <!-- Compute path to pose -->
        <ComputePathToPose
          goal="{goal}"
          path="{path}"
          planner_id="{selected_planner}"/>

        <!-- Check SLA compliance -->
        <SLAGoalChecker
          path="{path}"
          sla_deadline="{sla_deadline}"
          current_time="{current_time}"/>

        <!-- Smooth path -->
        <SmoothPath
          unsmoothed_path="{path}"
          smoothed_path="{smoothed_path}"
          smoother_id="ConstrainedSmoother"/>

        <!-- Energy-efficient controller selection -->
        <EnergyEfficientControllerSelector
          controller_ids="DWB;RPP;MPPI"
          battery_level="{battery_level}"
          terrain_type="{terrain_type}"/>

        <!-- Follow path -->
        <FollowPath
          path="{smoothed_path}"
          controller_id="{selected_controller}"/>

      </PipelineSequence>

      <!-- Recovery behaviors -->
      <ReactiveFallback name="RecoveryActions">
        <ClearCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear"/>
        <ClearCostmap name="ClearLocalCostmap" service_name="local_costmap/clear"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.5" backup_speed="0.1"/>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>
</root>
```

---

## Appendix D: Cost Model Integration Example

```python
# nav2_cost_tracker.py

from typing import Dict, Any
import time
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class Nav2CostTracker:
    """Tracks cost metrics for Nav2 navigation."""

    def __init__(self, robot_mass: float = 50.0, gravity: float = 9.81):
        self.robot_mass = robot_mass
        self.gravity = gravity
        self.reset()

    def reset(self):
        """Reset all tracked metrics."""
        self.metrics = {
            "start_time": time.time(),
            "end_time": None,
            "total_distance": 0.0,
            "total_energy": 0.0,
            "collision_count": 0,
            "recovery_count": 0,
            "planning_time_total": 0.0,
            "planning_count": 0,
            "goal_reached": False,
        }
        self.last_pose = None
        self.last_velocity = None

    def update_path(self, path: Path, planning_time: float):
        """Update metrics when new path is computed."""
        self.metrics["planning_time_total"] += planning_time
        self.metrics["planning_count"] += 1

        # Compute path length
        path_length = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            path_length += (dx**2 + dy**2)**0.5

        return path_length

    def update_velocity(self, cmd_vel: Twist, dt: float):
        """Update energy consumption based on velocity command."""
        # Simple energy model: E = m * g * v * dt
        speed = (cmd_vel.linear.x**2 + cmd_vel.linear.y**2)**0.5
        energy = self.robot_mass * self.gravity * speed * dt
        self.metrics["total_energy"] += energy
        self.last_velocity = cmd_vel

    def record_collision(self):
        """Record a collision event."""
        self.metrics["collision_count"] += 1

    def record_recovery(self):
        """Record a recovery behavior execution."""
        self.metrics["recovery_count"] += 1

    def record_goal_reached(self):
        """Record successful goal achievement."""
        self.metrics["goal_reached"] = True
        self.metrics["end_time"] = time.time()

    def compute_cost_metrics(self, sla_deadline: float,
                            revenue_per_delivery: float) -> Dict[str, Any]:
        """Compute final cost model metrics."""
        duration = (self.metrics["end_time"] or time.time()) - self.metrics["start_time"]

        # Energy cost ($/kWh)
        energy_kwh = self.metrics["total_energy"] / 3600000.0  # J to kWh
        energy_cost = energy_kwh * 0.12  # $0.12/kWh

        # Maintenance cost (based on collisions and distance)
        maintenance_cost = (
            self.metrics["collision_count"] * 50.0 +  # $50 per collision
            self.metrics["total_distance"] * 0.01      # $0.01 per meter
        )

        # SLA compliance
        sla_compliant = duration <= sla_deadline and self.metrics["goal_reached"]
        revenue = revenue_per_delivery if sla_compliant else 0.0

        # Operating margin
        total_cost = energy_cost + maintenance_cost
        operating_margin = (revenue - total_cost) / revenue if revenue > 0 else 0.0

        return {
            "duration": duration,
            "distance": self.metrics["total_distance"],
            "energy_kwh": energy_kwh,
            "energy_cost": energy_cost,
            "maintenance_cost": maintenance_cost,
            "total_cost": total_cost,
            "revenue": revenue,
            "operating_margin": operating_margin,
            "sla_compliant": sla_compliant,
            "collision_count": self.metrics["collision_count"],
            "recovery_count": self.metrics["recovery_count"],
            "avg_planning_time": (
                self.metrics["planning_time_total"] / self.metrics["planning_count"]
                if self.metrics["planning_count"] > 0 else 0.0
            ),
        }
```

---

## Appendix E: Quick Start Guide

### Minimal Nav2 Setup (Multi-Container Architecture)

**Prerequisites:**
- Docker and Docker Compose installed
- NVIDIA Container Toolkit configured
- ROS2 workspace built (in nav2 container)

**Step 1: Build both containers**
```bash
# Build Isaac Sim container (existing)
docker compose --profile isaac-lab build

# Build Nav2 container (new)
docker compose -f docker-compose.nav2.yml build

# Or build both at once
docker compose -f docker-compose.yml -f docker-compose.nav2.yml build
```

**Step 2: Start multi-container setup**
```bash
# Start both containers with shared network
docker compose -f docker-compose.yml -f docker-compose.nav2.yml up -d

# Verify containers are running
docker ps | grep costnav

# Expected output:
# costnav-isaac-sim    (Isaac Sim container)
# costnav-nav2         (ROS2/Nav2 container)
```

**Step 3: Launch Isaac Sim with ROS2 bridge (Isaac Sim container)**
```bash
# Enter Isaac Sim container
docker exec -it costnav-isaac-sim bash

# Launch Isaac Sim with Nav2 task
python scripts/nav2/launch_nav2.py --task=Template-Costnav-Isaaclab-v3-Nav2

# Or use the standalone script
./IsaacSim.sh --ext-folder /workspace/costnav_isaaclab/source/extensions \
              --enable omni.isaac.ros2_bridge
```

**Step 4: Verify ROS2 communication (Nav2 container)**
```bash
# In a new terminal, enter Nav2 container
docker exec -it costnav-nav2 bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash

# Verify topics from Isaac Sim are visible
ros2 topic list

# Expected topics:
# /odom
# /tf
# /tf_static
# /joint_states
# /camera/depth/points
# /scan (if configured)

# Check TF tree
ros2 run tf2_tools view_frames
# This generates frames.pdf showing the TF tree
```

**Step 5: Generate occupancy map (Nav2 container)**
```bash
# Option A: Use pre-generated map
# Maps are shared via volume mount at /workspace/maps

# Option B: Generate new map from Isaac Sim
# In Isaac Sim GUI:
# 1. Go to Tools > Robotics > Occupancy Map
# 2. Configure bounds and parameters
# 3. Click CALCULATE > VISUALIZE IMAGE
# 4. Save as PNG and YAML to /workspace/maps/
```

**Step 6: Launch Nav2 stack (Nav2 container)**
```bash
# Still in Nav2 container
ros2 launch costnav_nav2 coco_navigation.launch.py

# This launches:
# - Map Server
# - AMCL Localizer
# - Planner Server
# - Controller Server
# - Behavior Server
# - Lifecycle Manager
# - RViz2 (if DISPLAY is configured)
```

**Step 7: Visualize in RViz2 (Nav2 container or host)**
```bash
# Option A: From Nav2 container (requires X11 forwarding)
ros2 run rviz2 rviz2 -d /workspace/ros2_ws/src/costnav_nav2/rviz/nav2_default.rviz

# Option B: From host machine (if ROS2 installed locally)
# Set ROS_DOMAIN_ID to match containers
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2 -d ros2_ws/src/costnav_nav2/rviz/nav2_default.rviz
```

**Step 8: Send navigation goal**
```bash
# Method 1: Using RViz2
# - Click "2D Pose Estimate" to set initial pose
# - Click "Nav2 Goal" and click/drag on map to set goal

# Method 2: Using command line
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 10.0, y: 5.0, z: 0.0}, \
           orientation: {w: 1.0}}}"

# Method 3: Using Python script
python /workspace/scripts/nav2/send_navigation_goal.py --x 10.0 --y 5.0 --theta 0.0

# Method 4: Using Nav2 Simple Commander API
python3 << EOF
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy

rclpy.init()
navigator = BasicNavigator()
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 10.0
goal_pose.pose.position.y = 5.0
goal_pose.pose.orientation.w = 1.0
navigator.goToPose(goal_pose)
EOF
```

**Step 9: Monitor navigation (Nav2 container)**
```bash
# Monitor robot pose
ros2 topic echo /odom

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Monitor cost metrics (custom)
ros2 topic echo /cost_metrics
```

**Step 10: Shutdown**
```bash
# Stop containers
docker compose -f docker-compose.yml -f docker-compose.nav2.yml down

# Or stop individual containers
docker stop costnav-isaac-sim costnav-nav2
```

---

### Troubleshooting Multi-Container Setup

**Issue: Topics not visible across containers**
```bash
# Check network connectivity
docker exec costnav-nav2 ping costnav-isaac-sim

# Check ROS_DOMAIN_ID matches
docker exec costnav-isaac-sim printenv | grep ROS_DOMAIN_ID
docker exec costnav-nav2 printenv | grep ROS_DOMAIN_ID

# Check DDS discovery
docker exec costnav-nav2 ros2 daemon stop
docker exec costnav-nav2 ros2 daemon start
docker exec costnav-nav2 ros2 topic list
```

**Issue: High latency between containers**
```bash
# Switch to CycloneDDS (if using FastDDS)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Increase DDS buffer sizes in cyclonedds.xml
# See docker/nav2/cyclonedds.xml
```

**Issue: RViz2 not displaying**
```bash
# Enable X11 forwarding
xhost +local:docker

# Add to docker-compose.yml:
environment:
  - DISPLAY=$DISPLAY
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

**Issue: Map not loading**
```bash
# Verify map files exist
docker exec costnav-nav2 ls -la /workspace/maps/

# Check map_server logs
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/workspace/maps/coco_warehouse.yaml

# Verify map topic
ros2 topic echo /map --once
```

---

## Appendix F: Docker Configuration Examples

### Dockerfile.nav2

```dockerfile
# ROS2 Nav2 Container for CostNav
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install Nav2 and dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    nav2-simple-commander \
    transforms3d \
    numpy

# Create workspace
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

# Copy ROS2 packages
COPY ros2_ws/src /workspace/ros2_ws/src

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Copy DDS configuration
COPY docker/nav2/cyclonedds.xml /config/cyclonedds.xml

# Copy entrypoint script
COPY docker/nav2/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

### docker-compose.nav2.yml

```yaml
version: '3.8'

services:
  nav2:
    build:
      context: .
      dockerfile: Dockerfile.nav2
    image: costnav-nav2:latest
    container_name: costnav-nav2
    hostname: costnav-nav2

    networks:
      - costnav_network

    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - CYCLONEDDS_URI=/config/cyclonedds.xml
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1

    volumes:
      # ROS2 workspace
      - ./ros2_ws:/workspace/ros2_ws
      # Shared maps directory
      - ./maps:/workspace/maps
      # Shared config directory
      - ./config/nav2:/workspace/config
      # Scripts
      - ./scripts:/workspace/scripts
      # X11 for RViz2
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # DDS configuration
      - ./docker/nav2/cyclonedds.xml:/config/cyclonedds.xml:ro

    depends_on:
      - isaac-sim

    stdin_open: true
    tty: true

    command: bash

networks:
  costnav_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
```

### Updated docker-compose.yml (Isaac Sim)

```yaml
version: '3.8'

services:
  isaac-sim:
    build:
      context: .
      dockerfile: Dockerfile
      target: isaac-lab
    image: costnav-isaac-lab:latest
    container_name: costnav-isaac-sim
    hostname: costnav-isaac-sim

    networks:
      - costnav_network

    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - ACCEPT_EULA=Y
      - PRIVACY_CONSENT=Y

    volumes:
      - ./costnav_isaaclab:/workspace/costnav_isaaclab
      - ./maps:/workspace/maps
      - ./config:/workspace/config
      - ./scripts:/workspace/scripts
      - isaac-sim-cache:/root/.nvidia-omniverse/cache
      - isaac-sim-data:/root/.local/share/ov/data

    runtime: nvidia

    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]

    stdin_open: true
    tty: true

    ports:
      - "8211:8211"  # Isaac Sim streaming
      - "8899:8899"  # Isaac Sim web UI

    command: bash

networks:
  costnav_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

volumes:
  isaac-sim-cache:
  isaac-sim-data:
```

### docker/nav2/entrypoint.sh

```bash
#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if built
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Set up DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/config/cyclonedds.xml

# Print environment info
echo "ROS2 Nav2 Container Ready"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"

# Execute command
exec "$@"
```

### docker/nav2/cyclonedds.xml

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>stdout</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
```

### .env (Environment Variables)

```bash
# ROS2 Configuration
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Display for RViz2
DISPLAY=:0

# Isaac Sim Configuration
ACCEPT_EULA=Y
PRIVACY_CONSENT=Y

# Network Configuration
COSTNAV_NETWORK_SUBNET=172.20.0.0/16
```

### scripts/nav2/launch_multi_container.sh

```bash
#!/bin/bash
# Launch both Isaac Sim and Nav2 containers

set -e

echo "Starting CostNav Multi-Container Setup..."

# Build containers if needed
echo "Building containers..."
docker compose -f docker-compose.yml -f docker-compose.nav2.yml build

# Start containers
echo "Starting containers..."
docker compose -f docker-compose.yml -f docker-compose.nav2.yml up -d

# Wait for containers to be ready
echo "Waiting for containers to start..."
sleep 5

# Verify containers are running
echo "Verifying containers..."
docker ps | grep costnav

# Check network connectivity
echo "Testing network connectivity..."
docker exec costnav-nav2 ping -c 3 costnav-isaac-sim

# Verify ROS2 environment in Nav2 container
echo "Verifying ROS2 environment..."
docker exec costnav-nav2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

echo ""
echo "Multi-container setup complete!"
echo ""
echo "Next steps:"
echo "1. Launch Isaac Sim:"
echo "   docker exec -it costnav-isaac-sim bash"
echo "   python scripts/nav2/launch_nav2.py --task=Template-Costnav-Isaaclab-v3-Nav2"
echo ""
echo "2. Launch Nav2 (in another terminal):"
echo "   docker exec -it costnav-nav2 bash"
echo "   ros2 launch costnav_nav2 coco_navigation.launch.py"
echo ""
echo "3. Send navigation goals:"
echo "   docker exec -it costnav-nav2 bash"
echo "   python /workspace/scripts/nav2/send_navigation_goal.py --x 10.0 --y 5.0"
```

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2025-11-21 | CostNav Team | Initial draft |
| 0.2 | 2025-11-21 | CostNav Team | Updated with multi-container architecture and official Isaac Sim Nav2 tutorial references |

---

**End of Document**

