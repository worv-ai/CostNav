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

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2025-11-21 | CostNav Team | Initial draft |
| 0.2 | 2025-11-21 | CostNav Team | Updated with multi-container architecture and official Isaac Sim Nav2 tutorial references |

---

**End of Document**

