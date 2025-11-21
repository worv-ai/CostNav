# Nav2 (ROS2 Navigation Stack 2) Implementation Plan

**Issue Reference:** [#5 - Support rule-based navigation with nav2](https://github.com/worv-ai/CostNav/issues/5)

**Status:** Planning Phase  
**Target Version:** CostNav v0.2.0  
**Last Updated:** 2025-11-21

---

## Executive Summary

This document outlines the implementation plan for integrating ROS2 Navigation Stack 2 (Nav2) into CostNav to enable rule-based navigation alongside the existing learning-based approaches. This integration will allow for:

- Direct comparison between learning-based and rule-based navigation policies
- Hybrid approaches combining both paradigms
- Industry-standard navigation baselines for benchmarking
- Flexible rule-based constraints and decision-making in dynamic environments

---

## Table of Contents

1. [Background & Motivation](#background--motivation)
2. [Architecture Overview](#architecture-overview)
3. [Implementation Phases](#implementation-phases)
4. [Technical Requirements](#technical-requirements)
5. [Integration Points](#integration-points)
6. [Testing Strategy](#testing-strategy)
7. [Success Metrics](#success-metrics)
8. [Timeline & Milestones](#timeline--milestones)
9. [Risk Assessment](#risk-assessment)
10. [References](#references)

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

## Implementation Phases

### Phase 1: Foundation & ROS2 Bridge (Weeks 1-3)

**Objective:** Establish ROS2 communication bridge between Isaac Sim and Nav2

**Tasks:**

1. **ROS2 Bridge Setup**
   - [ ] Install ROS2 Humble/Iron in Isaac Sim container
   - [ ] Configure ROS2-Isaac Sim bridge using `omni.isaac.ros2_bridge`
   - [ ] Verify TF tree publishing (REP-105 compliance)
   - [ ] Test sensor data publishing (RGB-D, LaserScan, Odometry)

2. **Environment Configuration**
   - [ ] Update Dockerfile to include ROS2 dependencies
   - [ ] Add Nav2 packages to container build
   - [ ] Configure environment variables for ROS2/Isaac Sim coexistence
   - [ ] Update docker-compose.yml with ROS2 networking

3. **URDF/Robot Description**
   - [ ] Generate URDF from COCO robot USD file
   - [ ] Configure robot_state_publisher
   - [ ] Define TF frames (base_link, odom, map, camera, sensors)
   - [ ] Validate robot footprint parameters

**Deliverables:**
- Working ROS2 bridge in Isaac Sim
- COCO robot URDF with proper TF tree
- Documentation: `docs/ros2_bridge_setup.md`

**Success Criteria:**
- `ros2 topic list` shows Isaac Sim sensor topics
- `ros2 run tf2_tools view_frames` generates valid TF tree
- Sensor data visualizes correctly in RViz2

---

### Phase 2: Costmap & Localization (Weeks 4-6)

**Objective:** Configure Nav2 costmap layers and localization system

**Tasks:**

1. **Map Server Configuration**
   - [ ] Convert USD sidewalk map to occupancy grid (map.yaml + map.pgm)
   - [ ] Configure map_server for static map publishing
   - [ ] Implement dynamic map updates from Isaac Sim scene
   - [ ] Test map visualization in RViz2

2. **Costmap 2D Setup**
   - [ ] Configure global costmap (static layer, inflation layer)
   - [ ] Configure local costmap (obstacle layer, voxel layer)
   - [ ] Integrate RGB-D camera data as obstacle source
   - [ ] Tune inflation radius and cost scaling parameters

3. **Localization System**
   - [ ] Configure AMCL for map-based localization
   - [ ] Alternative: Use ground-truth pose from Isaac Sim
   - [ ] Configure robot_localization for sensor fusion
   - [ ] Implement pose initialization from safe positions

4. **Sensor Integration**
   - [ ] Publish RGB-D as PointCloud2 for obstacle detection
   - [ ] Convert depth image to LaserScan (optional)
   - [ ] Configure sensor transforms and timing
   - [ ] Validate obstacle detection in costmap

**Deliverables:**
- Configured costmap parameters (YAML files)
- Map conversion tools/scripts
- Localization configuration
- Documentation: `docs/nav2_costmap_config.md`

**Success Criteria:**
- Costmap visualizes obstacles correctly in RViz2
- Robot localizes accurately on the map
- Dynamic obstacles appear in local costmap

---

### Phase 3: Planners & Controllers (Weeks 7-10)

**Objective:** Integrate and configure Nav2 planners and controllers

**Tasks:**

1. **Planner Server Configuration**
   - [ ] Configure NavFn planner (Dijkstra/A*)
   - [ ] Configure Smac Planner (Hybrid-A*, 2D, Lattice)
   - [ ] Configure Theta* planner
   - [ ] Implement planner selection API
   - [ ] Benchmark planner performance

2. **Controller Server Configuration**
   - [ ] Configure DWB controller (Dynamic Window Approach)
   - [ ] Configure Regulated Pure Pursuit (RPP)
   - [ ] Configure MPPI controller (Model Predictive Path Integral)
   - [ ] Tune controller parameters for COCO robot kinematics
   - [ ] Implement controller switching logic

3. **Smoother Server Configuration**
   - [ ] Configure Constrained Smoother
   - [ ] Configure Savitzky-Golay Smoother
   - [ ] Configure Simple Smoother
   - [ ] Tune smoothing parameters

4. **Behavior Server Configuration**
   - [ ] Configure recovery behaviors (Spin, BackUp, Wait)
   - [ ] Implement custom recovery behaviors if needed
   - [ ] Define behavior timeout parameters

**Deliverables:**
- Planner/controller configuration files
- Performance benchmarking results
- Tuning guidelines document
- Documentation: `docs/nav2_planners_controllers.md`

**Success Criteria:**
- Robot successfully navigates to goal poses
- Controllers respect kinematic constraints
- Recovery behaviors execute on failure

---

### Phase 4: Behavior Trees & Rule System (Weeks 11-14)

**Objective:** Implement custom behavior trees and rule-based navigation logic

**Tasks:**

1. **Behavior Tree Development**
   - [ ] Create base navigation BT (NavigateToPose)
   - [ ] Create waypoint following BT (NavigateThroughPoses)
   - [ ] Implement custom BT nodes for CostNav-specific logic
   - [ ] Add cost-aware decision nodes
   - [ ] Integrate with Groot2 for visualization

2. **Rule-Based Navigation API**
   - [ ] Design rule specification interface (YAML/Python)
   - [ ] Implement rule evaluation engine
   - [ ] Create rule types:
     - [ ] Speed limits based on area/context
     - [ ] Keepout zones (dynamic/static)
     - [ ] Priority zones (delivery areas, crosswalks)
     - [ ] Time-based constraints
     - [ ] Cost-based path selection
   - [ ] Integrate rules with BT decision nodes

3. **Custom BT Plugins**
   - [ ] Cost-aware planner selector node
   - [ ] SLA-aware goal checker node
   - [ ] Energy-efficient controller selector
   - [ ] Maintenance-risk evaluator node

4. **Integration with Existing System**
   - [ ] Create Nav2 task environment (costnav_isaaclab_v3_Nav2)
   - [ ] Implement observation/action wrappers
   - [ ] Maintain compatibility with cost model
   - [ ] Support hybrid RL+Nav2 approaches

**Deliverables:**
- Custom behavior tree XML files
- Rule specification schema and examples
- Custom BT plugin library
- New task environment: `costnav_isaaclab_v3_Nav2`
- Documentation: `docs/nav2_behavior_trees.md`, `docs/rule_based_navigation.md`

**Success Criteria:**
- Custom BTs execute successfully
- Rules correctly influence navigation decisions
- Cost model evaluates Nav2 navigation
- Groot2 visualizes BT execution

---

### Phase 5: Testing & Benchmarking (Weeks 15-17)

**Objective:** Comprehensive testing and performance comparison

**Tasks:**

1. **Unit Testing**
   - [ ] Test ROS2 bridge components
   - [ ] Test costmap layer integration
   - [ ] Test planner/controller plugins
   - [ ] Test custom BT nodes
   - [ ] Test rule evaluation engine

2. **Integration Testing**
   - [ ] End-to-end navigation tests
   - [ ] Multi-waypoint mission tests
   - [ ] Recovery behavior tests
   - [ ] Sensor failure handling tests
   - [ ] Dynamic obstacle avoidance tests

3. **Performance Benchmarking**
   - [ ] Compare Nav2 vs RL-based navigation
   - [ ] Measure planning time, success rate, path quality
   - [ ] Evaluate cost metrics (SLA, energy, maintenance)
   - [ ] Test scalability (multiple robots, large maps)
   - [ ] Profile computational overhead

4. **Scenario Testing**
   - [ ] Sidewalk navigation scenarios
   - [ ] Crosswalk handling
   - [ ] Narrow passage navigation
   - [ ] Dynamic pedestrian avoidance
   - [ ] Delivery mission completion

**Deliverables:**
- Test suite for Nav2 integration
- Benchmark results and comparison report
- Performance optimization recommendations
- Documentation: `docs/nav2_testing_guide.md`, `docs/nav2_benchmarks.md`

**Success Criteria:**
- All tests pass with >95% success rate
- Nav2 baseline established for comparison
- Cost model accurately captures Nav2 performance
- Documentation complete and validated

---

### Phase 6: Documentation & Examples (Weeks 18-19)

**Objective:** Complete documentation and provide usage examples

**Tasks:**

1. **User Documentation**
   - [ ] Installation guide for Nav2 integration
   - [ ] Configuration guide for planners/controllers
   - [ ] Rule specification tutorial
   - [ ] Behavior tree customization guide
   - [ ] Troubleshooting guide

2. **Developer Documentation**
   - [ ] Architecture documentation
   - [ ] API reference for rule system
   - [ ] Custom plugin development guide
   - [ ] Integration points documentation

3. **Examples & Tutorials**
   - [ ] Basic Nav2 navigation example
   - [ ] Rule-based navigation example
   - [ ] Hybrid RL+Nav2 example
   - [ ] Custom behavior tree example
   - [ ] Multi-robot coordination example

4. **Video Demonstrations**
   - [ ] Record navigation demonstrations
   - [ ] Create tutorial videos
   - [ ] Prepare presentation materials

**Deliverables:**
- Complete documentation set
- Example configurations and scripts
- Tutorial notebooks/guides
- Demo videos
- Updated README.md

**Success Criteria:**
- New users can set up Nav2 in <1 hour
- All examples run successfully
- Documentation covers all features
- Community feedback incorporated

---

## Technical Requirements

### Software Dependencies

**ROS2 Packages:**
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

  visualization:
    - ros-humble-rviz2
    - ros-humble-nav2-rviz-plugins
    - groot2  # Behavior tree visualization
```

**Isaac Sim Extensions:**
```yaml
isaac_sim_extensions:
  - omni.isaac.ros2_bridge
  - omni.isaac.sensor
  - omni.isaac.range_sensor
  - omni.isaac.wheeled_robots
  - omni.anim.navigation.core
  - omni.anim.navigation.bundle
```

**Python Dependencies:**
```python
# Add to pyproject.toml [project.optional-dependencies]
nav2 = [
    "rclpy>=3.3.0",
    "nav2-simple-commander>=1.0.0",
    "transforms3d>=0.4.1",
    "pyyaml>=6.0",
]
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

### 1. Isaac Sim ↔ Nav2 Bridge

**Data Flow:**

```
Isaac Sim                          ROS2 Nav2
─────────────────────────────────────────────────────
Robot State        ──────────────► /odom (nav_msgs/Odometry)
                   ──────────────► /tf, /tf_static
RGB-D Camera       ──────────────► /camera/depth/points (PointCloud2)
                   ──────────────► /scan (LaserScan, optional)
Contact Sensors    ──────────────► /bumper (sensor_msgs/Range)
Ground Truth       ──────────────► /ground_truth/pose

Nav2 Commands      ◄────────────── /cmd_vel (geometry_msgs/Twist)
Goal Poses         ◄────────────── /goal_pose (PoseStamped)
Map                ◄────────────── /map (OccupancyGrid)
```

**Implementation:**
- Use `omni.isaac.ros2_bridge` for topic publishing/subscribing
- Implement custom Isaac Sim action graph for ROS2 integration
- Create wrapper nodes for data format conversion

### 2. Cost Model Integration

**Approach:**
- Extend `rl_games_helpers.py` to support Nav2 metrics
- Create `nav2_cost_tracker.py` for cost computation
- Log Nav2-specific metrics (planning time, path length, etc.)
- Maintain unified cost reporting format

**Metrics to Track:**
```python
nav2_metrics = {
    "planning_time": float,  # Time to compute path
    "path_length": float,    # Total path distance
    "smoothness": float,     # Path curvature metric
    "collision_count": int,  # Number of collisions
    "recovery_count": int,   # Recovery behavior invocations
    "goal_reached": bool,    # Success indicator
    "time_to_goal": float,   # Mission completion time
}
```

### 3. Hybrid RL + Nav2 Architecture

**Option A: Sequential**
- Use RL for high-level planning (waypoint selection)
- Use Nav2 for low-level control (waypoint navigation)

**Option B: Parallel**
- Run both systems simultaneously
- Use arbitration logic to select best action
- Learn when to use which approach

**Option C: Hierarchical**
- RL learns to configure Nav2 parameters
- Nav2 executes with learned configuration
- Adaptive tuning based on context

---

## Testing Strategy

### Test Levels

**1. Unit Tests**
- ROS2 bridge message conversion
- Costmap layer updates
- Rule evaluation logic
- BT node execution
- Planner/controller plugins

**2. Integration Tests**
- End-to-end navigation pipeline
- Sensor data → Costmap → Planning → Control
- Recovery behavior triggering
- Multi-waypoint missions
- Cost model computation

**3. System Tests**
- Full navigation scenarios
- Performance benchmarks
- Stress tests (many obstacles, long paths)
- Failure mode testing
- Multi-robot scenarios

### Test Scenarios

**Scenario 1: Basic Navigation**
- Start: Random safe position
- Goal: Random safe position >10m away
- Success: Reach goal within 2x optimal time
- Metrics: Path length, time, collisions

**Scenario 2: Dynamic Obstacles**
- Start: Fixed position
- Goal: Fixed position
- Obstacles: Moving pedestrians
- Success: Reach goal without collision
- Metrics: Avoidance maneuvers, time penalty

**Scenario 3: Narrow Passages**
- Start: Open area
- Goal: Through narrow doorway/passage
- Success: Navigate without collision
- Metrics: Clearance, speed reduction

**Scenario 4: Delivery Mission**
- Start: Depot
- Waypoints: 5 delivery locations
- Goal: Return to depot
- Success: Complete all deliveries within SLA
- Metrics: SLA compliance, energy, profit

**Scenario 5: Recovery Behaviors**
- Start: Random position
- Goal: Random position
- Inject: Localization failure, sensor noise
- Success: Recover and reach goal
- Metrics: Recovery time, success rate

### Continuous Integration

```yaml
# .github/workflows/nav2_tests.yml
name: Nav2 Integration Tests

on: [push, pull_request]

jobs:
  nav2_tests:
    runs-on: ubuntu-22.04
    container: costnav-isaaclab:latest

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Build Nav2 integration
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --packages-select costnav_nav2

      - name: Run unit tests
        run: pytest tests/nav2/

      - name: Run integration tests
        run: python scripts/test_nav2_integration.py

      - name: Generate coverage report
        run: pytest --cov=costnav_nav2 --cov-report=xml

      - name: Upload coverage
        uses: codecov/codecov-action@v3
```

---

## Success Metrics

### Functional Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Navigation Success Rate | >90% | Goals reached / Total attempts |
| Collision-Free Rate | >95% | Collision-free runs / Total runs |
| Recovery Success Rate | >80% | Successful recoveries / Recovery attempts |
| SLA Compliance | >40% | Deliveries on-time / Total deliveries |
| Path Optimality | <1.5x | Actual path / Optimal path length |

### Performance Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Planning Time | <500ms | Average global planning time |
| Control Frequency | >20Hz | Controller update rate |
| Costmap Update Rate | >5Hz | Costmap refresh frequency |
| End-to-End Latency | <100ms | Sensor → Action latency |
| Memory Usage | <4GB | Peak RAM consumption |

### Cost Model Metrics

| Metric | Baseline (RL) | Target (Nav2) | Measurement |
|--------|---------------|---------------|-------------|
| Operating Margin | 46.5% | >40% | (Revenue - Costs) / Revenue |
| Break-Even Time | 0.90 years | <1.2 years | Time to ROI |
| Energy Efficiency | Baseline | >90% of RL | kWh per delivery |
| Maintenance Cost | 33.7% | <40% | % of total costs |
| SLA Compliance | 43.0% | >40% | % on-time deliveries |

### Comparison Metrics (Nav2 vs RL)

```python
comparison_metrics = {
    "success_rate": {
        "nav2": float,
        "rl": float,
        "improvement": float,  # percentage
    },
    "average_time_to_goal": {
        "nav2": float,
        "rl": float,
        "improvement": float,
    },
    "collision_rate": {
        "nav2": float,
        "rl": float,
        "improvement": float,
    },
    "energy_per_delivery": {
        "nav2": float,
        "rl": float,
        "improvement": float,
    },
    "computational_cost": {
        "nav2": float,  # CPU/GPU usage
        "rl": float,
        "improvement": float,
    },
}
```

---

## Timeline & Milestones

### Overall Timeline: 19 Weeks (~4.5 Months)

```
Week 1-3:   Phase 1 - Foundation & ROS2 Bridge
Week 4-6:   Phase 2 - Costmap & Localization
Week 7-10:  Phase 3 - Planners & Controllers
Week 11-14: Phase 4 - Behavior Trees & Rule System
Week 15-17: Phase 5 - Testing & Benchmarking
Week 18-19: Phase 6 - Documentation & Examples
```

### Key Milestones

**M1: ROS2 Bridge Operational (Week 3)**
- ✓ Isaac Sim publishes sensor data to ROS2
- ✓ Nav2 commands control robot in simulation
- ✓ TF tree validated

**M2: Basic Navigation Working (Week 6)**
- ✓ Robot navigates to goal using NavFn + DWB
- ✓ Costmap shows obstacles correctly
- ✓ Localization functional

**M3: Multi-Planner Support (Week 10)**
- ✓ All planners/controllers configured
- ✓ Performance benchmarks completed
- ✓ Parameter tuning documented

**M4: Rule-Based System Complete (Week 14)**
- ✓ Custom behavior trees operational
- ✓ Rule specification API functional
- ✓ Cost model integration complete

**M5: Testing Complete (Week 17)**
- ✓ All test scenarios pass
- ✓ Benchmark comparison published
- ✓ Performance validated

**M6: Documentation & Release (Week 19)**
- ✓ All documentation complete
- ✓ Examples validated
- ✓ Ready for community use

### Gantt Chart

```
Phase 1: Foundation          [████████████]
Phase 2: Costmap             [            ████████████]
Phase 3: Planners            [                        ████████████████]
Phase 4: Behavior Trees      [                                        ████████████████]
Phase 5: Testing             [                                                        ████████████]
Phase 6: Documentation       [                                                                    ████████]
         ─────────────────────────────────────────────────────────────────────────────────────────────────
         Week: 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
```

---

## Risk Assessment

### High-Priority Risks

**Risk 1: ROS2-Isaac Sim Integration Complexity**
- **Probability:** Medium
- **Impact:** High
- **Mitigation:**
  - Start with simple bridge examples
  - Leverage existing Isaac Sim ROS2 tutorials
  - Allocate buffer time in Phase 1
  - Engage NVIDIA support if needed

**Risk 2: Performance Overhead**
- **Probability:** Medium
- **Impact:** Medium
- **Mitigation:**
  - Profile early and often
  - Optimize critical paths
  - Consider async processing
  - Use GPU acceleration where possible

**Risk 3: Costmap Synchronization**
- **Probability:** Medium
- **Impact:** Medium
- **Mitigation:**
  - Use Isaac Sim's built-in occupancy grid
  - Implement efficient update mechanisms
  - Test with various map sizes
  - Consider hierarchical costmaps

**Risk 4: Parameter Tuning Complexity**
- **Probability:** High
- **Impact:** Medium
- **Mitigation:**
  - Start with default Nav2 parameters
  - Use systematic tuning approach
  - Document parameter effects
  - Provide tuning guidelines

### Medium-Priority Risks

**Risk 5: Behavior Tree Complexity**
- **Probability:** Medium
- **Impact:** Low
- **Mitigation:**
  - Start with simple BTs
  - Incrementally add complexity
  - Use Groot2 for visualization
  - Provide BT templates

**Risk 6: Rule System Scalability**
- **Probability:** Low
- **Impact:** Medium
- **Mitigation:**
  - Design efficient rule evaluation
  - Cache rule results
  - Profile rule engine
  - Limit rule complexity

**Risk 7: Multi-Robot Interference**
- **Probability:** Low
- **Impact:** Low
- **Mitigation:**
  - Test single robot first
  - Use ROS2 namespacing
  - Implement coordination layer
  - Document multi-robot setup

### Contingency Plans

**If ROS2 bridge fails:**
- Fallback to custom message passing
- Use shared memory for high-frequency data
- Consider alternative simulation platforms

**If performance is insufficient:**
- Reduce costmap resolution
- Decrease update frequencies
- Simplify planner algorithms
- Use cloud offloading for planning

**If timeline slips:**
- Prioritize core functionality
- Defer advanced features to v0.3.0
- Release incremental versions
- Engage community contributors

---

## References

### Nav2 Documentation
- [Nav2 Official Docs](https://docs.nav2.org/)
- [Nav2 GitHub Repository](https://github.com/ros-planning/navigation2)
- [Nav2 Tutorials](https://docs.nav2.org/tutorials/index.html)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/index.html)

### Isaac Sim Integration
- [Isaac Sim ROS2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/index.html)
- [Isaac Sim Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html)
- [Isaac Sim Multi-Robot Navigation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_multi_navigation.html)

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
costnav_isaaclab/
├── source/costnav_isaaclab/costnav_isaaclab/
│   ├── tasks/manager_based/
│   │   └── costnav_isaaclab_v3_Nav2/          # New Nav2 task
│   │       ├── __init__.py
│   │       ├── nav2_env_cfg.py
│   │       ├── nav2_bridge.py
│   │       ├── nav2_cost_tracker.py
│   │       ├── behavior_trees/
│   │       │   ├── navigate_to_pose.xml
│   │       │   ├── navigate_through_poses.xml
│   │       │   └── cost_aware_navigation.xml
│   │       ├── rules/
│   │       │   ├── rule_engine.py
│   │       │   ├── rule_schema.yaml
│   │       │   └── example_rules.yaml
│   │       └── config/
│   │           ├── nav2_params.yaml
│   │           ├── costmap_common.yaml
│   │           ├── global_costmap.yaml
│   │           ├── local_costmap.yaml
│   │           ├── planner_server.yaml
│   │           ├── controller_server.yaml
│   │           └── behavior_server.yaml
│   └── nav2_plugins/                          # Custom Nav2 plugins
│       ├── cost_aware_planner_selector.cpp
│       ├── sla_goal_checker.cpp
│       └── energy_efficient_controller.cpp
├── scripts/
│   ├── nav2/
│   │   ├── launch_nav2.py
│   │   ├── test_nav2_integration.py
│   │   ├── benchmark_nav2_vs_rl.py
│   │   └── visualize_nav2.py
│   └── tools/
│       ├── convert_usd_to_map.py
│       └── generate_nav2_config.py
└── config/
    └── nav2/
        ├── coco_robot.urdf
        └── robot_description.yaml

docs/
├── nav2_implementation_plan.md                # This file
├── ros2_bridge_setup.md
├── nav2_costmap_config.md
├── nav2_planners_controllers.md
├── nav2_behavior_trees.md
├── rule_based_navigation.md
├── nav2_testing_guide.md
└── nav2_benchmarks.md

tests/
└── nav2/
    ├── test_ros2_bridge.py
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

### Minimal Nav2 Setup (5 Steps)

**Step 1: Build Nav2-enabled container**
```bash
# Update Dockerfile to include ROS2 Humble + Nav2
docker compose --profile isaac-lab build
docker compose --profile isaac-lab up -d
docker exec -it costnav-isaac-lab bash
```

**Step 2: Source ROS2 environment**
```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

**Step 3: Launch Isaac Sim with ROS2 bridge**
```bash
python scripts/nav2/launch_nav2.py --task=Template-Costnav-Isaaclab-v3-Nav2
```

**Step 4: Launch Nav2 stack**
```bash
# In another terminal
ros2 launch costnav_nav2 navigation.launch.py
```

**Step 5: Send navigation goal**
```bash
# Using RViz2
ros2 run rviz2 rviz2 -d config/nav2/costnav_nav2.rviz

# Or using Python API
python scripts/nav2/send_goal.py --x 10.0 --y 5.0
```

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2025-11-21 | CostNav Team | Initial draft |

---

**End of Document**

