# COCO Robot Integration Summary

## Overview
This document summarizes the integration of the COCO robot from urban-sim into the costnav_isaaclab_v2_nav task for navigation learning on a custom map.

## Changes Made

### 1. New Files Created

#### `coco_robot_cfg.py`
- **Purpose**: Contains COCO robot configuration and action space definitions
- **Key Components**:
  - `COCO_CFG`: ArticulationCfg for the COCO robot with:
    - USD path: `assets/robots/coco_one/coco_one.usd`
    - Actuators: wheels (DelayedPDActuatorCfg), axle (DCMotorCfg), shock (ImplicitActuatorCfg)
    - Initial state: position (0, 0, 0.3), zero joint positions/velocities
  - `ClassicalCarAction`: Velocity-based action term (linear velocity + steering angle)
  - `ClassicalCarWaypointAction`: Waypoint-based action term (local dx, dy)
  - `COCOVelocityActionsCfg`: Configuration for velocity-based control
  - `COCOWaypointActionsCfg`: Configuration for waypoint-based control

### 2. Modified Files

#### `costnav_isaaclab_v2_env_cfg.py`
Major changes to integrate COCO robot and navigation functionality:

**Imports Added**:
- `isaaclab_tasks.manager_based.locomotion.velocity.mdp as loc_mdp`
- `isaaclab_tasks.manager_based.navigation.mdp as nav_mdp`
- `ContactSensorCfg, TiledCameraCfg, CameraCfg` from isaaclab.sensors
- `COCO_CFG, COCOVelocityActionsCfg` from coco_robot_cfg

**Scene Configuration (`CostnavIsaaclabV2SceneCfg`)**:
- Replaced CartPole robot with COCO robot
- Added contact sensor: `ContactSensorCfg` for collision detection
- Added camera sensor: `TiledCameraCfg` for visual observations
  - Resolution: 240x135 (1080/8 x 1920/8)
  - Data types: RGB + depth (distance_to_camera)
  - Update period: 0.1s
  - Mounted on: `{ENV_REGEX_NS}/Robot/base_link/front_cam`
- Kept custom map: `omniverse://10.50.2.21/Users/worv/map/Street_road.usd`

**Commands Configuration (`CommandsCfg`)**:
- Added `pose_command`: UniformPose2dCommandCfg
  - Generates 2D navigation goals
  - Position range: (7.0, 10.0) for both x and y
  - Heading range: (-π, π)
  - Resampling time: 30s

**Actions Configuration (`ActionsCfg`)**:
- Replaced joint effort actions with COCO velocity actions
- Action space: [linear_velocity, steering_angle]

**Observations Configuration (`ObservationsCfg`)**:
- **Policy Group**:
  - `pose_command`: Navigation goal in robot frame (normalized)
- **Sensor Group**:
  - `rgb`: RGBD processed images (4 channels: RGB + depth)

**Events Configuration (`EventCfg`)**:
- Replaced cart/pole reset with base position reset
- Reset position: (0.3, 0.3) with zero yaw
- Zero initial velocities

**Rewards Configuration (`RewardsCfg`)**:
- `arrived_reward`: +2000 for reaching goal
- `collision_penalty`: -200 for collisions
- `position_tracking`: Coarse position tracking (std=5.0, weight=10.0)
- `position_tracking_fine`: Fine position tracking (std=1.0, weight=50.0)
- `moving_towards_goal`: Reward for progress (weight=20.0)
- `target_vel_rew`: Reward for velocity alignment (weight=10.0)

**Terminations Configuration (`TerminationsCfg`)**:
- `time_out`: Episode timeout (30s)
- `collision`: Contact with obstacles (body_link, threshold=1.0)
- `arrive`: Reached goal (threshold=1.0m)

**Environment Configuration (`CostnavIsaaclabV2EnvCfg`)**:
- Decimation: 10 (50 Hz control frequency)
- Episode length: 30 seconds
- Simulation dt: 0.005s (200 Hz physics)
- Render interval: 4 (50 Hz rendering)
- Disabled contact processing for performance
- Camera update period: 0.05s (decimation * dt)

## Key Differences from Urban-Sim

### Similarities (Preserved from Urban-Sim)
1. **Robot Configuration**: Identical COCO robot setup with same actuators and parameters
2. **Action Space**: Same velocity-based control (linear velocity + steering angle)
3. **Reward Structure**: Similar navigation rewards (arrival, collision, position tracking)
4. **Termination Conditions**: Same termination logic (timeout, collision, arrival)
5. **Camera Configuration**: Same camera setup and RGBD processing

### Differences (Adapted for Custom Map)
1. **Map/Scene**:
   - Urban-sim: Procedurally generated urban environments
   - V2_nav: Custom static map from Omniverse server
2. **Goal Sampling**:
   - Urban-sim: Larger range (15-30m)
   - V2_nav: Smaller range (7-10m) suitable for custom map
3. **Number of Environments**:
   - Urban-sim: 256 parallel environments
   - V2_nav: 64 parallel environments (reduced for complex map)
4. **Terrain/Obstacles**:
   - Urban-sim: Dynamic pedestrians and procedural obstacles
   - V2_nav: Static custom map geometry

## Robot Specifications

### COCO Robot Details
- **Type**: Ackermann steering vehicle (car-like robot)
- **Joints**:
  - 4 wheel joints (front_left, front_right, rear_left, rear_right)
  - 1 front axle joint (steering)
  - Shock joints (passive suspension)
- **Control**:
  - Wheels: Velocity control with PD actuators
  - Steering: Position control with DC motor
  - Max wheel velocity: 4.0 m/s
  - Max steering angle: ±40°
- **Dimensions**:
  - Wheelbase: 1.5m
  - Track width: 1.8m
  - Rear wheel radius: 0.3m

### Action Space
- **Dimension**: 2
- **Components**:
  1. Linear velocity [0, 4.0] m/s
  2. Steering angle [-40°, +40°]
- **Control Frequency**: 50 Hz (decimation=10, dt=0.005s)
- **Action Interval**: Every 4 steps (internal to action term)

### Observation Space
- **Policy Observations**:
  - Pose command (2D): [x, y] in robot frame (normalized)
- **Sensor Observations**:
  - RGBD image: 4 channels (RGB + depth), 240x135 resolution
  - Depth range: [0, 20m], normalized

## Environment Registration
- **Gym ID**: `Template-Costnav-Isaaclab-v2-Nav`
- **Entry Point**: `isaaclab.envs:ManagerBasedRLEnv`
- **Config**: `costnav_isaaclab_v2_env_cfg:CostnavIsaaclabV2EnvCfg`

## Usage

### Training
```bash
# Using RSL-RL
python scripts/rsl_rl/train.py --task Template-Costnav-Isaaclab-v2-Nav

# Using RL-Games
python scripts/rl_games/train.py --task Template-Costnav-Isaaclab-v2-Nav
```

### Testing
```bash
# Play trained policy
python scripts/rsl_rl/play.py --task Template-Costnav-Isaaclab-v2-Nav --checkpoint <path_to_checkpoint>
```

## Important Notes

### Asset Requirements
1. **COCO Robot USD**: The robot USD file must be available at `assets/robots/coco_one/coco_one.usd`
   - This path is relative to the working directory when running the simulation
   - Ensure the COCO robot assets from urban-sim are accessible

2. **Custom Map**: The custom map must be accessible at:
   - `omniverse://10.50.2.21/Users/worv/map/Street_road.usd`
   - Verify Omniverse server connection and permissions

### Camera Configuration
- The camera is mounted on `base_link/front_cam`
- Ensure the COCO robot USD has this camera prim defined
- If not, the camera path may need adjustment based on actual robot structure

### Collision Detection
- Contact sensor monitors `body_link` for collisions
- Verify this body name exists in the COCO robot USD
- Adjust `body_names` in termination config if needed

### Performance Considerations
- 64 parallel environments with complex map and camera rendering
- GPU memory requirements may be high
- Adjust `num_envs` if running into memory issues
- PhysX buffers are pre-configured for complex collision geometry

## Next Steps

1. **Verify Asset Paths**:
   - Confirm COCO robot USD is accessible
   - Test custom map loading

2. **Test Environment**:
   - Run a simple test to verify environment creation
   - Check for any runtime errors

3. **Tune Hyperparameters**:
   - Adjust reward weights based on training performance
   - Modify goal sampling range if needed
   - Tune episode length for your specific task

4. **Training**:
   - Start with small number of environments for debugging
   - Scale up once stable
   - Monitor collision rate and arrival rate

## Troubleshooting

### Common Issues
1. **Asset Not Found**: Ensure COCO robot USD path is correct
2. **Camera Errors**: Verify camera prim path in robot USD
3. **Collision Detection**: Check body_link name in robot USD
4. **Memory Issues**: Reduce num_envs or image resolution
5. **Slow Performance**: Disable camera rendering during initial testing

### Debug Mode
To test without camera:
- Comment out camera sensor in SceneCfg
- Comment out sensor observation group in ObservationsCfg
- This will run faster for initial debugging

