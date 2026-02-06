# COCO Robot Configuration

This document explains the COCO delivery robot configuration, including its physical properties, actuators, sensors, and control system.

## Overview

The COCO robot is a four-wheeled sidewalk delivery robot with Ackermann steering. It's designed for autonomous navigation in urban environments, carrying payloads for last-mile delivery.

## Physical Configuration

### Robot Structure

The robot is defined in `coco_robot_cfg.py` using Isaac Lab's `ArticulationCfg`:

```python
COCO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="omniverse://localhost/Users/worv/coco_one_fix_prim.usd",
        activate_contact_sensors=True,
    ),
    ...
)
```

### Key Dimensions

- **Wheelbase**: 1.5 m (distance between front and rear axles)
- **Track Width**: 1.8 m (distance between left and right wheels)
- **Wheel Radius**: 0.3 m
- **Mass**: ~50 kg (estimated, used for energy calculations)

### Rigid Body Properties

```python
rigid_props=sim_utils.RigidBodyPropertiesCfg(
    disable_gravity=False,              # Robot affected by gravity
    retain_accelerations=False,         # Don't store acceleration history
    linear_damping=0.0,                 # No linear damping
    angular_damping=0.0,                # No angular damping
    max_linear_velocity=1000.0,         # High limit (not restrictive)
    max_angular_velocity=1000.0,        # High limit (not restrictive)
    max_depenetration_velocity=1.0,     # Collision resolution speed
)
```

### Articulation Properties

```python
articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    enabled_self_collisions=False,      # Disable self-collision
    solver_position_iteration_count=4,  # Physics solver iterations
    solver_velocity_iteration_count=0,  # Velocity solver iterations
    sleep_threshold=0.005,              # Sleep when velocity < threshold
)
```

## Joints and Actuators

The robot has three types of joints, each with different actuator configurations.

### 1. Wheel Joints (4 wheels)

**Joint Names**: 
- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

**Actuator**: `DelayedPDActuatorCfg`

```python
"wheels": DelayedPDActuatorCfg(
    joint_names_expr=[".*wheel_joint"],
    velocity_limit=100.0,               # Max wheel velocity (rad/s)
    min_delay=0,                        # Minimum actuation delay (steps)
    max_delay=4,                        # Maximum actuation delay (steps)
    stiffness={".*_wheel_joint": 0.0},  # No position control
    damping={".*_wheel_joint": 0.3},    # Velocity damping
    friction={".*_wheel_joint": 0.0},   # No friction
    armature={".*_wheel_joint": 0.0},   # No armature inertia
)
```

**Purpose**: 
- Velocity-controlled wheels
- Delayed actuation simulates realistic motor response
- Damping provides stability

### 2. Axle Joint (Steering)

**Joint Name**: `base_to_front_axle_joint`

**Actuator**: `DCMotorCfg`

```python
"axle": DCMotorCfg(
    joint_names_expr=["base_to_front_axle_joint"],
    saturation_effort=64.0,             # Max torque (Nm)
    effort_limit=64.0,                  # Effort limit (Nm)
    velocity_limit=20.0,                # Max steering velocity (rad/s)
    stiffness=25.0,                     # Position control stiffness
    damping=0.5,                        # Damping coefficient
    friction=0.0,                       # No friction
)
```

**Purpose**:
- Position-controlled steering
- DC motor model provides realistic torque limits
- Stiffness and damping create stable steering behavior

### 3. Shock Joints (Suspension)

**Joint Names**: `.*shock_joint` (pattern matches all shock joints)

**Actuator**: `ImplicitActuatorCfg`

```python
"shock": ImplicitActuatorCfg(
    joint_names_expr=[".*shock_joint"],
    stiffness=0.0,                      # No spring force
    damping=0.0,                        # No damping
)
```

**Purpose**:
- Passive suspension joints
- Allow vertical motion for terrain following
- No active control

## Initial State

```python
init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.3),                # Initial position (x, y, z)
    joint_pos={
        ".*wheel_joint*": 0.0,          # Wheels at zero angle
        "base_to_front_axle_joint": 0.0, # Steering centered
    },
    joint_vel={
        ".*wheel_joint": 0.0,           # Wheels stationary
        "base_to_front_axle_joint": 0.0, # Steering stationary
    },
)
```

## Action Space

### RestrictedCarAction

The `RestrictedCarAction` class converts high-level commands to low-level joint commands.

#### Input Actions

```python
action = [velocity, steering_angle]
```

- **velocity**: Desired forward velocity (m/s), range [0, 4.0]
- **steering_angle**: Desired front axle angle (radians), range [-0.698, 0.698] (±40°)

#### Action Processing Pipeline

1. **Clamp Actions**:
```python
velocity = clamp(action[0], 0.0, 4.0)
steering_angle = clamp(action[1], -0.698, 0.698)
```

2. **Compute Turning Radius**:
```python
R = wheelbase / tan(steering_angle)
```

3. **Compute Individual Wheel Angles** (Ackermann geometry):
```python
left_wheel_angle = arctan(wheelbase / (R - 0.5 * track_width))
right_wheel_angle = arctan(wheelbase / (R + 0.5 * track_width))
```

4. **Compute Wheel Velocities**:
```python
wheel_velocity = velocity / wheel_radius
```

5. **Apply to Actuators**:
```python
# Steering (average of left and right wheel angles)
steering_action.process_actions((left_wheel_angle + right_wheel_angle) / 2)

# All wheels same velocity (simplified model)
acceleration_action.process_actions([wheel_velocity] * 4)
```

#### Action Interval

Actions are applied every 4 simulation steps:

```python
if self._counter % ACTION_INTERVAL == 0:
    # Process and apply actions
    ...
self._counter += 1
```

This provides:
- More realistic control frequency (~10 Hz with 0.005s timestep)
- Reduced computational cost
- Smoother motion

## Sensors

### 1. Contact Sensors

```python
contact_forces = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*",
    history_length=3,
    track_air_time=True,
)
```

**Purpose**:
- Detect collisions with environment
- Monitor contact forces on all robot bodies
- Used for collision termination condition

### 2. RGB-D Camera

```python
tiled_camera = TiledCameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link/front_cam",
    offset=TiledCameraCfg.OffsetCfg(
        pos=(0.510, 0.0, 0.015),
        rot=(0.5, -0.5, 0.5, -0.5),
        convention="ros",
    ),
    data_types=["rgb", "distance_to_camera"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0,
        focus_distance=400.0,
        horizontal_aperture=20.955,
        clipping_range=(0.1, 1.0e5),
    ),
    width=80,
    height=80,
)
```

**Specifications**:
- **Resolution**: 80x80 pixels
- **Field of View**: Determined by focal length and aperture
- **Mounting**: Front of robot, facing forward
- **Outputs**: RGB image + depth map

**Purpose**:
- Visual perception of environment
- Obstacle detection
- Terrain understanding

## Control Architecture

```
Policy Network
      ↓
[velocity, steering_angle]
      ↓
RestrictedCarAction
      ↓
┌─────────────┬──────────────┐
│             │              │
Steering      Wheel          Wheel
Action        Velocities     Velocities
│             │              │
↓             ↓              ↓
Axle Joint    Front Wheels   Rear Wheels
(Position)    (Velocity)     (Velocity)
      ↓
Physics Simulation
      ↓
Robot Motion
```

## Energy Model

Energy consumption is estimated using a simple physics-based model:

```python
def compute_navigation_energy_step(env):
    # Get robot mass
    mass = 50.0  # kg (estimated)
    
    # Get planar velocity
    speed = norm(robot.root_lin_vel[:, :2])
    
    # Power = m * g * v (simplified model)
    power = mass * 9.81 * speed
    
    return {"power": power, "speed": speed, "mass": mass}
```

This provides a proxy for:
- Battery consumption
- Operational costs
- Energy efficiency metrics

## Comparison with Real COCO Robot

The simulated COCO robot approximates real delivery robots like:
- **Starship Technologies** delivery robots
- **Kiwibot** delivery robots
- **Amazon Scout**

Key similarities:
- Four-wheeled design
- Ackermann steering
- Sidewalk-scale dimensions
- Camera-based perception

Simplifications:
- Simplified suspension model
- Uniform wheel velocities (no differential)
- Simplified energy model
- No payload dynamics

These simplifications enable faster simulation while maintaining realistic navigation behavior.

