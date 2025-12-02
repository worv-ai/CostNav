# RMP Robot Extension

## Overview

The `omni.isaac.rmp_robot` extension provides an OmniGraph node for Ackermann-style robot motion control in NVIDIA Isaac Sim. It implements a Riemannian Motion Policy (RMP) controller that converts velocity commands into wheel velocities and steering angles.

## Installation

The extension is located at `extensions/omni.isaac.rmp_robot` and can be loaded in Isaac Sim through the Extension Manager or by adding the path to your extension search paths.

## RmpController Node

### Description

The `RmpController` node takes linear and angular velocity commands and computes:

- **Wheel velocities** for all four wheels (front-left, front-right, back-left, back-right)
- **Steering angle** for the front wheel bar

### Node Type

```
OmniIsaacRmp_robotExtension.RmpController
```

### Inputs

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `execIn` | execution | - | Execution trigger input |
| `linearVelocity` | double | 0.0 | Target linear velocity (m/s) |
| `angularVelocity` | double | 0.0 | Target angular/rotation velocity (rad/s) |
| `dt` | double | 0.0 | Delta time between updates (seconds) |
| `wheelRadius` | double | 0.11 | Radius of the wheels (meters) |
| `wheelBase` | double | 0.456 | Distance between front and rear axles (meters) |
| `trackWidth` | double | 0.545 | Distance between left and right rear wheels (meters) |
| `maxLinearSpeed` | double | 2.5 | Maximum linear speed (m/s), 0.0 = not set |
| `maxAngularSpeed` | double | 1.8 | Maximum angular speed (rad/s), 0.0 = not set |
| `maxAcceleration` | double | 2.8 | Maximum forward/reverse acceleration (m/s²) |
| `maxDeceleration` | double | 2.8 | Maximum braking deceleration (m/s²) |
| `maxWheelRotation` | double | 0.349 | Maximum steering angle (radians, ~20°) |
| `maxWheelRotationVelocity` | double | 1.257 | Maximum steering angular velocity (rad/s) |

### Outputs

| Parameter | Type | Description |
|-----------|------|-------------|
| `velocityCommand` | double[4] | Wheel angular velocities [FL, FR, BL, BR] (rad/s) |
| `frontWheelAngle` | double | Computed steering angle (radians) |

## Algorithm

### Velocity Control

1. **Speed Limiting**: Target linear velocity is clamped to `[-maxLinearSpeed, maxLinearSpeed]`
2. **Acceleration Ramping**: Velocity changes are limited by `maxAcceleration` or `maxDeceleration` per timestep
3. **Direction Handling**: Acceleration is used when speeding up; deceleration when slowing down or reversing

### Steering Control

1. **Turning Radius Calculation**: Computed from linear/angular velocity ratio
2. **Minimum Radius Enforcement**: Based on `wheelBase` and `maxWheelRotation`
3. **Angle Computation**: Uses Ackermann geometry: `θ = atan(1 / sqrt(R²/L² - 0.25))`
4. **Rate Limiting**: Steering angle changes limited by `maxWheelRotationVelocity`

### Wheel Velocity Distribution

For turning (non-zero steering angle):

```
R_front = wheelBase / sin(θ)
R_rear  = wheelBase / tan(θ)
ω = linearVelocity / turningRadius

v_FL = ω × (R_front - trackWidth/2) / wheelRadius
v_FR = ω × (R_front + trackWidth/2) / wheelRadius  
v_BL = ω × (R_rear - trackWidth/2) / wheelRadius
v_BR = ω × (R_rear + trackWidth/2) / wheelRadius
```

For straight driving: all wheels receive the same velocity.

## Usage Example

See `rmp_drive.usda` for a complete OmniGraph setup that:

1. Subscribes to ROS2 Twist messages
2. Processes velocity through the RmpController
3. Sends commands to articulation controllers

### Basic Graph Structure

```
ROS2 Twist → RmpController → ArticulationController (wheels)
                          → ArticulationController (steering)
```

## File Structure

```
extensions/omni.isaac.rmp_robot/
├── config/
│   └── extension.toml          # Extension configuration
├── docs/
│   └── README.md               # Basic extension info
└── omni/isaac/rmp_robot/
    ├── __init__.py             # Extension entry point
    ├── nodes/
    │   ├── OgnRmpController.ogn  # Node interface definition
    │   └── OgnRmpController.py   # Node implementation
    └── ogn/
        ├── OgnRmpControllerDatabase.py  # Auto-generated database
        └── tests/                        # Unit tests
```

## Dependencies

- `omni.kit.test`
- `omni.graph`
- `omni.isaac.core_nodes` (for `BaseResetNode`)

