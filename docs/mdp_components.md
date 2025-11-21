# MDP Components

This document provides detailed explanations of the Markov Decision Process (MDP) components that define the CostNav navigation task.

## Overview

The MDP is defined by:
- **State Space (Observations)**: What the agent perceives
- **Action Space**: What the agent can do
- **Reward Function**: What the agent optimizes for
- **Transition Dynamics**: How actions affect the state (handled by physics)
- **Termination Conditions**: When episodes end

## Observations

The observation space combines vector observations and visual observations.

### Vector Observations

#### 1. Pose Command (2D)
**Function**: `pose_command_2d` in `mdp/observations.py`

```python
def pose_command_2d(env: ManagerBasedEnv, command_name: str) -> torch.Tensor:
    """2D goal position in robot's base frame."""
```

- **Shape**: `(num_envs, 2)`
- **Content**: `[x, y]` position of goal relative to robot
- **Normalization**: Divided by 5.0 (assumes goals within 5m)
- **Purpose**: Tells the robot where to go

#### 2. Base Linear Velocity
**Source**: Isaac Lab's built-in `base_lin_vel` observation

- **Shape**: `(num_envs, 3)`
- **Content**: `[vx, vy, vz]` in robot's base frame
- **Purpose**: Robot's current linear motion

#### 3. Base Angular Velocity
**Source**: Isaac Lab's built-in `base_ang_vel` observation

- **Shape**: `(num_envs, 3)`
- **Content**: `[wx, wy, wz]` in robot's base frame
- **Purpose**: Robot's current rotational motion

### Visual Observations

#### RGB-D Images
**Function**: `rgbd_processed` in `mdp/observations.py`

```python
def rgbd_processed(
    env: ManagerBasedEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("tiled_camera"),
    convert_perspective_to_orthogonal: bool = False,
    normalize: bool = True,
    flatten: bool = False,
) -> torch.Tensor:
```

- **Shape**: `(num_envs, 4, height, width)` or `(num_envs, 4*height*width)` if flattened
- **Channels**: `[R, G, B, D]`
- **RGB Normalization**: Divided by 255.0 → [0, 1]
- **Depth Normalization**: Clamped to [0, 20m], divided by 20.0 → [0, 1]
- **Special Handling**: 
  - Infinity values in depth set to 0
  - NaN values replaced with 0
  - Ensures numerical stability

**Purpose**: Visual perception of environment (obstacles, terrain, etc.)

## Actions

### Action Space: Restricted Car Action

**Class**: `RestrictedCarAction` in `coco_robot_cfg.py`

The action space is 2-dimensional:

```python
action = [velocity, steering_angle]
```

#### Action Processing

1. **Input**: Raw actions from policy
   - `velocity`: Forward velocity (m/s)
   - `steering_angle`: Front axle steering angle (radians)

2. **Clamping**:
   - `velocity`: [0, 4.0] m/s (forward only)
   - `steering_angle`: [-40°, +40°] (±0.698 radians)

3. **Conversion to Joint Commands**:
   - **Wheel velocities**: Computed using Ackermann steering geometry
   - **Axle position**: Steering angle applied to front axle joint

#### Ackermann Steering Geometry

```python
# Given velocity v and steering angle α
wheel_base = 1.5  # Distance between front and rear axles
track_width = 1.8  # Distance between left and right wheels
radius_rear = 0.3  # Wheel radius

# Turning radius
R = wheel_base / tan(α)

# Individual wheel angles (for proper Ackermann steering)
left_wheel_angle = arctan(wheel_base / (R - 0.5 * track_width))
right_wheel_angle = arctan(wheel_base / (R + 0.5 * track_width))

# Wheel velocities (all wheels same speed for simplicity)
wheel_velocity = v / radius_rear
```

#### Action Interval

Actions are applied every 4 simulation steps (`ACTION_INTERVAL = 4`) to:
- Reduce computational cost
- Provide more realistic control frequency
- Allow physics to settle between commands

## Commands

### Safe Position Pose 2D Command

**Class**: `SafePositionPose2dCommand` in `mdp/commands.py`

Generates navigation goals from pre-validated safe positions.

#### Safe Position Validation

Safe positions are generated offline using raycasting:
- Upward raycasts check for overhead obstacles (buildings, overhangs)
- Positions inside buildings or under obstacles are filtered out
- Validated positions stored in `safe_positions_auto_generated.py`

#### Goal Sampling

```python
def _resample(self, env_ids: Sequence[int]):
    # Sample random indices from safe positions
    indices = torch.randint(0, num_safe_positions, (num_resets,))
    sampled_positions = self.safe_positions[indices]
    
    # Set goal position
    self.pos_command_w[env_ids] = sampled_positions
    
    # Set heading (two modes)
    if self.cfg.simple_heading:
        # Point towards goal
        target_vec = goal - robot_pos
        heading = atan2(target_vec.y, target_vec.x)
    else:
        # Random heading
        heading = uniform(-π, π)
```

#### Command Update

Every step, the command is transformed to the robot's base frame:

```python
def _update_command(self):
    # Transform goal from world frame to base frame
    target_vec = pos_command_w - robot_pos_w
    pos_command_b = quat_apply_inverse(robot_quat, target_vec)
    
    # Transform heading
    heading_command_b = wrap_to_pi(heading_w - robot_heading_w)
```

This ensures the observation is always relative to the robot's current pose.

## Rewards

The reward function is a weighted sum of multiple components.

### 1. Arrival Reward
**Weight**: +20,000

```python
arrived_reward = is_terminated_term(term_keys="arrive") * 20000.0
```

Large positive reward for successfully reaching the goal.

### 2. Collision Penalty
**Weight**: -200

```python
collision_penalty = is_terminated_term(term_keys="collision") * -200.0
```

Penalty for colliding with obstacles.

### 3. Position Command Error (Tanh)
**Weight**: +1.0

```python
def position_command_error_tanh(env, std: float, command_name: str):
    distance = norm(goal_position_base_frame)
    return (1 - tanh(distance / std))
```

- Smooth reward that decreases with distance to goal
- `std` parameter controls the falloff rate
- Returns values in [0, 1]

### 4. Heading Command Error
**Weight**: -0.5

```python
def heading_command_error_abs(env, command_name: str):
    heading_error = command[:, 3]  # Heading in base frame
    return abs(heading_error)
```

Penalty for not facing the goal direction.

### 5. Distance Progress Reward
**Weight**: +100.0

```python
def distance_to_goal_progress(env, command_name: str, slack_penalty: float):
    progress = previous_distance - current_distance
    return progress - slack_penalty
```

- Rewards reducing distance to goal
- Penalizes increasing distance
- `slack_penalty` prevents reward hacking

### 6. Moving Towards Goal
**Weight**: +1.0

```python
def moving_towards_goal_reward(env, command_name: str):
    vel_direction = target_pos / distance
    movement_reward = (velocity * vel_direction).sum()
    return movement_reward * (episode_length >= 10)
```

- Rewards velocity component towards goal
- Only active after 10 steps (avoids instability at start)

## Terminations

### 1. Arrive (Success)
**Function**: `arrive` in `mdp/terminations.py`

```python
def arrive(env, threshold: float, command_name: str):
    distance = norm(goal_position_base_frame[:, :2])
    return distance <= threshold
```

- **Threshold**: 0.5 meters
- Terminates episode successfully when robot is within threshold of goal

### 2. Collision (Failure)
**Function**: Isaac Lab's `illegal_contact`

```python
collision = illegal_contact(
    sensor_cfg=SceneEntityCfg("contact_forces", body_names="body_link"),
    threshold=1.0,
)
```

- Monitors contact forces on robot body
- Terminates if contact force exceeds 1.0 N
- Indicates collision with obstacles

### 3. Time Out
**Function**: Isaac Lab's `time_out`

```python
time_out = time_out(time_out=True)
```

- Episode length limit (configured in environment)
- Prevents infinite episodes

## Events

### Reset Event
**Function**: `reset_root_state_from_safe_positions` in `mdp/events.py`

```python
def reset_root_state_from_safe_positions(env, safe_positions, velocity_range):
    # Sample random safe position
    position = random_choice(safe_positions)
    
    # Random yaw orientation
    yaw = uniform(-π, π)
    
    # Zero velocity
    velocity = [0, 0, 0]
    angular_velocity = [0, 0, 0]
    
    # Set robot state
    env.scene["robot"].write_root_state_to_sim(position, orientation, velocity, angular_velocity)
```

Ensures robot starts in valid positions at episode reset.

## Summary

The MDP components work together to create a navigation task where:
- The robot observes its goal and surroundings
- It controls velocity and steering
- It's rewarded for reaching goals efficiently
- It's penalized for collisions
- Episodes end on success, failure, or timeout

This design enables learning cost-effective navigation policies that balance speed, safety, and goal achievement.

