# Architecture Overview

This document provides a comprehensive overview of the CostNav codebase architecture, explaining how different components work together to create a cost-driven navigation benchmark for sidewalk robots.

## High-Level Architecture

CostNav is built on top of NVIDIA Isaac Sim and Isaac Lab, providing a simulation environment for evaluating navigation policies with business-oriented metrics. The architecture consists of several key layers:

```
┌─────────────────────────────────────────────────────────┐
│                   Training Scripts                       │
│         (RL-Games, RSL-RL, SB3, SKRL)                   │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              Environment Configurations                  │
│    (v0: CartPole, v1: CustomMap, v2: NavRL)            │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                  MDP Components                          │
│  (Observations, Actions, Rewards, Terminations,         │
│   Commands, Events)                                      │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              Isaac Lab Framework                         │
│  (ManagerBasedRLEnv, Scene, Sensors, Assets)           │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                  Isaac Sim                               │
│         (Physics, Rendering, USD)                        │
└─────────────────────────────────────────────────────────┘
```

## Directory Structure

### Core Source Code

```
costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/
├── __init__.py                    # Package initialization, environment registration
├── compat.py                      # Compatibility layer for Gymnasium and Isaac Lab
├── rl_games_helpers.py           # Helper functions for RL-Games integration
├── ui_extension_example.py       # UI extension example
└── tasks/                        # Task implementations
    └── manager_based/            # Manager-based environment tasks
        ├── costnav_isaaclab_v0/  # Version 0: CartPole baseline
        ├── costnav_isaaclab_v1_CustomMap/  # Version 1: Custom map navigation
        └── costnav_isaaclab_v2_NavRL/      # Version 2: Full navigation with RL
            ├── __init__.py
            ├── costnav_isaaclab_env_cfg.py  # Main environment configuration
            ├── coco_robot_cfg.py            # COCO robot configuration
            ├── safe_positions_auto_generated.py  # Pre-validated spawn positions
            └── mdp/                         # MDP component implementations
                ├── __init__.py
                ├── commands.py              # Command generators
                ├── observations.py          # Observation functions
                ├── rewards.py               # Reward functions
                ├── terminations.py          # Termination conditions
                └── events.py                # Event handlers
```

### Scripts

```
costnav_isaaclab/scripts/
├── list_envs.py              # List all registered environments
├── test_controller.py        # Test deterministic controller
├── test_v2_rewards.py        # Test reward functions
├── zero_agent.py             # Zero action baseline
├── random_agent.py           # Random action baseline
└── rl_games/                 # RL-Games training scripts
    ├── train.py              # Training script
    ├── play.py               # Inference/visualization script
    └── evaluate.py           # Evaluation script
```

## Key Components

### 1. Environment Configuration (`costnav_isaaclab_env_cfg.py`)

The environment configuration defines the complete MDP specification:

- **Scene Configuration**: Defines all assets in the simulation (robot, map, sensors)
- **Observation Configuration**: Specifies what the agent observes
- **Action Configuration**: Defines the action space
- **Command Configuration**: Goal generation and command management
- **Reward Configuration**: Reward function components and weights
- **Termination Configuration**: Success and failure conditions
- **Event Configuration**: Reset and initialization logic

### 2. MDP Components

#### Commands (`mdp/commands.py`)
- **SafePositionPose2dCommand**: Generates navigation goals from pre-validated safe positions
- Ensures goals are not inside buildings or obstacles
- Supports both simple heading (pointing towards goal) and random heading

#### Observations (`mdp/observations.py`)
- **pose_command_2d**: 2D goal position in robot's base frame
- **rgbd_processed**: RGB-D camera images (normalized and processed)
- **base_lin_vel**: Robot's linear velocity
- **base_ang_vel**: Robot's angular velocity

#### Rewards (`mdp/rewards.py`)
- **position_command_error_tanh**: Reward for being close to goal
- **heading_command_error_abs**: Penalty for heading error
- **moving_towards_goal_reward**: Reward for velocity towards goal
- **distance_to_goal_progress**: Reward for reducing distance to goal
- **arrived_reward**: Large reward for reaching goal
- **collision_penalty**: Penalty for collisions

#### Terminations (`mdp/terminations.py`)
- **arrive**: Success condition (within threshold of goal)
- **collision**: Failure condition (contact force exceeds threshold)
- **time_out**: Episode length limit

#### Events (`mdp/events.py`)
- **reset_base**: Reset robot to safe position with random orientation
- **print_rewards**: Debug logging of reward components

### 3. Robot Configuration (`coco_robot_cfg.py`)

Defines the COCO delivery robot:

- **Physical Properties**: Mass, inertia, collision shapes
- **Actuators**: 
  - Wheels: DelayedPDActuator for realistic wheel dynamics
  - Axle: DCMotor for steering
  - Shock: ImplicitActuator for suspension
- **Action Space**: RestrictedCarAction (velocity and steering angle)
- **Sensors**: Cameras, contact sensors

### 4. Scene Configuration

The scene includes:

- **Custom Map**: USD file from Omniverse Nucleus (sidewalk environment)
- **Robot**: COCO delivery robot with sensors
- **Contact Sensors**: For collision detection
- **Cameras**: RGB-D cameras for visual observations

## Data Flow

### Training Loop

1. **Environment Reset**:
   - Robot spawned at safe position (from `safe_positions_auto_generated.py`)
   - Goal sampled from safe positions
   - Sensors initialized

2. **Observation Collection**:
   - Goal position transformed to robot's base frame
   - RGB-D images captured and processed
   - Robot state (velocity, orientation) collected
   - All observations concatenated into policy input

3. **Action Execution**:
   - Policy outputs action (velocity, steering angle)
   - Action processed by `RestrictedCarAction`
   - Low-level joint commands sent to actuators
   - Physics simulation steps forward

4. **Reward Computation**:
   - Multiple reward components computed
   - Weighted sum produces total reward
   - Reward components logged for analysis

5. **Termination Check**:
   - Check if robot reached goal (success)
   - Check if robot collided (failure)
   - Check if episode timeout reached

6. **Repeat** until termination, then reset

### Inference/Evaluation

Same as training loop but:
- No gradient computation
- Policy in evaluation mode
- Additional metrics logged (success rate, episode length, etc.)

## Integration with Isaac Lab

CostNav extends Isaac Lab's `ManagerBasedRLEnv`:

- **Managers**: Observation, Action, Command, Reward, Termination, Event managers
- **Scene**: Interactive scene with assets and sensors
- **Simulation**: Physics simulation context
- **Logging**: TensorBoard integration for metrics

## Cost Model Integration

The `rl_games_helpers.py` module provides cost model integration:

- **Energy Computation**: `compute_navigation_energy_step()` calculates power consumption
- **Business Metrics**: SLA compliance, operational costs, profitability
- **Logging**: Custom scalars for TensorBoard

These metrics are computed during training and logged alongside standard RL metrics, enabling evaluation of policies based on business objectives rather than just task success.

