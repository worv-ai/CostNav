# Environment Versions

CostNav provides three environment versions with increasing complexity. This document explains the differences and use cases for each version.

## Version Comparison

| Feature | v0 | v1 | v2 |
|---------|----|----|-----|
| **Task** | CartPole | Custom Map Navigation | Full Navigation with RL |
| **Robot** | CartPole | CartPole | COCO Delivery Robot |
| **Map** | None | Custom USD Map | Sidewalk USD Map |
| **Observations** | Joint states | Joint states | Goal + Velocity + RGB-D |
| **Actions** | Cart force | Cart force | Velocity + Steering |
| **Sensors** | None | None | Contact + RGB-D Camera |
| **Complexity** | Low | Medium | High |
| **Use Case** | Testing | Development | Production |

## Version 0: CartPole Baseline

### Purpose
- Verify Isaac Lab installation
- Test training pipeline
- Baseline for debugging

### Task Description
Classic CartPole task: balance a pole on a moving cart.

### Configuration

**Scene**:
```python
scene = CostnavIsaaclabSceneCfg(
    num_envs=4096,
    env_spacing=4.0,
)
```

**Robot**: CartPole articulation
- 1 DOF cart (slider joint)
- 1 DOF pole (revolute joint)

**Observations**:
- Cart position
- Cart velocity
- Pole angle
- Pole angular velocity

**Actions**:
- Force applied to cart [-1, 1]

**Rewards**:
- `alive`: +1.0 for staying upright
- `terminating`: -2.0 for falling
- `pole_pos`: Penalty for pole angle deviation
- `cart_vel`: Penalty for cart velocity

**Terminations**:
- Pole angle > 12 degrees
- Cart position out of bounds
- Episode timeout

### Usage

```bash
# Train
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v0

# Evaluate
python scripts/rl_games/play.py --task=Template-Costnav-Isaaclab-v0
```

### Expected Performance
- Training time: 5-10 minutes
- Success rate: >95% after convergence
- Useful for verifying setup

## Version 1: Custom Map Navigation

### Purpose
- Test custom USD map loading
- Develop map-specific features
- Intermediate complexity

### Task Description
CartPole navigation on custom map (still using CartPole robot for simplicity).

### Configuration

**Scene**:
```python
scene = CostnavIsaaclabSceneCfg(
    num_envs=64,
    env_spacing=0.0,  # No spacing (using custom map)
)

custom_map = AssetBaseCfg(
    prim_path="/World/custom_map",
    spawn=sim_utils.UsdFileCfg(
        usd_path="omniverse://10.50.2.21/Users/worv/map/Street_sidewalk.usd"
    ),
)
```

**Robot**: CartPole (same as v0)

**Observations**: Same as v0

**Actions**: Same as v0

**Rewards**: Same as v0

**Terminations**: Same as v0

### Key Differences from v0
- Custom USD map loaded from Omniverse Nucleus
- No environment spacing (map provides spatial separation)
- Tests map loading and rendering

### Usage

```bash
# Train
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v1-CustomMap

# Evaluate
python scripts/rl_games/play.py --task=Template-Costnav-Isaaclab-v1-CustomMap
```

### Expected Performance
- Similar to v0 (task unchanged)
- Verifies custom map integration

## Version 2: Full Navigation with RL

### Purpose
- Production navigation task
- Full COCO robot with sensors
- Cost-aware evaluation

### Task Description
Navigate COCO delivery robot to goal positions on sidewalk map, avoiding obstacles.

### Configuration

**Scene**:
```python
scene = CostnavIsaaclabSceneCfg(
    num_envs=64,
    env_spacing=0.0,
)

# Custom sidewalk map
custom_map = AssetBaseCfg(
    prim_path="/World/custom_map",
    spawn=sim_utils.UsdFileCfg(
        usd_path="omniverse://10.50.2.21/Users/worv/map/Street_sidewalk.usd"
    ),
)

# COCO delivery robot
robot = COCO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

# Contact sensors
contact_forces = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*",
    history_length=3,
    track_air_time=True,
)

# RGB-D camera
tiled_camera = TiledCameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link/front_cam",
    width=80,
    height=80,
    data_types=["rgb", "distance_to_camera"],
)
```

**Observations**:
- **Vector** (8D):
  - Goal position (x, y) in base frame [2D]
  - Base linear velocity (vx, vy, vz) [3D]
  - Base angular velocity (wx, wy, wz) [3D]
  
- **Visual** (optional, 25,600D):
  - RGB-D image (4 channels × 80 × 80 pixels)
  - Flattened to 1D vector for concatenation

**Actions** (2D):
- Forward velocity [0, 4.0] m/s
- Steering angle [-40°, +40°]

**Rewards**:
- `arrived_reward`: +20,000 (reaching goal)
- `collision_penalty`: -200 (hitting obstacles)
- `position_command_error_tanh`: +1.0 (proximity to goal)
- `heading_command_error_abs`: -0.5 (facing goal)
- `distance_to_goal_progress`: +100.0 (making progress)
- `moving_towards_goal_reward`: +1.0 (velocity towards goal)

**Terminations**:
- `arrive`: Within 0.5m of goal (success)
- `collision`: Contact force > 1.0 N (failure)
- `time_out`: Episode length limit (timeout)

**Commands**:
- Goals sampled from pre-validated safe positions
- Ensures goals not inside buildings
- Random or goal-directed heading

### Key Features

#### Safe Position System
Goals are sampled from `safe_positions_auto_generated.py`:
- Positions validated using upward raycasting
- Filters out positions inside buildings
- Ensures valid spawn and goal locations

#### Cost Model Integration
Tracks business metrics:
- Energy consumption per episode
- SLA compliance rate
- Operating margin
- Break-even time

#### Multi-Modal Observations
Supports both vector-only and vision-based policies:
- **Vector-only**: Faster training, simpler policy
- **Vision-based**: More robust, handles unseen scenarios

### Usage

```bash
# Train with cameras (vision-based policy)
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --headless

# Train without cameras (vector-only policy)
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --headless

# Evaluate
python scripts/rl_games/evaluate.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras

# Visualize
python scripts/rl_games/play.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras
```

### Expected Performance

**Baseline RL-Games Policy**:
- Success rate: 43.0%
- SLA compliance: 43.0%
- Operating margin: 46.5%
- Break-even time: 0.90 years
- Training time: 4-8 hours (64 envs, GPU)

**Target Performance**:
- Success rate: >70%
- SLA compliance: >70%
- Operating margin: >60%
- Break-even time: <1 year

## Choosing a Version

### Use v0 if:
- Setting up CostNav for the first time
- Testing Isaac Lab installation
- Debugging training pipeline
- Learning RL basics

### Use v1 if:
- Developing custom map features
- Testing USD asset loading
- Intermediate development

### Use v2 if:
- Training production navigation policies
- Evaluating cost-aware navigation
- Benchmarking different approaches
- Research and publication

## Migration Path

### From v0 to v1
1. Add custom map to scene configuration
2. Update environment spacing to 0.0
3. Verify map loads correctly
4. No changes to robot, observations, or actions needed

### From v1 to v2
1. Replace CartPole with COCO robot configuration
2. Add contact sensors and cameras to scene
3. Update observation space (goal position, velocity, RGB-D)
4. Update action space (velocity, steering)
5. Update reward function (navigation-specific)
6. Update termination conditions (arrive, collision)
7. Add safe position system for spawning/goals
8. Integrate cost model logging

## Custom Versions

To create your own version:

1. **Copy existing version**:
```bash
cp -r costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL \
      costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/my_custom_version
```

2. **Update configuration**:
- Modify `costnav_isaaclab_env_cfg.py`
- Adjust scene, observations, actions, rewards, terminations

3. **Register environment**:
```python
# In __init__.py
gym.register(
    id="My-Custom-Version",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.my_custom_version:MyCustomEnvCfg",
    },
)
```

4. **Train**:
```bash
python scripts/rl_games/train.py --task=My-Custom-Version
```

