# Quick Reference

This page provides quick commands and code snippets for common tasks in CostNav.

## Installation

### Docker (Recommended)

```bash
# Clone repository
git clone https://github.com/worv-ai/CostNav.git
cd CostNav

# Initialize submodules
git submodule update --init --recursive

# Configure environment
cp .env.example .env
# Edit .env with your settings

# Start Isaac Lab container
docker compose --profile isaac-lab up -d

# Enter container
docker exec -it costnav-isaac-lab bash
```

### Manual Installation

```bash
# Install Isaac Lab first (see Isaac Lab docs)

# Install CostNav
cd CostNav
python -m pip install -e costnav_isaaclab/source/costnav_isaaclab
python -m pip install -e ".[dev]"

# Verify installation
python costnav_isaaclab/scripts/list_envs.py
```

## Training

### Quick Start

```bash
cd costnav_isaaclab

# Train with default settings (vector observations only)
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --headless

# Train with cameras (RGB-D observations)
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --headless
```

### Common Training Commands

```bash
# Resume from checkpoint
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --resume

# Train with more environments (faster)
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --num_envs=128 \
    --headless

# Train with visualization (slower, for debugging)
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras

# Train on SLURM cluster
sbatch train.sbatch
```

## Evaluation

### Evaluate Trained Policy

```bash
# Evaluate with metrics
python scripts/rl_games/evaluate.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --checkpoint=logs/rl_games/Template-Costnav-Isaaclab-v2-NavRL/nn/last_checkpoint.pth

# Visualize policy
python scripts/rl_games/play.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --checkpoint=logs/rl_games/Template-Costnav-Isaaclab-v2-NavRL/nn/last_checkpoint.pth
```

### Baseline Comparisons

```bash
# Zero agent (no actions)
python scripts/zero_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL

# Random agent
python scripts/random_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL

# Deterministic controller
python scripts/test_controller.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

## Monitoring

### TensorBoard

```bash
# Start TensorBoard
tensorboard --logdir costnav_isaaclab/logs/rl_games --port 6006

# Open in browser
# http://localhost:6006
```

### Key Metrics to Watch

- `rewards/iter`: Total reward per iteration
- `Episode/arrive_rate`: Success rate
- `Episode/collision_rate`: Collision rate
- `losses/kl`: Policy change magnitude
- `cost_model/sla_compliance`: SLA compliance rate

## Testing and Debugging

### Verify Environment Setup

```bash
# List all environments
python scripts/list_envs.py

# Test controller
python scripts/test_controller.py --task=Template-Costnav-Isaaclab-v2-NavRL

# Test rewards
python scripts/test_v2_rewards.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

### Debug Observations

```python
# In Python
import gymnasium as gym
env = gym.make("Template-Costnav-Isaaclab-v2-NavRL", num_envs=1)
obs, info = env.reset()

# Check observation shape
print(f"Observation shape: {obs['policy'].shape}")

# Check observation values
print(f"Observation min: {obs['policy'].min()}")
print(f"Observation max: {obs['policy'].max()}")
print(f"Observation mean: {obs['policy'].mean()}")
```

### Debug Rewards

```python
# Enable reward printing in config
rewards:
    print_rewards = RewTerm(
        func=mdp.print_rewards,
        weight=0.0,
        params={"print_every_n_steps": 10},
    )
```

## Configuration

### Modify Reward Weights

Edit `costnav_isaaclab_env_cfg.py`:

```python
@configclass
class RewardsCfg:
    # Increase arrival reward
    arrived_reward = RewTerm(
        func=loc_mdp.is_terminated_term,
        weight=30000.0,  # Changed from 20000.0
        params={"term_keys": "arrive"}
    )
    
    # Increase collision penalty
    collision_penalty = RewTerm(
        func=loc_mdp.is_terminated_term,
        weight=-500.0,  # Changed from -200.0
        params={"term_keys": "collision"}
    )
```

### Modify Observation Space

Edit `costnav_isaaclab_env_cfg.py`:

```python
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # Add new observation
        robot_height = ObsTerm(func=mdp.base_pos_z)
        
        # Modify existing observation
        pose_command = ObsTerm(
            func=mdp.pose_command_2d,
            params={"command_name": "pose_command"},
            scale=10.0,  # Changed from 5.0
        )
```

### Modify Action Space

Edit `coco_robot_cfg.py`:

```python
# Change velocity limits
max_velocity = 6.0  # Changed from 4.0

# Change steering limits
max_steering_angle = 50 * torch.pi / 180  # Changed from 40°
```

## Safe Position Management

### Generate Safe Positions

```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL

# Generate with visualization
python find_safe_positions.py --visualize_raycasts

# Generate without visualization (faster)
python find_safe_positions.py
```

### Validate Safe Positions

```bash
# Validate existing positions
python safe_area_validator.py

# Check for collisions
python check_impulse.py
```

## Common Issues

### Issue: "No module named 'isaaclab'"

**Solution**:
```bash
# Ensure Isaac Lab is in Python path
export PYTHONPATH=/path/to/isaac-lab/source:$PYTHONPATH

# Or use the compatibility layer (already included in CostNav)
```

### Issue: "CUDA out of memory"

**Solution**:
```bash
# Reduce number of environments
python scripts/rl_games/train.py --task=... --num_envs=32

# Disable cameras
python scripts/rl_games/train.py --task=... # Remove --enable_cameras

# Use smaller image resolution (edit config)
```

### Issue: "Reward is NaN"

**Solution**:
```bash
# Test reward function
python scripts/test_v2_rewards.py --task=Template-Costnav-Isaaclab-v2-NavRL

# Check for division by zero in reward functions
# Check observation normalization is enabled
```

### Issue: "Policy not learning"

**Solution**:
1. Verify reward function: `python scripts/test_v2_rewards.py`
2. Check observations are informative (not constant)
3. Try simpler task first (v0 or v1)
4. Reduce learning rate or increase minibatch size
5. Check for NaN/Inf in logs

## Code Snippets

### Custom Reward Function

```python
# In mdp/rewards.py
def my_custom_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Custom reward function."""
    robot = env.scene["robot"]
    
    # Get robot velocity
    velocity = robot.data.root_lin_vel_b[:, 0]
    
    # Reward forward motion
    reward = velocity.clamp(min=0.0)
    
    return reward

# In costnav_isaaclab_env_cfg.py
@configclass
class RewardsCfg:
    my_reward = RewTerm(func=mdp.my_custom_reward, weight=1.0)
```

### Custom Observation

```python
# In mdp/observations.py
def my_custom_observation(env: ManagerBasedEnv) -> torch.Tensor:
    """Custom observation function."""
    robot = env.scene["robot"]
    
    # Get robot height
    height = robot.data.root_pos_w[:, 2]
    
    return height.unsqueeze(-1)

# In costnav_isaaclab_env_cfg.py
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        robot_height = ObsTerm(func=mdp.my_custom_observation)
```

### Custom Termination

```python
# In mdp/terminations.py
def my_custom_termination(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Custom termination condition."""
    robot = env.scene["robot"]
    
    # Terminate if robot is too high
    height = robot.data.root_pos_w[:, 2]
    too_high = height > 2.0
    
    return too_high

# In costnav_isaaclab_env_cfg.py
@configclass
class TerminationsCfg:
    my_termination = DoneTerm(func=mdp.my_custom_termination)
```

## Useful Links

- **GitHub**: https://github.com/worv-ai/CostNav
- **Documentation**: https://worv-ai.github.io/CostNav
- **Isaac Lab**: https://isaac-sim.github.io/IsaacLab
- **Isaac Sim**: https://developer.nvidia.com/isaac-sim

