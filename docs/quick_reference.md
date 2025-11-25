# :zap: Quick Reference

This page provides quick commands and code snippets for common tasks in CostNav.

---

## :package: Installation

=== ":whale: Docker (Recommended)"

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

=== ":computer: Manual Installation"

    ```bash
    # Install Isaac Lab first (see Isaac Lab docs)

    # Install CostNav
    cd CostNav
    python -m pip install -e costnav_isaaclab/source/costnav_isaaclab
    python -m pip install -e ".[dev]"

    # Verify installation
    python costnav_isaaclab/scripts/list_envs.py
    ```

---

## :rocket: Training

### :sparkles: Quick Start

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

### :keyboard: Common Training Commands

| Use Case | Command |
|:---------|:--------|
| Resume from checkpoint | `--resume` |
| More environments (faster) | `--num_envs=128` |
| With visualization | Remove `--headless` |
| SLURM cluster | `sbatch train.sbatch` |

??? example "Full Command Examples"

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
    ```

---

## :bar_chart: Evaluation

### :clipboard: Evaluate Trained Policy

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

### :balance_scale: Baseline Comparisons

| Baseline | Command |
|:---------|:--------|
| :zero: Zero agent | `python scripts/zero_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL` |
| :game_die: Random agent | `python scripts/random_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL` |
| :robot: Deterministic | `python scripts/test_controller.py --task=Template-Costnav-Isaaclab-v2-NavRL` |

---

## :mag: Monitoring

### :chart_with_upwards_trend: TensorBoard

```bash
# Start TensorBoard
tensorboard --logdir costnav_isaaclab/logs/rl_games --port 6006

# Open in browser: http://localhost:6006
```

### :dart: Key Metrics to Watch

| Metric | Description | Target |
|:-------|:------------|:-------|
| `rewards/iter` | Total reward per iteration | :arrow_up: Increasing |
| `Episode/arrive_rate` | Success rate | :arrow_up: > 50% |
| `Episode/collision_rate` | Collision rate | :arrow_down: < 10% |
| `losses/kl` | Policy change magnitude | :wavy_dash: < 0.02 |
| `cost_model/sla_compliance` | SLA compliance rate | :arrow_up: > 70% |

---

## :hammer_and_wrench: Testing and Debugging

### :white_check_mark: Verify Environment Setup

```bash
# List all environments
python scripts/list_envs.py

# Test controller
python scripts/test_controller.py --task=Template-Costnav-Isaaclab-v2-NavRL

# Test rewards
python scripts/test_v2_rewards.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

### :eye: Debug Observations

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

### :moneybag: Debug Rewards

```python
# Enable reward printing in config
rewards:
    print_rewards = RewTerm(
        func=mdp.print_rewards,
        weight=0.0,
        params={"print_every_n_steps": 10},
    )
```

---

## :gear: Configuration

### :balance_scale: Modify Reward Weights

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

### :eye: Modify Observation Space

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

### :joystick: Modify Action Space

Edit `coco_robot_cfg.py`:

```python
# Change velocity limits
max_velocity = 6.0  # Changed from 4.0

# Change steering limits
max_steering_angle = 50 * torch.pi / 180  # Changed from 40°
```

---

## :world_map: Safe Position Management

### :mag: Generate Safe Positions

```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL

# Generate with visualization
python find_safe_positions.py --visualize_raycasts

# Generate without visualization (faster)
python find_safe_positions.py
```

### :white_check_mark: Validate Safe Positions

```bash
# Validate existing positions
python safe_area_validator.py

# Check for collisions
python check_impulse.py
```

---

## :rotating_light: Common Issues

### :x: "No module named 'isaaclab'"

??? solution "Solution"
    ```bash
    # Ensure Isaac Lab is in Python path
    export PYTHONPATH=/path/to/isaac-lab/source:$PYTHONPATH

    # Or use the compatibility layer (already included in CostNav)
    ```

### :boom: "CUDA out of memory"

??? solution "Solution"
    ```bash
    # Reduce number of environments
    python scripts/rl_games/train.py --task=... --num_envs=32

    # Disable cameras
    python scripts/rl_games/train.py --task=... # Remove --enable_cameras

    # Use smaller image resolution (edit config)
    ```

### :warning: "Reward is NaN"

??? solution "Solution"
    ```bash
    # Test reward function
    python scripts/test_v2_rewards.py --task=Template-Costnav-Isaaclab-v2-NavRL

    # Check for division by zero in reward functions
    # Check observation normalization is enabled
    ```

### :zzz: "Policy not learning"

??? solution "Solution"
    1. Verify reward function: `python scripts/test_v2_rewards.py`
    2. Check observations are informative (not constant)
    3. Try simpler task first (v0 or v1)
    4. Reduce learning rate or increase minibatch size
    5. Check for NaN/Inf in logs

---

## :memo: Code Snippets

### :moneybag: Custom Reward Function

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

### :eye: Custom Observation

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

### :stop_sign: Custom Termination

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

---

## :link: Useful Links

| Resource | Link |
|:---------|:-----|
| :octocat: GitHub | [github.com/worv-ai/CostNav](https://github.com/worv-ai/CostNav) |
| :book: Documentation | [worv-ai.github.io/CostNav](https://worv-ai.github.io/CostNav) |
| :green_book: Isaac Lab | [isaac-sim.github.io/IsaacLab](https://isaac-sim.github.io/IsaacLab) |
| :robot: Isaac Sim | [developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim) |
