# Scripts Reference

This document provides a comprehensive reference for all scripts in the CostNav repository.

## Training Scripts

### RL-Games Training

**Location**: `costnav_isaaclab/scripts/rl_games/train.py`

**Purpose**: Train navigation policies using RL-Games (PPO algorithm)

**Usage**:
```bash
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --headless \
    --num_envs=64
```

**Arguments**:
- `--task`: Environment name (required)
- `--enable_cameras`: Enable RGB-D camera observations
- `--headless`: Run without GUI (faster)
- `--num_envs`: Number of parallel environments
- `--resume`: Resume from latest checkpoint
- `--checkpoint`: Path to specific checkpoint
- `--seed`: Random seed for reproducibility

**Output**:
- Checkpoints: `costnav_isaaclab/logs/rl_games/<task>/nn/`
- TensorBoard logs: `costnav_isaaclab/logs/rl_games/<task>/summaries/`
- Config: `costnav_isaaclab/logs/rl_games/<task>/config.yaml`

### RSL-RL Training

**Location**: `costnav_isaaclab/scripts/rsl_rl/train.py`

**Purpose**: Train using RSL-RL (lightweight PPO implementation)

**Usage**:
```bash
python scripts/rsl_rl/train.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

**Advantages**:
- Faster training (less overhead)
- Simpler codebase
- Good for rapid prototyping

### Stable-Baselines3 Training

**Location**: `costnav_isaaclab/scripts/sb3/train.py`

**Purpose**: Train using Stable-Baselines3 (popular RL library)

**Usage**:
```bash
python scripts/sb3/train.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

**Advantages**:
- Well-documented
- Many algorithms (PPO, SAC, TD3, etc.)
- Easy to customize

### SKRL Training

**Location**: `costnav_isaaclab/scripts/skrl/train.py`

**Purpose**: Train using SKRL (modular RL library)

**Usage**:
```bash
python scripts/skrl/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --headless \
    --num_envs=64
```

**Arguments**:
- `--task`: Environment name (required)
- `--enable_cameras`: Enable RGB-D camera observations
- `--headless`: Run without GUI (faster)
- `--num_envs`: Number of parallel environments
- `--checkpoint`: Path to specific checkpoint to resume from
- `--seed`: Random seed for reproducibility
- `--track`: Enable wandb tracking
- `--wandb-project-name`: Wandb project name
- `--wandb-entity`: Wandb entity (username or team)
- `--wandb-name`: Wandb run name

**Advantages**:
- Modular design
- Multiple algorithms
- JAX support

## Evaluation Scripts

### RL-Games Evaluation

**Location**: `costnav_isaaclab/scripts/rl_games/evaluate.py`

**Purpose**: Evaluate trained policy on test scenarios

**Usage**:
```bash
python scripts/rl_games/evaluate.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --checkpoint=path/to/checkpoint.pth \
    --num_envs=64
```

**Output**:
- Success rate
- Average episode length
- Average reward
- SLA compliance
- Cost metrics

### RL-Games Play

**Location**: `costnav_isaaclab/scripts/rl_games/play.py`

**Purpose**: Visualize trained policy in simulation

**Usage**:
```bash
python scripts/rl_games/play.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --checkpoint=path/to/checkpoint.pth
```

**Arguments**:
- `--task`: Environment name (required)
- `--enable_cameras`: Enable RGB-D camera observations
- `--checkpoint`: Path to specific checkpoint
- `--use_last_checkpoint`: Use the last saved model instead of best (when no checkpoint provided)

**Features**:
- Real-time visualization
- Camera views
- Debug overlays
- Manual control (optional)
- Contact impulse metrics

### SKRL Evaluation

**Location**: `costnav_isaaclab/scripts/skrl/evaluate.py`

**Purpose**: Evaluate trained SKRL policy on test scenarios

**Usage**:
```bash
python scripts/skrl/evaluate.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --checkpoint=path/to/checkpoint.pt \
    --num_envs=64 \
    --num_episodes=100
```

**Arguments**:
- `--task`: Environment name (required)
- `--enable_cameras`: Enable RGB-D camera observations
- `--checkpoint`: Path to specific checkpoint
- `--use_last_checkpoint`: Use the last saved model instead of best (when no checkpoint provided)
- `--num_envs`: Number of parallel environments
- `--num_episodes`: Number of episodes to evaluate

**Output**:
- Success rate
- Average episode length
- Average reward
- Collision metrics
- Navigation energy metrics
- CSV export with per-episode data

### SKRL Play

**Location**: `costnav_isaaclab/scripts/skrl/play.py`

**Purpose**: Visualize trained SKRL policy in simulation

**Usage**:
```bash
python scripts/skrl/play.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --checkpoint=path/to/checkpoint.pt
```

**Arguments**:
- `--task`: Environment name (required)
- `--enable_cameras`: Enable RGB-D camera observations
- `--checkpoint`: Path to specific checkpoint
- `--use_last_checkpoint`: Use the last saved model instead of best (when no checkpoint provided)

**Features**:
- Real-time visualization
- Camera views
- Debug overlays
- Contact impulse metrics

## Testing Scripts

### List Environments

**Location**: `costnav_isaaclab/scripts/list_envs.py`

**Purpose**: List all registered environments

**Usage**:
```bash
python scripts/list_envs.py
```

**Output**:
```
Registered environments:
- Template-Costnav-Isaaclab-v0
- Template-Costnav-Isaaclab-v1-CustomMap
- Template-Costnav-Isaaclab-v2-NavRL
```

### Test Controller

**Location**: `costnav_isaaclab/scripts/test_controller.py`

**Purpose**: Test deterministic controller (sanity check)

**Usage**:
```bash
python scripts/test_controller.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras
```

**What it does**:
- Runs simple deterministic controller
- Verifies environment setup
- Checks observations and actions
- Useful for debugging

### Test Rewards

**Location**: `costnav_isaaclab/scripts/test_v2_rewards.py`

**Purpose**: Test reward function components

**Usage**:
```bash
python scripts/test_v2_rewards.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL
```

**What it does**:
- Prints reward components for each step
- Verifies reward computation
- Checks for NaN/Inf values
- Useful for reward debugging

### Zero Agent

**Location**: `costnav_isaaclab/scripts/zero_agent.py`

**Purpose**: Baseline agent that takes zero actions

**Usage**:
```bash
python scripts/zero_agent.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL
```

**Purpose**:
- Baseline performance (worst case)
- Verify environment doesn't crash with zero actions
- Check default behavior

### Random Agent

**Location**: `costnav_isaaclab/scripts/random_agent.py`

**Purpose**: Baseline agent that takes random actions

**Usage**:
```bash
python scripts/random_agent.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL
```

**Purpose**:
- Baseline performance (random exploration)
- Verify action space is correct
- Check environment stability

## Safe Position Scripts

### Find Safe Positions

**Location**: `costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/find_safe_positions.py`

**Purpose**: Generate safe spawn/goal positions using raycasting

**Usage**:
```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL
python find_safe_positions.py --visualize_raycasts
```

**What it does**:
- Samples candidate positions on map
- Performs upward raycasts to detect overhead obstacles
- Filters out positions inside buildings
- Saves validated positions to `safe_positions_auto_generated.py`

**Arguments**:
- `--visualize_raycasts`: Show raycasts in simulation (slower)
- `--num_samples`: Number of positions to sample
- `--grid_size`: Grid spacing for sampling

### Validate Safe Positions

**Location**: `costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/safe_area_validator.py`

**Purpose**: Validate existing safe positions

**Usage**:
```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL
python safe_area_validator.py
```

**What it does**:
- Loads positions from `safe_positions_auto_generated.py`
- Spawns robot at each position
- Checks for collisions or instability
- Reports invalid positions

### Check NavMesh

**Location**: `costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/check_navmesh.py`

**Purpose**: Visualize navigation mesh (if available)

**Usage**:
```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL
python check_navmesh.py
```

**What it does**:
- Loads navigation mesh from map
- Visualizes walkable areas
- Useful for understanding map topology

### Check Impulse

**Location**: `costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/check_impulse.py`

**Purpose**: Test contact impulse thresholds

**Usage**:
```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL
python check_impulse.py
```

**What it does**:
- Spawns robot at various positions
- Applies different velocities
- Measures contact forces on collision
- Helps tune collision detection threshold

## SLURM Scripts

### Training Batch Job

**Location**: `train.sbatch`

**Purpose**: Submit training job to SLURM cluster

**Usage**:
```bash
sbatch train.sbatch
```

**What it does**:
- Allocates GPU resources
- Launches Docker container
- Runs training script
- Saves logs to `logs/`

**Configuration**:
```bash
#SBATCH --job-name=costnav-train
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=8
#SBATCH --gres=gpu:1
#SBATCH --time=24:00:00
#SBATCH --output=logs/costnav-train_%j.out
#SBATCH --error=logs/costnav-train_%j.err
```

## Docker Scripts

### Docker Compose

**Location**: `docker-compose.yml`

**Purpose**: Define Docker services for development

**Usage**:
```bash
# Start Isaac Lab service
docker compose --profile isaac-lab up -d

# Start Isaac Sim service
docker compose --profile isaac-sim up -d

# Start dev service (no simulator)
docker compose --profile dev up -d

# Stop all services
docker compose down
```

**Services**:
- `isaac-lab`: Full Isaac Lab + Isaac Sim stack
- `isaac-sim`: Bare Isaac Sim
- `dev`: Lightweight dev environment

### Makefile

**Location**: `Makefile`

**Purpose**: Build Docker images

**Usage**:
```bash
# Build all images
make build-all

# Build specific image
make build-isaac-lab

# Build with custom version
make build-isaac-lab COSTNAV_VERSION=0.2.0
```

## Best Practices

### Before Training
1. Run `list_envs.py` to verify environment registration
2. Run `test_controller.py` to verify environment setup
3. Run `test_v2_rewards.py` to verify reward function
4. Run `zero_agent.py` and `random_agent.py` for baselines

### During Training
1. Monitor TensorBoard for metrics
2. Check logs for errors or warnings
3. Verify checkpoints are being saved
4. Compare against baselines

### After Training
1. Run `evaluate.py` for quantitative metrics
2. Run `play.py` for qualitative assessment
3. Analyze cost model metrics
4. Compare with other policies

### Debugging
1. Use `test_controller.py` to isolate environment issues
2. Use `test_v2_rewards.py` to debug reward function
3. Use `--headless=False` to visualize behavior
4. Check logs for NaN/Inf values
5. Reduce `num_envs` to simplify debugging
