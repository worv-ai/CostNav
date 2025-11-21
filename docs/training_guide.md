# Training Guide

This guide explains how to train navigation policies in CostNav, including the training pipeline, hyperparameters, and best practices.

## Training Pipeline Overview

```
1. Environment Setup
   ↓
2. Policy Initialization
   ↓
3. Training Loop
   ├─ Collect Rollouts
   ├─ Compute Advantages
   ├─ Update Policy
   └─ Log Metrics
   ↓
4. Evaluation
   ↓
5. Checkpoint Saving
```

## Supported RL Frameworks

CostNav supports multiple RL frameworks:

1. **RL-Games** (default, recommended)
2. **RSL-RL** (lightweight, fast)
3. **Stable-Baselines3** (popular, well-documented)
4. **SKRL** (modular, flexible)

## Training with RL-Games

### Basic Training Command

```bash
cd costnav_isaaclab
python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras \
    --headless
```

### Command-Line Arguments

#### Task Selection
```bash
--task=TASK_NAME
```

Available tasks:
- `Template-Costnav-Isaaclab-v0`: CartPole baseline
- `Template-Costnav-Isaaclab-v1-CustomMap`: Custom map navigation
- `Template-Costnav-Isaaclab-v2-NavRL`: Full navigation with RL (recommended)

#### Camera Control
```bash
--enable_cameras    # Enable RGB-D camera observations
```

- **With cameras**: Policy uses visual observations (RGB-D images)
- **Without cameras**: Policy uses only vector observations (goal position, velocity)

#### Rendering Mode
```bash
--headless          # Run without GUI (faster, for cluster training)
```

- **Headless**: No visualization, maximum performance
- **GUI**: Shows simulation, useful for debugging

#### Number of Environments
```bash
--num_envs=64       # Number of parallel environments
```

- More environments = faster data collection
- Limited by GPU memory
- Default: 64 (good balance for most GPUs)

#### Checkpoint Management
```bash
--resume            # Resume from latest checkpoint
--checkpoint=PATH   # Resume from specific checkpoint
```

### Training Configuration

RL-Games configuration is defined in YAML files or Python configs. Key hyperparameters:

#### PPO Hyperparameters

```yaml
params:
  algo:
    name: a2c_continuous
  
  model:
    name: continuous_a2c_logstd
  
  network:
    name: actor_critic
    separate: False
    space:
      continuous:
        mu_activation: None
        sigma_activation: None
        mu_init:
          name: default
        sigma_init:
          name: const_initializer
          val: 0
        fixed_sigma: True
    
    mlp:
      units: [256, 128, 64]
      activation: elu
      d2rl: False
      initializer:
        name: default
      regularizer:
        name: None
  
  config:
    name: Template-Costnav-Isaaclab-v2-NavRL
    env_name: isaacgym
    multi_gpu: False
    ppo: True
    mixed_precision: False
    normalize_input: True
    normalize_value: True
    value_bootstrap: True
    num_actors: 64
    reward_shaper:
      scale_value: 1.0
    normalize_advantage: True
    gamma: 0.99
    tau: 0.95
    learning_rate: 3e-4
    lr_schedule: adaptive
    kl_threshold: 0.008
    score_to_win: 20000
    max_epochs: 10000
    save_best_after: 100
    save_frequency: 100
    grad_norm: 1.0
    entropy_coef: 0.0
    truncate_grads: True
    e_clip: 0.2
    horizon_length: 24
    minibatch_size: 1024
    mini_epochs: 5
    critic_coef: 2
    clip_value: True
    seq_len: 4
    bounds_loss_coef: 0.0001
```

#### Key Parameters Explained

**Network Architecture**:
- `units: [256, 128, 64]`: Hidden layer sizes
- `activation: elu`: Activation function (ELU is smooth, helps training)
- `separate: False`: Shared trunk for actor and critic

**PPO Settings**:
- `gamma: 0.99`: Discount factor (how much to value future rewards)
- `tau: 0.95`: GAE lambda (advantage estimation)
- `learning_rate: 3e-4`: Adam learning rate
- `e_clip: 0.2`: PPO clipping parameter
- `entropy_coef: 0.0`: Entropy bonus (0 = no exploration bonus)

**Training Loop**:
- `horizon_length: 24`: Steps per rollout
- `minibatch_size: 1024`: Samples per gradient update
- `mini_epochs: 5`: Optimization epochs per rollout
- `num_actors: 64`: Parallel environments

**Normalization**:
- `normalize_input: True`: Normalize observations (running mean/std)
- `normalize_value: True`: Normalize value function targets
- `normalize_advantage: True`: Normalize advantages

## Training Process

### Phase 1: Initialization (Epochs 0-100)

**What happens**:
- Policy explores randomly
- Observation normalization statistics collected
- High variance in rewards
- Many collisions and timeouts

**Expected behavior**:
- Success rate: 0-5%
- Average reward: -100 to 0
- Episode length: Full timeout (varies by config)

**Tips**:
- Don't worry about poor performance
- Check that observations are reasonable (not NaN/Inf)
- Verify rewards are being computed correctly

### Phase 2: Learning (Epochs 100-1000)

**What happens**:
- Policy learns to avoid collisions
- Begins moving towards goals
- Success rate increases
- Reward variance decreases

**Expected behavior**:
- Success rate: 5-30%
- Average reward: 0-5000
- Episode length: Decreasing (reaching goals faster)

**Tips**:
- Monitor reward components (arrival, collision, progress)
- Check if policy is stuck in local optima
- Adjust reward weights if needed

### Phase 3: Refinement (Epochs 1000+)

**What happens**:
- Policy optimizes trajectory efficiency
- Success rate plateaus
- Learns to handle edge cases
- Reduces unnecessary movements

**Expected behavior**:
- Success rate: 30-50%
- Average reward: 5000-10000
- Episode length: Stable, efficient

**Tips**:
- Consider curriculum learning (harder goals)
- Fine-tune reward weights
- Evaluate on held-out test scenarios

## Monitoring Training

### TensorBoard

Launch TensorBoard to monitor training:

```bash
tensorboard --logdir costnav_isaaclab/logs/rl_games --port 6006
```

Open browser to `http://localhost:6006`

### Key Metrics

**Rewards**:
- `rewards/frame`: Total reward per step
- `rewards/iter`: Total reward per iteration
- `rewards/time`: Total reward over time

**Success Metrics**:
- `Episode/arrive_rate`: Percentage of successful episodes
- `Episode/collision_rate`: Percentage of collision episodes
- `Episode/timeout_rate`: Percentage of timeout episodes

**Policy Metrics**:
- `losses/a_loss`: Actor loss
- `losses/c_loss`: Critic loss
- `losses/entropy`: Policy entropy (exploration)
- `losses/kl`: KL divergence (policy change magnitude)

**Performance**:
- `performance/step_time`: Time per simulation step
- `performance/update_time`: Time per policy update
- `performance/total_time`: Total training time

### Custom Metrics

CostNav logs additional business metrics:

- `cost_model/energy_per_episode`: Energy consumption
- `cost_model/sla_compliance`: SLA compliance rate
- `cost_model/operating_margin`: Profit margin
- `cost_model/break_even_time`: Time to break even

## Troubleshooting

### Problem: Training is unstable (reward oscillates wildly)

**Solutions**:
1. Reduce learning rate: `learning_rate: 1e-4`
2. Increase minibatch size: `minibatch_size: 2048`
3. Reduce PPO clip: `e_clip: 0.1`
4. Check observation normalization is enabled

### Problem: Policy doesn't learn (reward stays flat)

**Solutions**:
1. Check reward function is working (use `test_v2_rewards.py`)
2. Verify observations are informative (not constant)
3. Increase learning rate: `learning_rate: 5e-4`
4. Reduce network size if overfitting: `units: [128, 64]`
5. Check for NaN/Inf in observations or rewards

### Problem: Policy learns to exploit reward function

**Solutions**:
1. Add slack penalty to progress reward
2. Increase collision penalty weight
3. Add smoothness penalty (action changes)
4. Use curriculum learning (start with easier goals)

### Problem: Out of memory

**Solutions**:
1. Reduce number of environments: `--num_envs=32`
2. Disable cameras: Remove `--enable_cameras`
3. Reduce image resolution in config
4. Use mixed precision: `mixed_precision: True`

### Problem: Training is too slow

**Solutions**:
1. Use headless mode: `--headless`
2. Increase number of environments: `--num_envs=128`
3. Reduce rendering interval in config
4. Use faster RL framework (RSL-RL)
5. Disable unnecessary logging

## Best Practices

### 1. Start Simple
- Train without cameras first (faster, easier to debug)
- Use smaller network (faster training)
- Shorter episodes (faster iteration)

### 2. Validate Incrementally
- Test reward function with `test_v2_rewards.py`
- Test controller with `test_controller.py`
- Verify observations are reasonable

### 3. Use Baselines
- Compare against random agent (`random_agent.py`)
- Compare against zero agent (`zero_agent.py`)
- Compare against deterministic controller

### 4. Hyperparameter Tuning
- Start with default hyperparameters
- Change one parameter at a time
- Use grid search or Bayesian optimization for systematic tuning

### 5. Reproducibility
- Set random seeds in config
- Save all hyperparameters
- Version control your configs
- Document changes and results

## Advanced Topics

### Curriculum Learning

Gradually increase task difficulty:

1. **Stage 1**: Short distances, no obstacles
2. **Stage 2**: Medium distances, sparse obstacles
3. **Stage 3**: Long distances, dense obstacles
4. **Stage 4**: Complex scenarios, dynamic obstacles

### Multi-Task Learning

Train on multiple scenarios simultaneously:
- Urban environments
- Suburban environments
- Rural environments
- Different weather conditions

### Transfer Learning

1. Pre-train on simple task
2. Fine-tune on complex task
3. Use frozen feature extractor
4. Progressive unfreezing

### Distributed Training

For large-scale training:

```bash
# SLURM cluster
sbatch train.sbatch

# Multi-GPU
python scripts/rl_games/train.py --task=... --multi_gpu=True
```

## Next Steps

After training:
1. **Evaluate**: Use `evaluate.py` to test on held-out scenarios
2. **Visualize**: Use `play.py` to watch trained policy
3. **Analyze**: Export metrics and create plots
4. **Deploy**: Convert policy to deployment format (ONNX, TorchScript)

