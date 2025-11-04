# COCO Robot Navigation on Custom Map

This environment enables navigation learning with the COCO robot on a custom map using reinforcement learning.

## Quick Start

### Prerequisites
1. COCO robot USD file accessible at: `assets/robots/coco_one/coco_one.usd`
2. Custom map accessible at: `omniverse://10.50.2.21/Users/worv/map/Street_road.usd`
3. Isaac Lab and dependencies installed

### Training

```bash
# Using RSL-RL (recommended)
python scripts/rsl_rl/train.py --task Template-Costnav-Isaaclab-v2-Nav --num_envs 64

# Using RL-Games
python scripts/rl_games/train.py --task Template-Costnav-Isaaclab-v2-Nav
```

### Testing/Playing

```bash
# Play trained policy
python scripts/rsl_rl/play.py --task Template-Costnav-Isaaclab-v2-Nav \
    --checkpoint /path/to/checkpoint.pt --num_envs 1
```

## Environment Details

### Robot
- **Type**: COCO (Ackermann steering vehicle)
- **Action Space**: 2D continuous
  - Linear velocity: [0, 4.0] m/s
  - Steering angle: [-40°, +40°]

### Task
- **Objective**: Navigate to randomly sampled 2D goals
- **Episode Length**: 30 seconds
- **Success Criteria**: Reach within 1.0m of goal
- **Failure Criteria**: Collision with obstacles

### Observations
- **Policy**: 2D goal position in robot frame (normalized)
- **Sensor**: RGBD camera (240x135, 4 channels)

### Rewards
- Arrival: +2000
- Collision: -200
- Position tracking (coarse): +10.0 * tanh_reward
- Position tracking (fine): +50.0 * tanh_reward
- Moving towards goal: +20.0
- Target velocity: +10.0

## Configuration Files

- `costnav_isaaclab_v2_env_cfg.py`: Main environment configuration
- `coco_robot_cfg.py`: COCO robot and action space definitions
- `mdp/rewards.py`: Custom reward functions (if any)

## Customization

### Adjust Goal Sampling Range
Edit `CommandsCfg` in `costnav_isaaclab_v2_env_cfg.py`:
```python
ranges=nav_mdp.UniformPose2dCommandCfg.Ranges(
    pos_x=(min_x, max_x),  # Adjust these
    pos_y=(min_y, max_y),  # Adjust these
    heading=(-math.pi, math.pi)
)
```

### Modify Reward Weights
Edit `RewardsCfg` in `costnav_isaaclab_v2_env_cfg.py`:
```python
position_tracking = RewTerm(
    func=nav_mdp.position_command_error_tanh,
    weight=10.0,  # Adjust this
    params={"std": 5.0, "command_name": "pose_command"},
)
```

### Change Number of Environments
```bash
python scripts/rsl_rl/train.py --task Template-Costnav-Isaaclab-v2-Nav --num_envs 128
```

### Disable Camera (for faster debugging)
Comment out in `CostnavIsaaclabV2SceneCfg`:
```python
# camera = TiledCameraCfg(...)
```
And in `ObservationsCfg`:
```python
# sensor: SensorCfg = SensorCfg()
```

## Troubleshooting

### Issue: "Asset not found"
- Verify COCO robot USD path is correct
- Check if running from correct working directory
- Ensure assets are accessible

### Issue: "Camera prim not found"
- Check if COCO robot USD has camera prim at `base_link/front_cam`
- Adjust camera path in scene config if needed

### Issue: "Out of memory"
- Reduce `num_envs` (try 32 or 16)
- Reduce camera resolution in scene config
- Disable camera for initial testing

### Issue: "Slow training"
- Disable camera rendering during initial experiments
- Reduce physics substeps
- Use fewer environments

## Performance Tips

1. **Start Small**: Begin with 16-32 environments for debugging
2. **Profile First**: Test without camera to establish baseline performance
3. **Scale Gradually**: Increase environments once stable
4. **Monitor Metrics**: Track collision rate, arrival rate, episode length

## File Structure

```
costnav_isaaclab_v2_nav/
├── __init__.py                      # Environment registration
├── costnav_isaaclab_v2_env_cfg.py  # Main environment config
├── coco_robot_cfg.py                # COCO robot configuration
├── mdp/
│   ├── __init__.py
│   └── rewards.py                   # Custom reward functions
├── agents/
│   ├── rsl_rl_ppo_cfg.py           # RSL-RL training config
│   ├── rl_games_ppo_cfg.yaml       # RL-Games training config
│   ├── skrl_ppo_cfg.yaml           # SKRL training config
│   └── sb3_ppo_cfg.yaml            # Stable-Baselines3 config
├── README.md                        # This file
└── INTEGRATION_SUMMARY.md          # Detailed integration notes
```

## Next Steps

1. Verify asset paths and environment creation
2. Run short training session to validate setup
3. Tune hyperparameters based on initial results
4. Scale up training with more environments
5. Evaluate on test scenarios

## Support

For detailed integration information, see `INTEGRATION_SUMMARY.md`.

For Isaac Lab documentation, visit: https://isaac-sim.github.io/IsaacLab/

