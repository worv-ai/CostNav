## [Currently Deprecated] Running RL

We have deprecated support on isaac-lab, due to limitations on physics dynamics fidelity on isaac-lab.
RL might be explored using isaac-lab or isaac-sim in future work.

### (Optional) Manual / bare-metal install

If you already installed Isaac Lab into your Python environment:

```bash
python -m pip install -e costnav_isaaclab/source/costnav_isaaclab
python -m pip install -e ".[dev]"
python costnav_isaaclab/scripts/list_envs.py
```


### Training RL

All commands assume `cd costnav_isaaclab` (which mirrors `/workspace/costnav_isaaclab` inside containers).

```bash
# Inspect available environments
python scripts/list_envs.py | grep Template-Costnav

# PPO baseline on COCO sidewalk navigation (RGB-D input + cameras)
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras 2>&1 | tee run_log.txt
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras --headless 2>&1 | tee run_log.txt

# Alternate curriculum / maps
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v1-CustomMap
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v0

# Evaluate, demo, or play with trained checkpoints
python scripts/rl_games/evaluate.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras
python scripts/rl_games/play.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras

# Deterministic controller + reward sanity checks
python scripts/test_controller.py --task Template-Costnav-Isaaclab-v2-NavRL --enable_cameras
python scripts/test_v2_rewards.py --task Template-Costnav-Isaaclab-v2-NavRL

# Dummy zero or random agents to confirm scene wiring
python scripts/zero_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL
python scripts/random_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

### Safe position discovery and NavMesh debugging

```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL
python find_safe_positions.py --visualize_raycasts
python safe_area_validator.py               # validates a set of candidate poses
python check_navmesh.py                     # optional NavMesh diagnostics
python check_impulse.py                     # contact impulse sweeps
```

Generated safe poses will be written back to `safe_positions_auto_generated.py`, which is consumed by both the command generator and environment reset hooks.

### Monitoring RL training

```bash
python -m tensorboard.main --logdir costnav_isaaclab/logs/rl_games/<experiment>/summaries --port 6006
```

TensorBoard logs include both standard RL metrics (success, distance, reward components) and cost-model summaries emitted as custom scalars.


### SLURM / cluster runs

Use the provided batch file to spin up per-job containers:

```bash
sbatch train.sbatch
```

`train.sbatch` derives the container name from `SLURM_JOB_ID`, pins GPUs via `NVIDIA_VISIBLE_DEVICES`, launches the `isaac-lab` compose profile, runs RL-Games training headlessly, and tears the container down when finished.
