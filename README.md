# CostNav


cd costnav_isaaclab/
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v0
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v1-CustomMap
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras 2>&1 | tee run_log.txt
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras --headless 2>&1 | tee run_log.txt


python scripts/rl_games/evaluate.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras


python scripts/test_controller.py --task Template-Costnav-Isaaclab-v2-NavRL --enable_cameras


cd /workspace/costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL && python find_safe_positions.py --visualize_raycasts

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-11_12-51-08/summaries --port 6006


## Best params
python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-06_07-18-39/summaries --port 6006

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-11_01-08-18/summaries --port 6006

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-12_00-42-40/summaries --port 6006

python -m tensorboard.main --logdir /mnt/home/haebin/harbor/CostNav/costnav_isaaclab/logs/rl_games/coco_static/2025-11-12_06-01-31/summaries --port 6006
---> this is much better with moving_towards_goal

python -m tensorboard.main --logdir /mnt/home/haebin/harbor/CostNav/costnav_isaaclab/logs/rl_games/coco_static/2025-11-12_07-23-08/summaries --port 6006
---> distance goal is weaker

## Docker

### Environment Configuration

The Docker containers use environment variables defined in the `.env` file. A template file `.env.example` is provided.

To customize your environment:
1. Copy `.env.example` to `.env` (already done by default)
2. Edit `.env` to set your custom values
3. The `.env` file is automatically loaded by docker-compose

Key environment variables:
- `DISPLAY`: X11 display for GUI applications (default: `:0`)
- `NVIDIA_VISIBLE_DEVICES`: Which GPUs to use (default: `all`)
- `GPU_DEVICE_IDS`: Specific GPU device IDs for containers (default: `0`)
- `ISAAC_PATH`, `CARB_APP_PATH`, `EXP_PATH`: Isaac Sim paths

### Running Containers

docker compose up -d isaac-lab
docker exec -it costnav-isaac-lab bash