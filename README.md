# CostNav


cd costnav_isaaclab/
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v0
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v1-CustomMap
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras 2>&1 | tee run_log.txt

python scripts/rl_games/evaluate.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras


python scripts/test_controller.py --task Template-Costnav-Isaaclab-v2-NavRL --enable_cameras


cd /workspace/costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL && python find_safe_positions.py --visualize_raycasts

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-11_12-51-08/summaries --port 6006


best
python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-06_07-18-39/summaries --port 6006

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-11_01-08-18/summaries --port 6006

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-12_00-42-40/summaries --port 6006