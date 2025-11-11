# CostNav


cd costnav_isaaclab/
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v0
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v1-CustomMap
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras

python scripts/rl_games/evaluate.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras


python scripts/test_controller.py --task Template-Costnav-Isaaclab-v2-NavRL --enable_cameras


cd /workspace/costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL && python find_safe_positions.py --visualize_raycasts

python -m tensorboard.main --logdir /workspace/costnav_isaaclab/logs/rl_games/coco_static/2025-11-11_09-34-10/summaries --port 6006