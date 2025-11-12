#!/bin/bash
#SBATCH --job-name=costnav-train
#SBATCH --gpus=1
#SBATCH --cpus-per-task=24
#SBATCH --mem=100G
#SBATCH --partition=h100
#SBATCH --nodelist=DGX-H100-11
#SBATCH --output=logs/costnav-train_%j.out
#SBATCH --error=logs/costnav-train_%j.err

echo "SLURM_JOB_GPUS: $SLURM_JOB_GPUS"
echo "SLURM_STEP_GPUS: $SLURM_STEP_GPUS"
echo "SLURM_JOB_ID: $SLURM_JOB_ID"

# Use SLURM_JOB_ID to create unique container name
export CONTAINER_NAME="costnav-isaac-lab-${SLURM_JOB_ID}"
export COMPOSE_PROJECT_NAME="costnav-${SLURM_JOB_ID}"

echo "Container name: $CONTAINER_NAME"

NVIDIA_VISIBLE_DEVICES=$SLURM_JOB_GPUS CONTAINER_NAME=$CONTAINER_NAME docker compose up -d isaac-lab

docker exec $CONTAINER_NAME bash -c "nvidia-smi -L"

docker exec $CONTAINER_NAME bash -c "cd /workspace/costnav_isaaclab \
    && python scripts/rl_games/train.py \
    --task=Template-Costnav-Isaaclab-v2-NavRL \
    --enable_cameras --headless"

CONTAINER_NAME=$CONTAINER_NAME docker compose down isaac-lab
