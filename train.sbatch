#!/bin/bash
#SBATCH --job-name=costnav-train
#SBATCH --gpus=1
#SBATCH --cpus-per-task=24
#SBATCH --mem=200G
#SBATCH --partition=h100
#SBATCH --nodelist=DGX-H100-12
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
    --enable_cameras --headless \
    --num_envs 8 \
    agent.params.config.horizon_length=16 \
    agent.params.config.minibatch_size=128 \
    agent.params.network.space.continuous.sigma_init.val=-3.0 \
    agent.params.config.learning_rate=5e-4"

# horizon length : 16 or 32, not 64 (does learn but becomes too slow)

# TODO: support skrl for better stability
# TODO: optimize reward scale

CONTAINER_NAME=$CONTAINER_NAME docker compose down isaac-lab
