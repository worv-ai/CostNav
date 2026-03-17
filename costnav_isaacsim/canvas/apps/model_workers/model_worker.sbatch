#!/bin/bash
#SBATCH --job-name=md_wk
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=12G
#SBATCH --output=slurm-%j.out

source .env.pub

export CUDA_VISIBLE_DEVICES=${SLURM_STEP_GPUS:-$SLURM_JOB_GPUS}
export MODEL_WORKER_PORT=${MODEL_WORKER_PORT}

echo "Starting model worker with:"
echo "  Model Path: ${MODEL_PATH}"
echo "  CUDA Devices: ${CUDA_VISIBLE_DEVICES}"
echo "  Port: ${MODEL_WORKER_PORT}"

docker compose --env-file .env.pub up &
DOCKER_PID=$!

trap 'echo "Stopping model worker..."; docker compose down' SIGTERM SIGINT EXIT

wait $DOCKER_PID
