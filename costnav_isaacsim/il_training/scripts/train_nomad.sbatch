#!/bin/bash
#SBATCH -p h100
#SBATCH --job-name=nomad-costnav
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --gpus-per-task=1
#SBATCH --cpus-per-gpu=32
#SBATCH --mem=256G
#SBATCH --output=/mnt/harbor/users/dyg/logs/slurm/train_nomad_%j.out
#SBATCH --error=/mnt/harbor/users/dyg/logs/slurm/train_nomad_%j.err

set -euo pipefail

export WANDB_DIR=/mnt/harbor/users/dyg/checkpoints/nomad_costnav/wandb
mkdir -p "$WANDB_DIR"

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="/mnt/home/doyounggil/work/CostNav/costnav_isaacsim/il_training/scripts"
PROJECT_ROOT="/mnt/home/doyounggil/work/CostNav"
UV_PROJECT="/mnt/home/doyounggil/work/CostNav/costnav_isaacsim/il_training"

# ── Environment ──────────────────────────────────────────────────────────────
export PROJECT_ROOT
export PYTHONPATH="$PROJECT_ROOT/third_party/diffusion_policy:${PYTHONPATH:-}"
export WANDB_MODE=online
export WANDB_INIT_TIMEOUT=600
export WANDB_HTTP_TIMEOUT=60
export WANDB__SERVICE_WAIT=300
export UV_PROJECT_ENVIRONMENT=/mnt/home/doyounggil/work/CostNav/costnav_isaacsim/il_training/.venv
export UV_PYTHON=/mnt/home/doyounggil/.local/share/uv/python/cpython-3.11.14-linux-x86_64-gnu/bin/python3.11

# Create log directory

echo "=== CostNav ViNT Training ==="
echo "PROJECT_ROOT : $PROJECT_ROOT"
echo "UV_PROJECT   : $UV_PROJECT"
echo "CUDA_VISIBLE : ${CUDA_VISIBLE_DEVICES:-auto}"
echo "SLURM_JOB_ID : ${SLURM_JOB_ID:-local}"
echo "=============================="

cd "$UV_PROJECT"

# ── Setup Environment ────────────────────────────────────────────────────────
echo "Running setup.sh to initialize environment..."
# bash "$SCRIPT_DIR/setup.sh" (disabled: uv sync done on login-1)

# ── Run training ─────────────────────────────────────────────────────────────
# Redirect stderr → stdout so wandb logs, tqdm progress bars, and warnings
# all appear in the .out file instead of .err.
uv run python -m il_training.training.train_vint \
--config training/visualnav_transformer/configs/nomad_costnav.yaml \
"$@" 2>&1
