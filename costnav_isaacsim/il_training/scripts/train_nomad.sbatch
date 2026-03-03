#!/bin/bash
#SBATCH -p h100
#SBATCH --job-name=nomad-costnav
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --gpus-per-task=1
#SBATCH --cpus-per-gpu=32
#SBATCH --mem=256G
#SBATCH --output=logs/slurm/train_nomad_%j.out
#SBATCH --error=logs/slurm/train_nomad_%j.err

set -euo pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="${SLURM_SUBMIT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$SCRIPT_DIR/.."

# ── Environment ──────────────────────────────────────────────────────────────
export PROJECT_ROOT
export PYTHONPATH="$PROJECT_ROOT/third_party/diffusion_policy:${PYTHONPATH:-}"
export WANDB_MODE="${WANDB_MODE:-online}"
export WANDB_INIT_TIMEOUT=600
export WANDB_HTTP_TIMEOUT=60
export WANDB__SERVICE_WAIT=300
export WANDB_DIR="${WANDB_DIR:-$PROJECT_ROOT/logs/wandb/nomad_costnav}"

# Create log directory
mkdir -p "$PROJECT_ROOT/logs/slurm"
mkdir -p "$WANDB_DIR"

echo "=== CostNav NoMaD Training ==="
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
