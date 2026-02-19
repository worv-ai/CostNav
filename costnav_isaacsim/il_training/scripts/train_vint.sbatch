#!/bin/bash
#SBATCH --job-name=vint-costnav
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --nodelist=DGX-H100-[11-12]
#SBATCH --gpus-per-task=1
#SBATCH --cpus-per-gpu=32
#SBATCH --mem=1T
#SBATCH --output=logs/slurm/train_vint_%j.out
#SBATCH --error=logs/slurm/train_vint_%j.err

set -euo pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="${SLURM_SUBMIT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$SCRIPT_DIR/.."

# ── Environment ──────────────────────────────────────────────────────────────
export PROJECT_ROOT
export WANDB_MODE="${WANDB_MODE:-online}"

# Create log directory
mkdir -p "$PROJECT_ROOT/logs/slurm"

echo "=== CostNav ViNT Training ==="
echo "PROJECT_ROOT : $PROJECT_ROOT"
echo "UV_PROJECT   : $UV_PROJECT"
echo "CUDA_VISIBLE : ${CUDA_VISIBLE_DEVICES:-auto}"
echo "SLURM_JOB_ID : ${SLURM_JOB_ID:-local}"
echo "=============================="

cd "$UV_PROJECT"

# ── Setup Environment ────────────────────────────────────────────────────────
echo "Running setup.sh to initialize environment..."
bash "$SCRIPT_DIR/setup.sh"

# ── Run training ─────────────────────────────────────────────────────────────
# Redirect stderr → stdout so wandb logs, tqdm progress bars, and warnings
# all appear in the .out file instead of .err.
uv run python -m il_training.training.train_vint \
--config training/visualnav_transformer/configs/vint_costnav.yaml \
"$@" 2>&1
