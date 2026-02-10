#!/bin/bash
#SBATCH --job-name=vint-costnav
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --nodelist=DGX-H100-[11-12]
#SBATCH --gpus-per-task=8
#SBATCH --cpus-per-gpu=16
#SBATCH --mem=500G
#SBATCH --output=logs/slurm/train_vint_%j.out
#SBATCH --error=logs/slurm/train_vint_%j.err

set -euo pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COSTNAV_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$SCRIPT_DIR/.."

# ── Environment ──────────────────────────────────────────────────────────────
export PROJECT_ROOT="$COSTNAV_ROOT"
export WANDB_MODE="${WANDB_MODE:-online}"

# Create log directory
mkdir -p "$COSTNAV_ROOT/logs/slurm"

echo "=== CostNav ViNT Training ==="
echo "COSTNAV_ROOT : $COSTNAV_ROOT"
echo "UV_PROJECT   : $UV_PROJECT"
echo "CUDA_VISIBLE : ${CUDA_VISIBLE_DEVICES:-auto}"
echo "SLURM_JOB_ID : ${SLURM_JOB_ID:-local}"
echo "=============================="

cd "$UV_PROJECT"

# ── Run training ─────────────────────────────────────────────────────────────
uv run python -m il_baselines.training.train_vint \
--config training/visualnav_transformer/configs/vint_costnav.yaml \
"$@"
