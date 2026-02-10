#!/bin/bash
#SBATCH --job-name=costnav-data
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --nodelist=DGX-H100-[11-12]
#SBATCH --gpus-per-task=8
#SBATCH --cpus-per-gpu=16
#SBATCH --mem=500G
#SBATCH --output=logs/slurm/process_data_%j.out
#SBATCH --error=logs/slurm/process_data_%j.err

set -euo pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COSTNAV_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$SCRIPT_DIR/.."

# ── Environment ──────────────────────────────────────────────────────────────
export PROJECT_ROOT="$COSTNAV_ROOT"

# Create log directory
mkdir -p "$COSTNAV_ROOT/logs/slurm"

echo "=== CostNav Data Processing ==="
echo "COSTNAV_ROOT : $COSTNAV_ROOT"
echo "UV_PROJECT   : $UV_PROJECT"
echo "SLURM_JOB_ID : ${SLURM_JOB_ID:-local}"
echo "================================"

cd "$UV_PROJECT"

# ── Step 1: ROS Bag → MediaRef ───────────────────────────────────────────────
echo "[Step 1] Converting ROS bags to MediaRef format..."
uv run python -m il_baselines.data_processing.converters.ray_batch_convert \
--input_dir "$COSTNAV_ROOT/data/sample_rosbags/" \
--output_dir "$COSTNAV_ROOT/data/mediaref/" \
--config data_processing/configs/processing_config.yaml \
--num_workers 8

# ── Step 2: MediaRef → ViNT Training Format ─────────────────────────────────
echo "[Step 2] Converting MediaRef to ViNT training format..."
uv run python -m il_baselines.data_processing.process_data.process_mediaref_bags \
--input-dir "$COSTNAV_ROOT/data/mediaref/" \
--output-dir "$COSTNAV_ROOT/data/vint_format/" \
--config data_processing/configs/vint_processing_config.yaml \
--sample-rate 4.0

echo "=== Data processing complete ==="
