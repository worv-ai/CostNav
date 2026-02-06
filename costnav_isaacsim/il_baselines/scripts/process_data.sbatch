#!/bin/bash
#SBATCH --job-name=costnav-data
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --nodelist=DGX-H100-11
#SBATCH --cpus-per-task=48
#SBATCH --mem=500G
#SBATCH --output=logs/slurm/process_data_%j.out
#SBATCH --error=logs/slurm/process_data_%j.err

set -euo pipefail

# ── Paths ────────────────────────────────────────────────────────────────────
# Use SLURM_SUBMIT_DIR which is set to the directory where sbatch was called
SCRIPT_DIR="${SLURM_SUBMIT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$SCRIPT_DIR/.."

# ── Environment ──────────────────────────────────────────────────────────────
export PROJECT_ROOT

echo "=== CostNav Data Processing ==="
echo "PROJECT_ROOT : $PROJECT_ROOT"
echo "UV_PROJECT   : $UV_PROJECT"
echo "SLURM_JOB_ID : ${SLURM_JOB_ID:-local}"
echo "================================"

# Create log directory
mkdir -p "$PROJECT_ROOT/logs/slurm"
cd "$UV_PROJECT"

# ── Setup Environment ────────────────────────────────────────────────────────
echo "Running setup.sh to initialize environment..."
bash "$SCRIPT_DIR/setup.sh"

# ── Activate venv ────────────────────────────────────────────────────────
# Ray workers inherit the Python executable from the main process.
# Using `uv run` causes workers to invoke uv in their temp directory, where
# the relative paths in pyproject.toml (e.g. ../../third_party/diffusion_policy)
# do not exist.  Activating the venv directly avoids this.
source "$UV_PROJECT/.venv/bin/activate"

# ── Step 1: ROS Bag → MediaRef ───────────────────────────────────────────────
echo "[Step 1] Converting ROS bags to MediaRef format..."
python -m il_baselines.data_processing.converters.ray_batch_convert \
--config data_processing/configs/processing_config.yaml

# ── Step 2: MediaRef → ViNT Training Format ─────────────────────────────────
echo ""
echo "=========================================================================="
echo "[Step 2] Converting MediaRef to ViNT training format..."
python -m il_baselines.data_processing.process_data.process_mediaref_bags \
--config data_processing/configs/vint_processing_config.yaml

echo "=== Data processing complete ==="
