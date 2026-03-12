#!/bin/bash
#SBATCH --job-name=navdp-costnav
#SBATCH --nodes=1
#SBATCH --ntasks=1
# NOTE: node selection should be overridden at submit time, e.g.:
#   sbatch --nodelist=DGX-H100-4 train_navdp.sbatch
#SBATCH --gpus-per-task=8
#SBATCH --cpus-per-gpu=28
#SBATCH --mem=1T
#SBATCH --exclusive
#SBATCH --output=/mnt/harbor/users/minhyeok/navdp/slurm/train_navdp_%j.out
#SBATCH --error=/mnt/harbor/users/minhyeok/navdp/slurm/train_navdp_%j.err

set -euo pipefail

# Paths
SCRIPT_DIR="${SLURM_SUBMIT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$SCRIPT_DIR/.."

# Environment
export PROJECT_ROOT
export WANDB_MODE="${WANDB_MODE:-online}"
export WANDB_DIR="${WANDB_DIR:-/mnt/harbor/users/minhyeok/navdp/wandb}"
export WANDB_PROJECT="${WANDB_PROJECT:-costnav-navdp}"
export PATH="$HOME/.local/bin:$PATH"

RUN_NAME="${RUN_NAME:-navdp_costnav}"
if [[ -z "${GPUS:-}" ]]; then
  if [[ -n "${SLURM_GPUS_ON_NODE:-}" ]] && [[ "${SLURM_GPUS_ON_NODE}" =~ ^[0-9]+$ ]]; then
    if [[ "${SLURM_GPUS_ON_NODE}" -gt 0 ]]; then
      GPUS="$(seq -s, 0 $((SLURM_GPUS_ON_NODE - 1)))"
    else
      GPUS="${CUDA_VISIBLE_DEVICES:-0}"
    fi
  else
    GPUS="${CUDA_VISIBLE_DEVICES:-0}"
  fi
fi
MASTER_PORT="${MASTER_PORT:-12345}"
CONFIG_PATH="${CONFIG_PATH:-training/configs/navdp_costnav.yaml}"
RUN_PREPROCESS="${RUN_PREPROCESS:-1}"
RUN_TRAIN="${RUN_TRAIN:-1}"
NAVDP_PROCESS_CONFIG="${NAVDP_PROCESS_CONFIG:-data_processing/configs/navdp_processing_config.yaml}"
PREPROCESS_SHARD_ID="${PREPROCESS_SHARD_ID:-0}"
PREPROCESS_NUM_SHARDS="${PREPROCESS_NUM_SHARDS:-1}"
PREPROCESS_LOCAL_WORKERS="${PREPROCESS_LOCAL_WORKERS:-1}"
PREPROCESS_START_INDEX="${PREPROCESS_START_INDEX:-0}"
PREPROCESS_END_INDEX="${PREPROCESS_END_INDEX:--1}"
SLURM_LOG_DIR="${SLURM_LOG_DIR:-/mnt/harbor/users/minhyeok/navdp/slurm}"

mkdir -p "$SLURM_LOG_DIR"
mkdir -p "$WANDB_DIR"
cd "$UV_PROJECT"

echo "=== CostNav NavDP Training (SLURM) ==="
echo "PROJECT_ROOT : $PROJECT_ROOT"
echo "UV_PROJECT   : $UV_PROJECT"
echo "RUN_NAME     : $RUN_NAME"
echo "GPUS         : $GPUS"
echo "MASTER_PORT  : $MASTER_PORT"
echo "CONFIG_PATH  : $CONFIG_PATH"
echo "RUN_PREPROCESS : $RUN_PREPROCESS"
echo "RUN_TRAIN    : $RUN_TRAIN"
echo "NAVDP_PROCESS_CONFIG : $NAVDP_PROCESS_CONFIG"
echo "PREPROCESS_SHARD_ID : $PREPROCESS_SHARD_ID"
echo "PREPROCESS_NUM_SHARDS : $PREPROCESS_NUM_SHARDS"
echo "PREPROCESS_LOCAL_WORKERS : $PREPROCESS_LOCAL_WORKERS"
echo "PREPROCESS_START_INDEX : $PREPROCESS_START_INDEX"
echo "PREPROCESS_END_INDEX : $PREPROCESS_END_INDEX"
echo "SLURM_LOG_DIR : $SLURM_LOG_DIR"
echo "WANDB_MODE    : ${WANDB_MODE:-unset}"
echo "WANDB_PROJECT : ${WANDB_PROJECT:-unset}"
echo "WANDB_DIR     : ${WANDB_DIR:-unset}"
echo "SLURM_JOB_NODELIST : ${SLURM_JOB_NODELIST:-unset}"
echo "SLURM_GPUS_ON_NODE : ${SLURM_GPUS_ON_NODE:-unset}"
echo "SLURM_JOB_ID : ${SLURM_JOB_ID:-local}"
echo "======================================"

if ! command -v uv >/dev/null 2>&1; then
  echo "ERROR: uv not found. Install uv or add ~/.local/bin to PATH." >&2
  exit 1
fi

if [[ "$PREPROCESS_LOCAL_WORKERS" -lt 1 ]]; then
  echo "ERROR: PREPROCESS_LOCAL_WORKERS must be >= 1" >&2
  exit 1
fi

IFS=',' read -r -a PREPROCESS_GPU_IDS <<< "${GPUS:-0}"
if [[ "${#PREPROCESS_GPU_IDS[@]}" -eq 0 ]]; then
  PREPROCESS_GPU_IDS=(0)
fi
PREPROCESS_GPU_COUNT="${#PREPROCESS_GPU_IDS[@]}"

if [[ "$RUN_PREPROCESS" == "1" && "$PREPROCESS_LOCAL_WORKERS" -gt "$PREPROCESS_GPU_COUNT" ]]; then
  echo "[Step 1] WARNING: PREPROCESS_LOCAL_WORKERS=$PREPROCESS_LOCAL_WORKERS exceeds visible GPU count=$PREPROCESS_GPU_COUNT. Workers will share GPUs round-robin."
fi

echo "[Setup] Running setup.sh (uv sync with supported Python ABI)"
bash "$SCRIPT_DIR/setup.sh"
source "$UV_PROJECT/.venv/bin/activate"

if [[ "$RUN_PREPROCESS" == "1" ]]; then
  echo "[Step 1] MediaRef -> NavDP processing"
  if [[ "$PREPROCESS_LOCAL_WORKERS" -gt 1 ]]; then
    echo "[Step 1] Launching $PREPROCESS_LOCAL_WORKERS local workers across GPUs: ${PREPROCESS_GPU_IDS[*]}"
    pids=()
    total_shards=$((PREPROCESS_NUM_SHARDS * PREPROCESS_LOCAL_WORKERS))
    for ((w = 0; w < PREPROCESS_LOCAL_WORKERS; w++)); do
      shard_id=$((PREPROCESS_SHARD_ID * PREPROCESS_LOCAL_WORKERS + w))
      gpu_id="${PREPROCESS_GPU_IDS[$((w % PREPROCESS_GPU_COUNT))]}"
      worker_log="$SLURM_LOG_DIR/preprocess_${SLURM_JOB_ID}_${w}.out"
      echo "[Step 1] worker=$w gpu=$gpu_id shard_id=$shard_id/$total_shards log=$worker_log"
      CUDA_VISIBLE_DEVICES="$gpu_id" python -m il_training.data_processing.process_data.process_mediaref_to_navdp \
        --config "$NAVDP_PROCESS_CONFIG" \
        --start-index "$PREPROCESS_START_INDEX" \
        --end-index "$PREPROCESS_END_INDEX" \
        --shard-id "$shard_id" \
        --num-shards "$total_shards" \
        >"$worker_log" 2>&1 &
      pids+=($!)
    done

    rc=0
    for pid in "${pids[@]}"; do
      if ! wait "$pid"; then
        rc=1
      fi
    done
    if [[ "$rc" -ne 0 ]]; then
      echo "ERROR: one or more preprocess workers failed" >&2
      exit 1
    fi
  else
    echo "[Step 1] Launching single preprocess worker on GPU ${PREPROCESS_GPU_IDS[0]}"
    CUDA_VISIBLE_DEVICES="${PREPROCESS_GPU_IDS[0]}" python -m il_training.data_processing.process_data.process_mediaref_to_navdp \
      --config "$NAVDP_PROCESS_CONFIG" \
      --start-index "$PREPROCESS_START_INDEX" \
      --end-index "$PREPROCESS_END_INDEX" \
      --shard-id "$PREPROCESS_SHARD_ID" \
      --num-shards "$PREPROCESS_NUM_SHARDS"
  fi
  echo "[Step 1] Done"
else
  echo "[Step 1] Skipped (RUN_PREPROCESS=$RUN_PREPROCESS)"
fi

if [[ "$RUN_TRAIN" == "1" ]]; then
  echo "[Step 2] NavDP training"
  echo "[Step 2a] Preflight import check (NavDP path)"
  python - <<'PY'
import os
import sys
from pathlib import Path

project_root = Path(os.environ["PROJECT_ROOT"])
sys.path.insert(0, str(project_root / "third_party" / "InternNav"))

checks = [
    "open3d",
    "internnav.dataset.navdp_lerobot_dataset",
    "internnav.trainer",
    "scripts.train.base_train.train",
]

for mod in checks:
    __import__(mod)
    print(f"[preflight] OK: {mod}")

from internnav.model import get_policy, get_config
P = get_policy("NavDP_Policy")
C = get_config("NavDP_Policy")
print(f"[preflight] OK: policy={P.__name__}, config={C.__name__}")
PY

  export SKIP_SETUP=1
  bash "$SCRIPT_DIR/train_navdp_internnav.sh" \
    --config "$CONFIG_PATH" \
    --name "$RUN_NAME" \
    --gpus "$GPUS" \
    --master-port "$MASTER_PORT"
else
  echo "[Step 2] Skipped (RUN_TRAIN=$RUN_TRAIN)"
fi
