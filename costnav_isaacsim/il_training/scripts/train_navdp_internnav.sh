#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
UV_PROJECT="$REPO_ROOT/costnav_isaacsim/il_training"
INTERNNAV_DIR="$REPO_ROOT/third_party/InternNav"

if [[ ! -d "$INTERNNAV_DIR" ]]; then
  echo "ERROR: InternNav not found at: $INTERNNAV_DIR" >&2
  exit 1
fi

export PROJECT_ROOT="$REPO_ROOT"

NAME="navdp_train"
GPUS="${CUDA_VISIBLE_DEVICES:-}"
MASTER_PORT="12345"
CONFIG_PATH="$REPO_ROOT/costnav_isaacsim/il_training/training/configs/navdp_costnav.yaml"

usage() {
  cat <<'EOF'
Usage: train_navdp_internnav.sh [--name NAME] [--gpus 0,1,2,3] [--master-port PORT] [--config PATH]

Runs InternNav NavDP training from the CostNav repo.

Options:
  --name         Experiment name (default: navdp_train)
  --gpus         Comma-separated GPU ids. If omitted, uses CUDA_VISIBLE_DEVICES
                 or defaults to "0".
  --master-port  torchrun master port (default: 12345)
  --config       Path to navdp_costnav.yaml
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --name)
      NAME="$2"
      shift 2
      ;;
    --gpus)
      GPUS="$2"
      shift 2
      ;;
    --master-port)
      MASTER_PORT="$2"
      shift 2
      ;;
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${GPUS}" ]]; then
  GPUS="0"
fi

export CUDA_VISIBLE_DEVICES="$GPUS"

IFS=',' read -r -a GPU_LIST <<< "$GPUS"
NUM_GPUS="${#GPU_LIST[@]}"

if [[ "$NUM_GPUS" -lt 1 ]]; then
  echo "ERROR: No GPUs resolved from CUDA_VISIBLE_DEVICES='$GPUS'" >&2
  exit 1
fi

export PYTHONPATH="$INTERNNAV_DIR:${PYTHONPATH:-}"

if [[ -z "${SKIP_SETUP:-}" ]]; then
  if [[ -f "$UV_PROJECT/scripts/setup.sh" ]]; then
    echo "Running setup.sh to initialize environment..."
    bash "$UV_PROJECT/scripts/setup.sh"
  fi
fi

RUN_PREFIX=()
if command -v uv >/dev/null 2>&1; then
  RUN_PREFIX=(uv run)
fi

echo "Starting NavDP training"
echo "  Repo:   $REPO_ROOT"
echo "  UV:     $UV_PROJECT"
echo "  CWD:    $INTERNNAV_DIR"
echo "  Name:   $NAME"
echo "  GPUs:   $CUDA_VISIBLE_DEVICES (count=$NUM_GPUS)"
echo "  Port:   $MASTER_PORT"
echo "  Config: $CONFIG_PATH"

if [[ "$NUM_GPUS" -gt 1 ]]; then
  "${RUN_PREFIX[@]}" torchrun \
    --nproc_per_node="$NUM_GPUS" \
    --master_addr=localhost \
    --master_port="$MASTER_PORT" \
    "$REPO_ROOT/costnav_isaacsim/il_training/training/train_navdp.py" \
    --name "$NAME" \
    --config "$CONFIG_PATH"
else
  "${RUN_PREFIX[@]}" python "$REPO_ROOT/costnav_isaacsim/il_training/training/train_navdp.py" \
    --name "$NAME" \
    --config "$CONFIG_PATH"
fi
