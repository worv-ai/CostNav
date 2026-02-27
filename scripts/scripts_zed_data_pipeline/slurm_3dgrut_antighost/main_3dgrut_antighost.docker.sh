#!/bin/bash
# =============================================================================
# 3DGRUT Anti-Ghosting Training - Docker 실행 스크립트
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 환경 변수 기본값
export DATA_ROOT="${DATA_ROOT:-/mnt/harbor}"
export SCRIPT_ROOT="${SCRIPT_ROOT:-$SCRIPT_DIR}"

# 3DGRUT 학습 경로 설정
export COLMAP_PATH="${COLMAP_PATH:-/mnt/harbor/users/samwoo/debug_10frames_real/colmap}"
export MESH_PATH="${MESH_PATH:-}"
export OUTPUT_DIR="${OUTPUT_DIR:-/mnt/harbor/users/samwoo/3dgrut_output}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-3dgut_antighost}"
export N_ITERATIONS="${N_ITERATIONS:-100000}"
export CONFIG_NAME="${CONFIG_NAME:-apps/cusfm_3dgut_mcmc}"
export DOWNSAMPLE_FACTOR="${DOWNSAMPLE_FACTOR:-1}"
export TEST_SPLIT_INTERVAL="${TEST_SPLIT_INTERVAL:-5}"

# Anti-Ghosting 설정 (환경변수로 전달)
export USE_SCALE_LOSS="${USE_SCALE_LOSS:-true}"
export LAMBDA_SCALE="${LAMBDA_SCALE:-0.1}"
export USE_OPACITY_LOSS="${USE_OPACITY_LOSS:-true}"
export LAMBDA_OPACITY="${LAMBDA_OPACITY:-0.01}"
export OPACITY_THRESHOLD="${OPACITY_THRESHOLD:-0.01}"
export POSITION_LR="${POSITION_LR:-0.0001}"
export POSITION_LR_FINAL="${POSITION_LR_FINAL:-1.0e-06}"
export MAX_GAUSSIANS="${MAX_GAUSSIANS:-800000}"

echo "================================================="
echo "3DGRUT Anti-Ghosting Training - Docker Container"
echo "================================================="
echo "COLMAP Path: $COLMAP_PATH"
echo "Mesh Path: ${MESH_PATH:-'(none)'}"
echo "Output Dir: $OUTPUT_DIR"
echo "Iterations: $N_ITERATIONS"
echo "Max Gaussians: $MAX_GAUSSIANS"
echo "★ NVIDIA_VISIBLE_DEVICES: ${NVIDIA_VISIBLE_DEVICES:-NOT SET}"
echo "Start Time: $(date)"
echo "================================================="

# Docker Compose 파일 경로
COMPOSE_FILE="$SCRIPT_DIR/docker/docker-compose-3dgrut-antighost.yml"

if [ ! -f "$COMPOSE_FILE" ]; then
    echo "Error: Docker Compose file not found: $COMPOSE_FILE"
    exit 1
fi

# 기존 컨테이너 정리
echo "Cleaning up existing containers..."
docker compose -f "$COMPOSE_FILE" -p "3dgrut_antighost_${SLURM_JOB_ID:-local}" down 2>/dev/null || true

# 출력 디렉토리 생성
mkdir -p "$OUTPUT_DIR"

# Docker 컨테이너 실행
echo ""
echo "Starting 3DGRUT Anti-Ghosting training container..."
echo "================================================="

docker compose -f "$COMPOSE_FILE" \
    -p "3dgrut_antighost_${SLURM_JOB_ID:-local}" \
    up \
    --abort-on-container-exit \
    --exit-code-from 3dgrut_antighost_training

EXIT_CODE=$?

# 컨테이너 정리
echo ""
echo "Cleaning up containers..."
docker compose -f "$COMPOSE_FILE" -p "3dgrut_antighost_${SLURM_JOB_ID:-local}" down

echo ""
echo "================================================="
echo "Training completed at: $(date)"
echo "Exit code: $EXIT_CODE"
echo "================================================="

exit $EXIT_CODE

