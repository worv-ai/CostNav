#!/bin/bash
# =============================================================================
# 3DGRUT Training - Anti-Ghosting Optimized Version
# =============================================================================
# 두겹/흔들림 현상 완화를 위한 최적화된 설정:
#   - Scale Regularization (큰 Gaussian 방지)
#   - Opacity Regularization (반투명 Gaussian 페널티)
#   - Higher Opacity Threshold (유령 Gaussian 제거)
#   - Lower Position LR (안정적 수렴)
#   - Reduced Max Gaussians (overfitting 방지)
#   - More Iterations (완전한 수렴)
#
# 사용법:
#   sbatch slurm_3dgrut_antighost.sbatch
#   COLMAP_PATH=/path/to/colmap sbatch slurm_3dgrut_antighost.sbatch
# =============================================================================

#SBATCH --job-name=3dgrut-antighost
#SBATCH --partition=h100
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=128GB
#SBATCH --time=16:00:00
#SBATCH --output=3dgrut_antighost.%j.out
#SBATCH --error=3dgrut_antighost.%j.err

# =============================================================================
# ★ GPU 격리: SLURM이 할당한 물리적 GPU만 Docker에 전달 ★
# =============================================================================
# 중요: SLURM은 CUDA_VISIBLE_DEVICES를 항상 "0"으로 재매핑함 (job에서 1개만 보이므로)
# Docker NVIDIA runtime은 물리적 GPU 인덱스가 필요하므로 SLURM_JOB_GPUS 사용
# 우선순위: SLURM_JOB_GPUS (물리 GPU) > GPU_DEVICE_ORDINAL > fallback 0
if [ -n "${SLURM_JOB_GPUS:-}" ]; then
    export SLURM_GPU_ID="$SLURM_JOB_GPUS"
elif [ -n "${GPU_DEVICE_ORDINAL:-}" ]; then
    export SLURM_GPU_ID="$GPU_DEVICE_ORDINAL"
else
    export SLURM_GPU_ID="0"
fi
export NVIDIA_VISIBLE_DEVICES="$SLURM_GPU_ID"

# =============================================================================
# 환경 정보 출력
# =============================================================================
echo "=============================================="
echo "3DGRUT Training - Anti-Ghosting Optimized"
echo "=============================================="
echo "Job ID: $SLURM_JOB_ID"
echo "Node: $SLURM_NODELIST"
echo "SLURM CUDA_VISIBLE_DEVICES: ${CUDA_VISIBLE_DEVICES:-unset}"
echo "SLURM_JOB_GPUS: ${SLURM_JOB_GPUS:-unset}"
echo "★ Allocated GPU ID: $SLURM_GPU_ID"
echo "★ NVIDIA_VISIBLE_DEVICES: $NVIDIA_VISIBLE_DEVICES"
echo "Start Time: $(date)"
echo "=============================================="
echo ""

# =============================================================================
# 스크립트 경로 설정
# =============================================================================
SCRIPT_DIR="/mnt/home/samwoo/real2sim-nurec/scripts/slurm_3dgrut_antighost"

# =============================================================================
# 기본 경로 설정
# =============================================================================
export COLMAP_PATH="${COLMAP_PATH:-/mnt/harbor/users/samwoo/debug_10frames_real/colmap}"
# MESH_PATH: 빈 문자열("")이 명시적으로 전달되면 PLY 없이 COLMAP sparse 사용
# ${MESH_PATH-default} 사용 (:-가 아님) → 빈 문자열과 미설정을 구분
export MESH_PATH="${MESH_PATH-/mnt/harbor/users/samwoo/debug_10frames_real/nvblox_mesh/mesh.ply}"
export OUTPUT_DIR="${OUTPUT_DIR:-/mnt/harbor/users/samwoo/3dgrut_output/antighost_job_${SLURM_JOB_ID}}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-3dgut_antighost_${SLURM_JOB_ID}}"

# =============================================================================
# 학습 기본 설정
# =============================================================================
export N_ITERATIONS="${N_ITERATIONS:-100000}"
export CONFIG_NAME="${CONFIG_NAME:-apps/cusfm_3dgut_mcmc}"
export DOWNSAMPLE_FACTOR="${DOWNSAMPLE_FACTOR:-1}"
export TEST_SPLIT_INTERVAL="${TEST_SPLIT_INTERVAL:-5}"

# =============================================================================
# ★ Anti-Ghosting 핵심 설정 ★
# =============================================================================
# 1. Scale Regularization - 너무 큰 Gaussian 방지 (흐릿한 blob 제거)
export USE_SCALE_LOSS="${USE_SCALE_LOSS:-true}"
export LAMBDA_SCALE="${LAMBDA_SCALE:-0.1}"

# 2. Opacity Regularization - 반투명 Gaussian 페널티 (겹침 현상 방지)
export USE_OPACITY_LOSS="${USE_OPACITY_LOSS:-true}"
export LAMBDA_OPACITY="${LAMBDA_OPACITY:-0.01}"

# 3. Opacity Threshold - 투명한 Gaussian 적극 제거 (기존 0.005 → 0.01)
export OPACITY_THRESHOLD="${OPACITY_THRESHOLD:-0.01}"

# 4. Position Learning Rate - 낮추면 더 안정적 수렴 (기존 0.00016 → 0.0001)
export POSITION_LR="${POSITION_LR:-0.0001}"
export POSITION_LR_FINAL="${POSITION_LR_FINAL:-1.0e-06}"

# 5. Max Gaussians - 줄이면 overfitting/ghosting 방지 (기존 2M → 800K)
export MAX_GAUSSIANS="${MAX_GAUSSIANS:-800000}"

# =============================================================================
# 설정 출력
# =============================================================================
echo "=== Input/Output Configuration ==="
echo "  COLMAP_PATH: $COLMAP_PATH"
echo "  MESH_PATH: ${MESH_PATH:-'(none)'}"
echo "  OUTPUT_DIR: $OUTPUT_DIR"
echo "  EXPERIMENT_NAME: $EXPERIMENT_NAME"
echo ""
echo "=== Training Configuration ==="
echo "  N_ITERATIONS: $N_ITERATIONS"
echo "  CONFIG_NAME: $CONFIG_NAME"
echo "  DOWNSAMPLE_FACTOR: $DOWNSAMPLE_FACTOR"
echo "  TEST_SPLIT_INTERVAL: $TEST_SPLIT_INTERVAL"
echo ""
echo "=== ★ Anti-Ghosting Settings ★ ==="
echo "  USE_SCALE_LOSS: $USE_SCALE_LOSS (lambda: $LAMBDA_SCALE)"
echo "  USE_OPACITY_LOSS: $USE_OPACITY_LOSS (lambda: $LAMBDA_OPACITY)"
echo "  OPACITY_THRESHOLD: $OPACITY_THRESHOLD (default: 0.005)"
echo "  POSITION_LR: $POSITION_LR → $POSITION_LR_FINAL (default: 0.00016)"
echo "  MAX_GAUSSIANS: $MAX_GAUSSIANS (default: 2,000,000)"
echo ""

# =============================================================================
# 입력 검증
# =============================================================================
if [ ! -d "$COLMAP_PATH" ]; then
    echo "Error: COLMAP path not found: $COLMAP_PATH"
    exit 1
fi

if [ ! -d "$COLMAP_PATH/sparse/0" ] && [ ! -d "$COLMAP_PATH/sparse" ]; then
    echo "Error: COLMAP sparse directory not found"
    exit 1
fi

if [ -n "$MESH_PATH" ] && [ ! -f "$MESH_PATH" ]; then
    echo "Warning: Mesh file not found, continuing without mesh..."
    export MESH_PATH=""
fi

# =============================================================================
# 출력 디렉토리 생성 및 실행
# =============================================================================
mkdir -p "$OUTPUT_DIR"
echo "Output directory created: $OUTPUT_DIR"
echo ""

echo "Starting 3DGRUT Anti-Ghosting training..."
echo "=============================================="

cd "$SCRIPT_DIR"
bash "$SCRIPT_DIR/main_3dgrut_antighost.docker.sh"

EXIT_CODE=$?

# =============================================================================
# 결과 요약
# =============================================================================
echo ""
echo "=============================================="
echo "Job Completed"
echo "=============================================="
echo "End Time: $(date)"
echo "Exit Code: $EXIT_CODE"
echo "Output: $OUTPUT_DIR"
if [ -d "$OUTPUT_DIR" ]; then
    echo ""
    echo "Output files:"
    find "$OUTPUT_DIR" -name "*.usdz" -o -name "*.ply" 2>/dev/null | head -10
fi
echo "=============================================="

exit $EXIT_CODE

