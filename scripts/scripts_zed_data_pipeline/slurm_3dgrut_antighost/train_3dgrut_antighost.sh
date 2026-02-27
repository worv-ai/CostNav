#!/bin/bash
# =============================================================================
# 3DGRUT Anti-Ghosting Training Script
# Docker 컨테이너 내부에서 실행됨
# =============================================================================
set -e

echo "================================================="
echo "3DGRUT Anti-Ghosting Training"
echo "================================================="
echo "Start Time: $(date)"
echo "================================================="

# =============================================================================
# 환경 변수 확인 (기본값은 Anti-Ghosting 최적화)
# =============================================================================
COLMAP_PATH="${COLMAP_PATH:-/mnt/harbor/users/samwoo/debug_10frames_real/colmap}"
MESH_PATH="${MESH_PATH:-}"
OUTPUT_DIR="${OUTPUT_DIR:-/mnt/harbor/users/samwoo/3dgrut_output}"
EXPERIMENT_NAME="${EXPERIMENT_NAME:-3dgut_antighost}"
N_ITERATIONS="${N_ITERATIONS:-100000}"
CONFIG_NAME="${CONFIG_NAME:-apps/cusfm_3dgut_mcmc}"
DOWNSAMPLE_FACTOR="${DOWNSAMPLE_FACTOR:-1}"
TEST_SPLIT_INTERVAL="${TEST_SPLIT_INTERVAL:-5}"

# ★ Anti-Ghosting 핵심 설정 ★
USE_SCALE_LOSS="${USE_SCALE_LOSS:-true}"
LAMBDA_SCALE="${LAMBDA_SCALE:-0.1}"
USE_OPACITY_LOSS="${USE_OPACITY_LOSS:-true}"
LAMBDA_OPACITY="${LAMBDA_OPACITY:-0.01}"
OPACITY_THRESHOLD="${OPACITY_THRESHOLD:-0.01}"
POSITION_LR="${POSITION_LR:-0.0001}"
POSITION_LR_FINAL="${POSITION_LR_FINAL:-1.0e-06}"
MAX_GAUSSIANS="${MAX_GAUSSIANS:-800000}"

# ★ Extended Parameters (beyond anti-ghosting) ★
EXTRA_ARGS="${EXTRA_ARGS:-}"

echo ""
echo "=== Configuration ==="
echo "  COLMAP_PATH: $COLMAP_PATH"
echo "  MESH_PATH: ${MESH_PATH:-'(none)'}"
echo "  OUTPUT_DIR: $OUTPUT_DIR"
echo "  N_ITERATIONS: $N_ITERATIONS"
echo "  CONFIG_NAME: $CONFIG_NAME"
echo ""
echo "=== ★ Anti-Ghosting Settings ★ ==="
echo "  USE_SCALE_LOSS: $USE_SCALE_LOSS (lambda: $LAMBDA_SCALE)"
echo "  USE_OPACITY_LOSS: $USE_OPACITY_LOSS (lambda: $LAMBDA_OPACITY)"
echo "  OPACITY_THRESHOLD: $OPACITY_THRESHOLD"
echo "  POSITION_LR: $POSITION_LR → $POSITION_LR_FINAL"
echo "  MAX_GAUSSIANS: $MAX_GAUSSIANS"
if [ -n "$EXTRA_ARGS" ]; then
echo ""
echo "=== ★ Extra Arguments ★ ==="
echo "  $EXTRA_ARGS"
fi
echo ""

# =============================================================================
# 입력 검증
# =============================================================================
if [ ! -d "$COLMAP_PATH" ]; then
    echo "Error: COLMAP path not found: $COLMAP_PATH"
    exit 1
fi

SPARSE_DIR=""
if [ -d "$COLMAP_PATH/sparse/0" ]; then
    SPARSE_DIR="$COLMAP_PATH/sparse/0"
elif [ -d "$COLMAP_PATH/sparse" ]; then
    SPARSE_DIR="$COLMAP_PATH/sparse"
else
    echo "Error: COLMAP sparse directory not found"
    exit 1
fi
echo "Using sparse directory: $SPARSE_DIR"

mkdir -p "$OUTPUT_DIR"

# =============================================================================
# Conda 환경 활성화
# =============================================================================
if [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    source /opt/conda/etc/profile.d/conda.sh
    conda activate 3dgrut 2>/dev/null || echo "Warning: 3dgrut conda env not found"
fi

pip install viser --quiet 2>/dev/null || true

# =============================================================================
# 3DGRUT 학습 명령 구성
# =============================================================================
cd /opt/3dgrut

CKPT_1=$((N_ITERATIONS * 14 / 100))
CKPT_2=$((N_ITERATIONS * 60 / 100))
CKPT_3=$N_ITERATIONS

echo "Derived settings:"
echo "  CHECKPOINTS: [$CKPT_1, $CKPT_2, $CKPT_3]"
echo ""

# 기본 명령
CMD="python train.py \
    --config-name $CONFIG_NAME \
    path=$COLMAP_PATH \
    out_dir=$OUTPUT_DIR \
    experiment_name=$EXPERIMENT_NAME \
    dataset.downsample_factor=$DOWNSAMPLE_FACTOR \
    dataset.test_split_interval=$TEST_SPLIT_INTERVAL \
    n_iterations=$N_ITERATIONS \
    checkpoint.iterations=\"[$CKPT_1,$CKPT_2,$CKPT_3]\" \
    model.progressive_training.max_n_features=3 \
    optimizer.type=selective_adam \
    export_usdz.enabled=true \
    export_usdz.apply_normalizing_transform=true \
    export_ply.enabled=true"

# ★ Anti-Ghosting 파라미터 추가 ★
CMD="$CMD \
    loss.use_scale=$USE_SCALE_LOSS \
    loss.lambda_scale=$LAMBDA_SCALE \
    loss.use_opacity=$USE_OPACITY_LOSS \
    loss.lambda_opacity=$LAMBDA_OPACITY \
    optimizer.params.positions.lr=$POSITION_LR \
    scheduler.positions.lr_final=$POSITION_LR_FINAL"

# MCMC 전략 파라미터 (MAX_GAUSSIANS 적용)
if [[ "$CONFIG_NAME" == *"mcmc"* ]]; then
    echo "Using MCMC strategy with Anti-Ghosting parameters..."
    STRATEGY_END=$((N_ITERATIONS * 80 / 100))
    CMD="$CMD \
        strategy.relocate.end_iteration=$STRATEGY_END \
        strategy.add.end_iteration=$STRATEGY_END \
        strategy.add.max_n_gaussians=$MAX_GAUSSIANS \
        strategy.opacity_threshold=$OPACITY_THRESHOLD"
else
    echo "Using GS strategy with Anti-Ghosting parameters..."
    DENSIFY_END=$((N_ITERATIONS * 60 / 100))
    PRUNE_END=$((N_ITERATIONS * 60 / 100))
    CMD="$CMD \
        strategy.densify.end_iteration=$DENSIFY_END \
        strategy.prune.end_iteration=$PRUNE_END \
        strategy.prune.density_threshold=$OPACITY_THRESHOLD"
fi

# ★ Extra arguments (for extended experiments beyond anti-ghosting) ★
if [ -n "$EXTRA_ARGS" ]; then
    echo "Applying extra arguments: $EXTRA_ARGS"
    CMD="$CMD $EXTRA_ARGS"
fi

# 초기화 포인트 클라우드 설정
if [ -n "$MESH_PATH" ] && [ -f "$MESH_PATH" ]; then
    echo "Using mesh for initialization: $MESH_PATH"
    CMD="$CMD initialization.fused_point_cloud_path=$MESH_PATH"
else
    POINTS3D_TXT="$COLMAP_PATH/sparse/0/points3D.txt"
    INIT_PLY="$OUTPUT_DIR/points3d_init.ply"

    if [ -f "$POINTS3D_TXT" ]; then
        echo "Converting COLMAP points3D.txt to PLY format..."
        python3 -c "
import numpy as np
points3d_path = '$POINTS3D_TXT'
output_ply = '$INIT_PLY'
points, colors = [], []
with open(points3d_path, 'r') as f:
    for line in f:
        if line.startswith('#') or not line.strip(): continue
        parts = line.strip().split()
        if len(parts) >= 7:
            points.append([float(parts[1]), float(parts[2]), float(parts[3])])
            colors.append([int(parts[4]), int(parts[5]), int(parts[6])])
points = np.array(points, dtype=np.float32)
colors = np.array(colors, dtype=np.uint8)
print(f'Loaded {len(points)} points from COLMAP')
with open(output_ply, 'w') as f:
    f.write('ply\nformat ascii 1.0\n')
    f.write(f'element vertex {len(points)}\n')
    f.write('property float x\nproperty float y\nproperty float z\n')
    f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    for i in range(len(points)):
        f.write(f'{points[i,0]} {points[i,1]} {points[i,2]} {colors[i,0]} {colors[i,1]} {colors[i,2]}\n')
print(f'Saved PLY to {output_ply}')
"
        CMD="$CMD initialization.fused_point_cloud_path=$INIT_PLY"
    else
        echo "No mesh or points3D.txt found, using 3DGRUT default initialization"
    fi
fi

# =============================================================================
# 학습 실행
# =============================================================================
echo ""
echo "================================================="
echo "Starting Anti-Ghosting Training..."
echo "================================================="
echo "Command:"
echo "$CMD"
echo ""

START_TIME=$(date +%s)
eval $CMD
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

# =============================================================================
# 완료 요약
# =============================================================================
echo ""
echo "================================================="
echo "Training Completed!"
echo "================================================="
echo "Duration: ${DURATION}s ($(($DURATION / 60))m $(($DURATION % 60))s)"
echo "Output: $OUTPUT_DIR"
echo ""
echo "Output files:"
find "$OUTPUT_DIR" -name "*.usdz" -o -name "*.ply" 2>/dev/null | head -10
echo ""
echo "End Time: $(date)"
echo "================================================="

