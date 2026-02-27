#!/bin/bash
# ZED Camera NuRec Pipeline
# ROS2 bag → cuSFM → FoundationStereo → nvblox
#
# Usage:
#   ./scripts/run_zed_pipeline.sh <bag_path> <output_dir> [options]
#
# Options:
#   --start-time <sec>   Start time in seconds (default: none)
#   --end-time <sec>     End time in seconds (default: none)
#   --skip <n>           Process every n-th frame (default: 1)
#
# Example:
#   ./scripts/run_zed_pipeline.sh \
#       /workspace/data/zed_bag \
#       /workspace/output/zed_test \
#       --start-time 38 --end-time 52 --skip 8

set -e

# Resolve script directory for relative imports
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }

# Parse arguments
if [ $# -lt 2 ]; then
    echo "Usage: $0 <bag_path> <output_dir> [--start-time <sec>] [--end-time <sec>] [--skip <n>]"
    exit 1
fi

BAG_PATH="$1"
OUTPUT_BASE="$2"
shift 2

START_TIME=""
END_TIME=""
SKIP=1

while [[ $# -gt 0 ]]; do
    case $1 in
        --start-time) START_TIME="$2"; shift 2 ;;
        --end-time) END_TIME="$2"; shift 2 ;;
        --skip) SKIP="$2"; shift 2 ;;
        *) log_error "Unknown option: $1"; exit 1 ;;
    esac
done

# Paths
EXTRACTED_DIR="$OUTPUT_BASE/extracted"
CUSFM_DIR="$OUTPUT_BASE/cusfm"
DEPTH_DIR="$OUTPUT_BASE/depth"
NVBLOX_DIR="$OUTPUT_BASE/nvblox"

# Start time
PIPELINE_START=$(date +%s)

echo "============================================================"
echo "ZED Camera NuRec Pipeline"
echo "============================================================"
echo ""
echo "Input:  $BAG_PATH"
echo "Output: $OUTPUT_BASE"
echo "Time Range: ${START_TIME:-start} ~ ${END_TIME:-end}"
echo "Skip: every $SKIP frame(s)"
echo ""

# =============================================================================
# Step 1: Extract ZED images from ROS2 bag
# =============================================================================
echo ""
log_info "=========================================="
log_info "Step 1/4: Extract ZED images from ROS2 bag"
log_info "=========================================="
STEP1_START=$(date +%s)

EXTRACT_ARGS="$BAG_PATH $EXTRACTED_DIR"
[ -n "$START_TIME" ] && EXTRACT_ARGS="$EXTRACT_ARGS --start-time $START_TIME"
[ -n "$END_TIME" ] && EXTRACT_ARGS="$EXTRACT_ARGS --end-time $END_TIME"
[ "$SKIP" != "1" ] && EXTRACT_ARGS="$EXTRACT_ARGS --skip $SKIP"

python3 "$SCRIPT_DIR/extract_zed_bag.py" $EXTRACT_ARGS

STEP1_END=$(date +%s)
log_success "Step 1 completed in $((STEP1_END - STEP1_START))s"

# Verify extraction
if [ ! -f "$EXTRACTED_DIR/frames_meta.json" ]; then
    log_error "frames_meta.json not found in $EXTRACTED_DIR"
    exit 1
fi

LEFT_COUNT=$(ls "$EXTRACTED_DIR/zed_left/"*.jpeg 2>/dev/null | wc -l)
log_info "Extracted $LEFT_COUNT stereo pairs"

# =============================================================================
# Step 2: cuSFM Pose Estimation
# =============================================================================
echo ""
log_info "=========================================="
log_info "Step 2/4: cuSFM Pose Estimation"
log_info "=========================================="
STEP2_START=$(date +%s)

cusfm_cli \
    --input_dir "$EXTRACTED_DIR" \
    --cusfm_base_dir "$CUSFM_DIR" \
    --min_inter_frame_distance 0.06 \
    --min_inter_frame_rotation_degrees 1.5

STEP2_END=$(date +%s)
log_success "Step 2 completed in $((STEP2_END - STEP2_START))s"

# Verify cuSFM output
POINTS_COUNT=$(grep -c "^[0-9]" "$CUSFM_DIR/sparse/points3D.txt" 2>/dev/null || echo "0")
log_info "Generated $POINTS_COUNT 3D points"

if [ "$POINTS_COUNT" -eq 0 ]; then
    log_error "No 3D points generated. Check cuSFM logs."
    exit 1
fi

# =============================================================================
# Step 3: FoundationStereo Depth Estimation
# =============================================================================
echo ""
log_info "=========================================="
log_info "Step 3/4: FoundationStereo Depth Estimation"
log_info "=========================================="
STEP3_START=$(date +%s)

python3 "$SCRIPT_DIR/run_depth.py" \
    --image_dir "$EXTRACTED_DIR" \
    --output_dir "$DEPTH_DIR" \
    --frames_meta_file "$CUSFM_DIR/keyframes/frames_meta.json" \
    --verbose

STEP3_END=$(date +%s)
log_success "Step 3 completed in $((STEP3_END - STEP3_START))s"

# Verify depth output
DEPTH_COUNT=$(ls "$DEPTH_DIR/zed_left/"*.png 2>/dev/null | wc -l)
log_info "Generated $DEPTH_COUNT depth maps"

# =============================================================================
# Step 4: nvblox Mesh Generation
# =============================================================================
echo ""
log_info "=========================================="
log_info "Step 4/4: nvblox Mesh Generation"
log_info "=========================================="
STEP4_START=$(date +%s)

mkdir -p "$NVBLOX_DIR"
export LD_LIBRARY_PATH=/opt/nvblox/build/nvblox:/opt/nvblox/build/nvblox/executables:$LD_LIBRARY_PATH

/opt/nvblox/build/nvblox/executables/fuse_cusfm \
    --color_image_dir "$EXTRACTED_DIR" \
    --depth_image_dir "$DEPTH_DIR" \
    --frames_meta_file "$CUSFM_DIR/keyframes/frames_meta.json" \
    --mesh_output_path "$NVBLOX_DIR/mesh.ply" \
    --save_2d_occupancy_map_path "$NVBLOX_DIR/occupancy_map" \
    --mapping_type_dynamic \
    --voxel_size 0.025 \
    --projective_integrator_max_integration_distance_m 5.0

STEP4_END=$(date +%s)
log_success "Step 4 completed in $((STEP4_END - STEP4_START))s"

# Verify nvblox output
if [ -f "$NVBLOX_DIR/mesh.ply" ]; then
    VERTEX_COUNT=$(grep "element vertex" "$NVBLOX_DIR/mesh.ply" | awk '{print $3}')
    FACE_COUNT=$(grep "element face" "$NVBLOX_DIR/mesh.ply" | awk '{print $3}')
    log_info "Generated mesh: $VERTEX_COUNT vertices, $FACE_COUNT faces"
else
    log_error "mesh.ply not generated"
    exit 1
fi

# =============================================================================
# Summary
# =============================================================================
PIPELINE_END=$(date +%s)
TOTAL_TIME=$((PIPELINE_END - PIPELINE_START))

echo ""
echo "============================================================"
echo "Pipeline Complete!"
echo "============================================================"
echo ""
echo "Timing Summary:"
echo "  Step 1 (Extract):  $((STEP1_END - STEP1_START))s"
echo "  Step 2 (cuSFM):    $((STEP2_END - STEP2_START))s"
echo "  Step 3 (Depth):    $((STEP3_END - STEP3_START))s"
echo "  Step 4 (nvblox):   $((STEP4_END - STEP4_START))s"
echo "  ----------------------------------------"
echo "  Total:             ${TOTAL_TIME}s ($(($TOTAL_TIME / 60))m $(($TOTAL_TIME % 60))s)"
echo ""
echo "Output Files:"
echo "  Extracted images: $EXTRACTED_DIR"
echo "  cuSFM output:     $CUSFM_DIR"
echo "  Depth maps:       $DEPTH_DIR"
echo "  3D Mesh:          $NVBLOX_DIR/mesh.ply"
echo ""
echo "Results:"
echo "  Stereo pairs:     $LEFT_COUNT"
echo "  3D points:        $POINTS_COUNT"
echo "  Depth maps:       $DEPTH_COUNT"
echo "  Mesh vertices:    $VERTEX_COUNT"
echo "  Mesh faces:       $FACE_COUNT"
echo ""

