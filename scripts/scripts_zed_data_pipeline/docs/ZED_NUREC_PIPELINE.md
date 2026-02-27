# ZED Camera NuRec Pipeline

ZED 스테레오 카메라 ROS2 bag 데이터를 사용하여 3D reconstruction을 수행하는 파이프라인입니다.

## Pipeline Overview

```
ROS2 Bag (ZED) → Extract → cuSFM → FoundationStereo → nvblox → Mesh
```

| Step | 도구 | 입력 | 출력 |
|------|------|------|------|
| 1 | `extract_zed_bag.py` | ROS2 bag | Stereo images + frames_meta.json |
| 2 | `cusfm_cli` | Images + meta | Camera poses + 3D points |
| 3 | `run-step3-depth-v2.py` | Stereo pairs | Depth maps (16-bit PNG) |
| 4 | `fuse_cusfm` | Color + Depth + Poses | 3D Mesh (PLY) |

---

## Step 1: ROS2 Bag에서 ZED 이미지 추출

### 명령어
```bash
python3 scripts/zed_pipeline/extract_zed_bag.py \
    /path/to/rosbag \
    /output/dir \
    --start-time 38 \
    --end-time 48 \
    --skip 5
```

### 파라미터
| 파라미터 | 설명 | 예시 |
|----------|------|------|
| `bag_path` | ROS2 bag 경로 (db3 또는 디렉토리) | `/data/zed.db3` |
| `output_dir` | 출력 디렉토리 | `/output/extracted` |
| `--start-time` | 시작 시간 (초) | `38` |
| `--end-time` | 종료 시간 (초) | `48` |
| `--skip` | N번째 프레임만 추출 | `5` |

### 출력 구조
```
output_dir/
├── zed_left/           # Left camera images (JPEG)
├── zed_right/          # Right camera images (JPEG)
├── frames_meta.json    # cuSFM 형식 메타데이터
├── stereo.edex         # Stereo calibration
└── frame_metadata.jsonl
```

---

## Step 2: cuSFM 포즈 추정

### 명령어
```bash
cusfm_cli \
    --input_dir /output/extracted \
    --cusfm_base_dir /output/cusfm \
    --min_inter_frame_distance 0.06 \
    --min_inter_frame_rotation_degrees 1.5
```

### 파라미터
| 파라미터 | 설명 | 기본값 |
|----------|------|--------|
| `--input_dir` | Step 1 출력 디렉토리 | - |
| `--cusfm_base_dir` | cuSFM 출력 디렉토리 | - |
| `--min_inter_frame_distance` | 최소 이동 거리 (m) | 0.06 |
| `--min_inter_frame_rotation_degrees` | 최소 회전 각도 (°) | 1.5 |

### 출력 구조
```
cusfm/
├── sparse/
│   ├── cameras.txt     # Camera intrinsics (COLMAP format)
│   ├── images.txt      # Camera poses
│   └── points3D.txt    # 3D points
├── keyframes/
│   └── frames_meta.json  # Keyframe metadata with poses
└── output_poses/
    └── camera_name-zed_left_pose_file.tum  # Poses in TUM format
```

---

## Step 3: FoundationStereo Depth 추정

### 명령어
```bash
python3 scripts/zed_pipeline/run_depth.py \
    --image_dir /output/extracted \
    --output_dir /output/depth \
    --frames_meta_file /output/cusfm/keyframes/frames_meta.json \
    --verbose
```

### 파라미터
| 파라미터 | 설명 |
|----------|------|
| `--image_dir` | Color image 디렉토리 (Step 1 출력) |
| `--output_dir` | Depth 출력 디렉토리 |
| `--frames_meta_file` | cuSFM keyframes metadata |
| `--scale` | 이미지 스케일 (기본: 1.0) |
| `--verbose` | 상세 출력 |

### 출력 구조
```
depth/
└── zed_left/
    ├── {timestamp}.png  # 16-bit depth (uint16, mm 단위)
    └── ...
```

### Depth 계산
```
depth_mm = (1000 * focal_length * baseline) / disparity
```
- ZED Baseline: **0.12m**
- Focal Length: **361.47px** (960x600 해상도 기준)

---

## Step 4: nvblox Mesh 생성

### 명령어
```bash
export LD_LIBRARY_PATH=/opt/nvblox/build/nvblox:$LD_LIBRARY_PATH

/opt/nvblox/build/nvblox/executables/fuse_cusfm \
    --color_image_dir /output/extracted \
    --depth_image_dir /output/depth \
    --frames_meta_file /output/cusfm/keyframes/frames_meta.json \
    --mesh_output_path /output/nvblox/mesh.ply \
    --save_2d_occupancy_map_path /output/nvblox/occupancy_map \
    --mapping_type_dynamic \
    --voxel_size 0.025 \
    --projective_integrator_max_integration_distance_m 2.0
```

### 파라미터
| 파라미터 | 설명 | 권장값 |
|----------|------|--------|
| `--voxel_size` | Voxel 크기 (m) | 0.025 (2.5cm) |
| `--projective_integrator_max_integration_distance_m` | 최대 depth 거리 | 2.0m |
| `--mapping_type_dynamic` | Dynamic mapping 모드 | - |

### 출력 구조
```
nvblox/
├── mesh.ply            # 3D Mesh (PLY format)
├── occupancy_map.png   # 2D Occupancy map
└── occupancy_map.yaml  # Occupancy map metadata
```

---

## Quick Start

### 전체 파이프라인 실행
```bash
./scripts/zed_pipeline/run_zed_pipeline.sh \
    /path/to/rosbag \
    /output/dir \
    --start-time 38 \
    --end-time 48 \
    --skip 5
```

### 예시 (테스트 데이터)
```bash
./scripts/zed_pipeline/run_zed_pipeline.sh \
    /workspace/data/2026_02_20_depth_adjusted/zed_nurec_compressed_adjusted_depth_point_cloud \
    /workspace/output/zed_test \
    --start-time 38 \
    --end-time 52 \
    --skip 8
```

