# Lab 6: 실제 3DGRUT 파이프라인

## 1. End-to-End 파이프라인

```
ZED ROS2 bag (.db3)
  │  Step 1: extract_zed_bag.py
  ▼
stereo images + frames_meta.json
  │  Step 2: cusfm_cli (CUDA SfM)
  ▼
camera poses + sparse 3D points
  │  Step 3: run_depth.py (FoundationStereo)
  ▼
depth maps (16-bit PNG, mm)
  │  Step 4: fuse_cusfm (nvblox)
  ▼
mesh.ply + occupancy_map
  │  Step 5: train.py (3DGRUT)  ← 이 Lab의 핵심
  ▼
Gaussian Scene (checkpoint .pt)
  │  Step 6: render.py
  ▼
Novel View Images
```

### SLAM과의 대응

| Step | 3DGRUT Pipeline | SLAM 대응 |
|------|-----------------|-----------|
| 1-2 | Feature extraction → SfM | Visual SLAM front-end |
| 3-4 | Stereo depth → TSDF fusion | Dense reconstruction |
| 5 | Gaussian fitting | Neural scene representation |
| 6 | Novel view synthesis | Map visualization |

---

## 2. Config 시스템 (Hydra)

### 구조

```
configs/
├── base_gs.yaml              # 기본 설정 (모든 app이 상속)
├── apps/                     # 완전한 실행 config
│   ├── colmap_3dgrt.yaml     # COLMAP + Ray Tracing
│   ├── colmap_3dgut.yaml     # COLMAP + Rasterization (UT)
│   └── cusfm_3dgut.yaml     # ZED pipeline + 3DGUT
├── dataset/                  # 데이터셋 형식
├── render/                   # 렌더링 백엔드
├── strategy/                 # Densification 전략
└── initialization/           # Gaussian 초기화 방법
```

### 핵심 파라미터 (base_gs.yaml)

| 파라미터 | 기본값 | 의미 |
|---------|-------|------|
| n_iterations | 30000 | 총 학습 반복 |
| loss.lambda_l1 | 0.8 | L₁ loss 가중치 |
| loss.lambda_ssim | 0.2 | SSIM loss 가중치 |
| optimizer.lr.position | 1.6e-4 | Position 학습률 |
| optimizer.lr.density | 5e-2 | Opacity 학습률 |
| strategy.densify_interval | 300 | Densification 주기 |

### 3DGUT 전용 파라미터

| 파라미터 | 기본값 | 의미 |
|---------|-------|------|
| splat.ut_alpha | 1.0 | UT α (sigma point spread) |
| splat.ut_beta | 2.0 | UT β (Gaussian prior) |
| splat.ut_kappa | 0.0 | UT κ (secondary scaling) |
| splat.k_buffer_size | 0 | MLAB buffer (0=unsorted) |
| particle_kernel_degree | 2 | Gaussian kernel degree |

---

## 3. 실행 가이드

### 방법 A: NeRF Synthetic (가장 간단)

```bash
cd /opt/3dgrut
conda activate 3dgrut

python train.py --config-name apps/nerf_synthetic_3dgrt.yaml \
    path=data/nerf_synthetic/lego \
    out_dir=runs \
    experiment_name=lego_3dgrt
```

### 방법 B: COLMAP 데이터

필요한 구조:
```
scene/
  images/               # 입력 이미지
  sparse/0/             # COLMAP 결과
    cameras.bin
    images.bin
    points3D.bin
```

```bash
python train.py --config-name apps/colmap_3dgut.yaml \
    path=data/my_scene \
    out_dir=runs \
    experiment_name=my_scene_3dgut \
    dataset.downsample_factor=2
```

### 방법 C: ZED Pipeline (이 repo)

```bash
# Step 1-4
cd /workspace
./scripts/zed_pipeline/run_zed_pipeline.sh \
    data/recording.db3 output/scene \
    --start-time 0 --end-time 30 --skip 5

# Step 5
cd /opt/3dgrut
python train.py --config-name apps/cusfm_3dgut.yaml \
    path=output/scene/extracted \
    out_dir=runs \
    experiment_name=zed_scene \
    initialization.fused_point_cloud_path=output/scene/nvblox/mesh.ply
```

---

## 4. 코드베이스 탐색 순서

```
1단계: 전체 흐름
  /opt/3dgrut/train.py                     # 진입점
  /opt/3dgrut/threedgrut/trainer.py        # 학습 루프

2단계: 모델
  /opt/3dgrut/threedgrut/model/model.py    # MixtureOfGaussians

3단계: 데이터
  /opt/3dgrut/threedgrut/datasets/         # COLMAP, NeRF loader

4단계: 전략
  /opt/3dgrut/threedgrut/strategy/         # GS, MCMC densification

5단계: CUDA (고급)
  /opt/3dgrut/threedgrt_tracer/            # OptiX ray tracing
  /opt/3dgrut/threedgut_tracer/            # UT rasterization
```

---

## 5. 실험 아이디어

| # | 실험 | 비교 대상 | 관찰 포인트 |
|---|------|----------|------------|
| 1 | Initialization | random vs COLMAP vs nvblox | 수렴 속도 |
| 2 | Renderer | 3DGRT vs 3DGUT | PSNR, 속도, 메모리 |
| 3 | Densification | GS vs MCMC | Gaussian 수, PSNR |
| 4 | UT 파라미터 | α=0.001 vs 1.0 | Fisheye에서의 차이 |
| 5 | Kernel degree | 1, 2, 4 | 속도, PSNR |
| 6 | SH degree | 0 only vs 0~3 | Specular 표현 |
| 7 | Loss 조합 | L₁ vs L₂ vs L₁+SSIM | 수렴, edge 보존 |

---

## 6. 트러블슈팅

| 문제 | 해결 |
|------|------|
| CUDA OOM | `dataset.downsample_factor=2` 또는 4 |
| Slang 컴파일 느림 | 첫 실행만. 이후 캐시됨 |
| COLMAP format 오류 | cameras.bin, images.bin, points3D.bin 확인 |
| 결과 흐릿 | n_iterations 늘리기 (50000) |
| 구멍 발생 | densify_grad_threshold 낮추기 (0.0001) |
