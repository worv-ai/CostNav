# 3DGRUT Anti-Ghosting 학습 파이프라인

> **최종 수정**: 2026-02-26
> **데이터셋**: ZED 카메라 COLMAP 재구성
> **클러스터**: DGX-H100 (SLURM, Docker)

---

## 0. 전체 흐름

```
[데이터 준비] → [COLMAP 재구성] → [Mesh 생성 (optional)] → [실험 제출] → [학습 (Docker)] → [결과 확인] → [로컬 다운로드]
```

---

## 1. 데이터 준비

### 1-1. ZED 카메라 rosbag → 이미지 추출

```bash
bash scripts/run-step1-rosbag-convert.sh
```

### 1-2. COLMAP 이미지 준비

```bash
python scripts/prepare_colmap_images.py
```

### 1-3. cuSFM (COLMAP) 실행 → sparse reconstruction

```bash
bash scripts/run-step2-cusfm.sh
```

- 출력: `{COLMAP_PATH}/sparse/0/` (cameras, images, points3D)

### 1-4. (선택) nvblox mesh 생성

```bash
bash scripts/run-step4-nvblox.sh
```

- 출력: `mesh.ply` → 학습 시 초기 Gaussian 포인트로 사용 가능
- mesh 없이도 학습 가능 (COLMAP sparse points로 초기화)

### 데이터셋별 경로

| 데이터셋 | COLMAP 경로 | Mesh | 이미지 수 |
|---------|-----------|------|---------|
| zed_nurec_3dgrut (round1) | `/mnt/harbor/users/samwoo/zed_trial_3/zed_nurec_3dgrut` | `dense_colored.ply` | 149장 |
| zed_front_short | `/mnt/harbor/users/samwoo/zed_trial_3/zed_front_short_colmap` | 없음 | 11장 |
| zed_pipeline_test | `/mnt/harbor/users/samwoo/zed_trial_3/zed_pipeline_test_0221_1702_colmap` | 없음 | 15장 |
| **zed_pipeline_round2** | `/mnt/harbor/users/samwoo/zed_trial_3/zed_pipeline_round_2026_0222_colmap` | `mesh.ply` | **54장** |

---

## 2. 실험 제출 (SLURM)

### 2-1. 파일 구조

```
scripts/slurm_3dgrut_antighost/
├── submit_experiments_round2.sh     # ★ 실험 제출 스크립트 (round2, 현재 사용)
├── submit_experiments.sh            # round1 제출 스크립트
├── submit_experiments_front_short.sh # front_short 데이터셋 제출
├── submit_experiments_pipeline_test.sh # pipeline_test 데이터셋 제출
├── slurm_3dgrut_antighost.sbatch    # SLURM job 스크립트
├── main_3dgrut_antighost.docker.sh  # Docker compose 실행
├── train_3dgrut_antighost.sh        # Docker 내부 학습 스크립트
├── docker/
│   └── docker-compose-3dgrut-antighost.yml
├── EXPERIMENT_DESIGN.md             # 33개 실험 설계 문서
├── logs/                            # SLURM job 로그
└── render_samples/                  # 렌더 결과 이미지
```

### 2-2. 실행 흐름

```
submit_experiments_round2.sh
  └→ sbatch slurm_3dgrut_antighost.sbatch (×33 실험)
       └→ main_3dgrut_antighost.docker.sh
            └→ docker compose up (3dgrut 이미지)
                 └→ train_3dgrut_antighost.sh
                      └→ python train.py (3DGRUT 학습)
```

### 2-3. 제출 명령

```bash
# 전체 33개 실험 제출
bash scripts/slurm_3dgrut_antighost/submit_experiments_round2.sh

# 미리보기 (dry-run)
bash scripts/slurm_3dgrut_antighost/submit_experiments_round2.sh --dry-run

# 특정 실험만
bash scripts/slurm_3dgrut_antighost/submit_experiments_round2.sh --only gs_best mcmc_best

# 특정 그룹만
bash scripts/slurm_3dgrut_antighost/submit_experiments_round2.sh --group best antighost
```

### 2-4. SLURM 리소스

- **GPU**: H100 × 1 (per job)
- **CPU**: 16 cores
- **Memory**: 128GB
- **Time**: 16시간
- **노드**: DGX-H100-1,2,3 고정 (3노드 × 8GPU = 최대 24 동시 실행)

---

## 3. 학습 모니터링

```bash
# 작업 상태 확인
squeue -u $USER

# 실시간 모니터링
watch -n 10 squeue -u $USER

# 로그 확인
tail -f scripts/slurm_3dgrut_antighost/logs/r2_*.out

# 특정 실험 로그
tail -f scripts/slurm_3dgrut_antighost/logs/r2_gs_best.*.out
```

---

## 4. 결과 확인

### 4-1. 서버 내 출력 구조

```
/mnt/harbor/users/samwoo/3dgrut_output_round2/
└── gs_best_20260222_143000/
    └── gs_best_20260222_143000/
        ├── ckpt_last.pt          # 체크포인트
        ├── export_last.ply       # Gaussian PLY
        ├── export_last.usdz      # USDZ (AR 뷰어용)
        ├── export_last.ingp      # INGP format
        ├── metrics.json          # PSNR/SSIM/LPIPS 메트릭
        ├── parsed.yaml           # 사용된 설정값
        ├── ours_{iter}/          # 체크포인트별 렌더 결과
        │   ├── renders/          # 렌더링 이미지
        │   └── gt/               # Ground Truth 이미지
        └── events.out.tfevents.* # TensorBoard 로그
```

### 4-2. 메트릭 확인

```bash
# 단일 실험 메트릭
cat /mnt/harbor/users/samwoo/3dgrut_output_round2/gs_best_*/*/metrics.json | python -m json.tool

# 전체 실험 메트릭 요약 (예시)
for d in /mnt/harbor/users/samwoo/3dgrut_output_round2/*/; do
  name=$(basename "$d")
  metrics=$(find "$d" -name "metrics.json" -exec cat {} \; 2>/dev/null)
  [ -n "$metrics" ] && echo "$name: $metrics"
done
```

---

## 5. 로컬로 결과 다운로드

### 5-1. 렌더링 이미지 가져오기 (scp)

```bash
# 로컬 머신에서 실행
# 특정 실험의 렌더 이미지 다운로드
scp -r samwoo@server:/mnt/harbor/users/samwoo/3dgrut_output_round2/gs_best_*/*/ours_*/renders/ ./render_samples_round2/gs_best/

# 전체 실험 렌더 결과 다운로드
for exp in gs_best mcmc_best gs_sorted mcmc_sorted; do
  mkdir -p render_samples_round2/$exp
  scp -r samwoo@server:/mnt/harbor/users/samwoo/3dgrut_output_round2/${exp}_*/*/ours_*/renders/ ./render_samples_round2/$exp/
done
```

### 5-2. Ground Truth 이미지 가져오기

```bash
scp -r samwoo@server:/mnt/harbor/users/samwoo/3dgrut_output_round2/gs_best_*/*/ours_*/gt/ ./render_samples_round2/_ground_truth/
```

### 5-3. USDZ 파일 (AR 뷰어)

```bash
scp samwoo@server:/mnt/harbor/users/samwoo/3dgrut_output_round2/gs_best_*/*/export_last.usdz ./
```

### 5-4. 로컬 렌더 결과 저장 위치

```
scripts/render_samples_round2/          # round2 렌더 결과
├── _ground_truth/                      # GT 이미지
├── gs_best/                            # gs_best 렌더링
├── mcmc_best/                          # mcmc_best 렌더링
├── gs_sorted/                          # ...
└── ...
```

---

## 6. gs_best 파라미터 요약 (Best Practice)

```
• Config: apps/cusfm_3dgut (GS)
• Iterations: 75,000
• Position LR: 0.0001 → 1.00e-06
• Max Gaussians: 350,000
• Test Split Interval: 6 (test ~9장, train ~45장)
• Mesh 초기화: nvblox/mesh.ply

Anti-Ghosting:
• USE_SCALE_LOSS: true (λ_scale = 0.1)
• USE_OPACITY_LOSS: true (λ_opacity = 0.01, threshold = 0.01)

Extra Args (best practice 조합):
• scheduler.positions.max_steps = 75000
• render.splat.k_buffer_size = 32 (sorted rendering)
• loss.lambda_l1 = 0.6
• loss.lambda_ssim = 0.4
• strategy.densify.clone_grad_threshold = 0.0001 (loose)
• strategy.densify.split_grad_threshold = 0.0001 (loose)
```

---

## 7. Legacy 파일

`scripts/legacy/`로 이동된 파일들:

| 폴더/파일 | 설명 |
|----------|------|
| `slurm_3dgrut_only/` | antighost 이전 초기 3dgrut 학습 스크립트 (backup 파일 포함) |
| `slurm_3dgrut_sweep/` | 파라미터 sweep 런처 (미사용) |
| `3dgut_slurm_90339/` | job #90339 출력 덤프 (초기 테스트) |
| `output/` | 과거 job 출력들 (90061~90071, 90287, 90349) |
| `3dgrut_antighost.90349.*` | 초기 antighost 단건 테스트 로그 |
| `3dgrut_antighost.90364.*` | 초기 antighost 단건 테스트 로그 |
| `render_samples_no_ply/` | round1 no-ply 모드 렌더 결과 |
| `render_samples_front_short/` | front_short (11장) 데이터셋 렌더 결과 |

---

## 8. 참고

- 실험 설계 상세: [`EXPERIMENT_DESIGN.md`](./EXPERIMENT_DESIGN.md)
- Docker 이미지: `docker-pri.maum.ai:443/worv/3dgrut:latest`
- 3DGRUT 코드: Docker 이미지 내 `/opt/3dgrut/`
- Conda 환경: `3dgrut` (Docker 이미지 내 `/opt/conda/`)
