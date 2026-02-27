# 3DGRUT Anti-Ghosting 학습 가이드

SLURM + Docker 환경에서 3DGRUT 학습 실험을 일괄 제출하는 방법을 설명합니다.

---

## 스크립트 구조

```
slurm_3dgrut_antighost/
├── submit_experiments_round2.sh       ← 실험 일괄 제출 (진입점)
├── slurm_3dgrut_antighost.sbatch      ← SLURM 배치 스크립트
├── main_3dgrut_antighost.docker.sh    ← Docker Compose 실행
├── train_3dgrut_antighost.sh          ← 컨테이너 내부 학습 스크립트
└── docker/
    └── docker-compose-3dgrut-antighost.yml
```

**실행 흐름:**
```
submit_experiments_round2.sh
  └─ sbatch slurm_3dgrut_antighost.sbatch   (SLURM 노드에서 실행)
       └─ main_3dgrut_antighost.docker.sh   (Docker Compose 구동)
            └─ train_3dgrut_antighost.sh    (컨테이너 내부 학습)
```

---

## 사전 요구사항

- SLURM 클러스터 접근 권한 (파티션: `h100`)
- Docker + NVIDIA Container Runtime
- 학습에 필요한 데이터:
  - **COLMAP 결과 디렉토리** (`sparse/0/` 또는 `sparse/` 하위 디렉토리 포함)
  - **Mesh PLY 파일** (선택, nvblox 출력 등): 없으면 COLMAP sparse points로 초기화

---

## 다른 사용자가 사용하기 위한 경로 수정

아래 **3개 파일**에서 경로를 본인 환경에 맞게 수정해야 합니다.

### 1. `submit_experiments_round2.sh`

```bash
# ★ 데이터 경로 수정 ★
COLMAP_PATH="/mnt/harbor/users/samwoo/..."   # ← 본인 COLMAP 경로로 변경
MESH_PATH="/mnt/harbor/users/samwoo/..."     # ← 본인 mesh.ply 경로로 변경 (없으면 ""로 설정)
OUTPUT_BASE="/mnt/harbor/users/samwoo/..."   # ← 출력 저장 경로로 변경

# ★ 노드 설정 수정 (필요 시) ★
ALLOWED_NODES="DGX-H100-1,DGX-H100-2,DGX-H100-3"  # ← 사용할 노드명으로 변경
EXCLUDE_NODES="DGX-H100-4,..."                       # ← 제외할 노드명으로 변경
```

### 2. `slurm_3dgrut_antighost.sbatch`

```bash
# ★ 스크립트 경로 수정 ★ (61번째 줄)
SCRIPT_DIR="/mnt/home/samwoo/real2sim-nurec/scripts/slurm_3dgrut_antighost"
# → 본인의 실제 절대 경로로 변경
# 예: SCRIPT_DIR="/mnt/home/yourname/real2sim-nurec/scripts/slurm_3dgrut_antighost"
```

### 3. `docker/docker-compose-3dgrut-antighost.yml` (필요 시)

```yaml
# Docker 이미지 변경이 필요한 경우
image: ${GRUT_IMAGE:-docker-pri.maum.ai:443/worv/3dgrut:latest}
# → 환경변수로 오버라이드: export GRUT_IMAGE=your-registry/3dgrut:tag
```

---

## 데이터 준비

학습에 필요한 데이터 디렉토리 구조:

```
<COLMAP_PATH>/
├── sparse/
│   └── 0/
│       ├── cameras.bin  (또는 cameras.txt)
│       ├── images.bin   (또는 images.txt)
│       └── points3D.bin (또는 points3D.txt)
└── images/              (학습 이미지)

<MESH_PATH>              (선택) mesh.ply 파일
```

> **Mesh 초기화**: `MESH_PATH`를 지정하면 nvblox 메쉬를 초기 포인트 클라우드로 사용합니다. 미지정 시 COLMAP sparse points를 PLY로 변환하여 사용합니다.

---

## 실험 제출 방법

### 기본 사용법

```bash
cd /path/to/slurm_3dgrut_antighost

# 전체 33개 실험 제출
bash submit_experiments_round2.sh

# 제출 없이 미리보기 (dry-run)
bash submit_experiments_round2.sh --dry-run

# 특정 실험만 제출
bash submit_experiments_round2.sh --only mcmc_mild gs_strong

# 특정 그룹만 제출
bash submit_experiments_round2.sh --group antighost best
```

### 노드/파티션 환경변수 오버라이드

```bash
# 파티션 변경
PARTITION=a100 bash submit_experiments_round2.sh

# 사용 노드 변경
ALLOWED_NODES="node1,node2" bash submit_experiments_round2.sh
```

---

## 실험 그룹 목록 (총 33개)

| 그룹 태그 | 실험 수 | 설명 |
|-----------|---------|------|
| `antighost` | 10 | Anti-Ghosting 강도 sweep (soft/mild/moderate/strong/aggressive × MCMC/GS) |
| `optimizer` | 2 | Adam optimizer 비교 (mcmc_adam, gs_adam) |
| `progressive` | 4 | SH (Spherical Harmonics) 점진적 학습 설정 (sh1/sh2/sh_fast/sh_slow) |
| `duration` | 4 | 학습 iteration 수 비교 (25k/75k × MCMC/GS) |
| `mcmc_tune` | 3 | MCMC 전략 튜닝 (noise_low/high, add_late) |
| `gs_tune` | 3 | GS densification 튜닝 (tight/loose/long) |
| `loss` | 3 | Loss 가중치 조정 (ssim_heavy, l1_heavy) |
| `render` | 2 | Sorted rendering (k_buffer_size=32) |
| `best` | 2 | 최적 조합 (75k iter + sorted + SSIM + 최적 densify) |

### 그룹별 실험 상세

<details>
<summary>GROUP A: Anti-Ghosting Sweep (10개)</summary>

| 실험명 | Config | Iters | PosLR | MaxGaussians | λ_scale | λ_opacity |
|--------|--------|-------|-------|--------------|---------|-----------|
| `mcmc_soft` | cusfm_3dgut_mcmc | 50k | 0.00014 | 500k | 0.008 | 0.005 |
| `mcmc_mild` | cusfm_3dgut_mcmc | 50k | 0.00012 | 400k | 0.01 | 0.01 |
| `mcmc_moderate` | cusfm_3dgut_mcmc | 50k | 0.0001 | 350k | 0.01 | 0.01 |
| `mcmc_strong` | cusfm_3dgut_mcmc | 50k | 0.0001 | 300k | 0.01 | 0.01 |
| `mcmc_aggressive` | cusfm_3dgut_mcmc | 50k | 0.00008 | 150k | 0.015 | 0.02 |
| `gs_soft` | cusfm_3dgut | 50k | 0.00014 | 500k | 0.008 | 0.005 |
| `gs_mild` | cusfm_3dgut | 50k | 0.00012 | 400k | 0.01 | 0.01 |
| `gs_moderate` | cusfm_3dgut | 50k | 0.0001 | 350k | 0.01 | 0.01 |
| `gs_strong` | cusfm_3dgut | 50k | 0.0001 | 300k | 0.01 | 0.01 |
| `gs_aggressive` | cusfm_3dgut | 50k | 0.00008 | 150k | 0.015 | 0.02 |

</details>

<details>
<summary>GROUP B~I: 나머지 실험 요약</summary>

| 실험명 | 그룹 | 핵심 설정 |
|--------|------|-----------|
| `mcmc_adam`, `gs_adam` | optimizer | `optimizer.type=adam` |
| `mcmc_sh1`, `mcmc_sh2` | progressive | `max_n_features=1/2` |
| `mcmc_sh_fast`, `mcmc_sh_slow` | progressive | `increase_frequency=150/600` |
| `mcmc_iter_short`, `gs_iter_short` | duration | 25k iterations |
| `mcmc_iter_long`, `gs_iter_long` | duration | 75k iterations |
| `mcmc_noise_low`, `mcmc_noise_high` | mcmc_tune | `noise_lr=30000/300000` |
| `mcmc_add_late` | mcmc_tune | `end_iteration=45000` |
| `gs_densify_tight`, `gs_densify_loose` | gs_tune | `grad_threshold=0.0004/0.0001` |
| `gs_densify_long` | gs_tune | `end_iteration=40000` |
| `mcmc_ssim_heavy`, `gs_ssim_heavy` | loss | `λ_l1=0.6, λ_ssim=0.4` |
| `mcmc_l1_heavy` | loss | `λ_l1=0.95, λ_ssim=0.05` |
| `mcmc_sorted`, `gs_sorted` | render | `k_buffer_size=32` |
| `mcmc_best`, `gs_best` | best | 75k + sorted + SSIM + 최적 전략 |

</details>

---

## Anti-Ghosting 파라미터 설명

두겹/흔들림(ghosting) 현상 완화를 위한 핵심 설정:

| 파라미터 | 기본값 | 효과 |
|----------|--------|------|
| `USE_SCALE_LOSS=true` + `LAMBDA_SCALE` | 0.01~0.015 | 지나치게 큰 Gaussian blob 억제 |
| `USE_OPACITY_LOSS=true` + `LAMBDA_OPACITY` | 0.005~0.02 | 반투명 Gaussian 페널티 |
| `OPACITY_THRESHOLD` | 0.01 | 이 값 미만 opacity Gaussian 제거 (기본 0.005보다 높음) |
| `POSITION_LR` | 0.00008~0.00014 | 낮을수록 안정적 수렴 (기본 0.00016보다 낮음) |
| `MAX_GAUSSIANS` | 150k~500k | 줄이면 overfitting/ghosting 방지 (기본 2M보다 낮음) |

---

## 작업 모니터링

```bash
# 내 작업 상태 확인
squeue -u $USER

# 실시간 모니터링 (10초 갱신)
watch -n 10 squeue -u $USER

# 특정 실험 로그 확인
tail -f logs/r2_<실험명>.<job_id>.out

# 전체 로그 스트리밍
tail -f logs/r2_*.out

# 작업 취소
scancel <job_id>
scancel -u $USER  # 내 전체 작업 취소
```

---

## 출력 결과

각 실험의 결과는 `OUTPUT_BASE/<실험명>_<타임스탬프>/` 에 저장됩니다.

```
<OUTPUT_BASE>/
└── mcmc_mild_20260227_120000/
    ├── *.ply          (3D Gaussian PLY 파일)
    ├── *.usdz         (USDZ 내보내기)
    └── checkpoints/   (학습 체크포인트)
```

체크포인트는 학습 iteration의 14%, 60%, 100% 시점에 저장됩니다.

---

## 자주 겪는 문제

**Q: `sbatch: error: job submit allocation failed`**
- SLURM 파티션 이름 확인: `PARTITION=h100` 또는 `sinfo`로 파티션 목록 확인

**Q: `Error: COLMAP path not found`**
- `COLMAP_PATH` 경로가 올바른지, `sparse/0/` 디렉토리가 존재하는지 확인

**Q: Docker 이미지 pull 실패**
- `GRUT_IMAGE` 환경변수로 접근 가능한 레지스트리의 이미지 지정

**Q: GPU가 여러 작업에서 중복 사용됨**
- `slurm_3dgrut_antighost.sbatch`의 `SCRIPT_DIR`이 본인 절대경로로 설정되었는지 확인
- `NVIDIA_VISIBLE_DEVICES`는 SLURM의 `SLURM_JOB_GPUS`를 사용하므로 자동으로 격리됨
