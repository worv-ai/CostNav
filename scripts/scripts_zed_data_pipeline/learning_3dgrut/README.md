# 3DGRUT Learning by Doing — SLAM 배경 기반 학습 자료

## 개요

NVIDIA 3DGRUT (3D Gaussian Ray-marching with Unified Transmittance)를
SLAM/Localization 배경 지식을 활용하여 learning-by-doing으로 학습하는 자료.

**핵심 연결고리**: SLAM에서 이미 사용하는 기술들이 3DGRUT의 기반이다.

| SLAM/Localization | 3DGRUT | Lab |
|---|---|---|
| Landmark (μ, Σ) | Gaussian Particle (μ, Σ, σ, SH) | 0, 1 |
| Mahalanobis distance | Gaussian Response ρ(x) | 1, 2 |
| Camera projection π(x) | Ray-Particle Intersection τ_max | 2 |
| Occupancy grid ray casting | Volume Rendering Integral | 3 |
| UKF Unscented Transform | 3DGUT Camera Projection | **4** |
| Bundle Adjustment | Differentiable Rendering | 5 |
| COLMAP / SfM | Data Pipeline Input | 6 |

## 파일 구조

```
labs/
├── lab0/                         # 개념 기초
│   ├── theory.md                 # 이론 (수식, SLAM 대응)
│   ├── exercise.py               # 실습 코드
│   └── results/                  # 실행 결과 (png)
├── lab1/                         # Gaussian 표현 심화
│   ├── theory.md
│   ├── exercise.py
│   └── results/
├── ...
└── lab6/                         # 실제 파이프라인
    ├── theory.md
    ├── exercise.py
    └── results/
```

각 Lab은 **theory.md를 먼저 읽고 → exercise.py를 실행**하는 순서로 진행.

## Lab 목록

```bash
conda activate 3dgrut
```

| Lab | 주제 | 실행 |
|-----|------|------|
| **0** | 개념 기초 — Point Cloud→Gaussian, Splatting vs Ray Tracing, EKF vs UT | `cd labs/lab0 && python exercise.py` |
| **1** | Gaussian 표현 — Quaternion, SH, Generalized Kernels | `cd labs/lab1 && python exercise.py` |
| **2** | Ray-Particle Intersection — τ_max 유도, 2D vs 3D eval, BVH | `cd labs/lab2 && python exercise.py` |
| **3** | Volume Rendering — Alpha Compositing, Early Termination | `cd labs/lab3 && python exercise.py` |
| **4** | **Unscented Transform** — UKF→3DGUT, 다양한 카메라 모델 | `cd labs/lab4 && python exercise.py` |
| **5** | Differentiable Rendering — PyTorch 최적화, Densification | `cd labs/lab5 && python exercise.py` |
| **6** | 실제 파이프라인 — Config, 실행 가이드, 실험 아이디어 | `cd labs/lab6 && python exercise.py` |

## 추천 학습 순서

### Phase 1: 직관 잡기 (Lab 0 → Lab 4)
UKF의 UT가 3DGUT의 핵심 혁신임을 확인

### Phase 2: 수학 구현 (Lab 1 → Lab 2 → Lab 3)
각 Lab의 연습 문제를 직접 풀기

### Phase 3: 최적화 체험 (Lab 5)
PyTorch로 Gaussian scene 최적화

### Phase 4: 실제 실행 (Lab 6)
3DGRUT 학습 파이프라인 실행

## 논문

1. **3DGRT** — SIGGRAPH Asia 2024: [arXiv:2407.07090](https://arxiv.org/abs/2407.07090)
2. **3DGUT** — CVPR 2025 Oral: [arXiv:2412.12507](https://arxiv.org/abs/2412.12507)
3. **3DGS** — SIGGRAPH 2023: Kerbl et al.

## 환경

```bash
conda activate 3dgrut    # PyTorch 2.10, CUDA, numpy, matplotlib
# 3DGRUT: /opt/3dgrut/
```
