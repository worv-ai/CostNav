# Lab 0: 개념 기초 — 3DGS → 3DGRT → 3DGUT → 3DGRUT

## 선수 지식 (이미 알고 있는 것)

- 카메라 모델 (pinhole, fisheye, distortion)
- SE(3) 변환, rotation matrix, quaternion
- 3D point cloud, depth map
- COLMAP / SfM → sparse reconstruction
- EKF / UKF에서의 Unscented Transform

---

## 1. Point Cloud → 3D Gaussian

### SLAM에서의 표현

SLAM에서 3D 공간은 **point cloud**로 표현된다.
각 point는 위치 `(x, y, z)`만 가지며, 연속적인 surface를 표현하기 어렵다.

### 3D Gaussian Splatting의 표현

3DGS는 각 point를 **Gaussian particle**로 확장한다:

| 파라미터 | 기호 | 의미 |
|---------|------|------|
| Position | μ ∈ ℝ³ | 중심 위치 |
| Covariance | Σ ∈ ℝ³ˣ³ | 크기와 방향 (ellipsoid) |
| Density | σ ∈ ℝ | 불투명도 (opacity) |
| Color | c | Spherical Harmonics로 view-dependent |

**SLAM과의 대응:**

```
SLAM:  landmark = (μ, Σ)         → 위치 불확실성
3DGS:  particle = (μ, Σ, σ, c)  → 공간 점유 + 외관
```

### Gaussian Response Function

```
ρ(x) = exp(-½ (x - μ)ᵀ Σ⁻¹ (x - μ))
```

이것은 **Mahalanobis distance**의 지수 함수이다.
SLAM에서 chi-squared test로 outlier를 판별할 때 쓰는 그 거리 공식과 동일하다:

```
d² = (x - μ)ᵀ Σ⁻¹ (x - μ)
```

---

## 2. Rendering 방식: Splatting vs Ray Tracing

### 방법 A: Splatting (3DGS, 3DGUT)

"각 Gaussian을 이미지 평면에 투영(project)한 후, 2D에서 합성"

1. 3D Gaussian → 카메라로 투영 → 2D Gaussian (타원)
2. 각 타원이 영향을 주는 pixel tile 계산
3. depth 순서로 alpha compositing

**SLAM 비유**: EKF에서 state를 measurement space로 변환하는 것

```
z = h(x)
Jacobian: H = ∂h/∂x
투영된 covariance: P_z = H Pₓ Hᵀ   ← 이것이 EWA splatting!
```

### 방법 B: Ray Tracing (3DGRT)

"각 pixel에서 ray를 쏘고, ray가 만나는 Gaussian들의 response를 적분"

1. pixel → ray 생성: `r(τ) = o + τd`
2. BVH로 ray가 만나는 Gaussian들 탐색
3. 각 Gaussian의 **ray 위 최대 response 지점** (τ_max) 계산
4. front-to-back alpha compositing

**SLAM 비유**: ray casting으로 occupancy grid를 업데이트하는 것

### 핵심 차이

| | Splatting (3DGS) | Ray Tracing (3DGRT) |
|---|---|---|
| Response 계산 위치 | **2D** (투영 후 image plane) | **3D** (ray 위 τ_max 지점) |
| 카메라 모델 | Pinhole만 (Jacobian 필요) | 임의의 모델 |
| Secondary ray | 불가 | 가능 (반사, 굴절) |

같은 Gaussian이라도 2D/3D 계산 결과가 미묘하게 다르다. (Lab 2에서 직접 확인)

---

## 3. SLAM의 UT → 3DGUT의 UT

이것이 **가장 핵심적인 연결고리**이다.

### UKF에서의 Unscented Transform

**문제**: nonlinear function `h(x)`를 통과한 후의 분포를 알고 싶다.

**EKF 방식** (Jacobian 선형화):
```
E[y] ≈ h(μ)
Cov[y] ≈ H Σ Hᵀ      (H = Jacobian at μ)
```

**UKF 방식** (Unscented Transform):
```
1. σ-points 생성: χᵢ = μ ± √((n+λ)Σ)
2. 각 σ-point를 h()에 통과: γᵢ = h(χᵢ)
3. 결과의 mean/cov 추정:
   E[y] = Σ wᵢ γᵢ
   Cov[y] = Σ wᵢ (γᵢ - E[y])(γᵢ - E[y])ᵀ
```

### 3DGUT에서의 Unscented Transform

**문제**: 3D Gaussian을 **distorted camera**를 통해 2D로 투영하고 싶다.

**EWA 방식** (= EKF):
```
Σ_2d = J Σ_3d Jᵀ      (J = 카메라 projection의 Jacobian)
→ Jacobian 유도 필요 → pinhole만 가능
```

**3DGUT 방식** (= UKF):
```
1. 3D Gaussian에서 7개 σ-points 생성
2. 각 σ-point를 카메라 모델로 정확히 투영 (fisheye든 뭐든)
3. 투영된 점들로 2D mean & covariance 추정
→ Jacobian 불필요 → 임의의 카메라 모델 가능
```

### 정확도 비교

| | 정확도 | 카메라 지원 |
|---|---|---|
| EKF / EWA | 1차 (Jacobian) | Pinhole만 |
| UKF / 3DGUT | **2차 이상** (σ-points) | **임의** (fisheye, rolling shutter) |

**Pinhole 카메라**: EKF ≈ UKF (거의 같음) → 그래서 원래 3DGS가 잘 동작했음

**Fisheye 카메라**: EKF ≠ UKF (차이 큼!) → 3DGUT가 필요한 이유

---

## 4. 3DGRUT = 3DGRT + 3DGUT (Hybrid)

| 방법 | 렌더링 | 장점 | 단점 |
|------|--------|------|------|
| 3DGRT | Ray Tracing (OptiX) | Secondary ray, 정확 | RT core 필요, 느림 |
| 3DGUT | Rasterization (UT) | 빠름, GPU 범용 | Secondary ray 불가 |
| **3DGRUT** | **Hybrid** | **둘 다** | 복잡 |

3DGRUT 렌더링 흐름:
1. Primary ray → **3DGUT** (rasterization, 빠르게)
2. Secondary ray (반사, 굴절) → **3DGRT** (ray tracing, 정확하게)

---

## 실습 코드에서 확인할 것

`code.py`를 실행하면 3개의 그래프가 생성된다:

1. **fig1**: Point Cloud vs Gaussian 표현, response heatmap
2. **fig2**: Splatting vs Ray Tracing 방식 비교도
3. **fig3**: EKF vs UT projection — Pinhole과 Fisheye에서의 정확도 차이

특히 fig3에서 **Fisheye 카메라에서 UT(빨간)가 EKF(파란)보다 MC ground truth(초록)에
가까운 것**을 직접 확인하라. 이것이 3DGUT의 존재 이유이다.
