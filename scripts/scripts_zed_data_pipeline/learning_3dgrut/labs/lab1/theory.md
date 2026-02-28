# Lab 1: 3D Gaussian 표현 심화 — Quaternion, SH, Opacity

## 1. Covariance 분해: Σ = R S² Rᵀ

### 왜 Covariance를 직접 저장하지 않는가?

3×3 symmetric PSD matrix는 6개 자유 파라미터가 있지만,
직접 최적화하면 **positive semi-definite 보장이 안 된다**.

대신 `(quaternion q, scale s)`를 저장하고 필요할 때 Σ를 계산한다:

```
R = quat_to_matrix(q)     # 3×3 rotation (3 DOF)
S = diag(s)                # 3×3 diagonal scale (3 DOF)
Σ = R S² Rᵀ               # 항상 PSD 보장!
```

**Local → World 변환**:
```
x_world = R · S · x_local + μ
```

로컬 좌표계에서 unit sphere → 월드에서 ellipsoid가 된다.

**SLAM 비유**: IMU preintegration에서 rotation을 quaternion으로
저장하여 numerical stability를 확보하는 것과 같은 원리.

---

## 2. Quaternion ↔ Rotation Matrix

### Quaternion (w, x, y, z), ‖q‖ = 1

```
        ⎡ 1-2(y²+z²)    2(xy-wz)     2(xz+wy)  ⎤
R(q) =  ⎢  2(xy+wz)   1-2(x²+z²)    2(yz-wx)  ⎥
        ⎣  2(xz-wy)    2(yz+wx)    1-2(x²+y²)  ⎦
```

SLAM에서 pose를 다룰 때 쓰는 것과 동일한 공식.
3DGRUT 코드에서도 이 변환을 사용한다.

---

## 3. Spherical Harmonics (SH) — View-Dependent Color

### 문제

현실의 물체는 보는 각도에 따라 색이 다르다:
- 금속의 specular highlight
- 물의 반사
- 차체의 glossy reflection

### SH 표현

구면 위의 함수를 주파수 분해한다:

```
f(θ, φ) = Σₗ Σₘ cₗᵐ Yₗᵐ(θ, φ)
```

3DGRUT에서는 `l_max = 3`:

| Degree l | 계수 개수 | 역할 |
|----------|----------|------|
| l=0 | 1 | DC (기본 색) — 어디서 봐도 같은 색 |
| l=1 | 3 | 방향성 — 한쪽이 밝고 반대가 어두움 |
| l=2 | 5 | 더 복잡한 패턴 |
| l=3 | 7 | Specular highlight |
| **합계** | **16 × 3(RGB)** | **= 48 파라미터** |

### SH Basis Functions (처음 몇 개)

```
Y₀⁰ = 0.2821           (상수)
Y₁⁻¹ = 0.4886 · y       (y 방향)
Y₁⁰ = 0.4886 · z        (z 방향)
Y₁¹ = 0.4886 · x        (x 방향)
Y₂⁰ = 0.3154 · (3z²-1)  (z축 대칭)
...
```

여기서 `(x, y, z)`는 **viewing direction** (단위벡터).

### Color 계산

```
color = sigmoid( Σ cₗᵐ · Yₗᵐ(view_dir) )
```

sigmoid로 [0, 1] 범위를 보장한다.

### Progressive SH

3DGRUT는 학습 중에 SH degree를 점진적으로 올린다:

```
iter    0 ~ 1000: degree 0만 (DC 색만 학습)
iter 1000 ~ 2000: degree 0~1
iter 2000 ~ 3000: degree 0~2
iter 3000 ~     : degree 0~3 (전체)
```

낮은 주파수(기본 색)를 먼저 맞추고, 점차 고주파(specular)를 추가한다.

---

## 4. Opacity (Density)

### 정의

```
opacity = sigmoid(σ_raw)     # σ_raw는 학습 파라미터 (unconstrained)
```

sigmoid로 (0, 1) 범위를 보장한다.

### 역할

- opacity ≈ 1 → 불투명 (빛을 완전히 차단)
- opacity ≈ 0 → 투명 (빛이 통과)

Volume rendering에서 각 particle의 기여:

```
α = opacity × ρ(x)     (opacity × Gaussian response)
```

### Density Reset

3DGRUT는 학습 중 주기적으로 모든 particle의 density를 낮은 값으로 reset한다.
→ 쓸모없는 particle이 자연스럽게 pruning된다.

---

## 5. Generalized Gaussian Kernels

### 문제

표준 Gaussian은 꼬리(tail)가 길어서 많은 particle과 겹친다.
→ ray 하나가 만나는 particle 수가 많아져서 느리다.

### 해법: 고차 Gaussian

```
Standard (n=1):  ρ(x) = exp(-½ d²)           ← 일반 Gaussian
Degree 2:        ρ̂(x) = exp(-(½ d²)²)        ← 더 sharp한 경계
Degree 3:        ρ̂(x) = exp(-(½ d²)³)        ← 매우 sharp
```

여기서 `d² = (x-μ)ᵀ Σ⁻¹ (x-μ)` (Mahalanobis distance²)

### 효과

- 높은 degree → 더 sharp한 경계 → 각 ray가 만나는 particle 수 감소 → 빠름
- 3DGRUT 기본값: degree 2 (3DGUT), degree 4 (3DGRT)

---

## 연습 문제

1. `GaussianParticle` 클래스에 generalized kernel을 추가하라:
   `def response(self, x, degree=1)`

2. SH degree를 0→3까지 올리면서 색상 패턴이 어떻게 변하는지 설명하라.
   (Progressive SH의 직관)

3. 주어진 Covariance Σ에서 quaternion q와 scale s를 역으로 추출하라.
   힌트: `Σ = V Λ Vᵀ` (eigendecomposition) → `s = √Λ`, `R = V` → `q = mat_to_quat(R)`
