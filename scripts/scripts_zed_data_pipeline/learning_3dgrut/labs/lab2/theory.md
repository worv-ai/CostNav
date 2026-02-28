# Lab 2: Ray-Particle Intersection — 3DGRT의 핵심

## 1. Ray Parameterization

```
r(τ) = o + τd
```

- `o` : ray origin (camera center)
- `d` : ray direction (단위벡터)
- `τ` : ray parameter (depth)

각 pixel `(u, v)`에서의 ray direction:

```
d = normalize([ (u - cx)/fx,  (v - cy)/fy,  1.0 ])
```

---

## 2. Ray → Gaussian 로컬 좌표 변환

### 변환 공식

```
o_g = S⁻¹ Rᵀ (o - μ)     # 원점을 Gaussian 중심으로 이동 & 정규화
d_g = S⁻¹ Rᵀ d           # 방향도 같은 변환
```

변환 후 Gaussian은 **단위 구** (identity covariance)가 된다:

```
ρ(x_g) = exp(-½ ‖x_g‖²)
```

**SLAM 비유**: body frame → sensor frame 변환과 동일한 구조

```
p_sensor = R_sensor_body · p_body + t
```

---

## 3. τ_max 해석적 해 — 핵심 수식

### 유도

로컬 좌표계에서 ray 위의 점:

```
x_g(τ) = o_g + τ · d_g
```

Gaussian response along ray:

```
ρ(τ) = exp(-½ ‖o_g + τ · d_g‖²)
```

최대값을 찾기 위해 미분:

```
dρ/dτ = ρ(τ) × (-(o_g + τ · d_g)ᵀ d_g) = 0
```

ρ(τ) > 0 이므로:

```
(o_g + τ_max · d_g)ᵀ d_g = 0
o_gᵀ d_g + τ_max · d_gᵀ d_g = 0
```

### 최종 공식

```
         -o_gᵀ d_g
τ_max = ──────────
         d_gᵀ d_g
```

또는 월드 좌표계에서:

```
         (μ - o)ᵀ Σ⁻¹ d
τ_max = ─────────────────
            dᵀ Σ⁻¹ d
```

### 기하학적 의미

τ_max는 **ray에서 Gaussian 중심까지의 Mahalanobis 수선의 발**이다.

즉, ray 위에서 Gaussian 중심에 "Mahalanobis 의미로 가장 가까운" 점.

### 두 공식의 동치성 증명

로컬 공식에서 출발:

```
-o_gᵀ d_g / (d_gᵀ d_g)
= -(S⁻¹Rᵀ(o-μ))ᵀ(S⁻¹Rᵀd) / ((S⁻¹Rᵀd)ᵀ(S⁻¹Rᵀd))
= -(o-μ)ᵀ R S⁻² Rᵀ d / (dᵀ R S⁻² Rᵀ d)
= (μ-o)ᵀ R S⁻² Rᵀ d / (dᵀ R S⁻² Rᵀ d)
```

Σ = R S² Rᵀ 이므로 Σ⁻¹ = R S⁻² Rᵀ. 따라서:

```
= (μ-o)ᵀ Σ⁻¹ d / (dᵀ Σ⁻¹ d)    ✓
```

---

## 4. Response at τ_max

```
α = σ × ρ(o + τ_max · d)
  = σ × exp(-½ (r(τ_max) - μ)ᵀ Σ⁻¹ (r(τ_max) - μ))
```

로컬 좌표계에서:

```
ω_g² = ‖o_g + τ_max · d_g‖²
α = σ × exp(-½ ω_g²)
```

`ω_g²`는 ray에서 Gaussian 중심까지의 최소 Mahalanobis distance²이다.

---

## 5. 2D Evaluation vs 3D Evaluation

### 3DGS (2D evaluation)

```
1. μ_2d = π(μ)                     # 3D → 2D 투영
2. Σ_2d = J Σ Jᵀ                   # Jacobian으로 covariance 투영
3. α = σ × exp(-½ (p - μ_2d)ᵀ Σ_2d⁻¹ (p - μ_2d))    # 2D에서 계산
```

### 3DGRT (3D evaluation)

```
1. τ_max = (μ-o)ᵀ Σ⁻¹ d / (dᵀ Σ⁻¹ d)    # ray 위 최적 지점
2. x = o + τ_max · d                       # 3D 점
3. α = σ × exp(-½ (x - μ)ᵀ Σ⁻¹ (x - μ))   # 3D에서 계산
```

### 차이가 발생하는 경우

- **큰 Gaussian**: 중심에서 먼 부분에서 Jacobian 근사가 부정확
- **카메라에 가까운 Gaussian**: perspective distortion이 큼
- **회전된 Gaussian**: 3D 형태와 2D 투영 형태가 많이 다름

---

## 6. BVH (Bounding Volume Hierarchy)

### 문제

N개 Gaussian × M개 ray → O(N×M) → 너무 느림

### 해법

각 Gaussian을 bounding box로 감싸고 hierarchical tree로 구성:

```
         [Root]
        /      \
    [Node]    [Node]
    /   \     /   \
  [G1] [G2] [G3] [G4]
```

Ray가 tree를 traverse하며 겹치는 node만 탐색 → **O(log N) per ray**

### 3DGRT의 BVH

- NVIDIA OptiX의 **hardware RT core**를 사용 (HW 가속)
- 각 Gaussian을 **stretched icosahedron**으로 감싸기
- 꼭짓점 스케일: `√(2 ln(σ / α_min)) × S` (α_min = 0.01)
  - opacity가 1% 미만인 영역은 무시

**SLAM 비유**: ICP에서 k-d tree로 nearest neighbor를 찾는 것과 같은 원리.

### Hit Buffer (k=16)

하나의 ray에 대해 가장 가까운 **k=16개** particle만 유지한다.
insertion sort로 τ_max 기준 정렬.

---

## 연습 문제

1. **Generalized Gaussian (degree n)**에 대한 τ_max를 유도하라.
   `ρ̂(τ) = exp(-(½ ‖o_g + τ d_g‖²)ⁿ)`, `dρ̂/dτ = 0`을 풀어라.
   힌트: chain rule → n에 관계없이 τ_max가 같다!

2. Per-ray hit buffer (k=16)를 insertion sort로 구현하라.

3. 2D에서 simple BVH를 직접 구현하라:
   `build_bvh_2d(gaussians) → tree`, `traverse_bvh(ray, tree) → hit list`
