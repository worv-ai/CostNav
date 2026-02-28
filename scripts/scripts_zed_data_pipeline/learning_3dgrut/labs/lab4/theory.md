# Lab 4: Unscented Transform — UKF에서 3DGUT으로

> 이 Lab이 SLAM 배경자에게 가장 핵심적인 Lab이다.
> 이미 알고 있는 UKF의 UT가 3DGUT의 핵심 혁신임을 확인한다.

## 1. EWA Splatting의 한계 (= EKF의 한계)

### EWA Splatting 공식

3D Gaussian을 2D로 투영:

```
μ_2d = π(μ_3d)              # nonlinear projection
Σ_2d = J Σ_3d Jᵀ            # linear approximation!
```

여기서 `J = ∂π/∂x |_{x=μ}` (Jacobian at the mean)

### EKF 대응

```
z = h(x) + v
S = H P Hᵀ + R              # H = Jacobian
```

**동일한 구조!** EWA splatting = EKF의 measurement update.

### 한계

1. **Jacobian은 mean에서만 계산** → nonlinearity가 크면 부정확
2. **카메라 distortion이 있으면** Jacobian 유도가 복잡하거나 불가능
3. **Rolling shutter**처럼 pixel마다 변환이 다르면 단일 Jacobian 적용 불가

---

## 2. Unscented Transform — 완전한 수식

### 파라미터

3DGUT에서 사용하는 값:

| 파라미터 | 값 | 의미 |
|---------|---|------|
| n | 3 | 차원 (3D Gaussian) |
| α | 1.0 | sigma point spread |
| β | 2.0 | prior (Gaussian이면 2) |
| κ | 0.0 | secondary scaling |
| λ | α²(n+κ) - n = **0.0** | combined scaling |

### Sigma Points 생성 (2n+1 = 7개)

```
χ₀ = μ
χᵢ = μ + (√((n+λ)Σ))ᵢ       for i = 1, 2, 3
χᵢ = μ - (√((n+λ)Σ))ᵢ₋ₙ     for i = 4, 5, 6
```

여기서 `√A` = Cholesky decomposition of A.

### Weights

```
w₀ᵐᵉᵃⁿ = λ / (n + λ)
w₀ᶜᵒᵛ  = λ / (n + λ) + (1 - α² + β)
wᵢᵐᵉᵃⁿ = wᵢᶜᵒᵛ = 1 / (2(n + λ))     for i = 1..6
```

### Transform

임의의 nonlinear function `g()` (= 카메라 투영):

```
1. 각 sigma point를 정확히 투영:
   γᵢ = g(χᵢ)

2. 투영된 mean 추정:
   μ_y = Σᵢ wᵢᵐᵉᵃⁿ × γᵢ

3. 투영된 covariance 추정:
   Σ_y = Σᵢ wᵢᶜᵒᵛ × (γᵢ - μ_y)(γᵢ - μ_y)ᵀ
```

---

## 3. UT의 정확도 — 왜 EKF보다 나은가

### Taylor 전개 관점

**EKF / EWA** (1차 근사):

```
E[y] ≈ h(μ)                    # 0차 정확
Cov[y] ≈ H Σ Hᵀ                # 1차 정확
```

**UT** (2차 이상):

```
E[y] = h(μ) + ½ Tr(H'' Σ) + O(Σ²)    # 2차까지 정확!
Cov[y] = H Σ Hᵀ + higher order        # 3차까지 정확 (β=2일 때)
```

UT는 Jacobian도, Hessian도 명시적으로 계산하지 않으면서
**자동으로 2차 이상의 정확도**를 얻는다.

### 직관적 설명

- EKF: "평균에서 기울기(Jacobian)를 구하고, 분포가 직선이라고 가정"
- UKF: "실제 대표 점들을 변환해서, 곡선도 자연스럽게 반영"

---

## 4. 카메라 모델별 적용

### Pinhole (선형에 가까움)

```
u = fx · x/z + cx
v = fy · y/z + cy
```

EWA ≈ UT → 차이 거의 없음 → 원래 3DGS가 잘 작동한 이유

### Fisheye Equidistant

```
θ = arctan(r)      # r = √(x²+y²) / z
θ_d = θ             # equidistant
u = fx · θ_d · x / √(x²+y²) + cx
```

EWA ≠ UT → **차이 큼!** → 3DGUT가 필요한 이유

### Fisheye Equisolid

```
θ_d = 2 sin(θ/2)
```

더 강한 nonlinearity → EWA의 오차가 더 큼

### OpenCV Radial Distortion

```
x_d = x_n(1 + k₁r² + k₂r⁴) + 2p₁x_ny_n + p₂(r² + 2x_n²)
```

### Rolling Shutter

```
각 scanline마다 다른 pose가 적용됨
→ 단일 Jacobian으로는 표현 불가능
→ UT는 각 sigma point를 해당 scanline의 정확한 pose로 투영 가능
```

---

## 5. 3DGUT의 UT 적용 흐름

```
1. 각 Gaussian (μ, Σ) 에서 7개 sigma points 생성
2. 각 sigma point를 카메라 모델로 정확히 투영 (fisheye든 뭐든)
3. 투영된 7개 점에서 2D Gaussian (μ_2d, Σ_2d) 추정
4. 2D Gaussian을 16×16 pixel tile로 확장
5. 각 tile의 pixel에서 response 계산 & alpha compositing
```

핵심: 2번에서 **어떤 카메라 모델이든** 사용 가능. Jacobian 유도 불필요.

---

## 6. MLAB (Multi-Layer Alpha Blending)

### 문제

Rasterization 기반이라 ray tracing처럼 정확한 depth ordering이 어렵다.

### 해법

k=16 크기의 hit buffer를 유지:

```
for each incoming hit:
    if buffer not full:
        insert and sort by depth
    else if hit closer than farthest in buffer:
        replace farthest, re-sort
```

### k 선택

| k | 정확도 | 메모리 |
|---|--------|--------|
| 1 | 나쁨 | 최소 |
| 4 | 보통 | 적음 |
| 16 | 좋음 (기본값) | 보통 |
| 32 | 매우 좋음 | 많음 |

**SLAM 비유**: sliding window optimization에서 window size를 정하는 것

---

## 연습 문제

1. Rolling shutter 카메라 모델을 구현하고 UT로 투영하라.
   ```
   def rolling_shutter_project(p, row, angular_vel):
       theta = angular_vel * row / H
       R = rotation_z(theta)
       return pinhole_project(R @ p)
   ```

2. UT의 α, β, κ 파라미터를 변경하면서 정확도 변화를 관찰하라.
   특히 α=0.001 (작은 spread) vs α=1.0 (큰 spread)

3. MLAB k=1,2,4,8,16,32에서 렌더링 품질(PSNR)을 비교하라.

4. 3DGUT 논문(arXiv:2412.12507)의 Table 1을 읽고,
   Pinhole에서 EWA ≈ UT인 이유를 이 Lab의 실험으로 설명하라.
