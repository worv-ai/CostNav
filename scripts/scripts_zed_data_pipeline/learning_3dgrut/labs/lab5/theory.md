# Lab 5: Differentiable Rendering & 최적화

## 1. Bundle Adjustment → Differentiable Rendering

### Bundle Adjustment (SLAM)

```
min  Σᵢ Σⱼ ‖π(Xⱼ, Tᵢ) - zᵢⱼ‖²
w.r.t.  Xⱼ (3D points), Tᵢ (camera poses)
```

- 관측 `zᵢⱼ` (pixel 좌표) 와 reprojection의 차이를 최소화
- Jacobian `∂π/∂X`, `∂π/∂T` 로 gradient 계산
- Levenberg-Marquardt 또는 Gauss-Newton으로 최적화

### 3DGRUT Training

```
min  Σᵢ ‖render(Θ, Tᵢ) - Iᵢ‖
w.r.t.  Θ (Gaussian parameters: μ, q, s, σ, SH)
```

- 렌더링 결과와 실제 사진의 차이를 최소화
- **Tᵢ는 고정** (COLMAP에서 이미 추정됨)
- 역전파(backprop)로 gradient 계산
- Adam optimizer로 최적화

### 핵심 대응

| Bundle Adjustment | 3DGRUT Training |
|---|---|
| 3D point Xⱼ | Gaussian position μ |
| Camera pose Tᵢ | 고정 (COLMAP 제공) |
| Reprojection error | Rendering error (L₁ + SSIM) |
| Jacobian (해석적) | Autograd (자동 미분) |
| Gauss-Newton / LM | Adam |

---

## 2. 학습 루프

```
for i in range(30000):                    # n_iterations
    # 1. Random view 선택
    camera_pose, target_image = sample_view()

    # 2. Forward: 렌더링
    rendered = render(gaussians, camera_pose)

    # 3. Loss 계산
    loss = 0.8 * L1(rendered, target) + 0.2 * SSIM_loss(rendered, target)

    # 4. Backward: gradient 계산
    loss.backward()

    # 5. Parameter update
    optimizer.step()

    # 6. (매 300 iter) Densification
    if i % 300 == 0:
        densify_and_prune(gaussians)
```

---

## 3. Backward Pass 수학

### Forward (복습)

```
α = σ × exp(-½ ω_g²)
```

여기서 `ω_g² = ‖o_g + τ_max · (d_g / ‖d_g‖)‖²`

### Backward

```
∂α/∂ω_g² = -½ σ × exp(-½ ω_g²) = -½ α
```

`ω_g²`는 `μ`, `s`, `q`의 함수이므로 chain rule로:

```
∂Loss/∂μ = (∂Loss/∂α) × (∂α/∂ω_g²) × (∂ω_g²/∂μ)
∂Loss/∂s = (∂Loss/∂α) × (∂α/∂ω_g²) × (∂ω_g²/∂s)
∂Loss/∂q = (∂Loss/∂α) × (∂α/∂ω_g²) × (∂ω_g²/∂q)
```

### 3D Evaluation의 장점

3D 평가 방식은 **projection function π()의 미분이 필요 없다!**

- EWA: Σ_2d = J Σ Jᵀ → backward에서 ∂J/∂params가 필요
  → 카메라 모델이 복잡하면 Jacobian의 Jacobian이 필요 (복잡)
- 3DGRT: ray-particle 상호작용만으로 gradient 계산
  → 임의의 카메라 모델에서도 학습 가능

---

## 4. Per-Parameter Learning Rate

3DGRUT는 파라미터별로 다른 learning rate를 사용한다:

| 파라미터 | LR | Decay | 이유 |
|---------|-----|-------|------|
| Position μ | 1.6e-4 → 1.6e-6 | Exponential | 초기에 크게, 후반에 미세 조정 |
| Density σ | 5e-2 | 없음 | 빠르게 투명/불투명 결정 |
| SH DC (c₀) | 2.5e-3 | 없음 | 기본 색은 중간 속도 |
| SH higher (c₁₋₃) | 1.25e-4 | 없음 | Specular는 매우 작은 LR |
| Rotation q | 1e-3 | 없음 | 방향 조정 |
| Scale s | 5e-3 | 없음 | 크기 조정 |

**SLAM 비유**: BA에서도 pose와 structure에 다른 step size를 쓰는 것과 같은 원리.

---

## 5. Densification 전략

### 문제

초기 Gaussian 수가 부족하면 디테일 표현 불가.
처음부터 수백만 개를 쓰면 학습 불안정.

### GS Strategy (Split / Clone / Prune)

매 300 iter마다 (iter 500 ~ 15000):

#### Split (큰 Gaussian 분할)

```
조건: gradient 큼 AND scale > threshold
동작: 1개 → 2개의 작은 Gaussian
     - 원본 scale을 1/1.6으로 축소
     - 두 Gaussian을 원본 covariance 방향으로 분리
```

**언제**: 하나의 큰 Gaussian이 복잡한 영역을 커버하려 할 때

#### Clone (작은 Gaussian 복제)

```
조건: gradient 큼 AND scale < threshold
동작: 원본 유지 + gradient 방향으로 복사본 생성
```

**언제**: 디테일이 필요한 영역에 Gaussian이 부족할 때

#### Prune (제거)

```
조건: opacity < threshold (예: 0.005)
동작: 해당 Gaussian 삭제
```

**언제**: 쓸모없는 투명한 Gaussian 정리

### MCMC Strategy (대안)

```
- Relocate: dead Gaussian을 high-gradient 영역으로 이동
- Perturb: 위치에 noise 추가 (탐색)
- Add: 새 Gaussian 생성
```

**SLAM 비유**: particle filter의 resampling과 유사

---

## 6. 학습 일정 (Training Schedule)

```
iter     0 ~   500: Warmup, densification 없음
iter   500 ~ 15000: 매 300 iter마다 densify/prune
iter  1000 씩:      SH degree +1 (progressive SH)
iter 15000 ~:       Densification 중단, fine-tuning
iter 15000 ~:       Incoherent ray sampling (batch 2¹⁹)
iter 30000:         학습 종료, 평가
```

---

## 연습 문제

1. `DifferentiableGaussianRenderer`에 rotation(quaternion)을 추가하라.

2. Densification을 구현하라:
   - gradient 누적기 추가
   - 매 50 iter마다 split/clone/prune 실행

3. 실제 사진을 target으로 학습하라 (PIL로 로드).

4. Multi-view 학습: 여러 camera pose에서 동시에 학습하라.
   각 iteration에서 random view를 선택.

5. Progressive SH: SH degree를 학습 중에 0→3으로 올려라.
