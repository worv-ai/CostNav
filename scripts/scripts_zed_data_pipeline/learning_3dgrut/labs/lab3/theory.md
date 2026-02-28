# Lab 3: Volume Rendering — Gaussian Scene을 이미지로

## 1. 연속적 Volume Rendering Integral

빛이 매질을 통과할 때 **흡수**(absorption)와 **방출**(emission)을 모델링한다:

```
                τ_f
L(o, d) = ∫     T(τ) × σ(τ) × c(τ) dτ
                τ_n
```

| 기호 | 의미 |
|------|------|
| `L(o, d)` | ray `(o, d)`에서 관측되는 radiance (색상) |
| `T(τ)` | transmittance — 빛이 살아남은 비율 |
| `σ(τ)` | volume density at τ |
| `c(τ)` | color (radiance) at τ |

### Transmittance

```
              τ
T(τ) = exp(-∫   σ(t) dt)
              τ_n
```

- `T(τ_n) = 1` : 시작점에서는 100% 투과
- `T → 0` : 밀도가 높은 매질을 지나면 빛이 다 흡수됨

**SLAM 비유**: occupancy grid에서 ray를 따라 free probability를 누적하는 것

```
Occupancy:  P(free) = ∏ (1 - P(occupied_i))
Rendering:  T = ∏ (1 - αᵢ)
```

---

## 2. 이산화: Alpha Compositing

Gaussian particle 장면에서 연속 적분을 이산화한다:

```
L = Σᵢ cᵢ × αᵢ × ∏ⱼ₌₁ⁱ⁻¹ (1 - αⱼ)
```

풀어쓰면:

```
L = c₁ × α₁                                    (첫 particle)
  + c₂ × α₂ × (1 - α₁)                         (두 번째)
  + c₃ × α₃ × (1 - α₁)(1 - α₂)                (세 번째)
  + ...
```

| 기호 | 의미 |
|------|------|
| `αᵢ = σᵢ × ρᵢ(r(τᵢ))` | particle i의 opacity × response |
| `cᵢ` | particle i의 color (SH 평가 결과) |
| `∏(1-αⱼ)` | 앞의 particle들이 얼마나 투과시켰는지 |

---

## 3. Front-to-Back Rendering

### 알고리즘

```python
color = [0, 0, 0]
T = 1.0                       # 초기 transmittance

for particle in sorted_by_depth:
    α = opacity × response(ray, particle)
    weight = T × α
    color += weight × particle.color
    T *= (1 - α)

color += T × background_color   # 남은 빛 = 배경
```

### 핵심 성질

- **ordering 필수**: depth 순서가 틀리면 결과가 틀림
- 3DGRT: BVH traversal이 자연스럽게 depth ordering 제공
- 3DGUT: MLAB (k=16 buffer)로 approximate ordering

---

## 4. Back-to-Front Rendering (비교용)

```python
color = background_color

for particle in reversed(sorted_by_depth):
    α = opacity × response(ray, particle)
    color = α × particle.color + (1 - α) × color
```

결과는 front-to-back과 **수학적으로 동일**하다.
하지만 back-to-front는 **early termination이 불가능**하다는 단점이 있다.

---

## 5. Early Ray Termination

### 원리

`T < T_min`이면 뒤의 particle은 거의 기여하지 않으므로 skip:

```
weight = T × α
if T < 0.001:    # T_min
    break        # 나머지 particle 무시
```

### T_min 선택

| T_min | 속도 | 품질 |
|-------|------|------|
| 0.001 | 느림 | 높음 (기본값) |
| 0.01 | 보통 | 약간 어두워짐 |
| 0.1 | 빠름 | 뒤쪽 디테일 손실 |

**SLAM 비유**: occupancy mapping에서 "이미 occupied인 cell은 더 이상 업데이트하지 않는" 최적화

---

## 6. Depth Map & Alpha Map

Volume rendering 과정에서 추가 정보도 구할 수 있다:

### Depth Map

```
depth = Σᵢ (T × αᵢ × τᵢ)     # weighted average depth
```

### Alpha Map

```
alpha_total = 1 - T_final      # 총 누적 opacity
```

- alpha ≈ 1 → 물체가 있는 pixel
- alpha ≈ 0 → 배경

---

## 7. Loss Functions

### 3DGRUT 기본 loss

```
L = 0.8 × L₁ + 0.2 × L_SSIM
```

### L₁ Loss

```
L₁ = (1/N) Σ |rendered - target|
```

pixel별 절대값 차이. 간단하고 안정적.

### SSIM (Structural Similarity)

```
                (2μ_x μ_y + C₁)(2σ_xy + C₂)
SSIM(x, y) = ─────────────────────────────────
              (μ_x² + μ_y² + C₁)(σ_x² + σ_y² + C₂)
```

- window 기반으로 local mean, variance, covariance 계산
- 구조적 유사성을 측정 → edge, texture 보존에 좋음
- `L_SSIM = 1 - SSIM` (loss이므로 뒤집음)

---

## 연습 문제

1. Back-to-front rendering을 구현하고 front-to-back 결과와 비교하라.
   수학적으로 같아야 한다.

2. SSIM loss를 직접 구현하라. (window_size=5, C₁=0.01², C₂=0.03²)

3. Anti-aliasing: 각 pixel에 여러 ray를 쏘고 평균하는 supersampling을 구현하라.

4. Gaussian 수백 개로 장면을 만들고 렌더링하라.
   numpy가 느린 이유 → GPU 가속이 필요한 이유를 체감.
