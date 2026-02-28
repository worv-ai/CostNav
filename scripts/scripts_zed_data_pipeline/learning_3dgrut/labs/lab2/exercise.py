#!/usr/bin/env python3
"""Lab 2: Ray-Particle Maximum Response Point -- 3DGRT core intersection.
See theory.md for mathematical derivations of tau_max and BVH.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

RESULTS = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS, exist_ok=True)

# ---- Utility functions ----

def quat_to_rotation_matrix(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


class Gaussian3D:
    def __init__(self, mu, q, s, opacity=0.9, color=np.array([1, 0, 0])):
        self.mu = np.array(mu, dtype=np.float64)
        self.q = np.array(q, dtype=np.float64)
        self.q /= np.linalg.norm(self.q)
        self.s = np.array(s, dtype=np.float64)
        self.opacity = opacity
        self.color = np.array(color, dtype=np.float64)

    @property
    def R(self):
        return quat_to_rotation_matrix(self.q)

    @property
    def S(self):
        return np.diag(self.s)

    @property
    def S_inv(self):
        return np.diag(1.0 / self.s)

    @property
    def Sigma(self):
        R = self.R; S = self.S
        return R @ S.T @ S @ R.T

    @property
    def Sigma_inv(self):
        return np.linalg.inv(self.Sigma)


# ---- Part 1: Ray -> Gaussian local coordinate transform ----
print("=" * 70)
print("Part 1: Ray -> Gaussian local coordinates")
print("=" * 70)


def transform_ray_to_gaussian_local(o, d, gaussian):
    """World space ray -> Gaussian local space (isotropic unit Gaussian)."""
    R = gaussian.R
    S_inv = gaussian.S_inv
    o_g = S_inv @ R.T @ (o - gaussian.mu)
    d_g = S_inv @ R.T @ d
    return o_g, d_g


# ---- Part 2: tau_max -- analytical maximum response point ----
print("\n" + "=" * 70)
print("Part 2: tau_max -- max Gaussian response on ray")
print("=" * 70)


def compute_tau_max(o, d, gaussian):
    """Compute tau_max via both local and world coordinate methods."""
    # Local coordinates: tau = -o_g^T d_g / (d_g^T d_g)
    o_g, d_g = transform_ray_to_gaussian_local(o, d, gaussian)
    tau_local = -np.dot(o_g, d_g) / np.dot(d_g, d_g)

    # World coordinates: tau = (mu - o)^T Sigma^{-1} d / (d^T Sigma^{-1} d)
    Sigma_inv = gaussian.Sigma_inv
    diff = gaussian.mu - o
    tau_world = np.dot(diff, Sigma_inv @ d) / np.dot(d, Sigma_inv @ d)

    return tau_local, tau_world


def compute_response_at_tau(tau, o, d, gaussian):
    """Compute alpha = opacity * rho(o + tau*d)."""
    x = o + tau * d
    diff = x - gaussian.mu
    Sigma_inv = gaussian.Sigma_inv
    mahal_sq = diff @ Sigma_inv @ diff
    rho = np.exp(-0.5 * mahal_sq)
    alpha = gaussian.opacity * rho
    return alpha, rho, mahal_sq


# Test with a single Gaussian and multiple rays
g = Gaussian3D(
    mu=[2.0, 1.0, 5.0],
    q=[0.924, 0, 0.383, 0],
    s=[0.8, 0.4, 0.6],
    opacity=0.95,
    color=[1, 0.3, 0.3]
)

camera_origin = np.array([0.0, 0.0, 0.0])
fx, fy = 500, 500
cx, cy = 320, 240
test_pixels = [(320, 240), (400, 200), (250, 300), (350, 180)]

print(f"\n  Gaussian: mu={g.mu}, s={g.s}")
print(f"  Camera: origin={camera_origin}")
print(f"  {'Pixel':>10} | {'tau_local':>10} | {'tau_world':>10} | {'alpha':>10} | {'rho':>10}")

for u, v in test_pixels:
    d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    d = d / np.linalg.norm(d)

    tau_l, tau_w = compute_tau_max(camera_origin, d, g)
    alpha, rho, _ = compute_response_at_tau(tau_l, camera_origin, d, g)

    print(f"  ({u:3d},{v:3d}) | {tau_l:10.4f} | {tau_w:10.4f} | {alpha:10.6f} | {rho:10.6f}")
    assert abs(tau_l - tau_w) < 1e-10, "Local/world methods must agree!"

print("\n  Local/world coordinate methods agree.")

# ---- Part 3: Ray response profile visualization ----
print("\n" + "=" * 70)
print("Part 3: Ray response profile")
print("=" * 70)

scene_gaussians = [
    Gaussian3D([1.5, 0, 3], [1, 0, 0, 0], [0.5, 0.4, 0.3], 0.9, [1, 0.2, 0.2]),
    Gaussian3D([0.5, 0.3, 5], [0.924, 0, 0, 0.383], [0.6, 0.3, 0.4], 0.7, [0.2, 0.8, 0.2]),
    Gaussian3D([-0.3, -0.2, 7], [1, 0, 0, 0], [0.4, 0.5, 0.3], 0.85, [0.2, 0.2, 1.0]),
    Gaussian3D([0.8, -0.5, 9], [0.866, 0.5, 0, 0], [0.7, 0.2, 0.5], 0.6, [0.8, 0.8, 0.2]),
]

o = np.array([0.0, 0.0, 0.0])
d = np.array([0.0, 0.0, 1.0])

fig, axes = plt.subplots(2, 1, figsize=(14, 10))

ax = axes[0]
tau_range = np.linspace(0, 12, 1000)
colors = ['red', 'green', 'blue', 'orange']
tau_maxes = []

for i, (g_i, c) in enumerate(zip(scene_gaussians, colors)):
    responses = []
    for tau in tau_range:
        alpha, rho, _ = compute_response_at_tau(tau, o, d, g_i)
        responses.append(alpha)

    tau_l, _ = compute_tau_max(o, d, g_i)
    tau_maxes.append(tau_l)
    alpha_max, _, _ = compute_response_at_tau(tau_l, o, d, g_i)

    ax.plot(tau_range, responses, color=c, linewidth=2,
            label=f'Gaussian {i}: mu={g_i.mu}, alpha_max={alpha_max:.3f}')
    ax.axvline(x=tau_l, color=c, linestyle='--', alpha=0.5)
    ax.plot(tau_l, alpha_max, 'o', color=c, markersize=10, zorder=5)

ax.set_xlabel('tau (ray parameter)', fontsize=12)
ax.set_ylabel('alpha(tau)', fontsize=12)
ax.set_title('Ray Gaussian Response Profile', fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)

# Transmittance along ray
ax2 = axes[1]
sorted_indices = np.argsort(tau_maxes)

# Volume rendering (front-to-back)
T = 1.0
cumulative = np.zeros(3)
print(f"\n  Volume Rendering order (front to back):")
print(f"  {'#':>4} | {'tau_max':>8} | {'alpha':>8} | {'T_before':>10} | {'T_after':>10} | {'weight':>10}")

for rank, i in enumerate(sorted_indices):
    g_i = scene_gaussians[i]
    alpha, _, _ = compute_response_at_tau(tau_maxes[i], o, d, g_i)
    weight = T * alpha
    cumulative += weight * g_i.color
    T_before = T
    T *= (1 - alpha)
    print(f"  {rank:4d} | {tau_maxes[i]:8.3f} | {alpha:8.4f} | {T_before:10.6f} | {T:10.6f} | {weight:10.6f}")

print(f"\n  Final color: RGB = [{cumulative[0]:.3f}, {cumulative[1]:.3f}, {cumulative[2]:.3f}]")
print(f"  Final transmittance: {T:.6f}")

# Transmittance curve
T_curve = np.ones(len(tau_range))
for j in range(len(tau_range)):
    T_val = 1.0
    for i in sorted_indices:
        g_i = scene_gaussians[i]
        if tau_range[j] > tau_maxes[i]:
            alpha, _, _ = compute_response_at_tau(tau_maxes[i], o, d, g_i)
            T_val *= (1 - alpha)
    T_curve[j] = T_val

ax2.plot(tau_range, T_curve, 'k-', linewidth=2, label='Transmittance T(tau)')
ax2.fill_between(tau_range, 0, T_curve, alpha=0.1, color='gray')

for i, c in zip(sorted_indices, [colors[j] for j in sorted_indices]):
    ax2.axvline(x=tau_maxes[i], color=colors[i], linestyle='--', alpha=0.7)
    ax2.annotate(f'G{i}', xy=(tau_maxes[i], 0.5), fontsize=10, color=colors[i], fontweight='bold')

ax2.set_xlabel('tau (ray parameter)', fontsize=12)
ax2.set_ylabel('Transmittance T(tau)', fontsize=12)
ax2.set_title('Transmittance decay along ray', fontsize=13)
ax2.set_ylim(0, 1.1)
ax2.legend(fontsize=10)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig1_ray_response_profile.png"), dpi=150)
plt.close()
print("\n-> Figure saved: fig1_ray_response_profile.png")

# ---- Part 4: 2D vs 3D Evaluation comparison ----
print("\n" + "=" * 70)
print("Part 4: 2D Evaluation (Splatting) vs 3D Evaluation (Ray Tracing)")
print("=" * 70)

g_test = Gaussian3D(
    mu=[0.5, 0.3, 3.0],
    q=[0.924, 0, 0.383, 0],
    s=[1.0, 0.5, 0.8],
    opacity=0.8
)

W, H = 100, 80
fx_t, fy_t, cx_t, cy_t = 200, 200, 50, 40

image_2d_eval = np.zeros((H, W))
image_3d_eval = np.zeros((H, W))

# 2D evaluation: EWA splatting (project 3D Gaussian to 2D)
J = np.array([
    [fx_t / g_test.mu[2], 0, -fx_t * g_test.mu[0] / g_test.mu[2]**2],
    [0, fy_t / g_test.mu[2], -fy_t * g_test.mu[1] / g_test.mu[2]**2]
])
mu_2d = np.array([
    fx_t * g_test.mu[0] / g_test.mu[2] + cx_t,
    fy_t * g_test.mu[1] / g_test.mu[2] + cy_t
])
Sigma_2d = J @ g_test.Sigma @ J.T
Sigma_2d_inv = np.linalg.inv(Sigma_2d)

for v in range(H):
    for u in range(W):
        # 2D evaluation
        pixel = np.array([u + 0.5, v + 0.5])
        diff_2d = pixel - mu_2d
        mahal_2d = diff_2d @ Sigma_2d_inv @ diff_2d
        image_2d_eval[v, u] = g_test.opacity * np.exp(-0.5 * mahal_2d)

        # 3D evaluation (ray tracing)
        d = np.array([(u + 0.5 - cx_t) / fx_t, (v + 0.5 - cy_t) / fy_t, 1.0])
        d = d / np.linalg.norm(d)
        tau_l, _ = compute_tau_max(np.zeros(3), d, g_test)
        if tau_l > 0:
            alpha, _, _ = compute_response_at_tau(tau_l, np.zeros(3), d, g_test)
            image_3d_eval[v, u] = alpha

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

im0 = axes[0].imshow(image_2d_eval, cmap='hot', aspect='auto')
axes[0].set_title('2D Evaluation (EWA Splatting)', fontsize=11)
plt.colorbar(im0, ax=axes[0])

im1 = axes[1].imshow(image_3d_eval, cmap='hot', aspect='auto')
axes[1].set_title('3D Evaluation (Ray Tracing)', fontsize=11)
plt.colorbar(im1, ax=axes[1])

diff = image_3d_eval - image_2d_eval
max_abs_diff = max(abs(diff.min()), abs(diff.max()))
im2 = axes[2].imshow(diff, cmap='RdBu', aspect='auto',
                      vmin=-max_abs_diff, vmax=max_abs_diff)
axes[2].set_title(f'Difference (3D - 2D)\nmax |diff| = {max_abs_diff:.4f}', fontsize=11)
plt.colorbar(im2, ax=axes[2])

plt.suptitle('2D vs 3D Evaluation Comparison', fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig2_2d_vs_3d_eval.png"), dpi=150)
plt.close()
print("-> Figure saved: fig2_2d_vs_3d_eval.png")

# ---- Part 5: BVH (Bounding Volume Hierarchy) concept ----
print("\n" + "=" * 70)
print("Part 5: BVH concept visualization")
print("=" * 70)

fig, axes = plt.subplots(1, 2, figsize=(16, 7))

np.random.seed(42)
n_gaussians = 20
centers = np.random.rand(n_gaussians, 2) * 8 - 1
sizes = np.random.rand(n_gaussians) * 0.3 + 0.1

# Brute force
ax = axes[0]
ax.set_title('Brute Force: check all Gaussians\nO(N) per ray', fontsize=12)

ray_o = np.array([-1.5, 4])
ray_d = np.array([1.0, -0.3])
ray_d /= np.linalg.norm(ray_d)

for i, (c, s) in enumerate(zip(centers, sizes)):
    circle = plt.Circle(c, s, fill=True, alpha=0.3, color='lightblue', edgecolor='blue')
    ax.add_patch(circle)
    ax.plot(c[0], c[1], 'rx', markersize=6)

t_end = 12
ax.annotate('', xy=ray_o + t_end * ray_d, xytext=ray_o,
            arrowprops=dict(arrowstyle='->', color='red', lw=2))
ax.set_xlim(-2, 8); ax.set_ylim(-2, 6)
ax.set_aspect('equal')
ax.grid(True, alpha=0.2)
ax.text(0, 5.5, f'Checks: {n_gaussians}', fontsize=12,
        bbox=dict(boxstyle='round', facecolor='lightyellow'))

# With BVH
ax = axes[1]
ax.set_title('BVH: check only relevant Gaussians\nO(log N) per ray', fontsize=12)

quadrants = [
    ([-1, -1], [3, 3]),
    ([3, -1], [7, 3]),
    ([-1, 3], [3, 7]),
    ([3, 3], [7, 7]),
]

for (bl, tr) in quadrants:
    rect = plt.Rectangle(bl, tr[0]-bl[0], tr[1]-bl[1],
                          fill=False, edgecolor='gray', linewidth=2, linestyle='--')
    ax.add_patch(rect)

hit_quadrants = [0, 2]
for qi, (bl, tr) in enumerate(quadrants):
    color = 'lightgreen' if qi in hit_quadrants else 'lightyellow'
    rect = plt.Rectangle(bl, tr[0]-bl[0], tr[1]-bl[1],
                          fill=True, alpha=0.2, color=color)
    ax.add_patch(rect)

checked = 0
for i, (c, s) in enumerate(zip(centers, sizes)):
    in_hit_quadrant = False
    for qi in hit_quadrants:
        bl, tr = quadrants[qi]
        if bl[0] <= c[0] <= tr[0] and bl[1] <= c[1] <= tr[1]:
            in_hit_quadrant = True

    circle = plt.Circle(c, s, fill=True, alpha=0.3,
                         color='lightblue' if in_hit_quadrant else 'lightgray',
                         edgecolor='blue' if in_hit_quadrant else 'gray')
    ax.add_patch(circle)
    if in_hit_quadrant:
        ax.plot(c[0], c[1], 'gx', markersize=6)
        checked += 1

ax.annotate('', xy=ray_o + t_end * ray_d, xytext=ray_o,
            arrowprops=dict(arrowstyle='->', color='red', lw=2))
ax.set_xlim(-2, 8); ax.set_ylim(-2, 6)
ax.set_aspect('equal')
ax.grid(True, alpha=0.2)
ax.text(0, 5.5, f'Checks: {checked} (/{n_gaussians})',
        fontsize=12, bbox=dict(boxstyle='round', facecolor='lightgreen'))

plt.suptitle('BVH accelerates Ray-Particle search', fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig3_bvh_concept.png"), dpi=150)
plt.close()
print("-> Figure saved: fig3_bvh_concept.png")

# ---- Hit buffer utility ----

def ray_hit_buffer(o, d, gaussians, k=16):
    """Per-ray hit buffer: keep k closest hits by tau_max."""
    hits = []
    for i, g_i in enumerate(gaussians):
        tau_l, _ = compute_tau_max(o, d, g_i)
        if tau_l > 0:
            alpha, rho, mahal_sq = compute_response_at_tau(tau_l, o, d, g_i)
            if alpha > 0.01:
                hits.append((tau_l, i, alpha, rho))
    hits.sort(key=lambda x: x[0])
    return hits[:k]


test_ray_o = np.array([0.0, 0.0, 0.0])
test_ray_d = np.array([0.1, 0.05, 1.0])
test_ray_d /= np.linalg.norm(test_ray_d)

hits = ray_hit_buffer(test_ray_o, test_ray_d, scene_gaussians, k=16)
print(f"\n  Hit buffer for test ray (k=16):")
for tau, idx, alpha, rho in hits:
    print(f"    tau={tau:.3f}, Gaussian {idx}, alpha={alpha:.4f}")

print("\nLab 2 complete!")
