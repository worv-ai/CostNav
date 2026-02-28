#!/usr/bin/env python3
"""Lab 3: Volume Rendering -- Alpha compositing Gaussian particles into images.
See theory.md for the volume rendering integral derivation.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from time import time

RESULTS = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS, exist_ok=True)

# ---- Utility functions (from Labs 1-2) ----

def quat_to_rotation_matrix(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


class Gaussian3D:
    def __init__(self, mu, q, s, opacity=0.9, sh_dc=None):
        self.mu = np.array(mu, dtype=np.float64)
        self.q = np.array(q, dtype=np.float64)
        self.q /= np.linalg.norm(self.q)
        self.s = np.array(s, dtype=np.float64)
        self.opacity = opacity
        self.sh_dc = np.array(sh_dc if sh_dc is not None else [0.5, 0.5, 0.5])

    @property
    def R(self): return quat_to_rotation_matrix(self.q)
    @property
    def S_inv(self): return np.diag(1.0 / self.s)
    @property
    def Sigma(self):
        R = self.R; S = np.diag(self.s)
        return R @ S.T @ S @ R.T
    @property
    def Sigma_inv(self): return np.linalg.inv(self.Sigma)


def compute_tau_max(o, d, g):
    o_g = g.S_inv @ g.R.T @ (o - g.mu)
    d_g = g.S_inv @ g.R.T @ d
    return -np.dot(o_g, d_g) / np.dot(d_g, d_g)


def compute_alpha(tau, o, d, g):
    x = o + tau * d
    diff = x - g.mu
    mahal_sq = diff @ g.Sigma_inv @ diff
    rho = np.exp(-0.5 * mahal_sq)
    return g.opacity * rho


# ---- Part 1: Continuous vs discrete volume rendering ----
print("=" * 70)
print("Part 1: Volume Rendering -- continuous vs discrete")
print("=" * 70)

fig, axes = plt.subplots(2, 2, figsize=(16, 10))
tau_range = np.linspace(0, 15, 1000)

demo_gaussians = [
    Gaussian3D([0, 0, 3], [1, 0, 0, 0], [0.4, 0.3, 0.5], 0.85, [0.9, 0.2, 0.1]),
    Gaussian3D([0, 0, 6], [1, 0, 0, 0], [0.5, 0.4, 0.3], 0.7, [0.1, 0.8, 0.1]),
    Gaussian3D([0, 0, 10], [1, 0, 0, 0], [0.3, 0.3, 0.6], 0.9, [0.1, 0.2, 0.9]),
]

o = np.zeros(3)
d = np.array([0, 0, 1.0])

# Density along ray
ax = axes[0, 0]
ax.set_title('Density sigma(tau) per Gaussian')
total_density = np.zeros_like(tau_range)
colors_list = ['red', 'green', 'blue']

for i, g in enumerate(demo_gaussians):
    density = np.array([compute_alpha(t, o, d, g) for t in tau_range])
    ax.fill_between(tau_range, 0, density, alpha=0.3, color=colors_list[i])
    ax.plot(tau_range, density, color=colors_list[i], linewidth=2,
            label=f'G{i}: mu_z={g.mu[2]:.0f}')
    total_density += density

ax.plot(tau_range, total_density, 'k--', linewidth=1.5, label='Total')
ax.set_xlabel('tau'); ax.set_ylabel('sigma(tau)')
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# Transmittance
ax = axes[0, 1]
ax.set_title('Transmittance T(tau)')

cumulative_density = np.cumsum(total_density) * (tau_range[1] - tau_range[0])
T_continuous = np.exp(-cumulative_density)
ax.plot(tau_range, T_continuous, 'k-', linewidth=2, label='Continuous T(tau)')

tau_maxes = [compute_tau_max(o, d, g) for g in demo_gaussians]
alphas = [compute_alpha(t, o, d, g) for t, g in zip(tau_maxes, demo_gaussians)]
sorted_idx = np.argsort(tau_maxes)

T_discrete = np.ones_like(tau_range)
for i in sorted_idx:
    for j in range(len(tau_range)):
        if tau_range[j] >= tau_maxes[i]:
            T_discrete[j] *= (1 - alphas[i])

ax.plot(tau_range, T_discrete, 'r--', linewidth=2, label='Discrete T(tau)')
for i in sorted_idx:
    ax.axvline(x=tau_maxes[i], color=colors_list[i], linestyle=':', alpha=0.5)

ax.set_xlabel('tau'); ax.set_ylabel('T(tau)')
ax.set_ylim(0, 1.1)
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# Per-particle weights
ax = axes[1, 0]
ax.set_title('Per-particle weight: T * alpha')

for rank, i in enumerate(sorted_idx):
    T_before = 1.0
    for prev_rank in range(rank):
        prev_i = sorted_idx[prev_rank]
        T_before *= (1 - alphas[prev_i])
    weight = T_before * alphas[i]
    ax.bar(rank, weight, color=colors_list[i],
           label=f'G{i}: w={weight:.3f}, alpha={alphas[i]:.3f}')

ax.set_xlabel('Depth order'); ax.set_ylabel('Weight (T * alpha)')
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# Final color composition
ax = axes[1, 1]
ax.set_title('Final color accumulation')

final_color = np.zeros(3)
T = 1.0
color_accumulation = []

for rank, i in enumerate(sorted_idx):
    weight = T * alphas[i]
    contribution = weight * demo_gaussians[i].sh_dc
    final_color += contribution
    T *= (1 - alphas[i])
    color_accumulation.append(final_color.copy())

bar_width = 0.25
for ch, ch_name, offset in [(0, 'R', -bar_width), (1, 'G', 0), (2, 'B', bar_width)]:
    values = [ca[ch] for ca in color_accumulation]
    ax.bar(np.arange(len(values)) + offset, values, bar_width,
           label=ch_name, color=['red', 'green', 'blue'][ch], alpha=0.7)

ax.set_xlabel('After adding particle #')
ax.set_ylabel('Accumulated color')
ax.set_xticks(range(len(color_accumulation)))
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

color_patch = plt.Rectangle((0.7, 0.7), 0.25, 0.25,
                              facecolor=np.clip(final_color, 0, 1),
                              edgecolor='black', linewidth=2,
                              transform=ax.transAxes)
ax.add_patch(color_patch)
ax.text(0.82, 0.65, f'RGB: [{final_color[0]:.2f}, {final_color[1]:.2f}, {final_color[2]:.2f}]',
        transform=ax.transAxes, ha='center', fontsize=8)

plt.suptitle('Volume Rendering: Continuous -> Discrete', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig1_volume_rendering_math.png"), dpi=150)
plt.close()
print("-> Figure saved: fig1_volume_rendering_math.png")

# ---- Part 2: Complete Volume Renderer ----
print("\n" + "=" * 70)
print("Part 2: Complete Volume Renderer")
print("=" * 70)


def render_pixel(o, d, gaussians, bg_color=np.array([1, 1, 1]), T_min=0.001):
    """Render a single pixel via front-to-back alpha compositing."""
    hits = []
    for i, g in enumerate(gaussians):
        tau = compute_tau_max(o, d, g)
        if tau > 0.01:
            alpha = compute_alpha(tau, o, d, g)
            if alpha > 0.001:
                hits.append((tau, i, alpha))
    hits.sort(key=lambda x: x[0])

    color = np.zeros(3)
    depth = 0.0
    T = 1.0

    for tau, idx, alpha in hits:
        if T < T_min:
            break  # Early termination
        weight = T * alpha
        color += weight * gaussians[idx].sh_dc
        depth += weight * tau
        T *= (1 - alpha)

    color += T * bg_color
    alpha_total = 1.0 - T
    return color, depth, alpha_total


def render_image(gaussians, width, height, fx, fy, cx, cy,
                 camera_pose=np.eye(4), bg_color=np.array([1, 1, 1])):
    """Render full image from a set of Gaussians and camera parameters."""
    image = np.zeros((height, width, 3))
    depth_map = np.zeros((height, width))
    alpha_map = np.zeros((height, width))

    R_cam = camera_pose[:3, :3]
    t_cam = camera_pose[:3, 3]
    o_world = -R_cam.T @ t_cam

    for v in range(height):
        for u in range(width):
            d_cam = np.array([(u + 0.5 - cx) / fx, (v + 0.5 - cy) / fy, 1.0])
            d_world = R_cam.T @ d_cam
            d_world = d_world / np.linalg.norm(d_world)

            color, depth, alpha = render_pixel(o_world, d_world, gaussians, bg_color)
            image[v, u] = np.clip(color, 0, 1)
            depth_map[v, u] = depth
            alpha_map[v, u] = alpha

    return image, depth_map, alpha_map


# ---- Part 3: Scene rendering ----
print("\n" + "=" * 70)
print("Part 3: Scene rendering")
print("=" * 70)

# Simple scene: "traffic light"
scene = [
    Gaussian3D([0, -0.8, 5], [1, 0, 0, 0], [0.35, 0.35, 0.2], 0.95, [0.9, 0.1, 0.1]),
    Gaussian3D([0, 0, 5], [1, 0, 0, 0], [0.35, 0.35, 0.2], 0.95, [0.9, 0.9, 0.1]),
    Gaussian3D([0, 0.8, 5], [1, 0, 0, 0], [0.35, 0.35, 0.2], 0.95, [0.1, 0.8, 0.1]),
    Gaussian3D([0, 0, 5.2], [1, 0, 0, 0], [0.5, 1.3, 0.15], 0.98, [0.15, 0.15, 0.15]),
    Gaussian3D([0, 3, 6], [1, 0, 0, 0], [4.0, 0.1, 3.0], 0.95, [0.3, 0.3, 0.35]),
    Gaussian3D([0, -4, 10], [1, 0, 0, 0], [8.0, 2.0, 0.5], 0.6, [0.5, 0.7, 1.0]),
    Gaussian3D([-3, -0.5, 8], [1, 0, 0, 0], [1.0, 2.0, 0.5], 0.85, [0.6, 0.5, 0.4]),
    Gaussian3D([3, -0.3, 7], [1, 0, 0, 0], [0.8, 1.8, 0.4], 0.85, [0.5, 0.4, 0.35]),
]

W, H = 120, 90
fx, fy = 120, 120
cx, cy = W / 2, H / 2

print(f"  Rendering {W}x{H} image with {len(scene)} Gaussians...")
t_start = time()
image, depth, alpha = render_image(scene, W, H, fx, fy, cx, cy)
t_elapsed = time() - t_start
print(f"  Done in {t_elapsed:.2f}s ({W*H} rays)")

# Side view
pose2 = np.eye(4)
theta = np.radians(15)
pose2[:3, :3] = np.array([
    [np.cos(theta), 0, np.sin(theta)],
    [0, 1, 0],
    [-np.sin(theta), 0, np.cos(theta)]
])
pose2[:3, 3] = np.array([-2, 0, 0])

print(f"  Rendering from side view...")
t_start = time()
image2, depth2, alpha2 = render_image(scene, W, H, fx, fy, cx, cy, pose2)
t_elapsed = time() - t_start
print(f"  Done in {t_elapsed:.2f}s")

fig, axes = plt.subplots(2, 3, figsize=(18, 11))

axes[0, 0].imshow(image); axes[0, 0].set_title('Front View -- RGB'); axes[0, 0].axis('off')
axes[0, 1].imshow(depth, cmap='turbo'); axes[0, 1].set_title('Front View -- Depth'); axes[0, 1].axis('off')
axes[0, 2].imshow(alpha, cmap='gray'); axes[0, 2].set_title('Front View -- Alpha'); axes[0, 2].axis('off')

axes[1, 0].imshow(image2); axes[1, 0].set_title('Side View -- RGB\n(15 deg rotation + 2m translation)'); axes[1, 0].axis('off')
axes[1, 1].imshow(depth2, cmap='turbo'); axes[1, 1].set_title('Side View -- Depth'); axes[1, 1].axis('off')
axes[1, 2].imshow(alpha2, cmap='gray'); axes[1, 2].set_title('Side View -- Alpha'); axes[1, 2].axis('off')

plt.suptitle('Volume Rendering: same scene from different viewpoints',
             fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig2_rendered_scene.png"), dpi=150)
plt.close()
print("-> Figure saved: fig2_rendered_scene.png")

# ---- Part 4: Early Termination ----
print("\n" + "=" * 70)
print("Part 4: Early Termination")
print("=" * 70)

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

for idx, T_min in enumerate([0.001, 0.01, 0.1]):
    d = np.array([0, 0, 1.0])
    hits = []
    for i, g in enumerate(scene):
        tau = compute_tau_max(np.zeros(3), d, g)
        if tau > 0.01:
            a = compute_alpha(tau, np.zeros(3), d, g)
            if a > 0.001:
                hits.append((tau, i, a))
    hits.sort()

    T = 1.0
    evaluated = 0
    for tau, i, a in hits:
        if T < T_min:
            break
        T *= (1 - a)
        evaluated += 1

    ax = axes[idx]
    ax.bar(range(len(hits)), [h[2] for h in hits], color='lightblue', edgecolor='blue')
    ax.bar(range(evaluated), [hits[j][2] for j in range(evaluated)],
           color='lightgreen', edgecolor='green', label=f'Evaluated ({evaluated})')
    if evaluated < len(hits):
        ax.bar(range(evaluated, len(hits)),
               [hits[j][2] for j in range(evaluated, len(hits))],
               color='lightcoral', edgecolor='red', alpha=0.5, label=f'Skipped ({len(hits)-evaluated})')
    ax.set_title(f'T_min = {T_min}\n{evaluated}/{len(hits)} particles evaluated')
    ax.set_xlabel('Hit order (by depth)')
    ax.set_ylabel('alpha')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

plt.suptitle('Early Ray Termination', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig3_early_termination.png"), dpi=150)
plt.close()
print("-> Figure saved: fig3_early_termination.png")

print("\nLab 3 complete!")
