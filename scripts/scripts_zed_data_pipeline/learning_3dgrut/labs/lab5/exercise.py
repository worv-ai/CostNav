#!/usr/bin/env python3
"""Lab 5: Differentiable Rendering -- PyTorch Gaussian scene optimization.
See theory.md for backward pass math and densification strategies.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from time import time

import torch
import torch.nn as nn
import torch.nn.functional as F

RESULTS = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS, exist_ok=True)

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}")

# ---- Part 1: Differentiable Rendering concept (see theory.md) ----
print("=" * 70)
print("Part 1: Differentiable Rendering")
print("=" * 70)

# ---- Part 2: PyTorch Differentiable Gaussian Renderer ----
print("\n" + "=" * 70)
print("Part 2: PyTorch Differentiable Gaussian Renderer")
print("=" * 70)


class DifferentiableGaussianRenderer(nn.Module):
    """Mini 3DGRUT: simplified differentiable Gaussian renderer in PyTorch."""

    def __init__(self, n_gaussians, height, width, fx, fy, cx, cy):
        super().__init__()
        self.H = height
        self.W = width
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

        # Learnable parameters (same structure as 3DGRUT)
        self.positions = nn.Parameter(torch.randn(n_gaussians, 3) * 0.5)
        self.log_scales = nn.Parameter(torch.randn(n_gaussians, 3) * 0.1 - 1.0)
        # Axis-aligned (no quaternion) for simplicity
        self.raw_opacity = nn.Parameter(torch.zeros(n_gaussians))
        self.colors = nn.Parameter(torch.rand(n_gaussians, 3))

    @property
    def scales(self):
        return torch.exp(self.log_scales)

    @property
    def opacity(self):
        return torch.sigmoid(self.raw_opacity)

    def forward(self, camera_pose=None):
        """Full differentiable rendering -> [H, W, 3] image."""
        v_coords, u_coords = torch.meshgrid(
            torch.arange(self.H, dtype=torch.float32, device=device),
            torch.arange(self.W, dtype=torch.float32, device=device),
            indexing='ij'
        )

        # Pixel -> ray directions
        dx = (u_coords + 0.5 - self.cx) / self.fx
        dy = (v_coords + 0.5 - self.cy) / self.fy
        dz = torch.ones_like(dx)
        ray_dirs = torch.stack([dx, dy, dz], dim=-1)
        ray_dirs = F.normalize(ray_dirs, dim=-1)

        n = self.positions.shape[0]
        scales = self.scales
        opacity = self.opacity
        colors = torch.sigmoid(self.colors)

        ray_origin = torch.zeros(3, device=device)

        image = torch.zeros(self.H, self.W, 3, device=device)
        transmittance = torch.ones(self.H, self.W, device=device)

        # Sort Gaussians by depth (z)
        depths = self.positions[:, 2]
        sorted_indices = torch.argsort(depths)

        for idx in sorted_indices:
            mu = self.positions[idx]
            s = scales[idx]
            alpha_base = opacity[idx]
            c = colors[idx]

            diff = mu  # o = 0
            s_inv_sq = 1.0 / (s ** 2)

            # tau_max per pixel
            numerator = torch.sum(diff * s_inv_sq * ray_dirs, dim=-1)
            denominator = torch.sum(s_inv_sq * ray_dirs ** 2, dim=-1)
            tau = numerator / (denominator + 1e-10)

            valid = tau > 0.1

            # 3D point at tau_max
            point = ray_dirs * tau.unsqueeze(-1)
            point_diff = point - mu
            mahal_sq = torch.sum(point_diff ** 2 * s_inv_sq, dim=-1)

            rho = torch.exp(-0.5 * mahal_sq)
            alpha = alpha_base * rho * valid.float()

            # Front-to-back compositing
            weight = transmittance * alpha
            image += weight.unsqueeze(-1) * c
            transmittance = transmittance * (1.0 - alpha)

        # White background
        image += transmittance.unsqueeze(-1)
        return image


# ---- SSIM Loss ----

def ssim_loss(img1, img2, window_size=5):
    """Simplified SSIM loss. 3DGRUT uses L = 0.8*L1 + 0.2*SSIM_loss."""
    img1_t = img1.permute(2, 0, 1).unsqueeze(0)
    img2_t = img2.permute(2, 0, 1).unsqueeze(0)

    C1 = 0.01 ** 2
    C2 = 0.03 ** 2

    pad = window_size // 2
    kernel = torch.ones(1, 1, window_size, window_size, device=device) / (window_size ** 2)

    ssim_val = 0
    for ch in range(3):
        x = img1_t[:, ch:ch+1]
        y = img2_t[:, ch:ch+1]

        mu_x = F.conv2d(x, kernel, padding=pad)
        mu_y = F.conv2d(y, kernel, padding=pad)

        sigma_x = F.conv2d(x * x, kernel, padding=pad) - mu_x ** 2
        sigma_y = F.conv2d(y * y, kernel, padding=pad) - mu_y ** 2
        sigma_xy = F.conv2d(x * y, kernel, padding=pad) - mu_x * mu_y

        ssim_map = ((2 * mu_x * mu_y + C1) * (2 * sigma_xy + C2)) / \
                   ((mu_x ** 2 + mu_y ** 2 + C1) * (sigma_x + sigma_y + C2))
        ssim_val += ssim_map.mean()

    return 1.0 - ssim_val / 3.0


# ---- Part 3: Target image and optimization ----
print("\n" + "=" * 70)
print("Part 3: Gradient Descent optimization")
print("=" * 70)

H, W = 64, 64
fx, fy = 60.0, 60.0
cx, cy = 32.0, 32.0


def create_target_image(H, W):
    """Simple target: 3 colored circles."""
    img = torch.ones(H, W, 3, device=device)
    yy, xx = torch.meshgrid(torch.arange(H, device=device), torch.arange(W, device=device), indexing='ij')

    circles = [
        (20, 20, 8, torch.tensor([0.9, 0.2, 0.1], device=device)),
        (40, 30, 10, torch.tensor([0.1, 0.8, 0.2], device=device)),
        (25, 45, 7, torch.tensor([0.1, 0.2, 0.9], device=device)),
    ]

    for cy_c, cx_c, r, color in circles:
        mask = ((xx - cx_c) ** 2 + (yy - cy_c) ** 2) < r ** 2
        img[mask] = color

    return img


target = create_target_image(H, W)

n_gaussians = 30
model = DifferentiableGaussianRenderer(n_gaussians, H, W, fx, fy, cx, cy).to(device)

# Initialize positions in front of camera
with torch.no_grad():
    model.positions[:, 2] = torch.rand(n_gaussians, device=device) * 3 + 2
    model.positions[:, :2] = torch.randn(n_gaussians, 2, device=device) * 0.5

# Per-parameter learning rates (same strategy as 3DGRUT)
optimizer = torch.optim.Adam([
    {'params': [model.positions], 'lr': 0.01},
    {'params': [model.log_scales], 'lr': 0.005},
    {'params': [model.raw_opacity], 'lr': 0.05},
    {'params': [model.colors], 'lr': 0.01},
])

# ---- Training loop ----
print("\n  Training loop (mini 3DGRUT)...")
n_iterations = 300
losses = []
snapshots = []

t_start = time()

for i in range(n_iterations):
    optimizer.zero_grad()

    rendered = model()

    # Loss: 0.8*L1 + 0.2*SSIM (3DGRUT default)
    l1_loss = F.l1_loss(rendered, target)
    s_loss = ssim_loss(rendered, target)
    loss = 0.8 * l1_loss + 0.2 * s_loss

    loss.backward()
    optimizer.step()

    losses.append(loss.item())

    if i % 50 == 0 or i == n_iterations - 1:
        psnr = -10 * torch.log10(F.mse_loss(rendered, target)).item()
        print(f"    iter {i:4d}: loss={loss.item():.4f}, "
              f"L1={l1_loss.item():.4f}, SSIM_loss={s_loss.item():.4f}, "
              f"PSNR={psnr:.2f} dB")
        snapshots.append((i, rendered.detach().cpu().numpy().copy()))

t_elapsed = time() - t_start
print(f"  Training done in {t_elapsed:.1f}s")

# ---- Visualization: training progress ----
fig, axes = plt.subplots(2, len(snapshots) + 1, figsize=(4 * (len(snapshots) + 1), 8))

axes[0, 0].imshow(target.cpu().numpy())
axes[0, 0].set_title('Target')
axes[0, 0].axis('off')
axes[1, 0].axis('off')

for idx, (iteration, snap) in enumerate(snapshots):
    axes[0, idx + 1].imshow(np.clip(snap, 0, 1))
    psnr = -10 * np.log10(np.mean((snap - target.cpu().numpy()) ** 2) + 1e-10)
    axes[0, idx + 1].set_title(f'Iter {iteration}\nPSNR: {psnr:.1f} dB')
    axes[0, idx + 1].axis('off')

    diff = np.abs(snap - target.cpu().numpy()).mean(axis=-1)
    axes[1, idx + 1].imshow(diff, cmap='hot', vmin=0, vmax=0.5)
    axes[1, idx + 1].set_title(f'|Error|')
    axes[1, idx + 1].axis('off')

plt.suptitle('Differentiable Rendering: Gradient Descent Optimization',
             fontsize=12, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig1_training_progress.png"), dpi=150)
plt.close()
print("-> Figure saved: fig1_training_progress.png")

# ---- Loss curve and Gaussian positions ----
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

axes[0].plot(losses, 'b-', linewidth=1)
axes[0].set_xlabel('Iteration')
axes[0].set_ylabel('Loss (0.8*L1 + 0.2*SSIM)')
axes[0].set_title('Training Loss Curve')
axes[0].grid(True, alpha=0.3)
axes[0].set_yscale('log')

positions = model.positions.detach().cpu().numpy()
scales = model.scales.detach().cpu().numpy()
opacities = model.opacity.detach().cpu().numpy()
colors_vis = torch.sigmoid(model.colors).detach().cpu().numpy()

axes[1].scatter(positions[:, 0], positions[:, 1],
                s=scales[:, :2].mean(axis=1) * 500,
                c=colors_vis, alpha=opacities,
                edgecolors='black', linewidth=0.5)
axes[1].set_xlabel('X'); axes[1].set_ylabel('Y')
axes[1].set_title(f'Optimized Gaussian Positions (top view)\n'
                  f'{n_gaussians} Gaussians')
axes[1].set_aspect('equal')
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig2_loss_and_positions.png"), dpi=150)
plt.close()
print("-> Figure saved: fig2_loss_and_positions.png")

# ---- Part 4: Densification visualization ----
print("\n" + "=" * 70)
print("Part 4: Gaussian Densification -- Split & Clone")
print("=" * 70)

fig, axes = plt.subplots(1, 3, figsize=(18, 5))
from matplotlib.patches import Ellipse as Ell

# Before
ax = axes[0]
ax.set_title('Before Densification', fontsize=12)

ell_big = Ell((3, 3), 4, 2, angle=30, fill=True, alpha=0.3, color='red',
              edgecolor='red', linewidth=2)
ax.add_patch(ell_big)
ax.annotate('Large G\n(high grad)', xy=(3, 3), fontsize=10, ha='center',
            fontweight='bold', color='red')

ell_small = Ell((7, 6), 1, 0.8, angle=-15, fill=True, alpha=0.3, color='blue',
                edgecolor='blue', linewidth=2)
ax.add_patch(ell_small)
ax.annotate('Small G\n(high grad)', xy=(7, 6), fontsize=10, ha='center',
            fontweight='bold', color='blue')

ell_trans = Ell((6, 2), 2, 1.5, angle=0, fill=True, alpha=0.05, color='gray',
                edgecolor='gray', linewidth=2, linestyle='--')
ax.add_patch(ell_trans)
ax.annotate('Low opacity\n(prune)', xy=(6, 2), fontsize=10, ha='center', color='gray')

ax.set_xlim(0, 10); ax.set_ylim(0, 8)
ax.set_aspect('equal'); ax.grid(True, alpha=0.2)

# After Split
ax = axes[1]
ax.set_title('After SPLIT (large -> 2 small)', fontsize=12)

ell_s1 = Ell((2, 3.5), 2, 1.2, angle=30, fill=True, alpha=0.4, color='orange',
             edgecolor='red', linewidth=2)
ell_s2 = Ell((4, 2.5), 2, 1.2, angle=30, fill=True, alpha=0.4, color='orange',
             edgecolor='red', linewidth=2)
ax.add_patch(ell_s1)
ax.add_patch(ell_s2)
ax.annotate('Split 1', xy=(2, 3.5), fontsize=10, ha='center')
ax.annotate('Split 2', xy=(4, 2.5), fontsize=10, ha='center')

ax.set_xlim(0, 10); ax.set_ylim(0, 8)
ax.set_aspect('equal'); ax.grid(True, alpha=0.2)

# After Clone
ax = axes[2]
ax.set_title('After CLONE (small -> original + copy)', fontsize=12)

ell_c1 = Ell((7, 6), 1, 0.8, angle=-15, fill=True, alpha=0.3, color='blue',
             edgecolor='blue', linewidth=2)
ell_c2 = Ell((7.3, 5.7), 1, 0.8, angle=-15, fill=True, alpha=0.3, color='cyan',
             edgecolor='blue', linewidth=2, linestyle='--')
ax.add_patch(ell_c1)
ax.add_patch(ell_c2)
ax.annotate('Original', xy=(7, 6.5), fontsize=10, ha='center')
ax.annotate('Clone\n(shifted)', xy=(7.3, 5.2), fontsize=10, ha='center', color='blue')

ax.set_xlim(0, 10); ax.set_ylim(0, 8)
ax.set_aspect('equal'); ax.grid(True, alpha=0.2)

plt.suptitle('Gaussian Densification: Split & Clone',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig3_densification.png"), dpi=150)
plt.close()
print("-> Figure saved: fig3_densification.png")

print("\nLab 5 complete!")
