#!/usr/bin/env python3
"""Lab 1: 3D Gaussian Representation - Quaternion, SH, Opacity, Covariance.
See theory.md for full mathematical derivations.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

RESULTS = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS, exist_ok=True)

# ---- Part 1: Quaternion <-> Rotation Matrix ----
print("=" * 70)
print("Part 1: Quaternion <-> Rotation Matrix")
print("=" * 70)


def quat_to_rotation_matrix(q):
    """Quaternion (w, x, y, z) -> 3x3 Rotation Matrix."""
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    return R


def rotation_matrix_to_quat(R):
    """Rotation Matrix -> Quaternion (Shepperd's method)."""
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([w, x, y, z])


# Round-trip verification
q_test = np.array([0.7071, 0.0, 0.7071, 0.0])  # y-axis 90 deg rotation
q_test = q_test / np.linalg.norm(q_test)
R_test = quat_to_rotation_matrix(q_test)
q_back = rotation_matrix_to_quat(R_test)

print(f"  Input quaternion:     {q_test}")
print(f"  Rotation matrix:\n{R_test}")
print(f"  Back to quaternion:   {q_back}")
print(f"  Round-trip error:     {np.linalg.norm(np.abs(q_test) - np.abs(q_back)):.2e}")

# ---- Part 2: Covariance = R^T S^T S R decomposition ----
print("\n" + "=" * 70)
print("Part 2: Covariance decomposition")
print("=" * 70)


class GaussianParticle:
    """Single Gaussian Particle with position, quaternion, log_scale, density, SH DC."""

    def __init__(self, position, quaternion, log_scale, density, sh_dc=None):
        self.position = np.array(position, dtype=np.float64)
        self.quaternion = np.array(quaternion, dtype=np.float64)
        self.quaternion /= np.linalg.norm(self.quaternion)
        self.log_scale = np.array(log_scale, dtype=np.float64)
        self.density = float(density)
        if sh_dc is None:
            self.sh_dc = np.array([0.5, 0.5, 0.5])
        else:
            self.sh_dc = np.array(sh_dc)

    @property
    def scale(self):
        return np.exp(self.log_scale)

    @property
    def rotation_matrix(self):
        return quat_to_rotation_matrix(self.quaternion)

    @property
    def covariance(self):
        R = self.rotation_matrix
        S = np.diag(self.scale)
        return R @ S.T @ S @ R.T

    @property
    def opacity(self):
        return 1.0 / (1.0 + np.exp(-self.density))

    def response(self, x):
        diff = x - self.position
        Sigma_inv = np.linalg.inv(self.covariance)
        return np.exp(-0.5 * diff @ Sigma_inv @ diff)

    def __repr__(self):
        return (f"GaussianParticle(pos={self.position}, "
                f"scale={self.scale}, opacity={self.opacity:.3f})")


particles = [
    GaussianParticle(
        position=[0, 0, 0],
        quaternion=[1, 0, 0, 0],
        log_scale=[np.log(0.5), np.log(0.3), np.log(0.2)],
        density=2.0,
        sh_dc=[0.8, 0.2, 0.2]
    ),
    GaussianParticle(
        position=[1.5, 0, 0],
        quaternion=[0.924, 0, 0, 0.383],  # z-axis 45 deg
        log_scale=[np.log(0.4), np.log(0.1), np.log(0.3)],
        density=1.0,
        sh_dc=[0.2, 0.8, 0.2]
    ),
    GaussianParticle(
        position=[0.5, 1.2, 0],
        quaternion=[0.866, 0.5, 0, 0],  # x-axis 60 deg
        log_scale=[np.log(0.3), np.log(0.3), np.log(0.5)],
        density=0.5,
        sh_dc=[0.2, 0.2, 0.8]
    ),
]

for i, p in enumerate(particles):
    print(f"\n  Particle {i}: {p}")
    print(f"    Covariance:\n{p.covariance}")
    print(f"    Eigenvalues of Sigma: {np.linalg.eigvalsh(p.covariance)}")
    print(f"    (All positive? {np.all(np.linalg.eigvalsh(p.covariance) > 0)} -> PSD guaranteed!)")

# ---- Visualization: 3D Gaussians ----
fig = plt.figure(figsize=(14, 6))

ax = fig.add_subplot(121, projection='3d')
ax.set_title('3D Gaussian Particles\n(ellipsoids = 1-sigma contour)')

for p in particles:
    eigenvalues, eigenvectors = np.linalg.eigh(p.covariance)
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))

    sphere_pts = np.stack([x.ravel(), y.ravel(), z.ravel()])
    transform = eigenvectors @ np.diag(np.sqrt(eigenvalues))
    ellipsoid_pts = transform @ sphere_pts + p.position[:, None]

    x_e = ellipsoid_pts[0].reshape(x.shape)
    y_e = ellipsoid_pts[1].reshape(y.shape)
    z_e = ellipsoid_pts[2].reshape(z.shape)

    ax.plot_surface(x_e, y_e, z_e, alpha=p.opacity * 0.5, color=p.sh_dc)
    ax.scatter(*p.position, s=50, color=p.sh_dc, edgecolors='black')

ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_xlim(-1, 3); ax.set_ylim(-1, 2); ax.set_zlim(-1, 1)

# XY slice heatmap
ax2 = fig.add_subplot(122)
ax2.set_title('XY slice (z=0) -- Gaussian Response Sum')
xx, yy = np.meshgrid(np.linspace(-1, 3, 200), np.linspace(-1, 2.5, 200))
response_map = np.zeros_like(xx)

for p in particles:
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            x_pt = np.array([xx[i, j], yy[i, j], 0.0])
            response_map[i, j] += p.opacity * p.response(x_pt)

im = ax2.contourf(xx, yy, response_map, levels=30, cmap='magma')
plt.colorbar(im, ax=ax2, label='Sum opacity * rho(x)')
ax2.set_xlabel('X'); ax2.set_ylabel('Y')
ax2.set_aspect('equal')

for p in particles:
    ax2.plot(p.position[0], p.position[1], 'w+', markersize=10, markeredgewidth=2)

plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig1_gaussian_particles.png"), dpi=150)
plt.close()
print("\n-> Figure saved: fig1_gaussian_particles.png")

# ---- Part 3: Spherical Harmonics ----
print("\n" + "=" * 70)
print("Part 3: Spherical Harmonics -- View-Dependent Color")
print("=" * 70)


def sh_basis(l, m, direction):
    """Spherical Harmonics basis Y_l^m(d). direction: unit vector [x, y, z]."""
    x, y, z = direction
    if l == 0:
        return 0.28209479
    if l == 1:
        if m == -1: return 0.48860251 * y
        if m == 0:  return 0.48860251 * z
        if m == 1:  return 0.48860251 * x
    if l == 2:
        if m == -2: return 1.09254843 * x * y
        if m == -1: return 1.09254843 * y * z
        if m == 0:  return 0.31539157 * (3*z*z - 1)
        if m == 1:  return 1.09254843 * x * z
        if m == 2:  return 0.54627422 * (x*x - y*y)
    if l == 3:
        if m == -3: return 0.59004360 * y * (3*x*x - y*y)
        if m == -2: return 2.89061144 * x * y * z
        if m == -1: return 0.45704579 * y * (5*z*z - 1)
        if m == 0:  return 0.37317633 * z * (5*z*z - 3)
        if m == 1:  return 0.45704579 * x * (5*z*z - 1)
        if m == 2:  return 1.44530572 * z * (x*x - y*y)
        if m == 3:  return 0.59004360 * x * (x*x - 3*y*y)
    return 0.0


def evaluate_sh_color(sh_coeffs, view_dir):
    """Evaluate view-dependent color from SH coefficients [3, 16]."""
    color = np.zeros(3)
    idx = 0
    for l in range(4):
        for m in range(-l, l + 1):
            basis = sh_basis(l, m, view_dir)
            color += sh_coeffs[:, idx] * basis
            idx += 1
    color = 1.0 / (1.0 + np.exp(-color))  # sigmoid
    return color


# Metallic red sphere SH coefficients
sh_coeffs = np.zeros((3, 16))
sh_coeffs[0, 0] = 2.0
sh_coeffs[1, 0] = -0.5
sh_coeffs[2, 0] = -0.5
sh_coeffs[0, 4:9] = [0.0, 0.0, 1.5, 0.0, 0.0]
sh_coeffs[1, 4:9] = [0.0, 0.0, 1.0, 0.0, 0.0]
sh_coeffs[2, 4:9] = [0.0, 0.0, 1.0, 0.0, 0.0]

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

for deg_max, ax_idx in [(0, 0), (1, 1), (3, 2)]:
    ax = axes[ax_idx]
    n_theta = 50
    n_phi = 100
    theta = np.linspace(0, np.pi, n_theta)
    phi = np.linspace(0, 2*np.pi, n_phi)
    color_map = np.zeros((n_theta, n_phi, 3))

    for i, t in enumerate(theta):
        for j, p in enumerate(phi):
            view_dir = np.array([
                np.sin(t) * np.cos(p),
                np.sin(t) * np.sin(p),
                np.cos(t)
            ])
            sh_truncated = np.zeros_like(sh_coeffs)
            n_coeffs = sum(2*l+1 for l in range(deg_max+1))
            sh_truncated[:, :n_coeffs] = sh_coeffs[:, :n_coeffs]
            color_map[i, j] = evaluate_sh_color(sh_truncated, view_dir)

    ax.imshow(color_map, extent=[0, 360, 180, 0], aspect='auto')
    ax.set_xlabel('Azimuth phi (deg)')
    ax.set_ylabel('Elevation theta (deg)')
    ax.set_title(f'SH Degree 0~{deg_max}\n({sum(2*l+1 for l in range(deg_max+1))} coefficients per channel)')

plt.suptitle('Spherical Harmonics: View-Dependent Color', fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig2_spherical_harmonics.png"), dpi=150)
plt.close()
print("-> Figure saved: fig2_spherical_harmonics.png")

# ---- Part 4: Generalized Gaussian Kernels ----
print("\n" + "=" * 70)
print("Part 4: Generalized Gaussian Kernels")
print("=" * 70)

fig, ax = plt.subplots(1, 1, figsize=(10, 6))

r = np.linspace(0, 4, 500)
for n, style, label in [(1, '-', 'Standard (n=1): exp(-1/2 r^2)'),
                         (2, '--', 'Degree 2: exp(-(1/2 r^2)^2)'),
                         (3, ':', 'Degree 3: exp(-(1/2 r^2)^3)')]:
    response = np.exp(-(0.5 * r**2)**n)
    ax.plot(r, response, style, linewidth=2.5, label=label)

ax.axhline(y=0.01, color='gray', linestyle='-.', alpha=0.5, label='alpha_min = 0.01 (cutoff)')
ax.set_xlabel('Mahalanobis Distance r', fontsize=12)
ax.set_ylabel('Response rho(r)', fontsize=12)
ax.set_title('Generalized Gaussian Kernels', fontsize=13)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig3_generalized_kernels.png"), dpi=150)
plt.close()
print("-> Figure saved: fig3_generalized_kernels.png")

# ---- Covariance decomposition utility ----

def decompose_covariance(Sigma):
    """Decompose covariance into quaternion + scale via eigendecomposition."""
    eigenvalues, eigenvectors = np.linalg.eigh(Sigma)
    scales = np.sqrt(eigenvalues)
    R = eigenvectors.T
    if np.linalg.det(R) < 0:
        R = -R
    q = rotation_matrix_to_quat(R)
    return q, scales


# Verification
test_p = particles[1]
q_recovered, s_recovered = decompose_covariance(test_p.covariance)
Sigma_recovered = quat_to_rotation_matrix(q_recovered).T @ np.diag(s_recovered**2) @ quat_to_rotation_matrix(q_recovered)

print(f"\n  Original scale:    {test_p.scale}")
print(f"  Recovered scale:   {np.sort(s_recovered)}")
print(f"  Sigma recovery error: {np.linalg.norm(test_p.covariance - Sigma_recovered):.2e}")

print("\nLab 1 complete!")
