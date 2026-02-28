#!/usr/bin/env python3
"""Lab 4: Unscented Transform -- from UKF to 3DGUT projection.
See theory.md for UT accuracy analysis and camera model details.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

RESULTS = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS, exist_ok=True)

# ---- Part 1: EWA Splatting limitations (see theory.md) ----
print("=" * 70)
print("Part 1: EWA Splatting -- Jacobian linearization limitations")
print("=" * 70)

# ---- Part 2: UT Sigma Points ----
print("\n" + "=" * 70)
print("Part 2: UT Sigma Points (same as UKF)")
print("=" * 70)


class UnscentedTransform:
    """Unscented Transform -- identical to UKF sigma point machinery.
    3DGUT applies this with n=3 for 3D Gaussian projection.
    """

    def __init__(self, n, alpha=1.0, beta=2.0, kappa=0.0):
        self.n = n
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lam = alpha**2 * (n + kappa) - n

        self.w_mean = np.zeros(2*n + 1)
        self.w_cov = np.zeros(2*n + 1)

        self.w_mean[0] = self.lam / (n + self.lam)
        self.w_cov[0] = self.lam / (n + self.lam) + (1 - alpha**2 + beta)

        for i in range(1, 2*n + 1):
            self.w_mean[i] = 1.0 / (2 * (n + self.lam))
            self.w_cov[i] = 1.0 / (2 * (n + self.lam))

        print(f"  UT configured: n={n}, alpha={alpha}, beta={beta}, kappa={kappa}")
        print(f"  lambda = {self.lam:.4f}, sigma points: {2*n+1}")
        print(f"  Weights sum (mean): {np.sum(self.w_mean):.6f}")

    def generate_sigma_points(self, mu, Sigma):
        """Generate 2n+1 sigma points via Cholesky decomposition."""
        n = self.n
        sqrt_matrix = np.linalg.cholesky((n + self.lam) * Sigma)

        sigma_points = np.zeros((2*n + 1, n))
        sigma_points[0] = mu

        for i in range(n):
            sigma_points[i + 1] = mu + sqrt_matrix[:, i]
            sigma_points[n + i + 1] = mu - sqrt_matrix[:, i]

        return sigma_points

    def transform(self, sigma_points, transform_fn):
        """Transform sigma points through nonlinear function, recover mean & cov."""
        y_points = np.array([transform_fn(sp) for sp in sigma_points])
        m = y_points.shape[1]

        y_mean = np.sum(self.w_mean[:, None] * y_points, axis=0)

        y_cov = np.zeros((m, m))
        for i in range(2*self.n + 1):
            diff = y_points[i] - y_mean
            y_cov += self.w_cov[i] * np.outer(diff, diff)

        return y_mean, y_cov, y_points


print("\n3DGUT UT configuration (n=3, alpha=1, beta=2, kappa=0):")
ut = UnscentedTransform(n=3, alpha=1.0, beta=2.0, kappa=0.0)

# ---- Part 3: Camera models & UT projection comparison ----
print("\n" + "=" * 70)
print("Part 3: Gaussian projection across camera models")
print("=" * 70)


def pinhole_project(p, fx=500, fy=500, cx=320, cy=240):
    x, y, z = p
    if z <= 0: return np.array([cx, cy])
    return np.array([fx * x / z + cx, fy * y / z + cy])


def fisheye_equidistant(p, fx=500, fy=500, cx=320, cy=240):
    x, y, z = p
    r3d = np.sqrt(x**2 + y**2 + z**2)
    if r3d < 1e-10: return np.array([cx, cy])
    theta = np.arccos(np.clip(z / r3d, -1, 1))
    r2d = np.sqrt(x**2 + y**2)
    if r2d < 1e-10: return np.array([cx, cy])
    u = fx * theta * x / r2d + cx
    v = fy * theta * y / r2d + cy
    return np.array([u, v])


def fisheye_equisolid(p, fx=500, fy=500, cx=320, cy=240):
    x, y, z = p
    r3d = np.sqrt(x**2 + y**2 + z**2)
    if r3d < 1e-10: return np.array([cx, cy])
    theta = np.arccos(np.clip(z / r3d, -1, 1))
    theta_d = 2 * np.sin(theta / 2)
    r2d = np.sqrt(x**2 + y**2)
    if r2d < 1e-10: return np.array([cx, cy])
    u = fx * theta_d * x / r2d + cx
    v = fy * theta_d * y / r2d + cy
    return np.array([u, v])


def opencv_distortion(p, fx=500, fy=500, cx=320, cy=240, k1=-0.3, k2=0.1, p1=0.0, p2=0.0):
    x, y, z = p
    if z <= 0: return np.array([cx, cy])
    xn, yn = x / z, y / z
    r2 = xn**2 + yn**2
    radial = 1 + k1 * r2 + k2 * r2**2
    xd = xn * radial + 2*p1*xn*yn + p2*(r2 + 2*xn**2)
    yd = yn * radial + p1*(r2 + 2*yn**2) + 2*p2*xn*yn
    return np.array([fx * xd + cx, fy * yd + cy])


# 3D Gaussian to project
mu_3d = np.array([1.0, 0.5, 4.0])
Sigma_3d = np.array([
    [0.15, 0.03, 0.01],
    [0.03, 0.10, 0.02],
    [0.01, 0.02, 0.08]
])

camera_models = [
    ("Pinhole", pinhole_project),
    ("Fisheye\n(Equidistant)", fisheye_equidistant),
    ("Fisheye\n(Equisolid)", fisheye_equisolid),
    ("OpenCV\n(k1=-0.3, k2=0.1)", opencv_distortion),
]

fig, axes = plt.subplots(2, 4, figsize=(20, 10))

for col, (name, proj_fn) in enumerate(camera_models):
    sigma_points = ut.generate_sigma_points(mu_3d, Sigma_3d)

    # EWA (Jacobian) method
    eps = 1e-5
    J = np.zeros((2, 3))
    mu_2d_ewa = proj_fn(mu_3d)
    for i in range(3):
        delta = np.zeros(3)
        delta[i] = eps
        J[:, i] = (proj_fn(mu_3d + delta) - proj_fn(mu_3d - delta)) / (2 * eps)
    Sigma_2d_ewa = J @ Sigma_3d @ J.T

    # UT method
    mu_2d_ut, Sigma_2d_ut, projected_pts = ut.transform(sigma_points, proj_fn)

    # Ground truth (Monte Carlo)
    np.random.seed(42)
    n_mc = 10000
    samples_3d = np.random.multivariate_normal(mu_3d, Sigma_3d, n_mc)
    samples_2d = np.array([proj_fn(s) for s in samples_3d])
    mu_2d_mc = np.mean(samples_2d, axis=0)
    Sigma_2d_mc = np.cov(samples_2d.T)

    # Top row: projected Gaussian ellipses
    ax = axes[0, col]
    ax.set_title(f'{name}', fontsize=11)

    ax.scatter(samples_2d[::10, 0], samples_2d[::10, 1], s=1, alpha=0.2, c='gray',
               label='MC samples')

    # EWA ellipse
    for n_sig, ls in [(1, '-'), (2, '--')]:
        evals, evecs = np.linalg.eigh(Sigma_2d_ewa)
        angle = np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0]))
        ell = Ellipse(mu_2d_ewa, 2*n_sig*np.sqrt(max(evals[0], 1e-10)),
                      2*n_sig*np.sqrt(max(evals[1], 1e-10)), angle=angle,
                      fill=False, color='blue', linewidth=2, linestyle=ls)
        ax.add_patch(ell)

    # UT ellipse
    for n_sig, ls in [(1, '-'), (2, '--')]:
        evals, evecs = np.linalg.eigh(Sigma_2d_ut)
        angle = np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0]))
        ell = Ellipse(mu_2d_ut, 2*n_sig*np.sqrt(max(evals[0], 1e-10)),
                      2*n_sig*np.sqrt(max(evals[1], 1e-10)), angle=angle,
                      fill=False, color='red', linewidth=2, linestyle=ls)
        ax.add_patch(ell)

    ax.plot(projected_pts[:, 0], projected_pts[:, 1], 'r^', markersize=8, zorder=5)

    # MC ground truth ellipse
    for n_sig in [1]:
        evals, evecs = np.linalg.eigh(Sigma_2d_mc)
        angle = np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0]))
        ell = Ellipse(mu_2d_mc, 2*n_sig*np.sqrt(max(evals[0], 1e-10)),
                      2*n_sig*np.sqrt(max(evals[1], 1e-10)), angle=angle,
                      fill=False, color='green', linewidth=2, linestyle='-.')
        ax.add_patch(ell)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    if col == 0:
        ax.legend(['MC', 'EWA (Jacobian)', 'UT (Sigma Points)', 'MC (1-sigma)'],
                  fontsize=7, loc='upper left')

    # Bottom row: error comparison
    ax = axes[1, col]
    metrics = {
        'Mean Error\n(EWA)': np.linalg.norm(mu_2d_ewa - mu_2d_mc),
        'Mean Error\n(UT)': np.linalg.norm(mu_2d_ut - mu_2d_mc),
        'Cov Error\n(EWA)': np.linalg.norm(Sigma_2d_ewa - Sigma_2d_mc, 'fro'),
        'Cov Error\n(UT)': np.linalg.norm(Sigma_2d_ut - Sigma_2d_mc, 'fro'),
    }
    colors_bar = ['#4a90d9', '#d94a4a', '#4a90d9', '#d94a4a']
    bars = ax.bar(range(len(metrics)), list(metrics.values()), color=colors_bar)
    ax.set_xticks(range(len(metrics)))
    ax.set_xticklabels(list(metrics.keys()), fontsize=8)
    ax.set_ylabel('Error vs Monte Carlo')
    ax.grid(True, alpha=0.3)

    for bar, val in zip(bars, metrics.values()):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                f'{val:.2f}', ha='center', va='bottom', fontsize=8)

plt.suptitle('EWA (Jacobian/EKF) vs UT (3DGUT/UKF) -- Multiple Camera Models\n'
             'Green = Ground Truth, Blue = EWA, Red = UT',
             fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig1_ewa_vs_ut_cameras.png"), dpi=150)
plt.close()
print("-> Figure saved: fig1_ewa_vs_ut_cameras.png")

# ---- Part 4: UT accuracy analysis (1D example) ----
print("\n" + "=" * 70)
print("Part 4: UT accuracy -- Taylor expansion perspective")
print("=" * 70)


def nonlinear_func(x):
    return np.arctan(x)


mu_1d = 2.0
sigma_1d = 0.8

# EKF (1st order)
J_1d = 1.0 / (1 + mu_1d**2)
mu_ekf = nonlinear_func(mu_1d)
var_ekf = J_1d**2 * sigma_1d**2

# UT (1D: 3 sigma points)
ut_1d = UnscentedTransform(n=1, alpha=1.0, beta=2.0, kappa=0.0)
sp_1d = ut_1d.generate_sigma_points(np.array([mu_1d]), np.array([[sigma_1d**2]]))
mu_ut, var_ut, _ = ut_1d.transform(sp_1d, lambda x: np.array([nonlinear_func(x[0])]))

# Ground truth (Monte Carlo)
samples = np.random.normal(mu_1d, sigma_1d, 100000)
y_samples = nonlinear_func(samples)
mu_mc = np.mean(y_samples)
var_mc = np.var(y_samples)

fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# Input distribution
ax = axes[0]
x_range = np.linspace(mu_1d - 3*sigma_1d, mu_1d + 3*sigma_1d, 200)
pdf_x = np.exp(-0.5 * ((x_range - mu_1d) / sigma_1d)**2) / (sigma_1d * np.sqrt(2*np.pi))
ax.fill_between(x_range, 0, pdf_x, alpha=0.3, color='blue')
ax.plot(x_range, pdf_x, 'b-', linewidth=2, label='Input: N(mu, sigma^2)')

ax.plot([sp[0] for sp in sp_1d], [0] * len(sp_1d), 'r^', markersize=15, zorder=5,
        label='UT Sigma Points')

ax2 = ax.twinx()
y_range = nonlinear_func(x_range)
ax2.plot(x_range, y_range, 'g-', linewidth=2, label='y = atan(x)')
y_linear = mu_ekf + J_1d * (x_range - mu_1d)
ax2.plot(x_range, y_linear, 'r--', linewidth=2, label='EKF linear approx')

ax.set_xlabel('x')
ax.set_ylabel('p(x)', color='blue')
ax2.set_ylabel('y = atan(x)', color='green')
ax.set_title('Input Distribution + Nonlinear Function')
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, fontsize=9)

# Output distributions
ax = axes[1]
y_range_out = np.linspace(0, np.pi/2, 200)

std_ekf = np.sqrt(var_ekf)
pdf_ekf = np.exp(-0.5 * ((y_range_out - mu_ekf) / std_ekf)**2) / (std_ekf * np.sqrt(2*np.pi))
ax.plot(y_range_out, pdf_ekf, 'b-', linewidth=2, label=f'EKF: mu={mu_ekf:.3f}, sigma={std_ekf:.3f}')

std_ut = np.sqrt(var_ut[0, 0])
pdf_ut = np.exp(-0.5 * ((y_range_out - mu_ut[0]) / std_ut)**2) / (std_ut * np.sqrt(2*np.pi))
ax.plot(y_range_out, pdf_ut, 'r--', linewidth=2, label=f'UT: mu={mu_ut[0]:.3f}, sigma={std_ut:.3f}')

ax.hist(y_samples, bins=100, density=True, alpha=0.3, color='green', label='MC Ground Truth')
ax.axvline(x=mu_mc, color='green', linestyle=':', linewidth=2, label=f'MC mu={mu_mc:.3f}')
ax.axvline(x=mu_ekf, color='blue', linestyle=':', linewidth=1.5)
ax.axvline(x=mu_ut[0], color='red', linestyle=':', linewidth=1.5)

ax.set_xlabel('y = atan(x)')
ax.set_ylabel('p(y)')
ax.set_title('Output Distribution Comparison\n(UT closer to MC ground truth)')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

plt.suptitle('EKF vs UT accuracy: y = atan(x), x ~ N(2, 0.64)',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig2_ut_accuracy.png"), dpi=150)
plt.close()
print("-> Figure saved: fig2_ut_accuracy.png")

# ---- Part 5: MLAB (Multi-Layer Alpha Blending) ----
print("\n" + "=" * 70)
print("Part 5: MLAB hit buffer strategy")
print("=" * 70)


def mlab_hit_buffer(hits, k=16):
    """Multi-Layer Alpha Blending hit buffer.
    Maintains k closest hits sorted by depth.
    """
    buffer = []
    for tau, idx, alpha in hits:
        if len(buffer) < k:
            buffer.append((tau, idx, alpha))
            buffer.sort(key=lambda x: x[0])
        else:
            if tau < buffer[-1][0]:
                buffer[-1] = (tau, idx, alpha)
                buffer.sort(key=lambda x: x[0])
    return buffer


np.random.seed(42)
random_hits = [(np.random.uniform(1, 20), i, np.random.uniform(0.1, 0.9))
               for i in range(30)]

buffer_4 = mlab_hit_buffer(random_hits, k=4)
buffer_8 = mlab_hit_buffer(random_hits, k=8)
buffer_16 = mlab_hit_buffer(random_hits, k=16)

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

all_taus = [h[0] for h in random_hits]

for ax, buf, k in [(axes[0], buffer_4, 4), (axes[1], buffer_8, 8), (axes[2], buffer_16, 16)]:
    ax.bar(range(len(all_taus)), sorted(all_taus), color='lightgray',
           edgecolor='gray', alpha=0.5, label='All hits')
    buf_taus = sorted([b[0] for b in buf])
    ax.bar(range(len(buf_taus)), buf_taus, color='lightgreen',
           edgecolor='green', label=f'MLAB k={k}')
    ax.set_title(f'MLAB k={k}\n{k}/{len(random_hits)} closest hits retained')
    ax.set_xlabel('Index (sorted by depth)')
    ax.set_ylabel('tau (depth)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

plt.suptitle('MLAB Hit Buffer: larger k = more accurate but more memory',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig3_mlab_buffer.png"), dpi=150)
plt.close()
print("-> Figure saved: fig3_mlab_buffer.png")

print("\nLab 4 complete!")
