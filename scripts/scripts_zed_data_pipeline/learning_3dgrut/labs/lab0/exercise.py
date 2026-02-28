#!/usr/bin/env python3
"""
Lab 0: 개념 기초 — Point Cloud→Gaussian, Splatting vs Ray Tracing, EKF vs UT
이론: theory.md 참고
실행: conda activate 3dgrut && python code.py
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

RESULTS = os.path.join(os.path.dirname(__file__), "results")
os.makedirs(RESULTS, exist_ok=True)

# ============================================================
# Part 1: Point Cloud → Gaussian 표현
# ============================================================

def make_gaussian_3d(mu, scales, rotation_euler_deg):
    """3D Gaussian 파라미터 생성: (μ, Σ)"""
    mu = np.array(mu, dtype=np.float64)
    s = np.array(scales, dtype=np.float64)
    S = np.diag(s)
    rx, ry, rz = np.radians(rotation_euler_deg)
    Rx = np.array([[1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]])
    Ry = np.array([[np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]])
    Rz = np.array([[np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]])
    R = Rz @ Ry @ Rx
    Sigma = R @ S.T @ S @ R.T  # Σ = R S² Rᵀ
    return mu, Sigma, R, S

gaussians = [
    make_gaussian_3d([0, 0, 0], [0.5, 0.2, 0.3], [0, 0, 0]),
    make_gaussian_3d([1.5, 0.5, 0], [0.3, 0.4, 0.2], [0, 0, 45]),
    make_gaussian_3d([0.5, -1, 0.5], [0.2, 0.2, 0.6], [30, 0, 0]),
]

# --- Fig 1: Gaussian basics ---
fig, axes = plt.subplots(1, 3, figsize=(18, 5))

# Point cloud vs Gaussian
ax1 = axes[0]
np.random.seed(42)
points = np.random.randn(50, 2) * 0.3
ax1.scatter(points[:, 0], points[:, 1], c='blue', s=10, alpha=0.5, label='Point Cloud')
mu_2d = np.array([0, 0])
cov_2d = np.array([[0.09, 0.02], [0.02, 0.06]])
evals, evecs = np.linalg.eigh(cov_2d)
angle = np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0]))
for ns in [1, 2, 3]:
    ell = Ellipse(mu_2d, 2*ns*np.sqrt(evals[0]), 2*ns*np.sqrt(evals[1]),
                  angle=angle, fill=False, color='red', linewidth=2-ns*0.5,
                  linestyle=['solid','dashed','dotted'][ns-1])
    ax1.add_patch(ell)
ax1.set_xlim(-1.5, 1.5); ax1.set_ylim(-1.5, 1.5)
ax1.set_aspect('equal'); ax1.set_title('Point Cloud vs Gaussian'); ax1.legend(); ax1.grid(True, alpha=0.3)

# Response heatmap
ax2 = axes[1]
xx, yy = np.meshgrid(np.linspace(-1.5, 1.5, 100), np.linspace(-1.5, 1.5, 100))
cov_ex = np.array([[0.3, 0.1], [0.1, 0.2]])
cov_ex_inv = np.linalg.inv(cov_ex)
zz = np.zeros_like(xx)
for i in range(xx.shape[0]):
    for j in range(xx.shape[1]):
        x = np.array([xx[i,j], yy[i,j]])
        zz[i,j] = np.exp(-0.5 * x @ cov_ex_inv @ x)
im = ax2.contourf(xx, yy, zz, levels=20, cmap='hot')
plt.colorbar(im, ax=ax2)
ax2.set_title('Gaussian Response'); ax2.set_aspect('equal')

# Multiple Gaussians
ax3 = axes[2]
colors = ['red', 'green', 'blue']
scene = np.zeros_like(xx)
for idx, (mu, Sigma, R, S) in enumerate(gaussians):
    S2d = Sigma[:2, :2]; S2d_inv = np.linalg.inv(S2d)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            d = np.array([xx[i,j], yy[i,j]]) - mu[:2]
            scene[i,j] += np.exp(-0.5 * d @ S2d_inv @ d)
    evals, evecs = np.linalg.eigh(S2d)
    ang = np.degrees(np.arctan2(evecs[1,0], evecs[0,0]))
    ax3.add_patch(Ellipse(mu[:2], 2*np.sqrt(evals[0]), 2*np.sqrt(evals[1]),
                           angle=ang, fill=False, color=colors[idx], linewidth=2))
ax3.contourf(xx, yy, scene, levels=20, cmap='viridis', alpha=0.7)
ax3.set_title('Multiple Gaussians = Scene'); ax3.set_xlim(-1.5, 3); ax3.set_ylim(-2.5, 2); ax3.set_aspect('equal')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig1_gaussian_basics.png"), dpi=150); plt.close()
print("-> fig1_gaussian_basics.png")

# ============================================================
# Part 2: Splatting vs Ray Tracing 비교도
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(16, 7))
cam = np.array([0, -3])
img_y = -1.5
gauss_c = [np.array([-1,2]), np.array([0.5,3]), np.array([1.5,1.5])]
gauss_s = [0.4, 0.6, 0.3]
gauss_rgb = ['#FF6B6B', '#4ECDC4', '#45B7D1']

# Splatting
ax = axes[0]; ax.set_title('Splatting (3DGS/3DGUT)')
ax.plot([-2,2], [img_y,img_y], 'k-', lw=3); ax.plot(0, -3, 'k^', ms=15)
for c, s, clr in zip(gauss_c, gauss_s, gauss_rgb):
    ax.add_patch(plt.Circle(c, s, color=clr, alpha=0.4))
    px = c[0]*(img_y+3)/(c[1]+3)
    ax.annotate('', xy=(px,img_y), xytext=c, arrowprops=dict(arrowstyle='->',color=clr,lw=1.5,ls='--'))
    ax.add_patch(plt.Rectangle((px-s*0.15, img_y-0.1), 2*s*0.15, 0.15, color=clr, alpha=0.7))
ax.set_xlim(-3,3); ax.set_ylim(-3.5,4.5); ax.set_aspect('equal'); ax.grid(True, alpha=0.2)

# Ray Tracing
ax = axes[1]; ax.set_title('Ray Tracing (3DGRT)')
ax.plot([-2,2], [img_y,img_y], 'k-', lw=3); ax.plot(0, -3, 'k^', ms=15)
for c, s, clr in zip(gauss_c, gauss_s, gauss_rgb):
    ax.add_patch(plt.Circle(c, s, color=clr, alpha=0.4))
for px in [-1.0, 0.0, 1.0]:
    d = np.array([px, img_y+3]); d = d/np.linalg.norm(d)
    ax.annotate('', xy=np.array([px,img_y])+d*7, xytext=(px,img_y),
                arrowprops=dict(arrowstyle='->',color='orange',lw=2))
    for c, s, clr in zip(gauss_c, gauss_s, gauss_rgb):
        t = np.dot(c - np.array([px,img_y]), d)
        closest = np.array([px,img_y]) + t*d
        if np.linalg.norm(closest - c) < s*1.5:
            ax.plot(closest[0], closest[1], '*', color='yellow', ms=12, mec='black', zorder=5)
ax.set_xlim(-3,3); ax.set_ylim(-3.5,4.5); ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig2_splatting_vs_raytracing.png"), dpi=150); plt.close()
print("-> fig2_splatting_vs_raytracing.png")

# ============================================================
# Part 3: EKF(Jacobian) vs UT(Sigma Points) 비교
# ============================================================

def pinhole_project(p, fx=500, fy=500, cx=320, cy=240):
    x,y,z = p; return np.array([fx*x/z+cx, fy*y/z+cy])

def fisheye_project(p, fx=500, fy=500, cx=320, cy=240, k1=-0.3, k2=0.1):
    x,y,z = p; xn,yn = x/z, y/z; r = np.sqrt(xn**2+yn**2)
    if r > 1e-8:
        theta = np.arctan(r); scale = theta*(1+k1*theta**2+k2*theta**4)/r
    else: scale = 1.0
    return np.array([fx*scale*xn+cx, fy*scale*yn+cy])

def ekf_project(mu, Sigma, proj_fn, **kw):
    """EKF/EWA: Σ_2d = J Σ Jᵀ"""
    mu_2d = proj_fn(mu, **kw)
    eps = 1e-5; J = np.zeros((2,3))
    for i in range(3):
        d = np.zeros(3); d[i] = eps
        J[:,i] = (proj_fn(mu+d, **kw) - proj_fn(mu-d, **kw))/(2*eps)
    return mu_2d, J @ Sigma @ J.T

def ut_project(mu, Sigma, proj_fn, **kw):
    """UT/3DGUT: sigma points -> exact projection"""
    n=3; lam=0.0  # alpha=1, kappa=0 -> lambda=0
    L = np.linalg.cholesky(n * Sigma)  # (n+lambda)*Sigma = n*Sigma when lambda=0
    sp = np.zeros((7,3)); sp[0] = mu
    for i in range(3):
        sp[i+1] = mu + L[:,i]; sp[3+i+1] = mu - L[:,i]
    # weights
    wm = np.full(7, 1/(2*n)); wm[0] = 0.0  # lambda/(n+lambda) = 0
    wc = wm.copy(); wc[0] = 2.0  # 0 + (1-1+2) = 2
    proj = np.array([proj_fn(s, **kw) for s in sp])
    mu2d = wm @ proj
    diff = proj - mu2d
    S2d = sum(wc[i]*np.outer(diff[i], diff[i]) for i in range(7))
    return mu2d, S2d, sp, proj

mu_3d = np.array([1.5, 1.0, 5.0])
Sigma_3d = np.array([[0.3,0.05,0],[0.05,0.2,0],[0,0,0.1]])

fig, axes = plt.subplots(2, 2, figsize=(14, 12))
for col, (fn, name, kw) in enumerate([
    (pinhole_project, "Pinhole", {}),
    (fisheye_project, "Fisheye (k1=-0.3)", {"k1":-0.3,"k2":0.1}),
]):
    mu_ekf, S_ekf = ekf_project(mu_3d, Sigma_3d, fn, **kw)
    mu_ut, S_ut, sp3d, sp2d = ut_project(mu_3d, Sigma_3d, fn, **kw)

    # 3D sigma points
    ax = axes[0, col]; ax.set_title(f'{name} - 3D Sigma Points')
    ev, evec = np.linalg.eigh(Sigma_3d[:2,:2])
    ang = np.degrees(np.arctan2(evec[1,0], evec[0,0]))
    for ns in [1,2]:
        ax.add_patch(Ellipse(mu_3d[:2], 2*ns*np.sqrt(ev[0]), 2*ns*np.sqrt(ev[1]),
                              angle=ang, fill=False, color='blue', alpha=0.5))
    ax.plot(sp3d[:,0], sp3d[:,1], 'r^', ms=10, label='Sigma Points')
    ax.plot(mu_3d[0], mu_3d[1], 'bo', ms=8, label='Mean')
    ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

    # 2D projected
    ax = axes[1, col]; ax.set_title(f'{name} - 2D Projection')
    for label, mu2d, S2d, clr, ls in [('EKF/EWA',mu_ekf,S_ekf,'blue','-'), ('UT/3DGUT',mu_ut,S_ut,'red','--')]:
        ev2, evec2 = np.linalg.eigh(S2d)
        ang2 = np.degrees(np.arctan2(evec2[1,0], evec2[0,0]))
        for ns in [1,2]:
            ax.add_patch(Ellipse(mu2d, 2*ns*np.sqrt(max(ev2[0],1e-10)),
                                  2*ns*np.sqrt(max(ev2[1],1e-10)), angle=ang2,
                                  fill=False, color=clr, lw=2, ls=ls,
                                  label=label if ns==1 else ''))
    ax.plot(sp2d[:,0], sp2d[:,1], 'r^', ms=8, alpha=0.7)
    mean_diff = np.linalg.norm(mu_ekf - mu_ut)
    cov_diff = np.linalg.norm(S_ekf - S_ut, 'fro')
    ax.text(0.02, 0.98, f'Mean diff: {mean_diff:.4f}\nCov diff: {cov_diff:.4f}',
            transform=ax.transAxes, fontsize=9, va='top',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

plt.suptitle('EKF/EWA vs UT/3DGUT (Pinhole: similar, Fisheye: UT wins)', fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, "fig3_ekf_vs_ut_projection.png"), dpi=150); plt.close()
print("-> fig3_ekf_vs_ut_projection.png")
print("Lab 0 done.")
