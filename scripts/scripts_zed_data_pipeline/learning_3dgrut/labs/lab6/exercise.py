#!/usr/bin/env python3
"""Lab 6: Real 3DGRUT Pipeline -- ZED data to 3D reconstruction.
See theory.md for full pipeline details and experiment ideas.
"""

import os
import sys

GRUT_ROOT = "/opt/3dgrut"

print("=" * 70)
print("Lab 6: Real 3DGRUT Pipeline")
print("=" * 70)

# ---- Pipeline overview ----
print("\nEnd-to-end pipeline: ZED bag -> 3D Gaussian Scene")
print("  Step 1: extract_zed_bag.py  -> stereo images + frames_meta.json")
print("  Step 2: cusfm_cli           -> camera poses + sparse 3D points")
print("  Step 3: run_depth.py        -> depth maps (16-bit PNG, mm)")
print("  Step 4: fuse_cusfm (nvblox) -> mesh.ply + occupancy_map.png")
print("  Step 5: train.py (3DGRUT)   -> Gaussian Scene checkpoint (.pt)")
print("  Step 6: render.py           -> novel view images")

# ---- Config structure ----
print(f"\nConfig directory: {GRUT_ROOT}/configs/")
print("  apps/colmap_3dgrt.yaml    -- COLMAP + Ray Tracing")
print("  apps/colmap_3dgut.yaml    -- COLMAP + Rasterization (UT)")
print("  apps/cusfm_3dgut.yaml     -- ZED pipeline + 3DGUT")
print("  dataset/colmap.yaml       -- COLMAP format")
print("  render/3dgrt.yaml         -- Ray tracing params")
print("  render/3dgut.yaml         -- Rasterization params")
print("  strategy/gs.yaml          -- Split/clone/prune")
print("  strategy/mcmc.yaml        -- MCMC densification")

# ---- Key training parameters ----
print("\nKey training parameters (base_gs.yaml):")
print("  n_iterations: 30000")
print("  loss: 0.8*L1 + 0.2*SSIM")
print("  lr: position=0.00016, density=0.05, features=0.0025")
print("  lr: rotation=0.001, scale=0.005")
print("  kernel_degree: 2, SH_degree: 3")
print("  densify_interval: 300, start: 500, end: 15000")
print("  UT params (3DGUT): alpha=1.0, beta=2.0, kappa=0.0")

# ---- Execution commands ----
print("\n" + "=" * 70)
print("Execution Guide")
print("=" * 70)

print(f"""
[A] NeRF Synthetic (simplest):
  cd {GRUT_ROOT}
  python train.py --config-name apps/nerf_synthetic_3dgrt.yaml \\
      path=data/nerf_synthetic/lego out_dir=runs experiment_name=lego

[B] COLMAP data:
  python train.py --config-name apps/colmap_3dgut.yaml \\
      path=data/mipnerf360/garden out_dir=runs experiment_name=garden

[C] ZED pipeline (this repo):
  ./scripts/zed_pipeline/run_zed_pipeline.sh data/recording.db3 output/scene
  cd {GRUT_ROOT}
  python train.py --config-name apps/cusfm_3dgut.yaml \\
      path=output/scene/extracted out_dir=runs experiment_name=zed \\
      initialization.fused_point_cloud_path=output/scene/nvblox/mesh.ply

[D] Custom photos + COLMAP:
  colmap automatic_reconstructor --image_path photos/ --workspace_path colmap_out/
  python train.py --config-name apps/colmap_3dgut.yaml path=colmap_out/ out_dir=runs
""")

# ---- Key source files ----
print("Key source files to read:")
print(f"  {GRUT_ROOT}/train.py                   -- entry point")
print(f"  {GRUT_ROOT}/threedgrut/trainer.py      -- training loop")
print(f"  {GRUT_ROOT}/threedgrut/model/model.py  -- MixtureOfGaussians")
print(f"  {GRUT_ROOT}/threedgrut/render.py       -- render()")
print(f"  {GRUT_ROOT}/threedgrut/strategy/       -- densification")
print(f"  {GRUT_ROOT}/threedgrt_tracer/          -- OptiX ray tracing (CUDA)")
print(f"  {GRUT_ROOT}/threedgut_tracer/          -- UT rasterization (CUDA)")

# ---- Troubleshooting ----
print("\nTroubleshooting:")
print("  CUDA OOM       -> dataset.downsample_factor=2 or 4")
print("  Slang compile  -> first run JIT is slow (minutes); check CUDA version")
print("  COLMAP format  -> needs sparse/0/{{cameras,images,points3D}}.bin")
print("  Blurry output  -> increase n_iterations or adjust densification")
print("  Holes in scene -> lower densify_grad_threshold or try MCMC strategy")

print("\nLab 6 complete!")
