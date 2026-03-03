# Teleop Pipeline (USDZ → Isaac Sim → Teleop)

This pipeline opens a **USDZ map** produced by ZED/3DGRUT in Isaac Sim, optionally imports an **nvblox mesh**, spawns a robot, runs teleoperation, captures frames, and (if configured) runs **Fixer → video** when you press **Stop** on the timeline.

Mesh handling is **auto** or **skip** only:
- `auto` imports a mesh only when the map has **no mesh geometry**.
- `skip` assumes the map already contains mesh geometry.

Mesh geometry is hidden for rendering, but colliders are applied/kept.

---

## Pipeline Overview

```
USDZ (3DGRUT) → Isaac Sim Open → [Mesh auto/skip + Collider] → Teleop → Capture → Fixer → Video
```

Inputs
- `export_last.usdz` (3DGRUT output)
- `mesh.ply` (nvblox output, optional)
- Robot USD (optional)

Outputs
- Isaac Sim session running
- (Optional) USD map saved with mesh included
- (Optional) Teleop capture frames
- (Optional) Fixer output + videos

---

## Desired Output Structure

When `CAPTURE_DIR` is set, each run creates a new trial folder:

```
CAPTURE_DIR/
  teleop_YYYYMMDD_HHMMSS/
    capture/        # raw frames
    fixer/          # Fixer output frames
    before.mp4      # video from raw frames
    after.mp4       # video from Fixer output
    comparison.mp4  # side-by-side video
```

---

## Quick Start (all-in-one)

This runs **map load → teleop → capture → Fixer → video**.

```bash
cd scripts/scripts_teleop_pipeline
cp .env.example .env

# Edit .env (ISAAC_SIM_PYTHON, ROS_SETUP_SCRIPT, DEFAULT_MAP_PATH, OMNI_SERVER, OMNI_USER, OMNI_PASS)
# Optional: MESH_MODE, MESH_PATH, CAPTURE_DIR, FIXER_SOURCE_PATH.

./start.sh --map "/path/to/map" \
  --teleop joystick \
  --capture-dir "/path/to/capture_root"
```

Notes
- After Isaac Sim finishes importing, the timeline **auto-plays**.
- **Pause is auto-resumed**. Only **Stop** ends the run.
- Fixer runs on **Stop** only when `RUN_FIXER_TO_VIDEO=1` (or `--run-fixer`).
- If `CAPTURE_MAX_FRAMES` is reached, the simulation stops and Fixer runs (if enabled).
- If Fixer is **not installed/configured**, the pipeline will fail at the Fixer step with an error. Make sure Fixer is installed before relying on the auto-run.
---

## Step 0. Prerequisites

Required
- Isaac Sim installed
- ROS 2 (for teleop)
- `joy`, `teleop_twist_joy` if using a joystick

Optional
- nvblox mesh (`.ply/.obj/.fbx/.stl`)
- Docker + ffmpeg + Hugging Face CLI (for Fixer automation)
- Nucleus account (if using omniverse paths)

---

## Step 1. Environment setup

```bash
cd scripts/scripts_teleop_pipeline
cp .env.example .env
```

Minimum required in `.env`
- `ISAAC_SIM_PYTHON`
- `ROS_SETUP_SCRIPT`
- `DEFAULT_MAP_PATH` (USDZ, **required**)

Mesh options (required choice, default `skip`)
- `MESH_MODE=skip` (default) or `MESH_MODE=auto`
- `MESH_PATH` (required for `auto`)
- `MESH_OFFSET`, `MESH_ROT`, `MESH_SCALE`
- `MESH_PRIM_PATH` (where auto-import is referenced)
- `SAVE_MAP_PATH` (optional save path)
- `MESH_USD_CACHE_DIR` (optional mesh conversion cache)
- `MESH_FORCE_CONVERT=1` to force mesh USD reconversion

Robot options (optional)
- `ROBOT_USD_PATH`
- `ROBOT_SPAWN_POS`
- `ROBOT_SPAWN_QUAT` (w,x,y,z)
- `ROBOT_SPAWN_ROT` (roll,pitch,yaw degrees)
- `ROBOT_SPAWN_YAW` (degrees, convenience)

Capture options (optional)
- `CAPTURE_DIR` (base directory for trials)
- `CAPTURE_CAMERA_PATH`, `CAPTURE_RES`
- `CAPTURE_EVERY`, `CAPTURE_MAX_FRAMES`

Fixer options (optional, for auto Fixer run)
- `FIXER_SOURCE_PATH` (local Fixer repo path)
- `FIXER_CONTAINER_WORKDIR`, `FIXER_CONTAINER_INPUT_DIR`, `FIXER_MODEL_PATH`
- `FIXER_FPS`, `FIXER_TIMESTEP`

Omniverse login (optional)
- `OMNI_SERVER`, `OMNI_USER`, `OMNI_PASS`

Notes
- `DEFAULT_MAP_PATH` has no default and must be set.
- `MESH_MODE=skip` assumes the map already contains mesh geometry (any `Mesh` prim under `/World/Map`).
- `MESH_MODE=auto` is allowed only when the map does **not** already contain mesh geometry.
- Empty Fixer values in `.env` do **not** override shell env vars.
- `SAVE_MAP_ON_START=1` auto-saves after auto mesh import.

---

## Step 2. Mesh handling

### Option A. Auto mesh import (map has no mesh geometry)
Assumes USDZ and mesh are in the **same coordinate system**. Offset/rotation/scale can correct alignment.
Auto mode is allowed only if the loaded map does **not** already contain mesh geometry.
The imported mesh is referenced at `MESH_PRIM_PATH`, colliders are applied, and visibility is hidden.

```bash
# .env
MESH_MODE=auto
MESH_PATH=/path/to/nvblox/mesh.ply
MESH_OFFSET=0,0,0
MESH_ROT=0,0,0
MESH_SCALE=1,1,1
SAVE_MAP_PATH=/path/to/maps/zed_map_with_mesh.usd
```

Direct run (Isaac Sim Python)
```bash
source "$ROS_SETUP_SCRIPT"
"$ISAAC_SIM_PYTHON" scripts/scripts_teleop_pipeline/run_pipeline.py \
  --map "$DEFAULT_MAP_PATH" \
  --mesh "$MESH_PATH" \
  --save-map "$SAVE_MAP_PATH"
```

If auto import fails (e.g., axis mismatch), you will see an English warning telling you to align the map manually and rerun with `--mesh-mode skip`.

### Option B. Skip (map already contains mesh)
Use this when you already have an aligned map with mesh/collider baked in.
If the map does **not** contain mesh geometry, the script will warn and exit.
Mesh geometry is hidden for rendering.

```bash
# .env
MESH_MODE=skip
DEFAULT_MAP_PATH=/path/to/aligned_map_with_mesh.usd
```

---

## Step 3. Run teleop

Recommended (all-in-one):
```bash
./start.sh --map "$DEFAULT_MAP_PATH" --teleop joystick --capture-dir "/path/to/capture_root"
```

Direct run (Isaac Sim Python):
```bash
source "$ROS_SETUP_SCRIPT"
"$ISAAC_SIM_PYTHON" scripts/scripts_teleop_pipeline/run_pipeline.py \
  --map "$DEFAULT_MAP_PATH" \
  --teleop joystick \
  --capture-dir "/path/to/capture_root"
```

Teleop modes
- `--teleop joystick` (default): launches `teleop_twist_joy` using `teleop_twist_joy.yaml`.
- `--teleop keyboard`: launches `teleop_twist_keyboard` in a terminal.
- `--teleop none`: runs without teleop.

Timeline behavior
- After imports finish, the timeline **auto-plays**.
- **Pause is auto-resumed**. Only **Stop** ends the run.
- On **Stop** (or when `CAPTURE_MAX_FRAMES` is reached), Fixer runs **only if** `RUN_FIXER_TO_VIDEO=1` (or `--run-fixer`).

---

## Step 4. Teleop capture (robot camera view)

Capture frames from a specified robot camera view. `CAPTURE_DIR` is the **base** directory; each run creates a new trial folder.

Example env:
```bash
# .env
CAPTURE_DIR=/path/to/capture_root
CAPTURE_CAMERA_PATH=/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left
CAPTURE_RES=1920,1080
CAPTURE_EVERY=1
CAPTURE_MAX_FRAMES=1000
```

- If you're doing ./start.sh script pipeline, then you only need step 4-5 below.
- Otherwise, you need to follow step 1-3 below.

Manual Capture flow (Script Editor)
1. Launch Isaac Sim (UI).
2. Open **Window → Script Editor**.
3. Run a capture script in the Script Editor.
- Example Script Editor snippet (matches pipeline defaults):
```python
import omni.replicator.core as rep

capture_dir = "/path/to/capture_root"
camera_path = "/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left"
res = (1920, 1080)

render_product = rep.create.render_product(camera_path, res)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=capture_dir, rgb=True)
writer.attach([render_product])
```

4. Start the simulation and drive the robot.
5. Press **Stop** on the timeline when done (or rely on `CAPTURE_MAX_FRAMES`).


Notes for Script Editor usage
- This writes frames into the folder you set in `capture_dir`.
- The pipeline itself will still create a **trial folder** when using `CAPTURE_DIR`. If you capture manually, you control the exact output path.
- When you press **Stop**, the pipeline (if running) will trigger Fixer.

Trial directory layout:
```
CAPTURE_DIR/
  teleop_YYYYMMDD_HHMMSS/
    capture/        # raw frames
```

Notes
- `CAPTURE_MAX_FRAMES=0` means unlimited capture.
- When the max is reached, the simulation stops and Fixer runs (if enabled).

---

## Step 5. Fixer → video output

Fixer runs automatically when the timeline stops **only if** `RUN_FIXER_TO_VIDEO=1` (or `--run-fixer`) and Fixer env is configured.
You can also run it manually.

Required Fixer env
- `FIXER_SOURCE_PATH`: local Fixer repo path (contains `Dockerfile.cosmos` and `src/`)
- `FIXER_CONTAINER_WORKDIR`, `FIXER_CONTAINER_INPUT_DIR`, `FIXER_MODEL_PATH`


If Fixer is not installed, follow the steps below.

### Fixer install (local repo + model)
```bash
git clone https://github.com/nv-tlabs/Fixer.git
cd Fixer
mkdir -p models
hf download nvidia/Fixer --local-dir models
```

Build Fixer Docker image (required for `fixer_to_video.sh`)
```bash
docker build -t fixer-cosmos-env -f Dockerfile.cosmos .
```

### Manual run (auto-detects latest trial)
```bash
./fixer_to_video.sh --input "/path/to/capture_root"
```


Input resolution rules
- If `--input` points to a **trial root** (contains `capture/`), it uses `trial/capture` → `trial/fixer` and writes videos to the trial root.
- If `--input` points to a **base dir** that contains `teleop_*`, it picks the latest trial automatically.
- If `--input` points directly to a **capture/** folder, it uses that folder and writes videos next to it by default.

Advanced
- Use `--output` or `--video-dir` to override defaults.
- `AUTO_FIXER_SETUP=1` will auto-clone/build/download (requires `FIXER_GIT_URL`, `FIXER_HF_REPO`).
- `RUN_FIXER_TO_VIDEO=1` runs Fixer after Isaac Sim exits if it did not already run on timeline stop.
- You can pass the Fixer repo path via `./start.sh --fixer-source-path "/path/to/Fixer"` (overrides `.env`).

Desirable output directory
```
CAPTURE_DIR/
  teleop_YYYYMMDD_HHMMSS/
    capture/        # raw frames
    fixer/          # Fixer output frames
    before.mp4      # video from raw frames
    after.mp4       # video from Fixer output
    comparison.mp4  # side-by-side video
```
