# Teleop Pipeline (USDZ → Isaac Sim → Teleop)

This pipeline opens a **USDZ map** produced by ZED/3DGRUT in Isaac Sim and runs teleoperation (joystick/keyboard).
Optionally, it can load an **nvblox mesh (.ply)** and attach colliders. Auto alignment is not possible, so adjust
offset/rotation or follow the manual alignment steps when needed.

---

## Pipeline Overview

```
USDZ (3DGRUT) → Isaac Sim Open → (optional) Mesh Import + Collider → Teleop
```

Inputs
- `export_last.usdz` (3DGRUT output)
- `mesh.ply` (nvblox output, optional)

Outputs
- Isaac Sim session running
- (Optional) USD map saved with mesh included
- (Optional) Teleop capture frames

---

## Quick Start (all-in-one)

One command to run **map load → teleop → capture → Fixer → video**:

```bash
./start.sh --map "/path/to/export_last.usdz" --teleop joystick \
  --capture-dir "/path/to/teleop_frames" \
  --capture-camera "/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left" \
  --capture-res "1920,1080" \
  --capture-every 1 \
  --capture-max-frames 1000 \
  --run-fixer \
  --fixer-input "/path/to/teleop_frames"
```

---

## Step 0. Prerequisites

Required
- Isaac Sim installed
- ROS 2 (for teleop)
- `joy`, `teleop_twist_joy` if using a joystick

Optional
- nvblox mesh (`.ply`)
- Nucleus account (if using omniverse paths)

---

## Step 1. Environment setup

```bash
cd scripts/scripts_teleop_pipeline
cp .env.example .env
```

Minimum required in `.env`
- `ISAAC_SIM_PYTHON`
- `DEFAULT_MAP_PATH` (USDZ, **required**)
- `ROS_SETUP_SCRIPT`

Mesh options (optional)
- `MESH_PATH` (mesh.ply)
- `MESH_MODE=auto`
- `MESH_OFFSET`, `MESH_ROT`, `MESH_SCALE`
- `SAVE_MAP_PATH` (path to save map with mesh)

Notes
- `DEFAULT_MAP_PATH` has no default and must be set.
- If `MESH_USD_CACHE_DIR` is empty, it uses `scripts/scripts_teleop_pipeline/_mesh_cache`.

---

## Step 2. Mesh handling

### Option A. Auto mesh import
Assumes USDZ and mesh are in the **same coordinate system**.
Use offset/rotation/scale to correct alignment when needed.
PLY conversion uses the Isaac Sim asset converter.

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

### Option B. Manual alignment
Use this if auto alignment does not match.

1. Run with `MESH_MODE=manual`.
2. Import the mesh directly in Isaac Sim.
3. Align position/rotation, then add a collider.
4. Save the USD and set it as `DEFAULT_MAP_PATH`.

You can also add a **plane mesh** to create a ground collider.
Steps:
1. Create a `Plane` mesh in the Stage.
2. Align it with the USDZ floor height/tilt.
3. Add a collider to the plane.
4. Save and reuse as `DEFAULT_MAP_PATH`.

---

## Step 3. Run teleop

Direct run (Isaac Sim Python):
```bash
# teleop start
$ source ${ROS_SETUP_SCRIPT}
$ ros2 run joy joy_node & ros2 run teleop_twist_joy teleop_node --ros-args --params-file ${TELEOP_CFG}
```

---

## Step 4. Teleop capture (robot camera view)

Capture frames from a specified robot camera view.

Example env:
```bash
# .env
CAPTURE_DIR=/path/to/teleop_frames
CAPTURE_CAMERA_PATH=/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left
CAPTURE_RES=1920,1080
CAPTURE_EVERY=1
CAPTURE_MAX_FRAMES=1000
```

```python
# Example capture in Isaac Sim Script Editor
import omni.replicator.core as rep

camera_path = "/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left"
render_product = rep.create.render_product(camera_path, (1920, 1080))

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="${TELEOP_FRAMES_DIR}",
    rgb=True,
    bounding_box_2d_tight=False
)
writer.attach([render_product])
rep.orchestrator.step()
```

Notes
- Control frame interval with `CAPTURE_EVERY`.
- `CAPTURE_MAX_FRAMES=0` means unlimited capture.
- Capture runs inside the Isaac Sim loop.
- You can paste the Python code into Isaac Sim → Window → Script Editor.

---

## Step 5. Fixer → video output

Enhance captured images with Fixer and export videos.

Required in `.env`
- `FIXER_SOURCE_PATH` (Fixer repo path)
- `FIXER_CONTAINER_WORKDIR` (container workdir)
- `FIXER_CONTAINER_INPUT_DIR` (container input dir)
- `FIXER_MODEL_PATH` (container model path)

Run Fixer
```bash
# Example input/output
$ export FIXER_INPUT_DIR=/path/to/frames_to_fixer
$ export FIXER_OUT_DIR=${FIXER_OUT_DIR:-${FIXER_INPUT_DIR}/output}

# Docker run
$ docker run -it --gpus=all --ipc=host \
  -v ${FIXER_SOURCE_PATH}:${FIXER_CONTAINER_WORKDIR} \
  -v ${FIXER_INPUT_DIR}:${FIXER_CONTAINER_INPUT_DIR} \
  --entrypoint python \
  ${FIXER_DOCKER_IMAGE} \
  ${FIXER_CONTAINER_WORKDIR}/src/inference_pretrained_model.py \
  --model ${FIXER_MODEL_PATH} \
  --input ${FIXER_CONTAINER_INPUT_DIR} \
  --output ${FIXER_OUT_DIR} \
  --timestep 250
```

```bash
# Encode videos
$ ffmpeg -start_number 0 -i frame.%04d.png \
       -c:v libx264 -pix_fmt yuv420p before.mp4

$ ffmpeg -start_number 0 -i output/frame.%04d.png \
       -c:v libx264 -pix_fmt yuv420p after.mp4

# Side-by-side comparison
$ ffmpeg -start_number 0 -i frame.%04d.png \
       -start_number 0 -i output/frame.%04d.png \
       -filter_complex hstack \
       -c:v libx264 -pix_fmt yuv420p comparison.mp4
```

Notes
- If input file names are not `frame.*`, rename or adjust input pattern.
- Use `-start_number` to align with your frame index.

Output:
```
${FIXER_OUT_DIR}/
├── after.mp4
├── before.mp4
└── comparison.mp4
```

---

## Step 6. Troubleshooting

- Mesh misalignment
  Adjust `MESH_OFFSET`, `MESH_ROT`, `MESH_SCALE`.

- Nucleus paths not opening
  Set `OMNI_SERVER`, `OMNI_USER`, `OMNI_PASS` in `.env`.

- Joystick not detected
  Check `JOYSTICK_BY_ID_DIR`, `JOYSTICK_DEV_GLOB`, `XBOX_ID`.

---

## References

- ZED pipeline docs
  `scripts/scripts_zed_data_pipeline/docs/ZED_NUREC_PIPELINE.md`
- 3DGRUT output example
  `export_last.usdz`
