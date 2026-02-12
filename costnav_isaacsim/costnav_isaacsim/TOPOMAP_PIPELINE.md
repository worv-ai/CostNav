# NavMesh-to-Topomap Pipeline

Automatically generate [ViNT](https://github.com/robodhruv/visualnav-transformer)-compatible topological maps from Isaac Sim NavMesh shortest paths — no manual rosbag recording required.

## Overview

The pipeline queries the NavMesh for the shortest path between two positions, interpolates dense waypoints at a fixed interval, then captures a forward-facing RGB image at each waypoint using a virtual camera. The output is a directory of sequentially numbered PNGs (`0.png`, `1.png`, …, `N.png`) that can be loaded directly by ViNT's `navigate.py`.

### Pipeline Steps

```
Start/Goal positions
        │
        ▼
┌──────────────────────┐
│  NavMesh shortest    │  query_shortest_path() → sparse waypoints (turns/corners)
│  path query          │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│  Waypoint            │  interpolate_waypoints() → dense waypoints every 0.5 m
│  interpolation       │  with heading = atan2(Δy, Δx)
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│  Virtual camera      │  setup_camera() → USD Camera prim + Replicator annotator
│  setup               │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│  Image capture loop  │  For each waypoint:
│                      │    1. Move camera to (x, y, z + offset)
│                      │    2. Set orientation from heading
│                      │    3. Step simulation (render flush)
│                      │    4. Save RGB as {i}.png
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│  Cleanup             │  Remove camera prim, release render resources
└──────────────────────┘
```

## Enable via Makefile

The topomap pipeline is controlled by the `TOPOMAP` Make variable (default: `False`):

```bash
# Enable topomap generation for Isaac Sim
make run-isaac-sim TOPOMAP=True

# Enable for teleop mode
make run-teleop TOPOMAP=True

# ViNT profile (always enables topomap)
make run-vint
```

The `TOPOMAP` variable flows through `docker-compose.yml` and maps to the existing `--goal-image-enabled` CLI argument internally.

## Quick Start (Python API)

```python
from costnav_isaacsim.config import TopoMapConfig
from costnav_isaacsim.mission_manager import NavMeshSampler, TopomapGenerator

# 1. Create a NavMesh sampler (requires a loaded USD stage with NavMesh baked)
sampler = NavMeshSampler(min_distance=5.0, max_distance=50.0)

# 2. Configure the topomap generator
config = TopoMapConfig(
    enabled=True,
    waypoint_interval=0.5,       # one image every 0.5 m
    image_width=640,
    image_height=360,
    output_dir="/tmp/my_topomap",
)

# 3. Create the generator (simulation_context comes from Isaac Sim)
generator = TopomapGenerator(sampler, config, simulation_context)

# 4. Sample start/goal or provide your own SampledPosition instances
start, goal = sampler.sample_start_goal_pair()

# 5. Generate the topomap
saved_paths = generator.generate_topomap(start, goal)
# → ["/tmp/my_topomap/0.png", "/tmp/my_topomap/1.png", …]
```

### Using Explicit Positions

```python
from costnav_isaacsim.mission_manager import SampledPosition

start = SampledPosition(x=10.0, y=20.0, z=0.0)
goal  = SampledPosition(x=50.0, y=80.0, z=0.0)

saved_paths = generator.generate_topomap(start, goal, output_dir="/tmp/route_A_B")
```

## Configuration

All settings live under the `topomap:` key in `config/mission_config.yaml`:

```yaml
topomap:
  enabled: false # Set to true to enable generation
  waypoint_interval: 0.5 # Distance between waypoints (meters)
  camera_height_offset: 0.3 # Camera height above ground (meters)
  image_width: 640 # Image width in pixels
  image_height: 360 # Image height in pixels
  output_dir: "/tmp/costnav_topomap"
  camera_prim_path: "/World/topomap_camera"
  render_settle_steps: 3 # Sim steps per capture (render flush)
  # Camera intrinsics (matching rgb_left.usda)
  focal_length: 2.87343
  horizontal_aperture: 5.76
  vertical_aperture: 3.6
  focus_distance: 0.6
```

You can also override any field programmatically:

```python
from costnav_isaacsim.config import load_mission_config

config = load_mission_config()              # loads mission_config.yaml
config.topomap.enabled = True
config.topomap.waypoint_interval = 1.0      # coarser sampling
config.topomap.output_dir = "/data/topomaps/run_01"
```

### Configuration Reference

| Field                  | Type    | Default                   | Description                                                                                              |
| ---------------------- | ------- | ------------------------- | -------------------------------------------------------------------------------------------------------- |
| `enabled`              | `bool`  | `False`                   | Master switch for topomap generation                                                                     |
| `waypoint_interval`    | `float` | `0.5`                     | Distance between interpolated waypoints (meters). Smaller = more images, higher fidelity                 |
| `camera_height_offset` | `float` | `0.3`                     | Camera height above the NavMesh ground plane (meters)                                                    |
| `image_width`          | `int`   | `640`                     | Captured image width in pixels                                                                           |
| `image_height`         | `int`   | `360`                     | Captured image height in pixels                                                                          |
| `output_dir`           | `str`   | `"/tmp/costnav_topomap"`  | Directory where numbered PNGs are saved                                                                  |
| `camera_prim_path`     | `str`   | `"/World/topomap_camera"` | USD prim path for the virtual camera                                                                     |
| `render_settle_steps`  | `int`   | `3`                       | Number of `simulation_context.step(render=True)` calls per capture. Increase if images appear incomplete |
| `focal_length`         | `float` | `2.87343`                 | Camera focal length (mm) — matches `rgb_left.usda`                                                       |
| `horizontal_aperture`  | `float` | `5.76`                    | Sensor horizontal aperture (mm)                                                                          |
| `vertical_aperture`    | `float` | `3.6`                     | Sensor vertical aperture (mm)                                                                            |
| `focus_distance`       | `float` | `0.6`                     | Focus distance (m)                                                                                       |

## ViNT Integration

The generated topomap is directly compatible with ViNT's `navigate.py`. The navigator loads images by sorting filenames numerically and walking the sequence from the closest node toward the goal.

```bash
# Copy or symlink the generated topomap into ViNT's expected directory
cp -r /tmp/my_topomap third_party/visualnav-transformer/deployment/topomaps/images/my_topomap

# Run ViNT navigation (goal-node -1 means "navigate to the last image")
cd third_party/visualnav-transformer/deployment/src
python navigate.py --dir my_topomap --goal-node -1
```

**How ViNT loads the topomap:**

```python
# From navigate.py — files are sorted by integer prefix
topomap_filenames = sorted(os.listdir(topomap_dir), key=lambda x: int(x.split(".")[0]))
for i in range(num_nodes):
    topomap.append(PILImage.open(os.path.join(topomap_dir, topomap_filenames[i])))
```

This means the output format (`0.png`, `1.png`, …, `N.png`) is the only requirement. No metadata files are needed.

## API Reference

### `TopomapGenerator(sampler, config, simulation_context)`

**Constructor parameters:**

| Parameter            | Type                          | Description                              |
| -------------------- | ----------------------------- | ---------------------------------------- |
| `sampler`            | `NavMeshSampler`              | Provides NavMesh access for path queries |
| `config`             | `TopoMapConfig`               | Camera, waypoint, and output settings    |
| `simulation_context` | Isaac Sim `SimulationContext` | Required for render stepping             |

### `generate_topomap(start, goal, output_dir=None) → List[str]`

Main entry point. Orchestrates the full pipeline and returns a list of saved image paths.

| Parameter    | Type              | Description                                                 |
| ------------ | ----------------- | ----------------------------------------------------------- |
| `start`      | `SampledPosition` | Start position on the NavMesh                               |
| `goal`       | `SampledPosition` | Goal position on the NavMesh                                |
| `output_dir` | `str \| None`     | Override output directory (defaults to `config.output_dir`) |

**Returns:** List of saved file paths, or empty list on failure.

### `get_shortest_path_waypoints(start, goal) → Optional[List[SampledPosition]]`

Queries the NavMesh for the shortest path. Returns sparse waypoints (only at turns/corners), or `None` if no path exists.

### `interpolate_waypoints(sparse_points, interval=0.5) → List[SampledPosition]` _(static)_

Inserts intermediate points between consecutive sparse waypoints at a fixed distance interval. Heading at each point is `atan2(Δy, Δx)` toward the next point.

### `setup_camera() → None`

Creates a USD Camera prim with Replicator RGB annotator. Must be called before `capture_image_at_position()`. Automatically called by `generate_topomap()`.

### `capture_image_at_position(position) → numpy.ndarray | None`

Moves the virtual camera to the given position, steps the simulation to flush the render pipeline, and returns an RGB array of shape `(H, W, 3)` with dtype `uint8`.

### `cleanup_camera() → None`

Removes the camera prim and releases render resources. Automatically called by `generate_topomap()` in a `finally` block.

## Advanced Usage

### Batch Generation for Multiple Routes

```python
import itertools
from costnav_isaacsim.mission_manager import SampledPosition

waypoints = [
    SampledPosition(x=10.0, y=20.0, z=0.0),
    SampledPosition(x=50.0, y=80.0, z=0.0),
    SampledPosition(x=30.0, y=60.0, z=0.0),
]

for i, (s, g) in enumerate(itertools.combinations(waypoints, 2)):
    generator.generate_topomap(s, g, output_dir=f"/data/topomaps/route_{i}")
```

### Waypoint-Only Mode (No Images)

You can use the path query and interpolation independently:

```python
sparse = generator.get_shortest_path_waypoints(start, goal)
dense  = TopomapGenerator.interpolate_waypoints(sparse, interval=0.25)

for wp in dense:
    print(f"x={wp.x:.2f}  y={wp.y:.2f}  heading={wp.heading:.3f}")
```

### Custom Camera Height Per Environment

```python
config.camera_height_offset = 0.5   # taller robot
config.render_settle_steps = 5      # more complex scene needs extra frames
```

## Limitations

1. **Linear chain only** — the output is a sequential image list (node 0 → 1 → … → N). There is no branching or graph structure. ViNT's navigator assumes this linear ordering.

2. **Requires baked NavMesh** — the USD stage must have a NavMesh volume baked via Isaac Sim's NavMesh tools before path queries will work.

3. **Isaac Sim runtime required** — image capture uses Omni Replicator and USD Camera prims, so the pipeline must run inside an active Isaac Sim session.

4. **No Algorithm 1 (ViNT exploration)** — this pipeline generates topomaps for known start/goal pairs. It does not implement the graph-based exploration algorithm from the ViNT paper.

## Troubleshooting

| Symptom                            | Cause                               | Fix                                                                        |
| ---------------------------------- | ----------------------------------- | -------------------------------------------------------------------------- |
| `No path found` warning            | Start or goal is off the NavMesh    | Verify positions are on walkable areas; use `sampler.check_path_exists()`  |
| Black / incomplete images          | Render pipeline not flushed         | Increase `render_settle_steps` (try 5–10)                                  |
| `NavMesh extensions not available` | Running outside Isaac Sim           | This pipeline requires the Isaac Sim runtime                               |
| Images have wrong FOV              | Camera intrinsic mismatch           | Adjust `focal_length` / `horizontal_aperture` to match your robot's camera |
| Too many / too few images          | Waypoint interval too small / large | Adjust `waypoint_interval` (0.25 m = dense, 1.0 m = sparse)                |
