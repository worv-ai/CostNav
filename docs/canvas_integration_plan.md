# CostNav → CANVAS Integration Plan

## Goal

Pass mission information (start/goal) from CostNav's **MissionManager** to CANVAS using **sketch-based navigation** (shortest path in image/pixel space). No goal images, goal poses, or topomaps are used.

---

## Architecture Overview

```
┌─────────────────────────┐
│  CostNav MissionManager │
│  (Isaac Sim)            │
│                         │
│  1. Sample start/goal   │
│     via NavMesh         │
│  2. Query NavMesh       │
│     shortest path       │
│  3. Convert path to     │
│     pixel coords via    │
│     pose2coords()       │
│  4. Build Scenario JSON │
│  5. Publish annotation  │
└────────┬────────────────┘
         │  /instruction_scenario  (std_msgs/String)
         │  /instruction_annotation (std_msgs/Int32MultiArray)
         │  /start_pause (std_msgs/Bool)
         ▼
┌─────────────────────────┐
│  CANVAS                 │
│                         │
│  - Receives Scenario    │
│  - Receives annotation  │
│    (pixel trajectory)   │
│  - Runs model inference │
│  - Publishes velocity   │
└────────┬────────────────┘
         │  /cmd_vel (geometry_msgs/TwistStamped)
         ▼
       Robot
```

---

## Step-by-Step Implementation

### Step 1: Add `get_shortest_path()` to NavMeshSampler

Currently `NavMeshSampler` only exposes `get_path_first_waypoint()` and `check_path_exists()`. Both call `navmesh.query_shortest_path()` internally but discard the full path. Add a new method that returns **all** waypoints:

```python
# In navmesh_sampler.py
def get_shortest_path(
    self,
    start: SampledPosition,
    goal: SampledPosition,
) -> Optional[list[SampledPosition]]:
    """Return all waypoints on the NavMesh shortest path from start to goal."""
    navmesh = self._get_navmesh()
    start_point = carb.Float3(start.x, start.y, start.z)
    end_point = carb.Float3(goal.x, goal.y, goal.z)

    path = navmesh.query_shortest_path(
        start_pos=start_point,
        end_pos=end_point,
        agent_radius=self.agent_radius,
    )
    if path is None:
        return None

    points = path.get_points()
    if points is None or len(points) < 2:
        return None

    return [
        SampledPosition(x=float(pt.x), y=float(pt.y), z=float(pt.z))
        for pt in points
    ]
```

### Step 2: Convert World Path → Pixel Coordinates

Use the **sidewalk map** parameters with `SketchMap.pose2coords()` logic:

| Parameter  | Value                       |
| ---------- | --------------------------- |
| image      | `sidewalk_orthographic.png` |
| size       | 4000 × 4000 px (RGBA)       |
| resolution | 0.05 m/pixel                |
| origin     | [-100.875, -100.875, 0.0]   |

**Conversion formula** (from `SketchMap.pose2coords`):

```python
import numpy as np

resolution = 0.05
origin = [-100.875, -100.875, 0.0]
width, height = 4000, 4000

# path_world: np.ndarray of shape (N, 2) with columns [x, y] in world coords
pixel_x = (path_world[:, 0] - origin[0]) / resolution
pixel_x = np.round(pixel_x).clip(0, width - 1)

pixel_y = height - (path_world[:, 1] - origin[1]) / resolution
pixel_y = np.round(pixel_y).clip(0, height - 1)

pixel_coords = np.stack([pixel_x, pixel_y], axis=1).astype(int)  # shape (N, 2), order (x, y)
```

### Step 3: Build a Scenario JSON (no CANVAS import needed)

The CANVAS model receives a JSON string on `/instruction_scenario` and parses it with `Scenario.model_validate_json()` internally.
CostNav does **not** import anything from CANVAS — just publish a plain JSON string with the required fields using `json.dumps`.

```python
import json

MAP_NAME = "costnav"
SKETCH_MAP_NAME = "orthographic_map"
DRIVE_MAP_NAME = "orthographic_map"

MODEL_GUIDELINE = (
    "You are segway_e1, measuring 62*89*115cm (width, length, and height),\n"
    "equipped with a wide-angle camera and moving on four wheel drive.\n\n"
    "You are driving on a sidewalk.\n"
    "You should act like a last-mile delivery robot.\n\n"
    "You must follow these driving instructions:\n"
    "1. You must avoid collisions.\n"
    "2. You should prioritize reaching the final destination.\n"
    "3. You should follow the Trajectory Instruction.\n"
    "    a. If the Trajectory Instruction cannot be followed due to any obstacles, "
    "you should deviate to bypass the obstacle.\n"
    "    b. You should try to evade any identifiable obstacles.\n"
    "4. You should maintain a constant driving speed.\n"
    "5. You must drive on the sidewalk.\n"
    "    a. If you need to cross the road, you must use the crosswalk."
)

scenario_json = json.dumps({
    "semantic_uuid": f"costnav_mission_{mission_id}",
    "map_name": MAP_NAME,                    # top-level key in map_list.yml
    "sketch_map_name": SKETCH_MAP_NAME,      # sub-key under costnav → used as HTL map
    "drive_map_name": DRIVE_MAP_NAME,        # sub-key under costnav → used as drive map
    "model_guideline": MODEL_GUIDELINE,
    "map_metadata": {},
})
```

**Scenario JSON schema** (fields parsed by `Scenario.model_validate_json()` on the CANVAS side):

| Field             | Type | Required | Description                                     |
| ----------------- | ---- | -------- | ----------------------------------------------- |
| `semantic_uuid`   | str  | ✅       | Unique mission identifier                       |
| `map_name`        | str  | ✅       | Top-level key in `map_list.yml` (= `"costnav"`) |
| `sketch_map_name` | str  | ✅       | Sub-key for HTL map (= `"orthographic_map"`)    |
| `drive_map_name`  | str  | ✅       | Sub-key for drive map (= `"orthographic_map"`)  |
| `model_guideline` | str  | ✅       | Driving instructions for the neural model       |
| `map_metadata`    | any  | ✅       | Extra metadata (can be `{}`)                    |

### Step 4: Publish to CANVAS Topics

From the new bridge node (or from MissionManager directly):

```python
from std_msgs.msg import String, Int32MultiArray, Bool

# 1. Publish Scenario JSON  (scenario_json built in Step 3 via json.dumps)
instruction_scenario_pub.publish(String(data=scenario_json))

# 2. Publish annotation (pixel trajectory, flattened)
#    annotation is pixel_coords from Step 2, shape (N, 2), order (x, y)
annotation_msg = Int32MultiArray(data=pixel_coords.flatten().tolist())
instruction_annotation_pub.publish(annotation_msg)

# 3. After CANVAS transitions to "waiting" state, send start signal
start_pause_pub.publish(Bool(data=True))
```

**Topics summary:**

| Topic                     | Type                         | Content                                                          |
| ------------------------- | ---------------------------- | ---------------------------------------------------------------- |
| `/instruction_scenario`   | `std_msgs/String`            | Scenario JSON                                                    |
| `/instruction_annotation` | `std_msgs/Int32MultiArray`   | Flattened pixel coords `[x0,y0,x1,y1,…]`                         |
| `/start_pause`            | `std_msgs/Bool`              | `True` to start inference                                        |
| `/stop_model`             | `std_msgs/Bool`              | `True` to stop                                                   |
| `/vel_predict`            | `sensor_msgs/PointCloud2`    | N×2 velocity predictions                                         |
| `/model_latency`          | `sensor_msgs/TimeReference`  | Inference latency                                                |
| `/cmd_vel`                | `geometry_msgs/TwistStamped` | Final velocity command                                           |
| `/reached_goal`           | `std_msgs/Bool`              | Goal reached signal                                              |
| `/model_state`            | `std_msgs/String`            | Current state (`init`/`resetting`/`waiting`/`running`/`stopped`) |

### Step 5: Monitor Completion

CANVAS publishes `/reached_goal` (Bool) and `/model_state` (String). The bridge node should:

1. Subscribe to `/reached_goal` — when `True`, the robot has reached the end of the sketch trajectory.
2. Subscribe to `/model_state` — track state transitions (`resetting` → `waiting` → `running` → `stopped`).
3. On completion or timeout, publish `/stop_model` (Bool `True`) to halt the model, then transition MissionManager to `COMPLETED`.

---

## Integration with MissionManager State Machine

Replace the Nav2 goal-publishing step with CANVAS scenario publishing:

| MissionManager State      | Current Behavior (Nav2)           | New Behavior (CANVAS)                                          |
| ------------------------- | --------------------------------- | -------------------------------------------------------------- |
| `READY`                   | Sample start/goal via NavMesh     | Same + query full NavMesh shortest path                        |
| `TELEPORTING`             | Teleport robot to start           | Same (unchanged)                                               |
| `SETTLING`                | Wait for physics                  | Same (unchanged)                                               |
| `PUBLISHING_INITIAL_POSE` | Publish `/initialpose` for AMCL   | Skip or keep for localization                                  |
| `PUBLISHING_GOAL`         | Publish `/goal_pose` to Nav2      | Convert path → pixels, publish Scenario + annotation to CANVAS |
| `WAITING_FOR_COMPLETION`  | Poll Nav2 result + distance check | Subscribe to `/reached_goal` + distance check                  |

---
