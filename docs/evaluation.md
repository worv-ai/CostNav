# :bar_chart: Evaluation

CostNav uses a unified evaluation script (`scripts/eval.sh`) that works across all navigation methods. It runs consecutive missions, collects metrics from the mission manager, and generates comprehensive logs.

---

## :rocket: Quick Start

```bash
# Requires a running navigation instance first (e.g. make run-nav2, make run-vint, etc.)

make run-eval-nav2       # Evaluate Nav2
make run-eval-vint       # Evaluate ViNT
make run-eval-gnm        # Evaluate GNM
make run-eval-nomad      # Evaluate NoMaD
make run-eval-navdp      # Evaluate NavDP
make run-eval-canvas     # Evaluate CANVAS
make run-eval-teleop     # Evaluate Teleop
```

### Custom Parameters

```bash
# Custom timeout (seconds) and number of missions
make run-eval-nav2 TIMEOUT=20 NUM_MISSIONS=100
```

### Direct Script Usage

```bash
./scripts/eval.sh <MODE> [TIMEOUT] [NUM_MISSIONS]

# Examples:
./scripts/eval.sh nav2 20 100
./scripts/eval.sh vint 30 50
./scripts/eval.sh canvas 241 10
```

| Argument | Default | Description |
|:---------|:--------|:------------|
| `MODE` | (required) | `teleop`, `nav2`, `vint`, `gnm`, `nomad`, `navdp`, or `canvas` |
| `TIMEOUT` | 20 | Mission timeout in seconds |
| `NUM_MISSIONS` | 10 | Number of missions to run |

### Controls

- **Right Arrow (→)**: Skip current mission during evaluation

---

## :clipboard: Collected Metrics

The eval script queries `/get_mission_result` after each mission and collects the following metrics:

### Episode Outcomes

| Metric | Description |
|:-------|:------------|
| Result | `SUCCESS`, `FAILURE_TIMEOUT`, `FAILURE_PHYSICALASSISTANCE`, `FAILURE_FOODSPOILED`, `SKIPPED` |
| Result Reason | Detailed reason (e.g. `orientation`, `impulse_health_depletion`) |
| Elapsed Time | Mission duration (seconds) |
| Traveled Distance | Total distance traveled (meters) |
| Distance to Goal | Final distance to goal (meters) |

### Physical Metrics

| Metric | Description |
|:-------|:------------|
| Avg Velocity | Average robot velocity (m/s) |
| Avg Mechanical Power | Average mechanical power draw (kW) |
| Total Contact Count | Total collision events |
| Total Impulse | Cumulative collision impulse (N·s) |
| People Contact Count | Pedestrian collision count |

### Safety Metrics (Pedestrian Injury)

| Metric | Description |
|:-------|:------------|
| Delta-v Count | Number of collision events with delta-v measurement |
| Delta-v Avg (m/s) | Average delta-v across collisions |
| Delta-v Avg (mph) | Average delta-v in mph |
| Total Injury Cost | Estimated pedestrian injury cost using AIS scale (\$) |

### Property Damage

| Property Type | Description |
|:--------------|:------------|
| Fire Hydrant | Contact count with fire hydrants |
| Traffic Light | Contact count with traffic lights |
| Street Lamp | Contact count with street lamps |
| Bollard | Contact count with bollards |
| Building | Contact count with buildings |
| Trash Bin | Contact count with trash bins |
| Mail Box | Contact count with mail boxes |
| Newspaper Box | Contact count with newspaper boxes |
| Bus Stop | Contact count with bus stops |
| **Total** | Sum of all property contacts |

### Food Delivery Metrics

| Metric | Description |
|:-------|:------------|
| Food Enabled | Whether food evaluation is active |
| Food Initial Pieces | Number of food pieces at start |
| Food Final Pieces | Number of food pieces at end |
| Food Loss Fraction | Fraction of pieces lost (0.0--1.0) |
| Food Spoiled | Whether spoilage threshold was exceeded |

---

## :page_facing_up: Log Output

Logs are saved to `./logs/<mode>_evaluation_<timestamp>.log` with per-mission details and a summary table.

```bash
# View logs
ls ./logs/
cat ./logs/nav2_evaluation_20260324_140000.log
```

---

## :link: Container Mapping

The eval script connects to the correct Docker container per mode:

| Mode | Container Name |
|:-----|:---------------|
| `nav2` | `costnav-ros2-nav2` |
| `vint` | `costnav-ros2-vint` |
| `gnm` | `costnav-ros2-gnm` |
| `nomad` | `costnav-ros2-nomad` |
| `navdp` | `costnav-ros2-navdp` |
| `canvas` | `costnav-ros2-rviz-nav2` |
| `teleop` | `costnav-ros2-teleop` |
