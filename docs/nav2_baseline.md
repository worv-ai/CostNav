# :robot: Nav2 Baseline

CostNav integrates the **ROS2 Navigation Stack 2** as a rule-based baseline for comparison with learning-based approaches. For IL baselines, see **[IL Baselines](baselines.md)**.

---

## :rocket: Quick Start

```bash
make run-nav2
# Defaults: NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True TUNED=True AMCL=False

# Then run ONE of:
make start-mission    # single mission
make run-eval-nav2    # batch evaluation
```

## Supported Robots

| Robot       | Command                               |
|:------------|:--------------------------------------|
| Nova Carter | `SIM_ROBOT=nova_carter make run-nav2` |
| Segway E1   | `SIM_ROBOT=segway_e1 make run-nav2`   |

## Features

- Full Nav2 integration with Isaac Sim
- Mission orchestration with automated start/goal sampling from NavMesh
- Two localization modes: AMCL (realistic) and GPS-only (ground truth)
- Tuned and untuned Nav2 parameter sets per robot
- Cost model integration: energy, distance, time, collision, and food spoilage tracking

## Localization Modes

```bash
# GPS mode (default — ground truth localization)
make run-nav2

# AMCL mode (realistic — particle filter localization)
make run-nav2 AMCL=True
```

## Nav2 Parameter Tuning

Each robot has tuned and untuned parameter files:

```
costnav_isaacsim/nav2_params/
├── segway_e1/
│   ├── navigation_params_tuned_true.yaml   # Tuned for CostNav
│   └── navigation_params_tuned_false.yaml  # Default Nav2 params
├── nova_carter/
│   ├── navigation_params_tuned_true.yaml
│   └── navigation_params_tuned_false.yaml
└── nav2.launch.py
```

```bash
# Use tuned parameters (default)
make run-nav2 TUNED=True

# Use default Nav2 parameters
make run-nav2 TUNED=False
```

## Evaluation

```bash
# Run batch evaluation (requires running nav2 instance)
make run-eval-nav2

# Custom timeout and mission count
make run-eval-nav2 TIMEOUT=20 NUM_MISSIONS=100
```

See **[Evaluation](evaluation.md)** for detailed metrics and the unified eval script.
