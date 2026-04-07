# :package: Third-Party References

The `third_party/` directory contains external projects checked out as git submodules. They are kept separate from the main CostNav code so we can inspect upstream implementations without mixing their code into our core packages.

## Setup

```bash
make fetch-third-party
```

## Submodules

### Simulation & Assets

| Submodule | Description |
|:----------|:------------|
| `IsaacLab/` | NVIDIA's Isaac Lab — reference for upstream tasks, assets, and training utilities |
| `urban-sim/` | Metadriverse urban simulator — reference for robot configuration and related assets |
| `PeopleAPI/` | Isaac Sim extension for spawning animated pedestrians with NavMesh-based walking |
| `FoodAssets/` | Isaac Sim extension — curated asset library for food-related scenes (popcorn cargo) |
| `ObstacleAssets/` | Isaac Sim extension for obstacle spawning with ground-snapping and dynamic avoidance |

### Navigation Baselines

| Submodule | Description |
|:----------|:------------|
| `visualnav-transformer/` | ViNT, NoMaD, GNM — visual navigation transformer baselines |
| `NavDP/` | Navigation Diffusion Policy — diffusion-based navigation baseline |
| `InternNav/` | InternNav — NavDP training framework with multi-baseline support |
| `diffusion_policy/` | Diffusion Policy — core diffusion policy library used by NavDP |
| `navigation2/` | ROS2 Navigation Stack 2 — rule-based navigation baseline |

## Usage

- Treat these directories as **read-only references**. All CostNav-specific code lives under `costnav_isaacsim/` or `costnav_isaaclab/`.
- These references are excluded from Docker build contexts and install hooks — they exist only to provide context while developing CostNav.
- When updating to newer upstream commits, use `git submodule update --remote` and capture the resulting commit hashes in your PR description.
