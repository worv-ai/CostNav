# CostNav

<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/ee4db2b1-ec4e-41c9-88b5-e6c58c78facd" />

<div align="center">
  <a href="https://arxiv.org/abs/2511.20216"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2511.20216-b31b1b.svg?style=flat"></a>
  <a href="https://worv.ghost.io/costnav-2/"><img alt="Blog Post" src="https://img.shields.io/badge/blog-post-blueviolet?style=flat&logo=ghost&logoColor=white"></a>
  <a href="https://github.com/worv-ai/CostNav/issues"><img alt="Open issues" src="https://img.shields.io/github/issues/worv-ai/CostNav?style=flat"></a>
  <a href="https://github.com/worv-ai/CostNav/stargazers"><img alt="GitHub stars" src="https://img.shields.io/github/stars/worv-ai/CostNav?style=flat"></a>
  <a href="https://github.com/worv-ai/CostNav"><img alt="Last commit" src="https://img.shields.io/github/last-commit/worv-ai/CostNav?style=flat&logo=github"></a>
  <img alt="Isaac Sim" src="https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?style=flat&logo=nvidia">
  <img alt="Isaac Lab" src="https://img.shields.io/badge/Isaac%20Lab-2.3.0-4CAF50?style=flat&logo=nvidia">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.11+-3776AB?style=flat&logo=python&logoColor=white">
  <a href="https://worv-ai.github.io/CostNav"><img alt="Documentation" src="https://img.shields.io/badge/docs-material-blue?style=flat&logo=materialformkdocs&logoColor=white"></a>

  <h3>CostNav is a cost-driven navigation benchmark for sidewalk robots, built on Isaac Sim.</h3>
</div>

---

## Overview

CostNav supports a wide range of robot platforms and diverse outdoor environments, and evaluates navigation policies with a unified cost model that captures SLA compliance, operational cost, profitability, and break-even time.
The toolkit enables scalable variation in robots, payloads, maps, and cloud-inference settings, and supports both learning-based and rule-based navigation stacks—making it easy to prototype and compare cost-aware policies without manual tuning for each scenario.

You can find more details in our [CostNav docs](https://worv-ai.github.io/CostNav).

## Highlights

- **Business-first benchmark:**
  Policies are evaluated not only on navigation success but also on their operational impact, including robot safety, SLA compliance, profitability, and break-even time—metrics directly tied to real-world deployment.
- **Diverse environment suite:**
  CostNav provides a set of tasks that span urban, suburban, rural, wild, port, and orchard-style maps, all using the COCO delivery robot with mixed observation (vector + RGB-D) pipelines for consistent evaluation.
- **Roadmap-ready:**
  Hooks are in place to compare learning vs. rule-based stacks, switch between on-device and cloud inference, and study cost-aware reward shaping.

## Simulation Overview

### Simulation Environment

| Scenario | Description                                                                                                                                             |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Sidewalk | City-scale sidewalk map featuring crosswalks, curbs, planters, and other street furniture, delivered via Omniverse USD assets for reproducible layouts. |

### Simulation Agents

| Agent               | Description                                                                                                                                                |
| ------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| COCO delivery robot | Four-wheeled sidewalk courier platform from `coco_robot_cfg.py` with configurable drive models, cameras, and LiDAR for learning or rule-based controllers. |

## Getting Started

### 1. Prerequisites

- Linux host with NVIDIA GPU, recent drivers, Docker, and `nvidia-container-toolkit`.
- Access to `nvcr.io/nvidia/isaac-sim:5.1.0` (login via NVIDIA NGC).
- Optional Omniverse Nucleus endpoint containing the street-scene USD referenced by the task configs.

### 2. Clone and fetch references

```bash
git clone https://github.com/worv-ai/CostNav.git
cd CostNav
```

### 3. Configure environment variables

1. Copy `.env.example` to `.env`.
2. Set `DISPLAY`, `NVIDIA_VISIBLE_DEVICES`, Isaac Sim search paths, and any Omniverse auth tokens your setup requires.
3. The same `.env` is used by Docker Compose, the devcontainer, and SLURM jobs.

### 4. Build/run with Docker Compose

```bash
# Isaac Lab + Isaac Sim stack (GPU + Omniverse)
docker compose --profile isaac-lab up -d
docker exec -it costnav-isaac-lab bash

# Alternate profiles
docker compose --profile isaac-sim up -d    # bare Isaac Sim
docker compose --profile dev up -d         # light dev image without simulator

# Tear down when finished
docker compose down
```

To rebuild the images locally, use the provided `Makefile`:

```bash
# build all three targets with default tags (costnav-*:<0.1.0>)
make build-all

# override the versions as needed
make build-isaac-lab COSTNAV_VERSION=0.2.0
```

Inside the container the project is mounted at `/workspace`. Isaac Sim is prevented from auto-starting so you control when to launch training scripts.

### 5. Manual / bare-metal install

If you already installed Isaac Lab into your Python environment:

```bash
python -m pip install -e costnav_isaaclab/source/costnav_isaaclab
python -m pip install -e ".[dev]"
python costnav_isaaclab/scripts/list_envs.py
```

### 6. IDE helpers

```bash
python tools/generate_vscode_settings.py --isaac-sim /path/to/isaac-sim
```

The script creates `.vscode/.python.env` with search paths so Pylance can index Isaac modules without indexing the entire Omniverse cache.

### 7. SLURM / cluster runs

Use the provided batch file to spin up per-job containers:

```bash
sbatch train.sbatch
```

`train.sbatch` derives the container name from `SLURM_JOB_ID`, pins GPUs via `NVIDIA_VISIBLE_DEVICES`, launches the `isaac-lab` compose profile, runs RL-Games training headlessly, and tears the container down when finished.

## Running Nav2 (Rule-Based Navigation)

For rule-based navigation using ROS2 Nav2 stack with Isaac Sim, see the [costnav_isaacsim README](costnav_isaacsim/README.md) for detailed setup and usage.

### Quick Start

```bash
# Build required images
make build-isaac-sim
make build-ros-ws
make build-ros2

# Run Nav2 navigation (Isaac Sim + ROS2 Nav2)
make run-nav2
```

This starts Isaac Sim with the Street Sidewalk environment and Nova Carter robot, along with the ROS2 Nav2 stack for classical navigation.

## Running Experiments

All commands assume `cd costnav_isaaclab` (which mirrors `/workspace/costnav_isaaclab` inside containers).

```bash
# Inspect available environments
python scripts/list_envs.py | grep Template-Costnav

# PPO baseline on COCO sidewalk navigation (RGB-D input + cameras)
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras 2>&1 | tee run_log.txt
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras --headless 2>&1 | tee run_log.txt

# Alternate curriculum / maps
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v1-CustomMap
python scripts/rl_games/train.py --task=Template-Costnav-Isaaclab-v0

# Evaluate, demo, or play with trained checkpoints
python scripts/rl_games/evaluate.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras
python scripts/rl_games/play.py --task=Template-Costnav-Isaaclab-v2-NavRL --enable_cameras

# Deterministic controller + reward sanity checks
python scripts/test_controller.py --task Template-Costnav-Isaaclab-v2-NavRL --enable_cameras
python scripts/test_v2_rewards.py --task Template-Costnav-Isaaclab-v2-NavRL

# Dummy zero or random agents to confirm scene wiring
python scripts/zero_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL
python scripts/random_agent.py --task=Template-Costnav-Isaaclab-v2-NavRL
```

### Safe position discovery and NavMesh debugging

```bash
cd costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL
python find_safe_positions.py --visualize_raycasts
python safe_area_validator.py               # validates a set of candidate poses
python check_navmesh.py                     # optional NavMesh diagnostics
python check_impulse.py                     # contact impulse sweeps
```

Generated safe poses will be written back to `safe_positions_auto_generated.py`, which is consumed by both the command generator and environment reset hooks.

### Monitoring training

```bash
python -m tensorboard.main --logdir costnav_isaaclab/logs/rl_games/<experiment>/summaries --port 6006
```

TensorBoard logs include both standard RL metrics (success, distance, reward components) and cost-model summaries emitted as custom scalars.

## Cost & Revenue Model

- **Lifecycle coverage:** CostNav accounts for capex (robot hardware amortized across operating years), energy costs per-kWh, street-level maintenance/repair, and SLA-driven revenue for each delivery slot.
- **Baseline economics:** The current RL-Games policy operating on-device reaches 43.0% SLA compliance, maintains a 46.5% operating margin, and breaks even in roughly 0.90 years.
- **Cost breakdown:** Maintenance (33.7%) and hardware amortization (34.9%) dominate the expense report, making collision avoidance and smoother driving the highest-leverage improvements.
- **Inputs & tuning:** Industry-sourced rates (utility tariffs, fleet service pricing) seed the configs. These parameters live alongside experiment metadata under `outputs/` so each run logs the assumptions used to compute profitability.
- **Future axes:** Rule-based navigation stacks, cloud inference policies, and cost-aware reinforcement learning curricula will reuse the same accounting layer, enabling apple-to-apple comparisons across control paradigms.

## Development Notes

- **Coding standards:** Python 3.10, `black` (100 char lines), `ruff`, and `mypy` settings live in `pyproject.toml`. Run `uv pip install --system -e ".[dev]"` or `pip install -e ".[dev]"` to install formatters and tests.
- **Testing:** Unit tests live under `tests/` (populate as features mature). Use `pytest` or targeted scripts like `scripts/test_v2_rewards.py` to validate reward shaping before launching long training jobs.
- **Submodules:** Refer to `third_party/README.md` for guidance on keeping IsaacLab and Urban-Sim references in sync without polluting CostNav source directories.
- **IDE & linting:** `tools/generate_vscode_settings.py` ensures VS Code/Pylance knows where Omniverse packages live. For PyCharm or other IDEs, mirror the generated `.python.env` paths.
- **Omniverse assets:** Update `CostnavIsaaclabSceneCfg` if your Nucleus path differs. Keep URLs in sync via the `.env` file so Docker and local runs agree on map locations.

## Roadmap

1. **Rule-based baselines:** Integrate classical planners/pure-pursuit controllers to ground the benchmark with non-learning references.
2. **Cloud inference toggles:** Decouple policy execution from the robot and simulate cloud latency + bandwidth costs.
3. **Cost-aware RL training:** Incorporate direct profit margins, SLA penalties, and maintenance risk into the reward to study economically aligned policies.
4. **Expanded asset suite:** Add night-time lighting variations, more COCO payload types (food containers, mail), and additional sidewalk geometries.
5. **Analytics tooling:** Publish notebooks/notebooks dashboards that convert `outputs/` records into ROI charts for entire fleets.

## Contributing

Issues and pull requests are welcome! Open a discussion or issue with

- The scenario or subsystem you want to improve (e.g., safe position sampling, cost accounting hooks, RL hyper-parameters).
- Links to supporting logs or Omniverse USD assets if relevant.

Please run formatters (`black`, `ruff`) and targeted validation scripts before submitting a PR.

## What's next?

- [ ] Paper release
- [ ] camera specsheet and cost
- [ ] sensor + robot + battery + compute specsheet and cost
- [ ] Repo Packaging
- [x] mkdocs
- [ ] isaac sim & nav2 support for rule-based navigation ([launch.py](costnav_isaacsim/launch.py))
- [ ] revenue references for delivery

## Acknowledgements

CostNav is built on NVIDIA Isaac Sim/Isaac Lab, Omniverse USD tooling, and the COCO delivery robot model. Many of the safe navigation utilities borrow ideas from the wider robotics community--huge thanks to everyone who made their work available.

## Contact

Maintained by the worv.ai robotics research team. For research collaborations or enterprise deployments, please contact your worv.ai point of contact or open an issue on GitHub.

## Citation

To Cite CostNav, please use the following bibtex citation

```
@misc{seong2025costnavnavigationbenchmarkcostaware,
      title={CostNav: A Navigation Benchmark for Cost-Aware Evaluation of Embodied Agents},
      author={Haebin Seong and Sungmin Kim and Minchan Kim and Yongjun Cho and Myunchul Joe and Suhwan Choi and Jaeyoon Jung and Jiyong Youn and Yoonshik Kim and Samwoo Seong and Yubeen Park and Youngjae Yu and Yunsung Lee},
      year={2025},
      eprint={2511.20216},
      archivePrefix={arXiv},
      primaryClass={cs.AI},
      url={https://arxiv.org/abs/2511.20216},
}
```
