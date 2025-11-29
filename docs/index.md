# CostNav :robot:

<div style="text-align: center;" markdown="1">

[![Open Issues](https://img.shields.io/github/issues/worv-ai/CostNav?style=flat)](https://github.com/worv-ai/CostNav/issues)
[![GitHub Stars](https://img.shields.io/github/stars/worv-ai/CostNav?style=flat)](https://github.com/worv-ai/CostNav/stargazers)
[![Last Commit](https://img.shields.io/github/last-commit/worv-ai/CostNav?style=flat&logo=github)](https://github.com/worv-ai/CostNav)
![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?style=flat&logo=nvidia)
![Isaac Lab](https://img.shields.io/badge/Isaac%20Lab-2.3.0-4CAF50?style=flat&logo=nvidia)
![Python](https://img.shields.io/badge/Python-3.11+-3776AB?style=flat&logo=python&logoColor=white)

**A cost-driven navigation benchmark for sidewalk robots, built on Isaac Sim.**

</div>

---

## :dart: Overview

CostNav supports a wide range of robot platforms and diverse outdoor environments, and evaluates navigation policies with a unified cost model that captures **SLA compliance**, **operational cost**, **profitability**, and **break-even time**.

The toolkit enables scalable variation in robots, payloads, maps, and cloud-inference settings, and supports both learning-based and rule-based navigation stacks—making it easy to prototype and compare cost-aware policies without manual tuning for each scenario.

## :star2: Highlights

- :moneybag: **Business-first benchmark:** Policies are evaluated not only on navigation success but also on their operational impact, including robot safety, SLA compliance, profitability, and break-even time—metrics directly tied to real-world deployment.
- :world_map: **Diverse environment suite:** CostNav provides a set of tasks that span urban, suburban, rural, wild, port, and orchard-style maps, all using the COCO delivery robot with mixed observation (vector + RGB-D) pipelines for consistent evaluation.
- :rocket: **Roadmap-ready:** Hooks are in place to compare learning vs. rule-based stacks, switch between on-device and cloud inference, and study cost-aware reward shaping.

## :movie_camera: Simulation Overview

### Environments

| Scenario | Description |
|:---------|:------------|
| :cityscape: Sidewalk | City-scale sidewalk map featuring crosswalks, curbs, planters, and other street furniture, delivered via Omniverse USD assets for reproducible layouts. |

### Agents

| Agent | Description |
|:------|:------------|
| :truck: COCO Robot | Four-wheeled sidewalk courier platform from `coco_robot_cfg.py` with configurable drive models, cameras, and LiDAR for learning or rule-based controllers. |

## :chart_with_upwards_trend: Cost Model

```mermaid
graph LR
    A[Navigation Policy] --> B{Simulation}
    B --> C[Task Metrics]
    B --> D[Business Metrics]
    C --> C1[Success Rate]
    C --> C2[Collision Rate]
    D --> D1[SLA Compliance]
    D --> D2[Operating Cost]
    D --> D3[Profitability]
```

## :book: Documentation

### :zap: Getting Started

- **[Quick Reference](quick_reference.md)**: Essential commands and configurations
- **[Architecture Overview](architecture.md)**: Understand the codebase structure and how components work together
- **[Environment Versions](environment_versions.md)**: Learn about v0, v1, and v2 environments and when to use each

### :brain: Core Concepts

- **[MDP Components](mdp_components.md)**: Deep dive into observations, actions, rewards, and terminations
- **[Robot Configuration](robot_configuration.md)**: Detailed explanation of the COCO robot's physical properties, actuators, and sensors
- **[Cost Model](cost_model.md)**: Understanding business metrics: SLA compliance, operational costs, and profitability

### :mortar_board: Guides

- **[Training Guide](training_guide.md)**: Complete guide to training navigation policies with RL-Games and other frameworks

### :books: Reference

- **[API Reference](api.md)**: Auto-generated API documentation for all modules
- **[Scripts Reference](scripts_reference.md)**: Comprehensive reference for all training, evaluation, and utility scripts
- **[Food Delivery Business](references/FOOD_DELIVERY_BUSINESS.md)**: Industry data and economics for food delivery robots
