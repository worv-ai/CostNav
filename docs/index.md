# CostNav

<div align="center">
  <a href="https://github.com/worv-ai/CostNav/issues">
    <img alt="Open issues" src="https://img.shields.io/github/issues/worv-ai/CostNav?style=flat">
  </a>
  <a href="https://github.com/worv-ai/CostNav/stargazers">
    <img alt="GitHub stars" src="https://img.shields.io/github/stars/worv-ai/CostNav?style=flat">
  </a>
  <a href="https://github.com/worv-ai/CostNav">
    <img alt="Last commit" src="https://img.shields.io/github/last-commit/worv-ai/CostNav?style=flat&logo=github">
  </a>
  <img alt="Isaac Sim" src="https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?style=flat&logo=nvidia">
  <img alt="Isaac Lab" src="https://img.shields.io/badge/Isaac%20Lab-2.3.0-4CAF50?style=flat&logo=nvidia">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.11+-3776AB?style=flat&logo=python&logoColor=white">


  <h3>CostNav is a cost-driven navigation benchmark for sidewalk robots, built on Isaac Sim.</h3>
</div>

---

## Overview

CostNav supports a wide range of robot platforms and diverse outdoor environments, and evaluates navigation policies with a unified cost model that captures SLA compliance, operational cost, profitability, and break-even time.
The toolkit enables scalable variation in robots, payloads, maps, and cloud-inference settings, and supports both learning-based and rule-based navigation stacks—making it easy to prototype and compare cost-aware policies without manual tuning for each scenario.

## Highlights

- **Business-first benchmark:**
Policies are evaluated not only on navigation success but also on their operational impact, including robot safety, SLA compliance, profitability, and break-even time—metrics directly tied to real-world deployment.
- **Diverse environment suite:**
CostNav provides a set of tasks that span urban, suburban, rural, wild, port, and orchard-style maps, all using the COCO delivery robot with mixed observation (vector + RGB-D) pipelines for consistent evaluation.
- **Roadmap-ready:**
Hooks are in place to compare learning vs. rule-based stacks, switch between on-device and cloud inference, and study cost-aware reward shaping.

## Simulation Overview

### Simulation Environment

| Scenario | Description |
| --- | --- |
| Sidewalk | City-scale sidewalk map featuring crosswalks, curbs, planters, and other street furniture, delivered via Omniverse USD assets for reproducible layouts. |

### Simulation Agents

| Agent | Description |
| --- | --- |
| COCO delivery robot | Four-wheeled sidewalk courier platform from `coco_robot_cfg.py` with configurable drive models, cameras, and LiDAR for learning or rule-based controllers. |
