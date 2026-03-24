# CostNav

<img alt="CostNav Logo" src="docs/assets/images/motivation2.png" />

<div align="center">
  <a href="https://arxiv.org/abs/2511.20216"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2511.20216-b31b1b.svg?style=flat"></a>
  <a href="https://worv.ghost.io/costnav-2/"><img alt="Blog Post" src="https://img.shields.io/badge/blog-post-blueviolet?style=flat&logo=ghost&logoColor=white"></a>
  <a href="https://github.com/worv-ai/CostNav/issues"><img alt="Open issues" src="https://img.shields.io/github/issues/worv-ai/CostNav?style=flat"></a>
  <a href="https://github.com/worv-ai/CostNav/stargazers"><img alt="GitHub stars" src="https://img.shields.io/github/stars/worv-ai/CostNav?style=flat"></a>
  <a href="https://github.com/worv-ai/CostNav"><img alt="Last commit" src="https://img.shields.io/github/last-commit/worv-ai/CostNav?style=flat&logo=github"></a>
  <img alt="Isaac Sim" src="https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?style=flat&logo=nvidia">
  <img alt="Isaac Lab" src="https://img.shields.io/badge/Isaac%20Lab-2.3.0-4CAF50?style=flat&logo=nvidia">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.11+-3776AB?style=flat&logo=python&logoColor=white">
  <a href="https://worv-ai.github.io/costnav/"><img alt="Project Page" src="https://img.shields.io/badge/Project%20Page-CostNav-blue?style=flat&logo=materialformkdocs&logoColor=white"></a>

  <h3>CostNav: A Navigation Benchmark for Real-World Economic-Cost Evaluation of Physical AI Agents</h3>
</div>

---

## Overview

CostNav introduces a **paradigm shift** in how we evaluate navigation systems: from purely technical metrics to actual economic cost and revenue.

Our key contributions are:

1. **High-Fidelity Physics Simulation with Dynamics for effective Real-World Economic Scenarios.**
   - Supporting Segway E1 delivery robot, food cargo dynamics with popcorn, detailed collision dynamics, pedestrians
1. **Real-world referenced Cost-Revenue Model with Break-Even Point Analysis.**
   - Supporting Energy Cost, Pedestrian Safety Cost, Property Damage Cost, Repair Cost
1. **Rule-Based and Learning-Based Navigation Evaluation with Multiple IL Baselines**
   - Comparing Profitability between Nav2 with GPS and Nav2 with AMCL localization
   - IL Baselines: ViNT, GNM, NoMaD, NavDP, and CANVAS

You can find more details in our [technical report](https://arxiv.org/abs/2511.20216).

The full cost benchmark formula with real world references is available in our google drive: https://drive.google.com/drive/folders/1j1MXm6NMkd6HHBwTJi_nSde7-shv4laX?usp=sharing

## Media

<video src="https://github.com/worv-ai/CostNav/raw/main/docs/assets/videos/comparison_part1.mp4" controls></video>

### Navigation Comparison

<video src="https://github.com/worv-ai/CostNav/raw/main/docs/assets/videos/comparison_part2.mp4" controls></video>

Side-by-side comparison of rule-based and learning-based navigation methods in CostNav's urban sidewalk environment.

### Physics Simulation

<video src="https://github.com/worv-ai/CostNav/raw/main/docs/assets/videos/costnav_popcorn.mp4" controls></video>

CostNav's high-fidelity physics simulation enables the modeling of real-world economic scenarios, including critical failures like food spoilage and robot rollovers.

### Benchmark Comparison

<img alt="Benchmark Comparison" src="docs/assets/images/bench_comparison.png" />

Comparison of existing navigation benchmarks (UnrealZoo, OpenBench, Arena-RosNav, Urban-Sim, DeliveryBench) that focus on task-oriented metrics versus CostNav's integration of physics simulation with comprehensive economic cost modeling.

### Economic Model

<img alt="Economic Model" src="docs/assets/images/figure2.drawio.png" />

CostNav's framework linking navigation performance to business value through profit-per-run measurement.

## Documentation

Full documentation is available at **[worv-ai.github.io/CostNav](https://worv-ai.github.io/CostNav/)**.

| Guide | Description |
|:------|:------------|
| [Quick Reference](https://worv-ai.github.io/CostNav/quick_reference/) | Installation, commands, and project structure |
| [Assets Setup](https://worv-ai.github.io/CostNav/assets_setup/) | Download and configure Omniverse USD assets |
| [Isaac Sim Integration](https://worv-ai.github.io/CostNav/isaacsim_guide/) | Mission manager, ROS2 topics, and launch.py reference |
| [Nav2 Baseline](https://worv-ai.github.io/CostNav/nav2_baseline/) | Rule-based navigation with ROS2 Nav2 |
| [IL Baselines](https://worv-ai.github.io/CostNav/baselines/) | ViNT, NoMaD, GNM, NavDP, and CANVAS |
| [Teleoperation](https://worv-ai.github.io/CostNav/teleop_guide/) | Joystick-based robot control for data collection |
| [Evaluation](https://worv-ai.github.io/CostNav/evaluation/) | Unified eval script, metrics, and log output |
| [Cost Model](https://worv-ai.github.io/CostNav/cost_model/) | CAPEX, OPEX, revenue, and break-even analysis |
| [Contributing](https://worv-ai.github.io/CostNav/contributing/) | How to contribute, roadmap, and PR guidelines |

## Contributing

Help us build a large-scale, ever-expanding benchmark!
We highly encourage contributions via issues and pull requests, especially adding more navigation baselines!

## Citation

To Cite CostNav, please use the following bibtex citation

```
@misc{seong2026costnavnavigationbenchmarkrealworld,
      title={CostNav: A Navigation Benchmark for Real-World Economic-Cost Evaluation of Physical AI Agents},
      author={Haebin Seong and Sungmin Kim and Yongjun Cho and Myunchul Joe and Geunwoo Kim and Yubeen Park and Sunhoo Kim and Yoonshik Kim and Suhwan Choi and Jaeyoon Jung and Jiyong Youn and Jinmyung Kwak and Sunghee Ahn and Jaemin Lee and Younggil Do and Seungyeop Yi and Woojin Cheong and Minhyeok Oh and Minchan Kim and Seongjae Kang and Samwoo Seong and Youngjae Yu and Yunsung Lee},
      year={2026},
      eprint={2511.20216},
      archivePrefix={arXiv},
      primaryClass={cs.AI},
      url={https://arxiv.org/abs/2511.20216},
}
```
