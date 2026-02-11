# CostNav

<img width="3200" height="1344" alt="image" src="https://github.com/user-attachments/assets/dd16349c-5126-415d-9d34-38c3e7137d9e" />

<div align="center">
  <a href="https://arxiv.org/abs/2511.20216"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2511.20216-b31b1b.svg?style=flat"></a>
  <a href="https://worv.ghost.io/costnav-2/"><img alt="Blog Post" src="https://img.shields.io/badge/blog-post-blueviolet?style=flat&logo=ghost&logoColor=white"></a>
  <a href="https://github.com/worv-ai/CostNav/issues"><img alt="Open issues" src="https://img.shields.io/github/issues/worv-ai/CostNav?style=flat"></a>
  <a href="https://github.com/worv-ai/CostNav/stargazers"><img alt="GitHub stars" src="https://img.shields.io/github/stars/worv-ai/CostNav?style=flat"></a>
  <a href="https://github.com/worv-ai/CostNav"><img alt="Last commit" src="https://img.shields.io/github/last-commit/worv-ai/CostNav?style=flat&logo=github"></a>
  <img alt="Isaac Sim" src="https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?style=flat&logo=nvidia">
  <img alt="Isaac Lab" src="https://img.shields.io/badge/Isaac%20Lab-2.3.0-4CAF50?style=flat&logo=nvidia">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.11+-3776AB?style=flat&logo=python&logoColor=white">
  <!--<a href="https://worv-ai.github.io/CostNav"><img alt="Documentation" src="https://img.shields.io/badge/docs-material-blue?style=flat&logo=materialformkdocs&logoColor=white"></a>-->

  <h3>CostNav: A Navigation Benchmark for Real-World Economic-Cost Evaluation of Physical AI Agents</h3>
</div>

---

## Overview

CostNav introduces a **paradigm shift** in how we evaluate navigation systems: from purely technical metrics to actual economic cost and revenue.

Our key contributions are:

1. **High-Fidelity Physics Simulation for effective Real-World Economic Scenarios.**  
   a. Supporting Segway E1 delivery robot, food cargo dynamics with popcorn, detailed collision dynamics, pedestrians
2. **Real-world referenced Cost-Revenue Model with Break-Even Point Analysis.**  
   a. Supporting Energy Cost, Pedestrian Safety Cost, Property Damage Cost, Repair Cost
3. **Rule based Navigation Evaluation (Coming up soon: Learning based Navigation Evaluation and Dataset)**  
   a. Comparing Profitability between Nav2 with GPS and Nav2 with AMCL localization

You can find more details in our [technical report](https://arxiv.org/abs/2511.20216).

## Getting Started

### 1. Prerequisites

- Linux host PC (Ubuntu 24.04 preferred)
- NVIDIA GPU (dependency of isaac-sim) with recent graphics drivers
- Docker with nvidia container toolkit.

### 2. Clone and fetch references

```bash
git clone https://github.com/worv-ai/CostNav.git
cd CostNav
make fetch-third-party # we use third-party submodules for reference or dependencies
```

### 3. Configure environment variables

1. Copy `.env.example` to `.env`.
2. Set `NGC_PASS` by making an account and an api key in https://org.ngc.nvidia.com/setup/api-keys
3. Set `PROJECT_ROOT` as the absolute path of cloned `CostNav`.

### 4. Build dependencies

```bash
cd CostNav
make build-ros2
make build-isaac-sim
```

### 5. Download Assets

```bash
# for open source users

# Download assets from HuggingFace
make download-assets-hf

# Start local Nucleus server, upload local assets to nucleus omniverse
make start-nucleus


# Stop Nucleus server
# make stop-nucleus
```

```bash
# for internal developers

# Or download from Omniverse (for internal developers)
# make download-assets-omniverse


# Upload assets to HuggingFace (for internal developers, requires HF_TOKEN in .env)
# make upload-assets-hf

```

## Running Nav2 (Rule-Based Navigation)

For rule-based navigation using ROS2 Nav2 stack with Isaac Sim, see the [costnav_isaacsim README](costnav_isaacsim/README.md) for detailed setup and usage.

```bash

# Run Nav2 navigation (Isaac Sim + ROS2 Nav2)
# Usage: make run-nav2 NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True TUNED=True AMCL=False
make run-nav2


make start-mission # start single mission

make run-eval-nav2 # run evaluation in nav2
```

This starts Isaac Sim with the Street Sidewalk environment and Segway E1 robot, along with the ROS2 Nav2 stack for classical navigation.

## Running Teleop (For Data Collection)

```bash

# Usage: make run-teleop NUM_PEOPLE=20 SIM_ROBOT=segway_e1 FOOD=True GOAL_IMAGE=True
make run-teleop

make run-rosbag # start rosbag record

make start-mission # start single mission

make stop-rosbag # stop rosbag record when mission is completed

make run-eval-teleop # run evaluation in teleop mode
```

> **Tip:** Press **Ctrl+C once** to stop teleop. The teardown will run automatically — do not press Ctrl+C again while containers are being cleaned up.

## Running IL Baselines (ViNT)

1. Download checkpoint or train a model  
   Link is from [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer)

```
gdown --folder https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg
```

Place the downloaded model files (e.g., `vint.pth`, `gnm.pth`, `nomad.pth`) in the `checkpoints/` directory.

2. Build a docker image

```bash
make build-vint
```

4. Run the evaluation

```bash
# Terminal 1: Start the ViNT stack
MODEL_CHECKPOINT=checkpoints/vint.pth make run-vint

# Terminal 2: Run evaluation
make run-eval-vint TIMEOUT=169 NUM_MISSIONS=10
```

## What's next?

- [x] Paper release
- [x] isaac sim & nav2 support for rule-based navigation
- [ ] cost formula and reference sheet
- [ ] imitation learning baseline, and collected dataset with teleoperation

## Contributing

Help us build a large-scale, ever-expanding benchmark!  
We highly encourage contributions via issues and pull requests, especially adding more navigation baselines!

## Contact

Maintained by the Maum.AI WoRV team.  
For research collaborations or enterprise deployments, please contact https://worv-ai.github.io/.

## Citation

To Cite CostNav, please use the following bibtex citation

```
@misc{seong2026costnavnavigationbenchmarkrealworld,
      title={CostNav: A Navigation Benchmark for Real-World Economic-Cost Evaluation of Physical AI Agents},
      author={Haebin Seong and Sungmin Kim and Yongjun Cho and Myunchul Joe and Geunwoo Kim and Yubeen Park and Sunhoo Kim and Yoonshik Kim and Suhwan Choi and Jaeyoon Jung and Jiyong Youn and Jinmyung Kwak and Sunghee Ahn and Jaemin Lee and Younggil Do and Seungyeop Yi and Woojin Cheong and Minhyeok Oh and Minchan Kim and Yoonseok Kang and Seongjae Kang and Samwoo Seong and Youngjae Yu and Yunsung Lee},
      year={2026},
      eprint={2511.20216},
      archivePrefix={arXiv},
      primaryClass={cs.AI},
      url={https://arxiv.org/abs/2511.20216},
}
```
