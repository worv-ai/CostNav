# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""IL Baselines Evaluation Package for CostNav.

This package provides ROS2 nodes for evaluating imitation learning baselines
(ViNT, NoMaD, GNM, etc.) in Isaac Sim using the CostNav infrastructure.

Key Components:
    - evaluation.models: Neural network architectures (ViNT, NoMaD, etc.)
    - evaluation.agents: Policy inference agents
    - evaluation.nodes: ROS2 nodes for robot control

Usage with uv:
    uv run vint_policy_node --ros-args \\
        -p checkpoint:=/workspace/checkpoints/vint_nova_carter.pth \\
        -p model_config:=configs/vint_eval.yaml \\
        -p robot_config:=configs/robot_carter.yaml
"""

from evaluation import agents, models

__all__ = ["models", "agents"]
