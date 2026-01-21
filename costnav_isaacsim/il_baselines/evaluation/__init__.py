# Copyright (c) 2024 CostNav Authors
# Licensed under the MIT License

"""IL Baselines Evaluation Package for CostNav.

This package provides ROS2 nodes for evaluating imitation learning baselines
(ViNT, NoMaD, GNM, etc.) in Isaac Sim using the CostNav infrastructure.

Key Components:
    - models: Neural network architectures (ViNT, NoMaD, etc.)
    - agents: Policy inference agents
    - nodes: ROS2 nodes for robot control

Usage:
    # Launch ViNT policy node
    ros2 launch costnav_il_baselines vint_policy.launch.py \\
        checkpoint:=/path/to/model.pth

    # Or run directly
    ros2 run costnav_il_baselines vint_policy_node \\
        --ros-args \\
        -p checkpoint:=/path/to/model.pth \\
        -p model_config:=/path/to/config.yaml \\
        -p robot_config:=/path/to/robot.yaml
"""

from . import models
from . import agents

__all__ = ["models", "agents"]
