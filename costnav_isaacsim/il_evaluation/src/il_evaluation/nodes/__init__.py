# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""ROS2 Nodes for IL Evaluation.

This module provides ROS2 nodes for running IL policies:
- vint_policy_node: ViNT visual navigation policy (inference at ~4 Hz)
- gnm_policy_node: GNM visual navigation policy (inference at ~4 Hz)
- nomad_policy_node: NoMaD diffusion policy (inference at ~4 Hz)
- trajectory_follower_node: Trajectory following controller (cmd_vel at ~20 Hz)

Usage with uv:
    uv run vint_policy_node --checkpoint /path/to/model.pth --model_config ... --robot_config ...
    uv run gnm_policy_node --checkpoint /path/to/model.pth --model_config ... --robot_config ...
    uv run nomad_policy_node --checkpoint /path/to/model.pth --model_config ... --robot_config ...
    uv run trajectory_follower_node --robot_config configs/robot_carter.yaml
"""

__all__ = ["vint_policy_node", "gnm_policy_node", "nomad_policy_node", "trajectory_follower_node"]
