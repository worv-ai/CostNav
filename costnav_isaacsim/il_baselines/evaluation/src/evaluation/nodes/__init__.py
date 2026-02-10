# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""ROS2 Nodes for IL Baseline Evaluation.

This module provides ROS2 nodes for running IL baseline policies:
- vint_policy_node: ViNT visual navigation policy (inference at ~10 Hz)
- trajectory_follower_node: Trajectory following controller (cmd_vel at ~20 Hz)

Usage with uv:
    uv run vint_policy_node --checkpoint /path/to/model.pth --model_config ... --robot_config ...
    uv run trajectory_follower_node --control_rate 20.0 --max_linear_vel 1.0 --max_angular_vel 1.0
"""

__all__ = ["vint_policy_node", "trajectory_follower_node"]
