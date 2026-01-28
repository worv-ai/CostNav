# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""ROS2 Nodes for IL Baseline Evaluation.

This module provides ROS2 nodes for running IL baseline policies:
- vint_policy_node: ViNT visual navigation policy

Usage with uv:
    uv run vint_policy_node --ros-args -p checkpoint:=/workspace/checkpoints/vint_nova_carter.pth
"""

__all__ = ["vint_policy_node"]
