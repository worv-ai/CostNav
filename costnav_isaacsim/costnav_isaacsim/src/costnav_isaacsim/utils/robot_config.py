# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Robot configuration constants and resolution utilities."""

import os
from typing import Optional

# Default simulation constants
DEFAULT_PHYSICS_DT = 1.0 / 120.0
DEFAULT_RENDERING_DT = 1.0 / 30.0
WARMUP_STEPS = 100

# Robot configuration
DEFAULT_ROBOT_NAME = "nova_carter"

OMNI_URL = os.environ.get("OMNI_URL", "omniverse://localhost")

DEFAULT_USD_PATHS = {
    "nova_carter": f"{OMNI_URL}/Users/worv/costnav/Street_sidewalk.usd",
    "segway_e1": f"{OMNI_URL}/Users/worv/costnav/street_sidewalk_segwaye1_Corrected.usd",
}

DEFAULT_ROBOT_PRIM_PATHS = {
    "nova_carter": "/World/Nova_Carter_ROS/chassis_link",
    "segway_e1": "/World/Segway_E1_ROS2/base_link",
}

DEFAULT_GOAL_CAMERA_HEIGHTS = {
    "nova_carter": 0.3,  # Nova Carter camera height
    "segway_e1": 0.825,  # Segway E1 camera height
}

DEFAULT_CAMERA_USD_PATHS = {
    "nova_carter": f"{OMNI_URL}/Users/worv/costnav/NovaCarter/camera.usd",
    "segway_e1": f"{OMNI_URL}/Users/worv/costnav/SegwayE1/camera.usd",
}

ROBOT_NAME_ALIASES = {
    "segway": "segway_e1",
    "segway-e1": "segway_e1",
    "segwaye1": "segway_e1",
}


def resolve_robot_name(robot_name: Optional[str]) -> str:
    """Resolve the robot name from CLI or environment.

    Args:
        robot_name: Optional robot name override.

    Returns:
        Normalized robot name.
    """
    selected_robot = robot_name or os.environ.get("SIM_ROBOT") or DEFAULT_ROBOT_NAME
    return ROBOT_NAME_ALIASES.get(selected_robot, selected_robot)


def resolve_usd_path(usd_path: Optional[str], robot_name: Optional[str]) -> str:
    """Resolve the USD path based on CLI or robot selection.

    Args:
        usd_path: Optional USD path override.
        robot_name: Optional robot name for default selection.

    Returns:
        Resolved USD path string.
    """
    if usd_path:
        return usd_path

    selected_robot = resolve_robot_name(robot_name)
    if selected_robot not in DEFAULT_USD_PATHS:
        supported = ", ".join(sorted(DEFAULT_USD_PATHS.keys()))
        raise ValueError(f"Unknown robot '{selected_robot}'. Supported: {supported}")

    return DEFAULT_USD_PATHS[selected_robot]
