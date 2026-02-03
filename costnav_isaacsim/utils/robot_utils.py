"""Robot and prim path resolution helpers."""

import logging
import os
from typing import Optional

from .launch_constants import DEFAULT_ROBOT_NAME, DEFAULT_ROBOT_PRIM_PATHS, DEFAULT_USD_PATHS, ROBOT_NAME_ALIASES

logger = logging.getLogger("costnav_launch")


def resolve_robot_name(robot_name: Optional[str]) -> str:
    """Resolve the robot name from CLI or environment."""
    selected_robot = robot_name or os.environ.get("SIM_ROBOT") or DEFAULT_ROBOT_NAME
    return ROBOT_NAME_ALIASES.get(selected_robot, selected_robot)


def resolve_usd_path(usd_path: Optional[str], robot_name: Optional[str]) -> str:
    """Resolve the USD path based on CLI or robot selection."""
    if usd_path:
        return usd_path

    selected_robot = resolve_robot_name(robot_name)
    if selected_robot not in DEFAULT_USD_PATHS:
        supported = ", ".join(sorted(DEFAULT_USD_PATHS.keys()))
        raise ValueError(f"Unknown robot '{selected_robot}'. Supported: {supported}")

    return DEFAULT_USD_PATHS[selected_robot]


def resolve_robot_prim_path(stage, robot_name: str) -> Optional[str]:
    """Resolve a robot prim path for pose lookups."""
    env_override = os.environ.get("ROBOT_PRIM_PATH")
    if env_override:
        prim = stage.GetPrimAtPath(env_override)
        if prim.IsValid():
            return env_override
        logger.warning("ROBOT_PRIM_PATH not found on stage: %s", env_override)

    default_path = DEFAULT_ROBOT_PRIM_PATHS.get(robot_name)
    if default_path:
        prim = stage.GetPrimAtPath(default_path)
        if prim.IsValid():
            return default_path
        logger.warning("Default robot prim path not found: %s", default_path)

    if robot_name == "segway_e1":
        return find_robot_prim_by_tokens(stage, ("Segway", "segway"))

    return None


def resolve_people_robot_prim(robot_prim_path: Optional[str]) -> str:
    """Resolve robot prim path for PeopleAPI usage."""
    if not robot_prim_path:
        return "/World/Nova_Carter_ROS"

    if robot_prim_path.endswith("/chassis_link"):
        return robot_prim_path.rsplit("/", 1)[0]

    return robot_prim_path


def find_robot_prim_by_tokens(stage, tokens: tuple[str, ...]) -> Optional[str]:
    """Find a prim path containing any of the supplied tokens."""
    from pxr import UsdGeom

    for prim in stage.Traverse():
        name = prim.GetName()
        if not name:
            continue
        if not any(token in name for token in tokens):
            continue
        if not (prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Xformable)):
            continue
        return prim.GetPath().pathString

    return None
