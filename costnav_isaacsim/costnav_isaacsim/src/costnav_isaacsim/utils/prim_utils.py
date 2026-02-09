# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""USD prim utility functions for robot and stage manipulation."""

import logging
from typing import Optional

from .robot_config import DEFAULT_ROBOT_PRIM_PATHS

logger = logging.getLogger("costnav_launch")


def resolve_robot_prim_path(
    stage,
    robot_name: str,
    env_override: Optional[str] = None,
) -> Optional[str]:
    """Resolve a robot prim path for pose lookups.

    Args:
        stage: USD stage instance.
        robot_name: Name of the robot (e.g., 'nova_carter', 'segway_e1').
        env_override: Optional environment variable override for prim path.

    Returns:
        Resolved prim path string or None if not found.
    """
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
    """Resolve robot prim path for PeopleAPI usage.

    Args:
        robot_prim_path: Original robot prim path.

    Returns:
        Adjusted prim path suitable for PeopleAPI.
    """
    if not robot_prim_path:
        return "/World/Nova_Carter_ROS"

    if robot_prim_path.endswith("/chassis_link"):
        return robot_prim_path.rsplit("/", 1)[0]

    return robot_prim_path


def find_robot_prim_by_tokens(stage, tokens: tuple[str, ...]) -> Optional[str]:
    """Find a prim path containing any of the supplied tokens.

    Args:
        stage: USD stage instance.
        tokens: Tuple of token strings to search for in prim names.

    Returns:
        Found prim path string or None.
    """
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


def get_prim_world_translation(stage, prim_path: str) -> Optional[tuple[float, float, float]]:
    """Return the world translation for a prim path.

    Args:
        stage: USD stage instance.
        prim_path: Path to the prim.

    Returns:
        Tuple of (x, y, z) world coordinates or None if prim not found.
    """
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        logger.warning("Prim not found for translation lookup: %s", prim_path)
        return None

    xform_cache = UsdGeom.XformCache()
    transform = xform_cache.GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    return (float(translation[0]), float(translation[1]), float(translation[2]))


def set_prim_world_translation(stage, prim_path: str, position: tuple[float, float, float]) -> None:
    """Set the world translation for a prim path.

    Args:
        stage: USD stage instance.
        prim_path: Path to the prim.
        position: Tuple of (x, y, z) world coordinates.
    """
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        logger.warning("Prim not found for translation update: %s", prim_path)
        return

    xform = UsdGeom.Xformable(prim)
    translate_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
    translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
