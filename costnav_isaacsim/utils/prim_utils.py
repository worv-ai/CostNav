"""Prim utility helpers."""

import logging
from typing import Optional

logger = logging.getLogger("costnav_launch")


def get_prim_world_translation(stage, prim_path: str) -> Optional[tuple[float, float, float]]:
    """Return the world translation for a prim path."""
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
    """Set the world translation for a prim path."""
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        logger.warning("Prim not found for translation update: %s", prim_path)
        return

    xform = UsdGeom.Xformable(prim)
    translate_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
    translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
