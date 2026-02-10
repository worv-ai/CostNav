"""Food spoilage evaluation helpers."""

from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger("costnav_mission_manager")


def update_food_root_prefix(state, mission_config) -> None:
    food_root = mission_config.food.prim_path
    if food_root:
        state.food_root_prim_path = food_root.rstrip("/")
        state.food_prefix_path = f"{state.food_root_prim_path}/"
    else:
        state.food_root_prim_path = None
        state.food_prefix_path = None


def count_food_pieces_in_bucket(state, mission_config) -> int:
    if not mission_config.food.enabled or state.food_pieces_prim_path is None:
        return 0

    try:
        import omni.usd
        from pxr import UsdGeom
        from isaacsim.core.utils import bounds as bounds_utils

        stage = omni.usd.get_context().get_stage()

        bucket_prim = stage.GetPrimAtPath(state.food_bucket_prim_path)
        if not bucket_prim.IsValid():
            logger.warning(f"[FOOD] Bucket prim not found: {state.food_bucket_prim_path}")
            return 0

        bbox_cache = bounds_utils.create_bbox_cache()
        bucket_bounds = bounds_utils.compute_aabb(bbox_cache, prim_path=state.food_bucket_prim_path)
        min_xyz = bucket_bounds[:3]
        max_xyz = bucket_bounds[3:]

        pieces_prim = stage.GetPrimAtPath(state.food_pieces_prim_path)
        if not pieces_prim.IsValid():
            logger.warning(f"[FOOD] Pieces prim not found: {state.food_pieces_prim_path}")
            return 0

        count = 0
        for child in pieces_prim.GetChildren():
            if not child.IsValid():
                continue
            xformable = UsdGeom.Xformable(child)
            if not xformable:
                continue

            world_transform = xformable.ComputeLocalToWorldTransform(0)
            position = world_transform.ExtractTranslation()

            if (
                min_xyz[0] <= position[0] <= max_xyz[0]
                and min_xyz[1] <= position[1] <= max_xyz[1]
                and min_xyz[2] <= position[2] <= max_xyz[2]
            ):
                count += 1

        return count
    except Exception as e:
        logger.error(f"[FOOD] Error counting food pieces: {e}")
        return 0


def _get_robot_z_offset(mission_config, manager_config) -> Optional[float]:
    robot_prim = mission_config.teleport.robot_prim.lower()

    if "segway" in robot_prim:
        return 0.33
    if "nova_carter" in robot_prim:
        return manager_config.teleport_height
    return None


def _spawn_food_at_position(
    state,
    mission_config,
    manager_config,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    remove_existing: bool = False,
) -> bool:
    food_config = mission_config.food
    base_path = food_config.prim_path.rstrip("/")

    z_offset = _get_robot_z_offset(mission_config, manager_config)
    if z_offset is None:
        robot_prim = mission_config.teleport.robot_prim.lower()
        logger.warning(
            f"[FOOD] Food spawning not implemented for robot: {robot_prim}. "
            f"Currently only segway_e1 is supported. Disabling food tracking."
        )
        mission_config.food.enabled = False
        return False

    try:
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()

        if remove_existing:
            food_prim = stage.GetPrimAtPath(base_path)
            if food_prim.IsValid():
                stage.RemovePrim(base_path)
                logger.info(f"[FOOD] Removed existing food prim at: {base_path}")

        food_prim = stage.GetPrimAtPath(base_path)
        if food_prim.IsValid():
            logger.info(f"[FOOD] Food prim already exists at: {base_path}")
            return True

        food_prim = stage.DefinePrim(base_path, "Xform")
        if not food_prim.IsValid():
            logger.error(f"[FOOD] Failed to create prim at: {base_path}")
            return False

        success = food_prim.GetReferences().AddReference(food_config.usd_path)
        if not success:
            logger.error(f"[FOOD] Failed to add USD reference: {food_config.usd_path}")
            return False

        final_z = z + z_offset
        xformable = UsdGeom.Xformable(food_prim)
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(x, y, final_z))

        logger.info(f"[FOOD] Spawned food asset from: {food_config.usd_path}")
        logger.info(f"[FOOD] Position: ({x:.2f}, {y:.2f}, {final_z:.2f})")
        return True

    except Exception as e:
        logger.error(f"[FOOD] Error spawning food asset: {e}")
        return False


def setup_food_tracking(state, mission_config, manager_config) -> None:
    if not mission_config.food.enabled:
        return

    if not _spawn_food_at_position(state, mission_config, manager_config, x=0.0, y=0.0, z=0.0):
        return

    food_config = mission_config.food
    base_path = food_config.prim_path.rstrip("/")
    state.food_pieces_prim_path = f"{base_path}/{food_config.pieces_prim_path}"
    state.food_bucket_prim_path = f"{base_path}/{food_config.bucket_prim_path}"

    logger.info("[FOOD] Food tracking enabled")
    logger.info(f"[FOOD] Pieces path: {state.food_pieces_prim_path}")
    logger.info(f"[FOOD] Bucket path: {state.food_bucket_prim_path}")


def check_food_spoilage(state, mission_config) -> bool:
    if not mission_config.food.enabled:
        return False

    state.final_food_piece_count = count_food_pieces_in_bucket(state, mission_config)

    if state.initial_food_piece_count == 0:
        return False

    pieces_lost = state.initial_food_piece_count - state.final_food_piece_count
    loss_fraction = pieces_lost / state.initial_food_piece_count

    logger.info(
        f"[FOOD] Pieces: initial={state.initial_food_piece_count}, "
        f"final={state.final_food_piece_count}, lost={pieces_lost} ({loss_fraction:.1%})"
    )

    return loss_fraction > mission_config.food.spoilage_threshold


def reset_food_for_teleport(state, mission_config, manager_config) -> bool:
    if not mission_config.food.enabled:
        return True

    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        robot_prim_path = mission_config.teleport.robot_prim
        robot_prim = stage.GetPrimAtPath(robot_prim_path)

        if not robot_prim.IsValid():
            logger.error(f"[FOOD] Robot prim not found: {robot_prim_path}")
            return False

        xformable = UsdGeom.Xformable(robot_prim)
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        robot_pos = world_transform.ExtractTranslation()

        logger.info(f"[FOOD] Robot position from prim: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f})")

        return _spawn_food_at_position(
            state,
            mission_config,
            manager_config,
            x=robot_pos[0],
            y=robot_pos[1],
            z=robot_pos[2],
            remove_existing=True,
        )
    except Exception as e:
        logger.error(f"[FOOD] Error getting robot position: {e}")
        return False
