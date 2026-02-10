"""Contact and impulse evaluation helpers."""

from __future__ import annotations

import logging
from typing import Callable, Optional

from . import injury

logger = logging.getLogger("costnav_mission_manager")

PROPERTY_PRIM_PATHS = {
    "fire_hydrant": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section19",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section53",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section57",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section55",
    ],
    "traffic_light": [
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section32",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section31",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section31",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section12",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section39",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section39",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section41",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section41",
    ],
    "street_lamp": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section1",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section1",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section1",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section1",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section78",
    ],
    "bollard": [
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section77",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section31",
    ],
    # Building prims are matched broadly via prefix; keep only explicit non-standard
    # building-related prims here (e.g., cubes that represent building parts).
    "building": [
        "/World/box/Cube",
        "/World/box/Cube_01",
        "/World/box/Cube_02",
        "/World/box/Cube_03",
        "/World/box/Cube_04",
        "/World/box/Cube_05",
        "/World/box/Cube_06",
        "/World/box/Cube_07",
        "/World/box/Cube_08",
        "/World/box/Cube_09",
        "/World/box/Cube_10",
        "/World/box/Cube_11",
        "/World/box/Cube_12",
        "/World/box/Cube_13",
    ],
    "trash_bin": [
        "/World/Environment/SM_Buidlng_032/SM_Buidlng_032/Section26",
    ],
    "mail_box": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section20",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section85",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section55",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section18",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section18",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section18",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section57",
    ],
    "newspaper_box": [
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section21",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section21",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section21",
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section23",
    ],
    "bus_stop": [
        "/World/Environment/SM_StreetDetails_001/SM_StreetDetails_001/Section8",
        "/World/Environment/SM_StreetDetails_002/SM_StreetDetails_03/SM_StreetDetails_002/Section6",
        "/World/Environment/SM_StreetDetails_003/SM_StreetDetails_003/Section6",
        "/World/Environment/SM_StreetDetails_004/SM_StreetDetails_004/Section6",
    ],
}

_COST_PROPERTY_CATEGORIES = {"mail_box", "trash_bin", "building", "bollard"}


def init_property_contact_counts() -> dict[str, int]:
    return {key: 0 for key in PROPERTY_PRIM_PATHS}


def reset_impulse_health(state) -> None:
    state.impulse_damage_accumulated = 0.0
    state.impulse_health = state.impulse_health_max
    state.last_damage_steps_remaining = 0
    state.contact_count = 0
    state.people_contact_count = 0
    state.total_impulse = 0.0
    state.property_contact_counts = init_property_contact_counts()
    state.delta_v_magnitudes_mps = []
    state.injury_costs = []
    state.total_injury_cost = 0.0


def setup_contact_reporting(state, mission_config, manager_config, on_contact_report: Callable) -> None:
    base_link_path = mission_config.teleport.robot_prim or manager_config.robot_prim_path
    if base_link_path:
        base_link_path = base_link_path.rstrip("/")

    robot_hint = base_link_path.lower() if base_link_path else ""
    if "segway" in robot_hint:
        base_link_path = "/World/Segway_E1_ROS2/base_link"
    elif "nova_carter" in robot_hint or "carter" in robot_hint:
        base_link_path = "/World/Nova_Carter_ROS/chassis_link"

    if not base_link_path:
        logger.warning("[CONTACT] Robot prim path not configured; skipping contact reporting")
        return

    try:
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysxSchema

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(base_link_path)
        if not prim.IsValid():
            logger.warning(f"[CONTACT] Base link prim not found: {base_link_path}")
            return

        contact_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        contact_api.CreateThresholdAttr().Set(state.impulse_min_threshold)
        state.contact_report_targets = {base_link_path}

        if state.contact_report_subscription is None:
            sim_interface = get_physx_simulation_interface()
            state.contact_report_subscription = sim_interface.subscribe_contact_report_events(on_contact_report)

        logger.info(f"[CONTACT] Contact reporting enabled for {base_link_path}")
    except ImportError as exc:
        logger.warning(f"[CONTACT] Isaac Sim modules not available: {exc}")
    except Exception as exc:
        logger.error(f"[CONTACT] Failed to setup contact reporting: {exc}")


def classify_property_from_prim_path(prim_path: str) -> Optional[str]:
    if not prim_path:
        return None
    # Exception in trash bin path.
    if prim_path.startswith("/World/Environment/SM_Buidlng_032/SM_Buidlng_032/Section26"):
        return "trash_bin"
    # Broad match for any building prims (handle both SM_Buidlng_ and SM_Buildlng_)
    if prim_path.startswith("/World/Environment/SM_Buidlng_") or prim_path.startswith(
        "/World/Environment/SM_Buildlng_"
    ):
        return "building"
    for category, paths in PROPERTY_PRIM_PATHS.items():
        for base_path in paths:
            if prim_path == base_path or prim_path.startswith(base_path + "/"):
                return category
    return None


def record_property_contact_from_pair(
    state, actor0_path: str, actor1_path: str, impulse_amount: float
) -> Optional[str]:
    if impulse_amount < state.property_contact_impulse_min_threshold:
        return None

    target = next(iter(state.contact_report_targets), None)
    if target:
        if actor0_path == target or actor0_path.startswith(target + "/"):
            category = classify_property_from_prim_path(actor1_path)
        elif actor1_path == target or actor1_path.startswith(target + "/"):
            category = classify_property_from_prim_path(actor0_path)
        else:
            category = None
    else:
        category = None

    if category is None:
        category = classify_property_from_prim_path(actor0_path)
    if category is None:
        category = classify_property_from_prim_path(actor1_path)

    if category is None:
        return None
    if category not in _COST_PROPERTY_CATEGORIES:
        return None

    print(f"[CONTACT] Property contact: {category} {impulse_amount}")

    state.property_contact_counts[category] += 1
    return category


def apply_impulse_damage(
    state,
    impulse_amount: float,
    is_mission_active: Callable[[], bool],
    injury_info: Optional[tuple[float, float, float]],
) -> None:
    if not is_mission_active():
        msg = f"[CONTACT] Impulse: {impulse_amount:.2f}"
        if injury_info:
            delta_v_mps, injury_cost, total_injury_cost = injury_info
            msg += (
                f" | delta_v={delta_v_mps:.4f} m/s ({delta_v_mps * injury.MPS_TO_MPH:.2f} mph), "
                f"{injury_cost=:.2f}, {total_injury_cost=:.2f}"
            )
        print(msg)
        return

    if state.impulse_health <= 0.0:
        return

    state.contact_count += 1
    state.total_impulse += impulse_amount

    state.impulse_damage_accumulated += impulse_amount
    state.impulse_health = max(0.0, state.impulse_health_max - state.impulse_damage_accumulated)
    msg = f"[CONTACT] Impulse: {impulse_amount:.2f}, Health: {state.impulse_health:.2f}, Count: {state.contact_count}"
    if injury_info:
        delta_v_mps, injury_cost, total_injury_cost = injury_info
        msg += (
            f" | delta_v={delta_v_mps:.4f} m/s ({delta_v_mps * injury.MPS_TO_MPH:.2f} mph), "
            f"cost={injury_cost:.2f}, total={total_injury_cost:.2f}"
        )
    print(msg)


def on_contact_report(
    state, contact_headers, contact_data, mission_config, is_mission_active: Callable[[], bool]
) -> None:
    if not state.contact_report_targets:
        return

    if state.last_damage_steps_remaining > 0:
        state.last_damage_steps_remaining = max(0, state.last_damage_steps_remaining - 1)
        return

    try:
        from pxr import PhysicsSchemaTools
    except ImportError:
        return

    food_root: str = state.food_root_prim_path or ""
    for header in contact_headers:
        actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
        if (actor0 not in state.contact_report_targets) and (actor1 not in state.contact_report_targets):
            continue

        if food_root:
            food_prefix_str: str = f"{food_root}/"
            if actor0 in state.contact_report_targets and (
                actor1 == food_root or actor1[: len(food_prefix_str)] == food_prefix_str
            ):
                continue
            if actor1 in state.contact_report_targets and (
                actor0 == food_root or actor0[: len(food_prefix_str)] == food_prefix_str
            ):
                continue

        if header.num_contact_data == 0:
            continue

        is_character_collision = False
        if "/World/Characters/" in actor0 or "/World/Characters/" in actor1:
            is_character_collision = True

        contact_range = range(
            header.contact_data_offset,
            header.contact_data_offset + header.num_contact_data,
        )
        for idx in contact_range:
            impulse = contact_data[idx].impulse
            impulse_amount = (impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z) ** 0.5
            if impulse_amount < state.impulse_min_threshold:
                continue
            if is_character_collision:
                state.people_contact_count += 1
            record_property_contact_from_pair(state, actor0, actor1, impulse_amount)
            injury_info = injury.process_collision_injury(state, mission_config, impulse_amount, is_character_collision)
            apply_impulse_damage(state, impulse_amount, is_mission_active, injury_info)
            state.last_damage_steps_remaining = state.damage_cooldown_steps
            return
