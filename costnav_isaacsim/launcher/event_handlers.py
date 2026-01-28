# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Simulation event handler utilities for stage and physics events."""

import logging
import os
import time
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from isaacsim.core.api import SimulationContext

logger = logging.getLogger("costnav_launch")


def register_stage_open_handler(callback) -> Optional[Any]:
    """Register a handler for stage open events.

    Args:
        callback: Function to call when stage is opened.

    Returns:
        Subscription object or None if registration failed.
    """
    try:
        import carb.eventdispatcher
        import omni.usd

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        subscription = dispatcher.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=callback,
            observer_name="costnav.launcher.stage_open",
        )
        return subscription
    except Exception as exc:
        logger.warning("Failed to register stage open handler: %s", exc)
        return None


def handle_physics_reinit(
    simulation_context: "SimulationContext",
    pending_physics_reinit: bool,
    pending_stage_reload: bool,
    last_physics_reset_time: Optional[float],
    mission_manager=None,
    people_manager=None,
    people_initialized: bool = False,
    refresh_robot_callback=None,
    simulation_app=None,
) -> tuple[bool, bool, Optional[float], bool]:
    """Handle pending physics reinitialization after stage reload or view invalidation.

    Args:
        simulation_context: The simulation context instance.
        pending_physics_reinit: Whether physics reinit is pending.
        pending_stage_reload: Whether stage reload is pending.
        last_physics_reset_time: Timestamp of last physics reset.
        mission_manager: Optional mission manager to notify of restart.
        people_manager: Optional people manager to reinitialize.
        people_initialized: Whether people have been initialized.
        refresh_robot_callback: Callback to refresh robot prim paths.
        simulation_app: The simulation app instance.

    Returns:
        Tuple of (pending_physics_reinit, pending_stage_reload, last_physics_reset_time, people_initialized).
    """
    try:
        from isaacsim.core.utils.stage import is_stage_loading

        if is_stage_loading():
            return pending_physics_reinit, pending_stage_reload, last_physics_reset_time, people_initialized
    except Exception as exc:
        logger.warning("Failed to query stage loading state: %s", exc)

    force_reset = False
    sim_view = simulation_context.physics_sim_view
    if sim_view is not None:
        try:
            if hasattr(sim_view, "is_valid") and not sim_view.is_valid:
                pending_physics_reinit = True
                force_reset = True
            elif hasattr(sim_view, "check") and not sim_view.check():
                pending_physics_reinit = True
                force_reset = True
        except Exception:
            pending_physics_reinit = True
            force_reset = True

    if not pending_physics_reinit and not pending_stage_reload:
        return pending_physics_reinit, pending_stage_reload, last_physics_reset_time, people_initialized

    stage_reloaded = pending_stage_reload
    needs_people_restart = stage_reloaded or force_reset

    if stage_reloaded and refresh_robot_callback is not None:
        refresh_robot_callback()

    if force_reset:
        now = time.perf_counter()
        if last_physics_reset_time is None or (now - last_physics_reset_time) > 1.0:
            last_physics_reset_time = now
            try:
                simulation_context.reset()
            except Exception as exc:
                logger.warning("Failed to reset simulation after view invalidation: %s", exc)

    try:
        simulation_context.initialize_physics()
    except Exception as exc:
        logger.warning("Failed to initialize physics after restart: %s", exc)

    if mission_manager is not None:
        try:
            mission_manager.handle_simulation_restart(stage_reloaded=stage_reloaded)
        except Exception as exc:
            logger.warning("Failed to refresh mission manager after restart: %s", exc)

    # Only restart people if the stage was reloaded OR if people haven't been initialized yet.
    should_restart_people = (
        needs_people_restart
        and people_manager is not None
        and (stage_reloaded or not people_initialized)
    )
    if should_restart_people and simulation_app is not None:
        try:
            people_manager.shutdown()
            people_manager.initialize(simulation_app, simulation_context)
            people_initialized = True
        except Exception as exc:
            logger.warning("Failed to reinitialize PeopleManager after physics reset: %s", exc)

    return False, False, last_physics_reset_time, people_initialized

