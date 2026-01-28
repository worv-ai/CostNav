# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Isaac Sim extension management utilities."""

import logging

from isaacsim import SimulationApp

logger = logging.getLogger("costnav_launch")


def enable_isaac_sim_extensions(simulation_app: SimulationApp, num_people: int = 0) -> bool:
    """Enable required Isaac Sim extensions.

    Args:
        simulation_app: The SimulationApp instance.
        num_people: Number of people to spawn (enables PeopleAPI if > 0).

    Returns:
        True if all extensions enabled successfully, False if PeopleAPI failed.
    """
    from isaacsim.core.utils.extensions import enable_extension
    import omni.kit.app

    # Navigation extensions (must be enabled before using navmesh)
    enable_extension("omni.anim.navigation.core")

    # Debugging extension
    enable_extension("omni.anim.navigation.bundle")
    enable_extension("omni.physx.bundle")

    # Core extensions
    enable_extension("omni.isaac.sensor")
    enable_extension("omni.replicator.core")

    # ROS2 bridge for Nav2 communication
    enable_extension("isaacsim.ros2.bridge")

    # People API extension (if people spawning is enabled)
    if num_people > 0:
        if not _enable_people_api(simulation_app):
            return False

    simulation_app.update()
    return True


def _enable_people_api(simulation_app: SimulationApp) -> bool:
    """Enable PeopleAPI extension and its dependencies.

    Args:
        simulation_app: The SimulationApp instance.

    Returns:
        True if enabled successfully, False otherwise.
    """
    from isaacsim.core.utils.extensions import enable_extension
    import omni.kit.app

    logger.info("Enabling PeopleAPI extension...")

    # Add extension search path for PeopleAPI
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_path = "/isaac-sim/extsUser"
    logger.info(f"Adding extension search path: {ext_path}")
    ext_manager.add_path(ext_path)

    # Verify the extension can be found
    if not ext_manager.is_extension_enabled("omni.anim.people_api"):
        logger.info("PeopleAPI extension not yet enabled, enabling dependencies...")

    # Enable required dependencies for PeopleAPI (in order)
    try:
        enable_extension("omni.anim.graph.core")
        simulation_app.update()
        enable_extension("omni.anim.graph.schema")
        simulation_app.update()
        enable_extension("omni.anim.retarget.core")
        simulation_app.update()
        enable_extension("omni.kit.scripting")
        simulation_app.update()
        enable_extension("omni.metropolis.utils")
        simulation_app.update()

        # Finally enable PeopleAPI
        enable_extension("omni.anim.people_api")
        simulation_app.update()
        logger.info("PeopleAPI extension enabled successfully")

        # Give extensions time to fully initialize
        for _ in range(10):
            simulation_app.update()

        return True

    except Exception as e:
        logger.error(f"Failed to enable PeopleAPI extension: {e}")
        logger.warning("People spawning will be disabled")
        return False

