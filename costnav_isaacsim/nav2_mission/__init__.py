# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Nav2 Mission Planning and Execution Package.

This package provides tools for:
- NavMesh-based position sampling for navigation goals
- Robot teleportation and mission initiation
- RViz marker visualization for start, goal, and robot positions
- Background mission execution via MissionRunner

Usage:
    from costnav_isaacsim.nav2_mission import NavMeshSampler, MissionRunner
    from costnav_isaacsim.config import load_mission_config

    # Load configuration
    config = load_mission_config()

    # Run missions in background
    runner = MissionRunner(config)
    runner.start()

Note:
    MarkerPublisher, MissionOrchestrator, and MissionRunner require ROS2 (rclpy).
    NavMeshSampler and SampledPosition can be used without ROS2.
"""

# Always available (no ROS2 dependency)
from .navmesh_sampler import NavMeshSampler, SampledPosition

# ROS2-dependent imports (lazy loading)
ROS2_AVAILABLE = False
try:
    import rclpy  # noqa: F401

    ROS2_AVAILABLE = True
except ImportError:
    pass

if ROS2_AVAILABLE:
    from .marker_publisher import MarkerPublisher
    from .mission_orchestrator import (
        MissionOrchestrator,
        MissionConfig,
        create_isaac_sim_teleport_callback,
    )
    from .mission_runner import MissionRunner

    __all__ = [
        "NavMeshSampler",
        "SampledPosition",
        "MarkerPublisher",
        "MissionOrchestrator",
        "MissionConfig",
        "MissionRunner",
        "create_isaac_sim_teleport_callback",
        "ROS2_AVAILABLE",
    ]
else:
    # Provide stub classes for documentation/type hints
    MarkerPublisher = None
    MissionOrchestrator = None
    MissionConfig = None
    MissionRunner = None
    create_isaac_sim_teleport_callback = None

    __all__ = [
        "NavMeshSampler",
        "SampledPosition",
        "ROS2_AVAILABLE",
    ]
