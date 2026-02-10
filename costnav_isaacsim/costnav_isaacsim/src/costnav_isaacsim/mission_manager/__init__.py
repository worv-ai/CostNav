# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Mission Planning and Execution Package.

This package provides tools for:
- NavMesh-based position sampling for navigation goals
- Robot teleportation and mission initiation
- RViz marker visualization for start and goal positions
- Mission execution via MissionManager (main loop integration)

Usage:
    from costnav_isaacsim.mission_manager import MissionManager
    from costnav_isaacsim.config import load_mission_config

    # Load configuration
    config = load_mission_config()

    # Create mission manager (runs in main simulation loop)
    manager = MissionManager(config, simulation_context)

    # Main simulation loop
    while running:
        simulation_context.step(render=True)
        manager.step()  # Step mission manager after physics step

Note:
    MarkerPublisher and MissionManager require ROS2 (rclpy).
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
    from .mission_manager import MissionManager

    __all__ = [
        "NavMeshSampler",
        "SampledPosition",
        "MarkerPublisher",
        "MissionManager",
        "ROS2_AVAILABLE",
    ]
else:
    # Provide stub classes for documentation/type hints
    MarkerPublisher = None
    MissionManager = None
    MissionManagerConfig = None

    __all__ = [
        "NavMeshSampler",
        "SampledPosition",
        "ROS2_AVAILABLE",
    ]
