# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""CLI configuration loading and override utilities."""

from typing import TYPE_CHECKING

from .robot_config import DEFAULT_CAMERA_USD_PATHS, DEFAULT_GOAL_CAMERA_HEIGHTS

if TYPE_CHECKING:
    from costnav_isaacsim.config import MissionConfig


def load_and_override_config(args, robot_name: str) -> "MissionConfig":
    """Load mission config from file and apply CLI overrides.

    Args:
        args: Parsed command line arguments.
        robot_name: Resolved robot name for robot-specific settings.

    Returns:
        MissionConfig instance with loaded settings.
    """
    from costnav_isaacsim.config import load_mission_config

    # Load from config file (or default)
    config = load_mission_config(args.config)

    # Apply CLI overrides (only if explicitly provided)
    if args.mission_timeout is not None:
        config.timeout = args.mission_timeout
    if args.min_distance is not None:
        config.min_distance = args.min_distance
    if args.max_distance is not None:
        config.max_distance = args.max_distance
    if args.nav2_wait is not None:
        config.nav2.wait_time = args.nav2_wait

    # Food evaluation overrides
    # CLI flag takes priority, then config default
    if args.food_enabled is not None:
        config.food.enabled = args.food_enabled.lower() in ("true", "1")
    if args.food_prim_path is not None:
        config.food.prim_path = args.food_prim_path
    if args.food_spoilage_threshold is not None:
        config.food.spoilage_threshold = args.food_spoilage_threshold

    # Goal image overrides (for ViNT ImageGoal mode)
    if args.goal_image_enabled is not None:
        config.goal_image.enabled = args.goal_image_enabled.lower() in ("true", "1")

    # Topomap overrides (NavMesh-based topological map generation)
    if args.topomap_enabled is not None:
        config.topomap.enabled = args.topomap_enabled.lower() in ("true", "1")

    # Set robot-specific camera heights
    if robot_name in DEFAULT_GOAL_CAMERA_HEIGHTS:
        config.goal_image.camera_height_offset = DEFAULT_GOAL_CAMERA_HEIGHTS[robot_name]
        config.topomap.camera_height_offset = DEFAULT_GOAL_CAMERA_HEIGHTS[robot_name]

    # Set robot-specific camera USD paths (for topomap and goal_image cameras)
    if robot_name in DEFAULT_CAMERA_USD_PATHS:
        camera_usd = DEFAULT_CAMERA_USD_PATHS[robot_name]
        if config.topomap.camera_usd_path is None:
            config.topomap.camera_usd_path = camera_usd
        if config.goal_image.camera_usd_path is None:
            config.goal_image.camera_usd_path = camera_usd

    return config
