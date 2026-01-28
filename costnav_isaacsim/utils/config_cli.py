# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""CLI configuration loading and override utilities."""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config import MissionConfig


def load_and_override_config(args) -> "MissionConfig":
    """Load mission config from file and apply CLI overrides.

    Args:
        args: Parsed command line arguments.

    Returns:
        MissionConfig instance with loaded settings.
    """
    from config import load_mission_config

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

    return config
