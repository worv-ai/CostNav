# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Command line argument parsing for CostNav launcher."""

import argparse

from .robot_config import DEFAULT_PHYSICS_DT, DEFAULT_RENDERING_DT


def parse_args():
    """Parse command line arguments before SimulationApp creation."""
    parser = argparse.ArgumentParser(description="CostNav Isaac Sim Launcher")

    # Simulation arguments
    sim_group = parser.add_argument_group("Simulation")
    sim_group.add_argument(
        "--usd_path",
        type=str,
        default=None,
        help="Path to USD file (overrides --robot)",
    )
    sim_group.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Robot name to select a default USD (nova_carter, segway_e1)",
    )
    sim_group.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI",
    )
    sim_group.add_argument(
        "--physics_dt",
        type=float,
        default=DEFAULT_PHYSICS_DT,
        help="Physics time step (default: 1/120)",
    )
    sim_group.add_argument(
        "--rendering_dt",
        type=float,
        default=DEFAULT_RENDERING_DT,
        help="Rendering time step (default: 1/30)",
    )
    sim_group.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging",
    )

    # Mission arguments
    mission_group = parser.add_argument_group("Nav2 Mission")
    mission_group.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to mission config YAML file (default: config/mission_config.yaml)",
    )
    # CLI overrides for config values
    mission_group.add_argument(
        "--mission-timeout",
        type=float,
        default=None,
        help="Override: Mission timeout in seconds",
    )
    mission_group.add_argument(
        "--min-distance",
        type=float,
        default=None,
        help="Override: Minimum start-goal distance in meters",
    )
    mission_group.add_argument(
        "--max-distance",
        type=float,
        default=None,
        help="Override: Maximum start-goal distance in meters",
    )
    mission_group.add_argument(
        "--nav2-wait",
        type=float,
        default=None,
        help="Override: Seconds to wait for Nav2 stack",
    )

    # Food evaluation arguments
    food_group = parser.add_argument_group("Food Evaluation")
    food_group.add_argument(
        "--food-enabled",
        type=str,
        default=None,
        help="Enable food spoilage evaluation (True/False or 1/0)",
    )
    food_group.add_argument(
        "--food-prim-path",
        type=str,
        default=None,
        help="Override: Prim path where food is spawned in the stage",
    )
    food_group.add_argument(
        "--food-spoilage-threshold",
        type=float,
        default=None,
        help="Override: Fraction of pieces that can be lost (0.0 = no loss allowed)",
    )

    # People arguments
    people_group = parser.add_argument_group("People")
    people_group.add_argument(
        "--people",
        type=int,
        default=20,
        help="Number of people to spawn in the scene (default: 20)",
    )

    # Goal image arguments (for ViNT ImageGoal mode)
    goal_image_group = parser.add_argument_group("Goal Image")
    goal_image_group.add_argument(
        "--goal-image-enabled",
        type=str,
        default=None,
        help="Enable goal image publishing for ViNT ImageGoal mode (True/False or 1/0)",
    )

    return parser.parse_args()
