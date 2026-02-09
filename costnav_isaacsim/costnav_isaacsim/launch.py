#!/isaac-sim/python.sh
"""
Isaac Sim launcher for CostNav project.
Runs Street_sidewalk.usd or street_sidewalk_segwaye1.usd based on robot selection.

Usage:
    # Basic simulation
    python launch.py

    # Use custom mission config file
    python launch.py --config /path/to/config.yaml

    # Override mission config values via CLI
    python launch.py --mission-timeout 600 --min-distance 10

    # Headless mode
    python launch.py --headless

    # Select robot preset
    python launch.py --robot segway_e1

Missions are triggered manually via /start_mission (e.g. `make start-mission`).
"""

import logging

from launcher import CostNavSimLauncher
from utils import (
    load_and_override_config,
    parse_args,
    resolve_robot_name,
    resolve_usd_path,
)

logger = logging.getLogger("costnav_launch")


def main():
    """Main entry point for CostNav Isaac Sim launcher."""
    args = parse_args()

    # Configure logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

    # Resolve robot name first (needed for robot-specific config settings)
    robot_name = resolve_robot_name(args.robot)
    usd_path = resolve_usd_path(args.usd_path, robot_name)

    # Load mission config (missions are triggered manually)
    mission_config = load_and_override_config(args, robot_name)

    logger.info("Mission manager armed (manual start via /start_mission)")
    logger.info(f"  Config: {args.config or 'default'}")
    logger.info(f"  Timeout: {mission_config.timeout}s")
    logger.info(f"  Distance: {mission_config.min_distance}m - {mission_config.max_distance}m")
    logger.info(f"  Nav2 wait: {mission_config.nav2.wait_time}s")
    logger.info(f"  Robot: {robot_name}")
    logger.info(f"  USD path: {usd_path}")

    if args.people > 0:
        logger.info(f"People spawning enabled: {args.people} people")
    else:
        logger.info("People spawning disabled")

    launcher = CostNavSimLauncher(
        usd_path=usd_path,
        robot_name=robot_name,
        headless=args.headless,
        physics_dt=args.physics_dt,
        rendering_dt=args.rendering_dt,
        mission_config=mission_config,
        num_people=args.people,
    )

    try:
        launcher.run()
    except Exception:
        logger.exception("CostNav Isaac Sim launcher crashed")
    finally:
        launcher.close()


if __name__ == "__main__":
    main()
