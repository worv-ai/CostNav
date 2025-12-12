#!/isaac-sim/python.sh
"""
Standalone Nav2 Mission Launcher for CostNav project.

This script runs Nav2 missions when Isaac Sim is ALREADY RUNNING.
For integrated launch (simulation + missions together), use:
    python launch.py --mission

This standalone script is useful for:
- Running additional missions on a running simulation
- Testing mission orchestration separately
- Interactive mission control

Features:
- NavMesh-based position sampling
- Robot teleportation
- AMCL initial pose publication
- Nav2 goal publication
- RViz marker visualization

Usage:
    # Exec into Isaac Sim container when simulation is already running:
    docker exec -it costnav-isaac-sim /isaac-sim/python.sh \\
        /workspace/costnav_isaacsim/launch_mission.py --mission-count 5
"""

import argparse
import logging
import sys
import time

logger = logging.getLogger("costnav_mission")


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="CostNav Nav2 Mission Launcher")
    parser.add_argument(
        "--mission-count",
        type=int,
        default=1,
        help="Number of missions to run (default: 1)",
    )
    parser.add_argument(
        "--mission-delay",
        type=float,
        default=30.0,
        help="Delay between missions in seconds (default: 30)",
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=5.0,
        help="Minimum distance between start and goal (default: 5.0m)",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=50.0,
        help="Maximum distance between start and goal (default: 50.0m)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Configure logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

    # Initialize ROS2
    try:
        import rclpy

        rclpy.init()
    except ImportError:
        logger.error("ROS2 (rclpy) not available. Make sure ROS2 bridge extension is enabled.")
        sys.exit(1)

    # Import mission components
    from nav2_mission import MissionOrchestrator, MissionConfig

    # Create mission configuration
    config = MissionConfig(
        min_distance=args.min_distance,
        max_distance=args.max_distance,
    )

    # Create orchestrator
    orchestrator = MissionOrchestrator(config=config)

    logger.info(f"Starting {args.mission_count} mission(s)...")
    logger.info(f"Distance range: {args.min_distance}m - {args.max_distance}m")

    try:
        for i in range(args.mission_count):
            logger.info(f"\n{'='*50}")
            logger.info(f"Mission {i + 1}/{args.mission_count}")
            logger.info(f"{'='*50}")

            success = orchestrator.run_mission()

            if success:
                logger.info(f"Mission {i + 1} initiated successfully")
            else:
                logger.error(f"Mission {i + 1} failed to start")

            # Wait between missions (except for last one)
            if i < args.mission_count - 1:
                logger.info(f"Waiting {args.mission_delay}s before next mission...")
                time.sleep(args.mission_delay)

        logger.info("\nAll missions completed!")

    except KeyboardInterrupt:
        logger.info("\nMission launcher interrupted by user")
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
