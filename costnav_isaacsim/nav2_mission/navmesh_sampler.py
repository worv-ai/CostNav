# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""NavMesh-based position sampling for Nav2 navigation goals.

This module provides position sampling using the NavMesh API from Isaac Sim's
omni.anim.navigation.core extension. It ensures sampled positions are on
navigable surfaces and enforces minimum distance thresholds between start and goal.

Reference:
    - Omniverse Navigation Mesh Extension:
      https://docs.omniverse.nvidia.com/extensions/latest/ext_navigation-mesh.html
"""

from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# Try to import NavMesh extensions (only available in Isaac Sim runtime)
NAVMESH_AVAILABLE = False
try:
    import carb
    import omni.anim.navigation.core as nav

    NAVMESH_AVAILABLE = True
except ImportError:
    logger.warning("NavMesh extensions not available. Running in non-Isaac Sim environment.")


@dataclass
class SampledPosition:
    """Represents a sampled navigation position."""

    x: float
    y: float
    z: float
    heading: float = 0.0  # Orientation in radians

    def distance_to(self, other: "SampledPosition") -> float:
        """Calculate 2D Euclidean distance to another position."""
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)

    def to_tuple(self) -> Tuple[float, float, float]:
        """Return position as (x, y, z) tuple."""
        return (self.x, self.y, self.z)

    def __repr__(self) -> str:
        return f"SampledPosition(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, heading={self.heading:.2f})"


class NavMeshSampler:
    """NavMesh-based position sampler for Nav2 navigation.

    This class uses the omni.anim.navigation.core extension to sample valid
    walkable positions from the NavMesh and ensures minimum distance constraints
    between sampled positions.

    Example usage:
        sampler = NavMeshSampler(min_distance=5.0, max_distance=50.0)

        # Sample a valid start/goal pair
        start, goal = sampler.sample_start_goal_pair()

        # Sample a random position on NavMesh
        position = sampler.sample_random_position()

        # Validate a specific position
        is_valid = sampler.validate_position(position)
    """

    def __init__(
        self,
        min_distance: float = 5.0,
        max_distance: float = 100.0,
        agent_radius: float = 0.5,
        agent_height: float = 1.8,
        default_z: float = 0.0,
        max_sampling_attempts: int = 100,
    ):
        """Initialize the NavMesh sampler.

        Args:
            min_distance: Minimum 2D distance between start and goal positions (meters).
            max_distance: Maximum 2D distance between start and goal positions (meters).
            agent_radius: Navigation agent radius for NavMesh queries (meters).
            agent_height: Navigation agent height (meters).
            default_z: Default Z coordinate for sampled positions.
            max_sampling_attempts: Maximum attempts to find valid position pairs.
        """
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.agent_radius = agent_radius
        self.agent_height = agent_height
        self.default_z = default_z
        self.max_sampling_attempts = max_sampling_attempts

        # NavMesh interface (will be acquired on first use)
        self._navmesh_interface: Optional[object] = None
        self._navmesh: Optional[object] = None

        # Random ID for NavMesh queries
        self._random_id = f"CostNav_Sampler_{random.randint(1000, 9999)}"

        logger.info(
            f"NavMeshSampler initialized: min_distance={min_distance}m, "
            f"max_distance={max_distance}m, agent_radius={agent_radius}m"
        )

    @property
    def is_available(self) -> bool:
        """Check if NavMesh is available and ready for queries."""
        if not NAVMESH_AVAILABLE:
            return False
        try:
            navmesh = self._get_navmesh()
            return navmesh is not None
        except Exception:
            return False

    def _get_navmesh_interface(self):
        """Acquire NavMesh interface lazily."""
        if not NAVMESH_AVAILABLE:
            raise RuntimeError("NavMesh extensions not available.")

        if self._navmesh_interface is None:
            self._navmesh_interface = nav.acquire_interface()
            if self._navmesh_interface is None:
                raise RuntimeError("Failed to acquire NavMesh interface.")
            logger.debug("NavMesh interface acquired.")

        return self._navmesh_interface

    def _get_navmesh(self):
        """Get the baked NavMesh object."""
        interface = self._get_navmesh_interface()
        self._navmesh = interface.get_navmesh()

        if self._navmesh is None:
            # Try to bake the navmesh automatically
            logger.info("NavMesh not found. Attempting to bake automatically...")
            if self._try_bake_navmesh():
                self._navmesh = interface.get_navmesh()

            if self._navmesh is None:
                raise RuntimeError(
                    "NavMesh is not baked. Please bake the NavMesh in Isaac Sim:\n"
                    "  1. Create > Navigation > NavMesh Include Volume\n"
                    "  2. Window > Navigation > Navigation Mesh\n"
                    "  3. Click 'Bake NavMesh'"
                )
        return self._navmesh

    def _try_bake_navmesh(self, max_wait_time: float = 30.0) -> bool:
        """Try to bake the NavMesh automatically.

        Args:
            max_wait_time: Maximum time to wait for baking to complete (seconds).

        Returns:
            True if baking succeeded, False otherwise.
        """
        try:
            import time

            interface = self._get_navmesh_interface()

            logger.info("Starting NavMesh baking...")
            logger.info("NOTE: This requires NavMesh volumes to be set up in the scene.")

            # Start NavMesh baking
            interface.start_navmesh_baking()

            # Wait for NavMesh to be baked
            logger.info(f"Waiting for NavMesh to be baked (max {max_wait_time} seconds)...")
            start_time = time.time()
            check_count = 0

            while True:
                navmesh = interface.get_navmesh()
                if navmesh is not None:
                    logger.info("✓ NavMesh baked successfully!")
                    return True

                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > max_wait_time:
                    logger.error(f"NavMesh baking timed out after {max_wait_time}s")
                    logger.error("This likely means no NavMesh volumes exist in the scene.")
                    return False

                check_count += 1
                if check_count % 10 == 0:  # Print progress every second
                    logger.debug(f"Still waiting for NavMesh... ({elapsed:.1f}s elapsed)")

                time.sleep(0.1)

        except Exception as e:
            logger.error(f"Error baking NavMesh: {e}")
            return False

    def sample_random_position(self, max_retries: int = 10) -> Optional[SampledPosition]:
        """Sample a random valid position from the NavMesh.

        Args:
            max_retries: Maximum number of retry attempts for sampling.

        Returns:
            SampledPosition if successful, None if sampling fails.
        """
        try:
            navmesh = self._get_navmesh()
            interface = self._get_navmesh_interface()
            area_count = interface.get_area_count()
            area_mask = [1] * area_count  # Include all areas

            # Try multiple times with different random IDs
            for _ in range(max_retries):
                # Use a different random ID for each retry
                random_id = f"CostNav_Sample_{random.randint(1000, 99999)}"
                point = navmesh.query_random_point(random_id, area_mask)

                if point is not None:
                    # Successfully sampled a point
                    return SampledPosition(
                        x=float(point[0]),
                        y=float(point[1]),
                        z=float(point[2]) if len(point) > 2 else self.default_z,
                        heading=random.uniform(-math.pi, math.pi),
                    )

            # All retries failed - only log once instead of per retry
            logger.warning(f"NavMesh random query failed after {max_retries} retries.")
            return None

        except Exception as e:
            logger.error(f"Failed to sample random position: {e}")
            return None

    def validate_position(self, position: SampledPosition) -> bool:
        """Validate if a position is on the navigable NavMesh.

        Args:
            position: Position to validate.

        Returns:
            True if position is on NavMesh and navigable.
        """
        try:
            navmesh = self._get_navmesh()

            # Query closest point on NavMesh
            query_point = carb.Float3(position.x, position.y, position.z)
            closest_result = navmesh.query_closest_point(query_point, agent_radius=self.agent_radius)

            if closest_result is None:
                return False

            closest_point = closest_result[0]

            # Check if the closest point is within tolerance
            tolerance = 0.5  # 50cm tolerance
            dx = abs(position.x - closest_point[0])
            dy = abs(position.y - closest_point[1])

            return dx < tolerance and dy < tolerance
        except Exception as e:
            logger.error(f"Position validation failed: {e}")
            return False

    def check_path_exists(
        self,
        start: SampledPosition,
        goal: SampledPosition,
    ) -> bool:
        """Check if a valid path exists between start and goal.

        Args:
            start: Start position.
            goal: Goal position.

        Returns:
            True if a valid path exists on the NavMesh.
        """
        try:
            navmesh = self._get_navmesh()

            start_point = carb.Float3(start.x, start.y, start.z)
            end_point = carb.Float3(goal.x, goal.y, goal.z)

            path = navmesh.query_shortest_path(
                start_pos=start_point,
                end_pos=end_point,
                agent_radius=self.agent_radius,
            )

            if path is None:
                return False

            points = path.get_points()
            return points is not None and len(points) > 0
        except Exception as e:
            logger.error(f"Path check failed: {e}")
            return False

    def sample_start_goal_pair(
        self,
        fixed_start: Optional[SampledPosition] = None,
    ) -> Tuple[Optional[SampledPosition], Optional[SampledPosition]]:
        """Sample a valid start/goal pair with minimum distance constraint.

        Args:
            fixed_start: If provided, use this as the start position and only sample goal.

        Returns:
            Tuple of (start, goal) positions, or (None, None) if sampling fails.
        """
        # Track failure reasons for better diagnostics
        failure_stats = {
            "sampling_failed": 0,
            "too_close": 0,
            "too_far": 0,
            "no_path": 0,
        }

        for attempt in range(self.max_sampling_attempts):
            try:
                # Sample or use fixed start position
                if fixed_start is not None:
                    start = fixed_start
                else:
                    start = self.sample_random_position()
                    if start is None:
                        failure_stats["sampling_failed"] += 1
                        continue

                # Sample goal position
                goal = self.sample_random_position()
                if goal is None:
                    failure_stats["sampling_failed"] += 1
                    continue

                # Check distance constraints
                distance = start.distance_to(goal)
                if distance < self.min_distance:
                    failure_stats["too_close"] += 1
                    # Only log every 20 attempts to reduce spam
                    if (attempt + 1) % 20 == 0:
                        logger.debug(
                            f"Progress: {attempt + 1}/{self.max_sampling_attempts} attempts. "
                            f"Last: distance {distance:.2f}m < min {self.min_distance}m"
                        )
                    continue
                if distance > self.max_distance:
                    failure_stats["too_far"] += 1
                    if (attempt + 1) % 20 == 0:
                        logger.debug(
                            f"Progress: {attempt + 1}/{self.max_sampling_attempts} attempts. "
                            f"Last: distance {distance:.2f}m > max {self.max_distance}m"
                        )
                    continue

                # Verify path exists between start and goal
                if not self.check_path_exists(start, goal):
                    failure_stats["no_path"] += 1
                    if (attempt + 1) % 20 == 0:
                        logger.debug(
                            f"Progress: {attempt + 1}/{self.max_sampling_attempts} attempts. Last: no valid path found"
                        )
                    continue

                # Set goal heading to point from start to goal
                goal.heading = math.atan2(goal.y - start.y, goal.x - start.x)

                logger.info(
                    f"✓ Sampled valid pair (attempt {attempt + 1}): start={start}, goal={goal}, distance={distance:.2f}m"
                )
                return start, goal

            except Exception as e:
                # Only log exceptions, not every failed attempt
                logger.warning(f"Sampling attempt {attempt + 1} failed with exception: {e}")
                continue

        # Log summary of failures
        logger.error(
            f"Failed to sample valid start/goal pair after {self.max_sampling_attempts} attempts. "
            f"Failure breakdown: sampling_failed={failure_stats['sampling_failed']}, "
            f"too_close={failure_stats['too_close']}, too_far={failure_stats['too_far']}, "
            f"no_path={failure_stats['no_path']}"
        )
        return None, None

    def get_closest_navmesh_point(
        self,
        x: float,
        y: float,
        z: float = 0.0,
    ) -> Optional[SampledPosition]:
        """Get the closest point on the NavMesh to the given position.

        Args:
            x: X coordinate.
            y: Y coordinate.
            z: Z coordinate.

        Returns:
            Closest SampledPosition on NavMesh, or None if not found.
        """
        try:
            navmesh = self._get_navmesh()
            query_point = carb.Float3(x, y, z)
            result = navmesh.query_closest_point(query_point, agent_radius=self.agent_radius)

            if result is None:
                return None

            closest = result[0]
            return SampledPosition(
                x=float(closest[0]),
                y=float(closest[1]),
                z=float(closest[2]) if len(closest) > 2 else self.default_z,
            )
        except Exception as e:
            logger.error(f"Failed to get closest NavMesh point: {e}")
            return None
