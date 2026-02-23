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
        edge_margin: float = 0.5,
        validate_path: bool = True,
    ):
        """Initialize the NavMesh sampler.

        Args:
            min_distance: Minimum 2D distance between start and goal positions (meters).
            max_distance: Maximum 2D distance between start and goal positions (meters).
            agent_radius: Navigation agent radius for NavMesh queries (meters).
            agent_height: Navigation agent height (meters).
            default_z: Default Z coordinate for sampled positions.
            max_sampling_attempts: Maximum attempts to find valid position pairs.
            edge_margin: Minimum distance from navmesh edges for sampled positions (meters).
            validate_path: Whether to validate that a path exists between start and goal.
        """
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.agent_radius = agent_radius
        self.agent_height = agent_height
        self.default_z = default_z
        self.max_sampling_attempts = max_sampling_attempts
        self.edge_margin = edge_margin
        self.validate_path = validate_path

        # NavMesh interface (will be acquired on first use)
        self._navmesh_interface: Optional[object] = None
        self._navmesh: Optional[object] = None

        # Random ID for NavMesh queries
        self._random_id = f"CostNav_Sampler_{random.randint(1000, 9999)}"

        logger.info(
            f"NavMeshSampler initialized: min_distance={min_distance}m, "
            f"max_distance={max_distance}m, agent_radius={agent_radius}m, "
            f"edge_margin={edge_margin}m"
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

        This method samples positions that are:
        1. On valid navigable navmesh
        2. At least edge_margin distance away from navmesh boundaries

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
                    position = SampledPosition(
                        x=float(point[0]),
                        y=float(point[1]),
                        z=float(point[2]) if len(point) > 2 else self.default_z,
                        heading=random.uniform(-math.pi, math.pi),
                    )

                    # Check if position is too close to edge
                    if self._is_too_close_to_edge(position):
                        continue  # Reject this position and try again

                    return position

            # All retries failed - only log once instead of per retry
            logger.warning(f"NavMesh random query failed after {max_retries} retries.")
            return None

        except Exception as e:
            logger.error(f"Failed to sample random position: {e}")
            return None

    def _is_too_close_to_edge(self, position: SampledPosition, num_directions: int = 8) -> bool:
        """Check if a position is too close to the navmesh edge.

        This method samples points in multiple directions around the given position
        at the edge_margin distance. If any of these offset points are not on valid
        navmesh, the position is considered too close to an edge.

        Args:
            position: Position to check.
            num_directions: Number of directions to check around the position.

        Returns:
            True if position is too close to edge, False otherwise.
        """
        if self.edge_margin <= 0:
            return False  # Edge checking disabled

        try:
            navmesh = self._get_navmesh()

            # Check points in a circle around the position at edge_margin distance
            for i in range(num_directions):
                angle = 2.0 * math.pi * i / num_directions
                offset_x = position.x + self.edge_margin * math.cos(angle)
                offset_y = position.y + self.edge_margin * math.sin(angle)

                # Query if this offset point is on valid navmesh
                query_point = carb.Float3(offset_x, offset_y, position.z)
                closest_result = navmesh.query_closest_point(query_point, agent_radius=self.agent_radius)

                if closest_result is None:
                    # No valid navmesh point found nearby - we're at an edge
                    return True

                closest_point = closest_result[0]

                # Check if the closest point is far from our query point
                # If it's far, it means the offset point is not on navmesh
                tolerance = self.edge_margin * 0.5  # Allow 50% deviation
                dx = abs(offset_x - closest_point[0])
                dy = abs(offset_y - closest_point[1])
                distance = math.sqrt(dx * dx + dy * dy)

                if distance > tolerance:
                    # The offset point is not on valid navmesh - we're at an edge
                    return True

            return False  # All directions checked out - not at edge
        except Exception as e:
            logger.error(f"Edge check failed: {e}")
            return True  # Assume at edge if check fails (conservative)

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

    def get_path_first_waypoint(
        self,
        start: SampledPosition,
        goal: SampledPosition,
    ) -> Optional[SampledPosition]:
        """Return the first non-start waypoint on the NavMesh shortest path.

        Useful for aligning the robot's initial heading with the path direction.
        If the path has only start and goal (no intermediate waypoints), the
        goal itself is returned.

        Args:
            start: Start position.
            goal: Goal position.

        Returns:
            The first waypoint after start on the shortest path,
            or None if no valid path exists.
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
                return None

            points = path.get_points()
            if points is None or len(points) < 2:
                return None

            # The second point (index 1) is the first waypoint after start
            pt = points[1]
            return SampledPosition(
                x=float(pt.x),
                y=float(pt.y),
                z=float(pt.z),
            )
        except Exception as e:
            logger.error(f"Failed to get first path waypoint: {e}")
            return None

    def sample_goal_in_annulus(
        self,
        start: SampledPosition,
        max_retries: int = 10,
    ) -> Optional[SampledPosition]:
        """Sample a goal position in an annulus around the start position.

        This is more efficient than global rejection sampling because it samples
        the goal in a ring around the start at distance [min_distance, max_distance].

        Args:
            start: Start position to sample around.
            max_retries: Maximum number of retry attempts.

        Returns:
            SampledPosition if successful, None if sampling fails.
        """
        try:
            navmesh = self._get_navmesh()

            for _ in range(max_retries):
                # Sample random distance in [min_distance, max_distance]
                distance = random.uniform(self.min_distance, self.max_distance)
                # Sample random angle
                angle = random.uniform(-math.pi, math.pi)

                # Calculate candidate position in annulus
                candidate_x = start.x + distance * math.cos(angle)
                candidate_y = start.y + distance * math.sin(angle)
                candidate_z = start.z

                # Project candidate onto navmesh
                query_point = carb.Float3(candidate_x, candidate_y, candidate_z)
                closest_result = navmesh.query_closest_point(query_point, agent_radius=self.agent_radius)

                if closest_result is None:
                    continue

                closest_point = closest_result[0]
                goal = SampledPosition(
                    x=float(closest_point[0]),
                    y=float(closest_point[1]),
                    z=float(closest_point[2]) if len(closest_point) > 2 else self.default_z,
                    heading=random.uniform(-math.pi, math.pi),
                )

                # Verify the projected point still satisfies distance constraints
                actual_distance = math.sqrt((goal.x - start.x) ** 2 + (goal.y - start.y) ** 2)
                if actual_distance < self.min_distance or actual_distance > self.max_distance:
                    continue

                # Check if goal is too close to edge
                if self._is_too_close_to_edge(goal):
                    continue

                return goal

            return None

        except Exception as e:
            logger.error(f"Failed to sample goal in annulus: {e}")
            return None

    def sample_start_goal_pair(
        self,
        fixed_start: Optional[SampledPosition] = None,
        use_annulus_sampling: bool = True,
    ) -> Tuple[Optional[SampledPosition], Optional[SampledPosition]]:
        """Sample a valid start/goal pair with minimum distance constraint.

        This method samples start and goal positions that are:
        1. On valid navigable navmesh
        2. At least edge_margin distance away from navmesh boundaries
        3. Between min_distance and max_distance apart
        4. Connected by a valid path

        Args:
            fixed_start: If provided, use this as the start position and only sample goal.
            use_annulus_sampling: If True, use efficient annulus sampling for goal.
                                 If False, use global rejection sampling (slower).

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

                # Sample goal position using annulus sampling (more efficient)
                if use_annulus_sampling:
                    goal = self.sample_goal_in_annulus(start, max_retries=5)
                    if goal is None:
                        failure_stats["sampling_failed"] += 1
                        continue
                    # Distance constraints already satisfied by annulus sampling
                    distance = start.distance_to(goal)
                else:
                    # Fallback to global rejection sampling
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

                # Verify path exists between start and goal (if enabled)
                if self.validate_path:
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
