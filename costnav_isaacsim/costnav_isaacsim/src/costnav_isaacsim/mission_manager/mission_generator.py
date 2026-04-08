# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Deterministic mission config generator for CostNav evaluation.

Generates mission configurations across three difficulty levels (none, easy, hard)
with deterministic obstacle placement. All randomness is controlled via a seeded
``random.Random`` instance so that identical seeds produce identical mission sets.

Typical usage::

    from costnav_isaacsim.mission_manager.navmesh_sampler import NavMeshSampler
    from costnav_isaacsim.mission_manager.mission_generator import MissionGenerator, save_missions

    sampler = NavMeshSampler(min_distance=5.0, max_distance=50.0)
    generator = MissionGenerator(sampler, seed=42)
    missions = generator.generate(num_none=50, num_easy=25, num_hard=25)
    save_missions(missions, "missions.json", map_name="Street_sidewalk", seed=42)
"""

from __future__ import annotations

import json
import logging
import math
import random
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from costnav_isaacsim.mission_manager.navmesh_sampler import NavMeshSampler, SampledPosition

logger = logging.getLogger(__name__)

# Obstacle type pools per difficulty
_EASY_OBSTACLE_TYPES = ["common", "minor"]
_HARD_OBSTACLE_TYPES = ["common", "minor", "major"]

# Minimum distance (metres) between any two obstacle centres
_MIN_OBSTACLE_SPACING = 2.0


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class ObstacleConfig:
    """Configuration for a single obstacle placed in the scene."""

    usd_path: str  # Relative path to USD asset (e.g. "common/trash_can.usd")
    type: str  # "common", "minor", or "major"
    x: float
    y: float
    z: float
    rotation: float  # Degrees around vertical axis


@dataclass
class MissionConfig:
    """Complete configuration for a single evaluation mission."""

    id: int
    difficulty: str  # "none", "easy", or "hard"
    start: dict  # {"x", "y", "z", "heading"}
    goal: dict  # {"x", "y", "z"}
    waypoints: list  # [[x, y], ...] from NavMesh shortest path
    obstacles: list = field(default_factory=list)  # list[ObstacleConfig]


# ---------------------------------------------------------------------------
# MissionGenerator
# ---------------------------------------------------------------------------


class MissionGenerator:
    """Generate deterministic mission configs with obstacle placement.

    Args:
        sampler: A ``NavMeshSampler`` used to sample start/goal pairs and query
            NavMesh shortest paths.
        seed: Random seed for reproducible generation.
    """

    def __init__(self, sampler: NavMeshSampler, seed: int = 42) -> None:
        self._sampler = sampler
        self._rng = random.Random(seed)

        # Cache for obstacle asset lists keyed by type prefix
        self._obstacle_cache: dict[str, list[str]] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate(
        self,
        num_none: int = 50,
        num_easy: int = 25,
        num_hard: int = 25,
    ) -> list[MissionConfig]:
        """Generate all missions across difficulty levels.

        Args:
            num_none: Number of missions with no obstacles.
            num_easy: Number of easy missions (obstacles near path, avoidable).
            num_hard: Number of hard missions (obstacles on path, forces detour).

        Returns:
            Ordered list of ``MissionConfig`` instances.
        """
        missions: list[MissionConfig] = []
        mission_id = 0

        # --- none difficulty ---
        logger.info("Generating %d 'none' missions...", num_none)
        for _ in range(num_none):
            config = self._generate_single(mission_id, "none")
            if config is not None:
                missions.append(config)
                mission_id += 1

        # --- easy difficulty ---
        logger.info("Generating %d 'easy' missions...", num_easy)
        for _ in range(num_easy):
            config = self._generate_single(mission_id, "easy")
            if config is not None:
                missions.append(config)
                mission_id += 1

        # --- hard difficulty ---
        logger.info("Generating %d 'hard' missions...", num_hard)
        for _ in range(num_hard):
            config = self._generate_single(mission_id, "hard")
            if config is not None:
                missions.append(config)
                mission_id += 1

        logger.info(
            "Generated %d missions total (none=%d, easy=%d, hard=%d).",
            len(missions),
            sum(1 for m in missions if m.difficulty == "none"),
            sum(1 for m in missions if m.difficulty == "easy"),
            sum(1 for m in missions if m.difficulty == "hard"),
        )
        return missions

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _generate_single(self, mission_id: int, difficulty: str) -> MissionConfig | None:
        """Generate a single mission config.

        Returns None if start/goal sampling or path query fails.
        """
        start, goal = self._sampler.sample_start_goal_pair()
        if start is None or goal is None:
            logger.warning("Failed to sample start/goal pair for mission %d.", mission_id)
            return None

        waypoints = self._get_waypoints(start, goal)
        if waypoints is None:
            logger.warning("Failed to query waypoints for mission %d.", mission_id)
            return None

        # Compute initial heading toward the first waypoint (or goal)
        heading = self._compute_heading(start, waypoints)

        # Place obstacles based on difficulty
        obstacles: list[ObstacleConfig] = []
        if difficulty == "easy":
            obstacles = self._place_easy_obstacles(waypoints)
        elif difficulty == "hard":
            obstacles = self._place_hard_obstacles(waypoints)

        return MissionConfig(
            id=mission_id,
            difficulty=difficulty,
            start={"x": start.x, "y": start.y, "z": start.z, "heading": heading},
            goal={"x": goal.x, "y": goal.y, "z": goal.z},
            waypoints=[[wp[0], wp[1]] for wp in waypoints],
            obstacles=obstacles,
        )

    def _get_waypoints(
        self,
        start: SampledPosition,
        goal: SampledPosition,
    ) -> list[list[float]] | None:
        """Query the NavMesh shortest path and return a list of [x, y] waypoints."""
        path_positions = self._sampler.get_shortest_path(start, goal)
        if path_positions is None or len(path_positions) < 2:
            return None
        return [[pos.x, pos.y] for pos in path_positions]

    @staticmethod
    def _compute_heading(start: SampledPosition, waypoints: list[list[float]]) -> float:
        """Compute initial heading (radians) from start toward the first waypoint."""
        if len(waypoints) >= 2:
            # First waypoint after start is at index 1
            dx = waypoints[1][0] - start.x
            dy = waypoints[1][1] - start.y
        else:
            dx = waypoints[0][0] - start.x
            dy = waypoints[0][1] - start.y
        return math.atan2(dy, dx)

    # ------------------------------------------------------------------
    # Obstacle placement
    # ------------------------------------------------------------------

    def _place_easy_obstacles(self, waypoints: list[list[float]]) -> list[ObstacleConfig]:
        """Place 2-3 obstacles near the path but not blocking it.

        Obstacles are spread evenly along the path by distance, then offset
        perpendicular so the robot can still pass without a detour.
        """
        num_obstacles = self._rng.randint(2, 3)
        obstacles: list[ObstacleConfig] = []

        chosen_indices = self._spread_indices_along_path(waypoints, num_obstacles)
        if not chosen_indices:
            return obstacles

        for idx in chosen_indices:
            wp = waypoints[idx]
            perp_x, perp_y = self._perpendicular_at(waypoints, idx)

            offset_dist = self._rng.uniform(2.0, 4.0)
            side = self._rng.choice([-1, 1])

            obs_x = wp[0] + perp_x * offset_dist * side
            obs_y = wp[1] + perp_y * offset_dist * side

            if self._too_close(obs_x, obs_y, obstacles):
                continue

            rotation = self._rng.uniform(0.0, 360.0)
            usd_path, obs_type = self._get_random_obstacle_usd(_EASY_OBSTACLE_TYPES)
            if usd_path is None:
                continue

            obstacles.append(
                ObstacleConfig(
                    usd_path=usd_path,
                    type=obs_type,
                    x=obs_x,
                    y=obs_y,
                    z=0.0,
                    rotation=rotation,
                )
            )

        return obstacles

    def _place_hard_obstacles(self, waypoints: list[list[float]]) -> list[ObstacleConfig]:
        """Place obstacles directly on the path to force a detour.

        1-2 blocking obstacles are spread across the middle third of the path
        (by distance, not index), and 1-2 extra obstacles are offset nearby.
        """
        obstacles: list[ObstacleConfig] = []

        if len(waypoints) <= 2:
            return obstacles

        # Pick 1-2 blocking points spread across the middle third of the path
        num_blocking = self._rng.randint(1, 2)
        blocking_indices = self._spread_indices_along_path(
            waypoints,
            num_blocking,
            start_frac=0.3,
            end_frac=0.7,
        )

        # Place obstacles directly on the path
        for idx in blocking_indices:
            wp = waypoints[idx]

            if self._too_close(wp[0], wp[1], obstacles):
                continue

            rotation = self._rng.uniform(0.0, 360.0)
            usd_path, obs_type = self._get_random_obstacle_usd(_HARD_OBSTACLE_TYPES)
            if usd_path is None:
                continue
            obstacles.append(
                ObstacleConfig(
                    usd_path=usd_path,
                    type=obs_type,
                    x=wp[0],
                    y=wp[1],
                    z=0.0,
                    rotation=rotation,
                )
            )

        # Place 1-2 additional obstacles offset from the blocking points
        num_extra = self._rng.randint(1, 2)
        for _ in range(num_extra):
            if not blocking_indices:
                break
            idx = self._rng.choice(blocking_indices)
            wp = waypoints[idx]
            perp_x, perp_y = self._perpendicular_at(waypoints, idx)

            offset_dist = self._rng.uniform(2.0, 3.5)
            side = self._rng.choice([-1, 1])
            obs_x = wp[0] + perp_x * offset_dist * side
            obs_y = wp[1] + perp_y * offset_dist * side

            if self._too_close(obs_x, obs_y, obstacles):
                continue

            rotation = self._rng.uniform(0.0, 360.0)
            usd_path, obs_type = self._get_random_obstacle_usd(_HARD_OBSTACLE_TYPES)
            if usd_path is None:
                continue
            obstacles.append(
                ObstacleConfig(
                    usd_path=usd_path,
                    type=obs_type,
                    x=obs_x,
                    y=obs_y,
                    z=0.0,
                    rotation=rotation,
                )
            )

        return obstacles

    def _spread_indices_along_path(
        self,
        waypoints: list[list[float]],
        count: int,
        start_frac: float = 0.1,
        end_frac: float = 0.9,
    ) -> list[int]:
        """Select *count* waypoint indices evenly spread by cumulative path distance.

        Only considers the portion of the path between *start_frac* and
        *end_frac* (0-1) of the total length, skipping the very start and end.
        A small random jitter is added so placements aren't perfectly uniform.
        """
        if len(waypoints) <= 2 or count <= 0:
            return []

        # Build cumulative distance along the path
        cum_dist = [0.0]
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i - 1][0]
            dy = waypoints[i][1] - waypoints[i - 1][1]
            cum_dist.append(cum_dist[-1] + math.hypot(dx, dy))

        total_dist = cum_dist[-1]
        if total_dist < 1e-3:
            return []

        lo = total_dist * start_frac
        hi = total_dist * end_frac
        span = hi - lo
        if span < _MIN_OBSTACLE_SPACING * count:
            return []

        # Target distances evenly spaced within the range, with jitter
        step = span / (count + 1)
        targets = []
        for k in range(1, count + 1):
            jitter = self._rng.uniform(-step * 0.25, step * 0.25)
            targets.append(lo + step * k + jitter)

        # Map each target distance to the nearest waypoint index
        indices = []
        wi = 0
        for target in sorted(targets):
            while wi < len(cum_dist) - 1 and cum_dist[wi + 1] < target:
                wi += 1
            idx = max(1, min(wi, len(waypoints) - 2))
            if idx not in indices:
                indices.append(idx)

        return indices

    @staticmethod
    def _too_close(
        x: float, y: float, obstacles: list[ObstacleConfig], min_dist: float = _MIN_OBSTACLE_SPACING
    ) -> bool:
        """Return True if (x, y) is within *min_dist* of any existing obstacle."""
        for obs in obstacles:
            if math.hypot(x - obs.x, y - obs.y) < min_dist:
                return True
        return False

    @staticmethod
    def _perpendicular_at(waypoints: list[list[float]], idx: int) -> tuple[float, float]:
        """Compute a unit perpendicular vector to the path at the given index.

        Uses the segment from ``waypoints[idx-1]`` to ``waypoints[idx+1]``
        (or the nearest available neighbors) to approximate the path tangent,
        then returns the left-hand perpendicular.
        """
        prev_idx = max(0, idx - 1)
        next_idx = min(len(waypoints) - 1, idx + 1)

        dx = waypoints[next_idx][0] - waypoints[prev_idx][0]
        dy = waypoints[next_idx][1] - waypoints[prev_idx][1]
        length = math.sqrt(dx * dx + dy * dy)

        if length < 1e-6:
            return (1.0, 0.0)

        # Perpendicular: rotate tangent 90 degrees
        return (-dy / length, dx / length)

    # ------------------------------------------------------------------
    # Obstacle asset selection
    # ------------------------------------------------------------------

    def _get_random_obstacle_usd(self, types: list[str]) -> tuple[str | None, str | None]:
        """Pick a random obstacle USD from the given obstacle type categories.

        Uses ``get_obstacle_list()`` from the obstacle extension and caches
        results per type prefix.

        Args:
            types: List of obstacle type prefixes (e.g. ``["common", "minor"]``).

        Returns:
            Tuple of ``(relative_usd_path, type_prefix)`` or ``(None, None)``
            if no assets are available.
        """
        # Build a combined pool of (type, absolute_path)
        pool: list[tuple[str, str]] = []
        for type_prefix in types:
            assets = self._get_obstacle_assets(type_prefix)
            pool.extend((type_prefix, path) for path in assets)

        if not pool:
            logger.warning("No obstacle assets found for types: %s", types)
            return None, None

        obs_type, abs_path = self._rng.choice(pool)
        rel_path = self._to_relative_path(abs_path)
        return rel_path, obs_type

    def _get_obstacle_assets(self, type_prefix: str) -> list[str]:
        """Retrieve (and cache) obstacle asset paths for a type prefix."""
        if type_prefix not in self._obstacle_cache:
            try:
                from omni.isaac.obstacle.impl.utils import get_obstacle_list

                self._obstacle_cache[type_prefix] = get_obstacle_list(prefix=type_prefix)
            except ImportError:
                logger.warning("omni.isaac.obstacle not available; obstacle asset listing disabled.")
                self._obstacle_cache[type_prefix] = []
        return self._obstacle_cache[type_prefix]

    @staticmethod
    def _to_relative_path(absolute_path: str) -> str:
        """Convert an absolute obstacle path to a relative path under the Object root.

        The obstacle extension stores paths like
        ``<server>/Users/worv/Object/common/trash_can.usd``.
        This strips everything up to and including ``Object/``.
        """
        marker = "Object/"
        idx = absolute_path.find(marker)
        if idx != -1:
            return absolute_path[idx + len(marker) :]
        # Fallback: return the filename only
        return Path(absolute_path).name


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------


def _mission_to_dict(mission: MissionConfig) -> dict:
    """Convert a MissionConfig to a JSON-serializable dict."""
    d = asdict(mission)
    # obstacles are already dicts after asdict()
    return d


def save_missions(
    missions: list[MissionConfig],
    path: str | Path,
    *,
    map_name: str = "Street_sidewalk",
    seed: int = 42,
) -> None:
    """Save missions to a JSON file.

    Args:
        missions: List of mission configs to save.
        path: Output file path.
        map_name: Name of the simulation map.
        seed: Random seed used during generation.
    """
    payload = {
        "map": map_name,
        "seed": seed,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "num_missions": len(missions),
        "missions": [_mission_to_dict(m) for m in missions],
    }

    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)

    logger.info("Saved %d missions to %s", len(missions), out_path)


def load_missions(path: str | Path) -> list[MissionConfig]:
    """Load missions from a JSON file.

    Args:
        path: Path to JSON file previously created by ``save_missions``.

    Returns:
        List of ``MissionConfig`` instances.
    """
    with open(path) as f:
        payload = json.load(f)

    missions: list[MissionConfig] = []
    for raw in payload["missions"]:
        obstacles = [ObstacleConfig(**obs) for obs in raw.get("obstacles", [])]
        missions.append(
            MissionConfig(
                id=raw["id"],
                difficulty=raw["difficulty"],
                start=raw["start"],
                goal=raw["goal"],
                waypoints=raw["waypoints"],
                obstacles=obstacles,
            )
        )

    logger.info(
        "Loaded %d missions from %s (map=%s, seed=%s)", len(missions), path, payload.get("map"), payload.get("seed")
    )
    return missions
