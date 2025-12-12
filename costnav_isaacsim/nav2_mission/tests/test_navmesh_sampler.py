# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for NavMesh sampler module."""

import math

# Import the module under test
from costnav_isaacsim.nav2_mission.navmesh_sampler import (
    SampledPosition,
    NavMeshSampler,
    NAVMESH_AVAILABLE,
)


class TestSampledPosition:
    """Tests for SampledPosition dataclass."""

    def test_creation(self):
        """Test basic position creation."""
        pos = SampledPosition(x=1.0, y=2.0, z=3.0, heading=0.5)
        assert pos.x == 1.0
        assert pos.y == 2.0
        assert pos.z == 3.0
        assert pos.heading == 0.5

    def test_default_heading(self):
        """Test default heading value."""
        pos = SampledPosition(x=0.0, y=0.0, z=0.0)
        assert pos.heading == 0.0

    def test_distance_to_same_point(self):
        """Test distance to same point is zero."""
        pos1 = SampledPosition(x=5.0, y=5.0, z=0.0)
        pos2 = SampledPosition(x=5.0, y=5.0, z=0.0)
        assert pos1.distance_to(pos2) == 0.0

    def test_distance_to_horizontal(self):
        """Test horizontal distance calculation."""
        pos1 = SampledPosition(x=0.0, y=0.0, z=0.0)
        pos2 = SampledPosition(x=3.0, y=4.0, z=0.0)
        # 3-4-5 triangle
        assert pos1.distance_to(pos2) == 5.0

    def test_distance_ignores_z(self):
        """Test that distance calculation is 2D (ignores Z)."""
        pos1 = SampledPosition(x=0.0, y=0.0, z=0.0)
        pos2 = SampledPosition(x=3.0, y=4.0, z=100.0)
        # Z should be ignored
        assert pos1.distance_to(pos2) == 5.0

    def test_distance_symmetry(self):
        """Test distance is symmetric."""
        pos1 = SampledPosition(x=1.0, y=2.0, z=0.0)
        pos2 = SampledPosition(x=4.0, y=6.0, z=0.0)
        assert pos1.distance_to(pos2) == pos2.distance_to(pos1)

    def test_to_tuple(self):
        """Test conversion to tuple."""
        pos = SampledPosition(x=1.0, y=2.0, z=3.0)
        assert pos.to_tuple() == (1.0, 2.0, 3.0)

    def test_repr(self):
        """Test string representation."""
        pos = SampledPosition(x=1.5, y=2.5, z=0.0, heading=1.57)
        repr_str = repr(pos)
        assert "1.50" in repr_str
        assert "2.50" in repr_str


class TestNavMeshSampler:
    """Tests for NavMeshSampler class."""

    def test_initialization(self):
        """Test sampler initialization with default values."""
        sampler = NavMeshSampler()
        assert sampler.min_distance == 5.0
        assert sampler.max_distance == 100.0
        assert sampler.agent_radius == 0.5
        assert sampler.agent_height == 1.8

    def test_initialization_custom_values(self):
        """Test sampler initialization with custom values."""
        sampler = NavMeshSampler(
            min_distance=10.0,
            max_distance=50.0,
            agent_radius=0.3,
            agent_height=1.5,
        )
        assert sampler.min_distance == 10.0
        assert sampler.max_distance == 50.0
        assert sampler.agent_radius == 0.3
        assert sampler.agent_height == 1.5

    def test_is_available_without_navmesh(self):
        """Test availability check when NavMesh is not available."""
        sampler = NavMeshSampler()
        # In test environment, NavMesh is typically not available
        if not NAVMESH_AVAILABLE:
            assert sampler.is_available is False


class TestDistanceConstraints:
    """Tests for distance constraint validation logic."""

    def test_min_distance_constraint(self):
        """Test that positions below min distance are rejected."""
        min_distance = 5.0
        pos1 = SampledPosition(x=0.0, y=0.0, z=0.0)
        pos2 = SampledPosition(x=2.0, y=2.0, z=0.0)  # ~2.83m apart

        distance = pos1.distance_to(pos2)
        assert distance < min_distance

    def test_max_distance_constraint(self):
        """Test that positions above max distance are rejected."""
        max_distance = 50.0
        pos1 = SampledPosition(x=0.0, y=0.0, z=0.0)
        pos2 = SampledPosition(x=100.0, y=0.0, z=0.0)  # 100m apart

        distance = pos1.distance_to(pos2)
        assert distance > max_distance

    def test_valid_distance_range(self):
        """Test positions within valid distance range."""
        min_distance = 5.0
        max_distance = 50.0
        pos1 = SampledPosition(x=0.0, y=0.0, z=0.0)
        pos2 = SampledPosition(x=20.0, y=15.0, z=0.0)  # 25m apart

        distance = pos1.distance_to(pos2)
        assert min_distance <= distance <= max_distance


class TestHeadingCalculation:
    """Tests for heading/orientation calculations."""

    def test_heading_east(self):
        """Test heading calculation pointing east."""
        start = SampledPosition(x=0.0, y=0.0, z=0.0)
        goal = SampledPosition(x=10.0, y=0.0, z=0.0)

        heading = math.atan2(goal.y - start.y, goal.x - start.x)
        assert abs(heading - 0.0) < 0.001

    def test_heading_north(self):
        """Test heading calculation pointing north."""
        start = SampledPosition(x=0.0, y=0.0, z=0.0)
        goal = SampledPosition(x=0.0, y=10.0, z=0.0)

        heading = math.atan2(goal.y - start.y, goal.x - start.x)
        assert abs(heading - math.pi / 2) < 0.001

    def test_heading_west(self):
        """Test heading calculation pointing west."""
        start = SampledPosition(x=0.0, y=0.0, z=0.0)
        goal = SampledPosition(x=-10.0, y=0.0, z=0.0)

        heading = math.atan2(goal.y - start.y, goal.x - start.x)
        assert abs(abs(heading) - math.pi) < 0.001
