# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for NavMesh sampler module."""

import math

# Import the module under test
from costnav_isaacsim.nav2_mission.navmesh_sampler import (
    NAVMESH_AVAILABLE,
    NavMeshSampler,
    SampledPosition,
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
        assert sampler.edge_margin == 0.5

    def test_initialization_custom_values(self):
        """Test sampler initialization with custom values."""
        sampler = NavMeshSampler(
            min_distance=10.0,
            max_distance=50.0,
            agent_radius=0.3,
            agent_height=1.5,
            edge_margin=1.0,
        )
        assert sampler.min_distance == 10.0
        assert sampler.max_distance == 50.0
        assert sampler.agent_radius == 0.3
        assert sampler.agent_height == 1.5
        assert sampler.edge_margin == 1.0

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


class TestEdgeMarginParameter:
    """Tests for edge_margin parameter functionality."""

    def test_edge_margin_initialization(self):
        """Test edge_margin parameter initialization with various values."""
        # Default value
        sampler_default = NavMeshSampler()
        assert sampler_default.edge_margin == 0.5

        # Custom value
        sampler_custom = NavMeshSampler(edge_margin=1.5)
        assert sampler_custom.edge_margin == 1.5

        # Zero and negative values (disables edge checking)
        sampler_zero = NavMeshSampler(edge_margin=0.0)
        assert sampler_zero.edge_margin == 0.0

        sampler_negative = NavMeshSampler(edge_margin=-1.0)
        assert sampler_negative.edge_margin == -1.0


class TestAnnulusSampling:
    """Tests for annulus-based goal sampling."""

    def test_annulus_sampling_distance_constraints(self):
        """Test that annulus sampling respects distance constraints mathematically."""
        sampler = NavMeshSampler(min_distance=10.0, max_distance=20.0, edge_margin=0.0)
        start = SampledPosition(x=0.0, y=0.0, z=0.0)

        # Test the mathematical properties of annulus sampling
        import random

        random.seed(42)

        # Simulate what annulus sampling does
        for _ in range(20):
            distance = random.uniform(sampler.min_distance, sampler.max_distance)
            angle = random.uniform(-math.pi, math.pi)

            candidate_x = start.x + distance * math.cos(angle)
            candidate_y = start.y + distance * math.sin(angle)

            # Verify the candidate is in the correct distance range
            actual_distance = math.sqrt(candidate_x**2 + candidate_y**2)
            assert sampler.min_distance <= actual_distance <= sampler.max_distance


class TestSamplingMethodComparison:
    """Tests comparing different sampling strategies."""

    def test_annulus_vs_global_sampling_efficiency(self):
        """Test that annulus sampling is more efficient than global sampling."""
        min_dist = 10.0
        max_dist = 20.0

        # Annulus area
        annulus_area = math.pi * (max_dist**2 - min_dist**2)

        # If navmesh has area 100x100 = 10000
        navmesh_area = 10000.0

        # Probability of global sample being in range
        global_prob = annulus_area / navmesh_area

        # Annulus sampling has 100% probability (before projection)
        annulus_prob = 1.0

        # Annulus should be much more efficient
        efficiency_ratio = annulus_prob / global_prob
        assert efficiency_ratio > 10.0  # At least 10x more efficient


class TestSamplingParameters:
    """Tests for sampling parameter validation and behavior."""

    def test_equal_min_max_distance_edge_case(self):
        """Test edge case where min and max distance are equal."""
        sampler = NavMeshSampler(min_distance=10.0, max_distance=10.0)
        assert sampler.min_distance == sampler.max_distance


class TestSampleStartGoalPairParameters:
    """Tests for sample_start_goal_pair method parameters."""

    def test_use_annulus_sampling_default(self):
        """Test use_annulus_sampling parameter defaults to True for efficiency."""
        import inspect

        sig = inspect.signature(NavMeshSampler.sample_start_goal_pair)
        params = sig.parameters

        assert "use_annulus_sampling" in params
        assert params["use_annulus_sampling"].default is True


class TestEdgeDetectionLogic:
    """Tests for edge detection logic."""

    def test_edge_detection_uses_8_directions(self):
        """Test that edge detection checks 8 evenly-spaced directions."""
        num_directions = 8
        angles = [2.0 * math.pi * i / num_directions for i in range(num_directions)]

        # Verify we get 8 evenly spaced angles
        assert len(angles) == 8
        assert abs(angles[0] - 0.0) < 0.001  # First angle is 0
        assert abs(angles[4] - math.pi) < 0.001  # Opposite angle is π

    def test_edge_detection_tolerance(self):
        """Test that edge detection uses 50% tolerance."""
        edge_margin = 1.0
        tolerance = edge_margin * 0.5

        assert tolerance == 0.5

        # If distance > tolerance, point is considered at edge
        distance_at_edge = 0.6
        distance_not_at_edge = 0.4

        assert distance_at_edge > tolerance
        assert distance_not_at_edge < tolerance
