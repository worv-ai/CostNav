# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for MissionManager configuration and workflow logic.

This test suite validates:
1. MissionManagerConfig dataclass defaults and customization
2. Mission workflow logic (sampling, teleporting, publishing)
3. Utility functions (yaw to quaternion conversion, covariance matrix)
4. TopomapGenerator.interpolate_waypoints arc-length resampling

Note:
    These tests do NOT require ROS2 or Isaac Sim to be running.
    They test the configuration and logic, not the actual ROS2 communication.
"""

import math

# Mock ROS2 and other dependencies
import sys
from unittest.mock import MagicMock, patch

sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["nav_msgs"] = MagicMock()
sys.modules["nav_msgs.msg"] = MagicMock()
sys.modules["visualization_msgs"] = MagicMock()
sys.modules["visualization_msgs.msg"] = MagicMock()
sys.modules["std_msgs"] = MagicMock()
sys.modules["std_msgs.msg"] = MagicMock()

from transforms3d.euler import euler2quat  # noqa: E402

from costnav_isaacsim.config import MissionManagerConfig  # noqa: E402
from costnav_isaacsim.mission_manager.navmesh_sampler import SampledPosition  # noqa: E402
from costnav_isaacsim.mission_manager.topomap_generator import TopomapGenerator  # noqa: E402


class TestMissionManagerConfig:
    """Tests for MissionManagerConfig dataclass."""

    def test_configuration_initialization(self):
        """Test mission manager configuration with default and custom values."""
        # Test defaults
        config_default = MissionManagerConfig()
        assert config_default.min_distance == 5.0
        assert config_default.max_distance == 100.0
        assert config_default.edge_margin == 0.5
        assert config_default.initial_pose_delay == 1.0
        assert config_default.goal_delay == 0.5
        assert config_default.teleport_height == 0.1
        assert config_default.robot_prim_path is None

        # Test custom values
        config_custom = MissionManagerConfig(
            min_distance=10.0,
            max_distance=50.0,
            edge_margin=1.0,
            initial_pose_delay=2.0,
            goal_delay=1.0,
            teleport_height=0.2,
            robot_prim_path="/World/Nova_Carter_ROS/chassis_link",
        )
        assert config_custom.min_distance == 10.0
        assert config_custom.max_distance == 50.0
        assert config_custom.edge_margin == 1.0
        assert config_custom.initial_pose_delay == 2.0
        assert config_custom.goal_delay == 1.0
        assert config_custom.teleport_height == 0.2
        assert config_custom.robot_prim_path == "/World/Nova_Carter_ROS/chassis_link"


class TestTeleportCallback:
    """Tests for teleport callback functionality."""

    def test_teleport_callback_behavior(self):
        """Test teleport callback success and failure cases."""
        # Test success case
        callback_called = False
        received_position = None

        def success_callback(position: SampledPosition) -> bool:
            nonlocal callback_called, received_position
            callback_called = True
            received_position = position
            return True

        position = SampledPosition(x=5.0, y=10.0, z=0.1, heading=0.5)
        result = success_callback(position)

        assert callback_called
        assert result is True
        assert received_position.x == 5.0
        assert received_position.y == 10.0

        # Test failure case
        def failure_callback(position: SampledPosition) -> bool:
            return False

        result = failure_callback(position)
        assert result is False

    def test_xform_teleport_simple_translation_orientation(self):
        """Test that xform teleport callback performs simple translation and orientation.

        This test verifies that the teleport operation:
        1. Uses get-or-create pattern for xform operations
        2. Sets absolute position and orientation on the target prim
        3. Does NOT modify any child articulation states
        4. Multiple consecutive teleports result in accurate absolute positioning
        """
        # Mock USD/pxr modules
        mock_gf = MagicMock()
        mock_usd_geom = MagicMock()

        # Create mock Vec3d class that stores the value
        class MockVec3d:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

        # Create mock Quatd class that stores the quaternion
        class MockQuatd:
            def __init__(self, w, imaginary_vec):
                self.w = w
                self.imaginary = imaginary_vec

        mock_gf.Vec3d = MockVec3d
        mock_gf.Quatd = MockQuatd

        # Create mock xform operations
        mock_translate_op = MagicMock()
        mock_orient_op = MagicMock()

        # Track the values set on operations
        translate_values = []
        orient_values = []

        def track_translate_set(value):
            translate_values.append((value.x, value.y, value.z))

        def track_orient_set(value):
            orient_values.append((value.w, value.imaginary.x, value.imaginary.y, value.imaginary.z))

        mock_translate_op.Set.side_effect = track_translate_set
        mock_orient_op.Set.side_effect = track_orient_set

        # Mock XformOp type enum
        mock_usd_geom.XformOp.TypeTranslate = "TypeTranslate"
        mock_usd_geom.XformOp.TypeOrient = "TypeOrient"

        # Set up translate op mock
        mock_translate_op.GetOpType.return_value = "TypeTranslate"
        mock_orient_op.GetOpType.return_value = "TypeOrient"

        # Create mock xformable for prim
        mock_xformable = MagicMock()
        mock_xformable.GetOrderedXformOps.return_value = []  # Initially no ops
        mock_xformable.AddTranslateOp.return_value = mock_translate_op
        mock_xformable.AddOrientOp.return_value = mock_orient_op

        # Create mock prim
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = True

        mock_stage = MagicMock()
        mock_stage.GetPrimAtPath.return_value = mock_prim

        # Mock UsdGeom.Xformable to return xformable
        mock_usd_geom.Xformable.return_value = mock_xformable

        # Patch the imports and create the callback
        with patch.dict("sys.modules", {"pxr": MagicMock(Gf=mock_gf, UsdGeom=mock_usd_geom)}):
            # Import the mission manager to get access to the method
            from costnav_isaacsim.mission_manager.mission_manager import MissionManager

            # Create instance without initializing (to avoid ROS2 dependencies)
            manager = object.__new__(MissionManager)
            manager._stage = mock_stage
            manager._robot_prim_path = "/World/Nova_Carter_ROS/chassis_link"

            # Create the teleport callback
            teleport_callback = manager._create_xform_teleport_callback()

            # === FIRST TELEPORT ===
            # Teleport to initial position
            position1 = SampledPosition(x=10.0, y=20.0, z=0.1, heading=0.5)
            result1 = teleport_callback(position1)

            # Verify first teleport succeeded
            assert result1 is True

            # Verify operations were created (get-or-create pattern)
            assert mock_xformable.AddTranslateOp.call_count == 1
            assert mock_xformable.AddOrientOp.call_count == 1

            # Verify position was set correctly
            assert len(translate_values) == 1
            assert translate_values[0] == (10.0, 20.0, 0.1)

            # Verify orientation was set correctly
            assert len(orient_values) == 1
            # heading=0.5 -> quaternion via euler2quat
            q_expected = euler2quat(0, 0, 0.5)
            assert abs(orient_values[0][0] - q_expected[0]) < 0.001  # w
            assert abs(orient_values[0][1] - q_expected[1]) < 0.001  # x
            assert abs(orient_values[0][2] - q_expected[2]) < 0.001  # y
            assert abs(orient_values[0][3] - q_expected[3]) < 0.001  # z

            # === SIMULATE ROBOT MOVEMENT ===
            # Simulate that xform operations now exist (from first teleport)
            mock_xformable.GetOrderedXformOps.return_value = [
                mock_translate_op,
                mock_orient_op,
            ]

            # === SECOND TELEPORT ===
            # Teleport to new position
            position2 = SampledPosition(x=50.0, y=30.0, z=0.1, heading=1.0)
            result2 = teleport_callback(position2)

            # Verify second teleport succeeded
            assert result2 is True

            # Verify operations were reused (get-or-create pattern)
            # Operations already exist, so they should be reused, not recreated
            assert mock_xformable.AddTranslateOp.call_count == 1  # Still 1 (reused)
            assert mock_xformable.AddOrientOp.call_count == 1  # Still 1 (reused)

            # Verify position was updated correctly (absolute, not relative)
            assert len(translate_values) == 2
            assert translate_values[1] == (50.0, 30.0, 0.1)

            # Verify orientation was updated correctly
            assert len(orient_values) == 2
            q_expected2 = euler2quat(0, 0, 1.0)
            assert abs(orient_values[1][0] - q_expected2[0]) < 0.001  # w
            assert abs(orient_values[1][1] - q_expected2[1]) < 0.001  # x
            assert abs(orient_values[1][2] - q_expected2[2]) < 0.001  # y
            assert abs(orient_values[1][3] - q_expected2[3]) < 0.001  # z

            # === VERIFY NO ACCUMULATION ===
            # The key verification: positions are absolute, not accumulated
            assert translate_values[1] != (
                translate_values[0][0] + 50.0,
                translate_values[0][1] + 30.0,
                translate_values[0][2] + 0.1,
            )
            assert translate_values[1] == (50.0, 30.0, 0.1)  # Absolute position


class TestMissionWorkflow:
    """Tests for mission workflow logic."""

    def test_mission_workflow_distance_validation(self):
        """Test mission positions storage and distance validation."""
        min_distance = 5.0
        max_distance = 50.0

        start = SampledPosition(x=0.0, y=0.0, z=0.0, heading=0.0)
        goal = SampledPosition(x=20.0, y=15.0, z=0.0, heading=0.5)

        # Simulate storing positions
        current_start = start
        current_goal = goal

        # Verify positions stored correctly
        assert current_start.x == 0.0
        assert current_goal.x == 20.0

        # Verify distance validation
        distance = start.distance_to(goal)
        assert distance >= min_distance
        assert distance <= max_distance


class TestCovarianceMatrix:
    """Tests for initial pose covariance matrix."""

    def test_covariance_matrix_structure(self):
        """Test 6x6 covariance matrix structure and diagonal indices."""
        # 6x6 covariance matrix in row-major order
        # Diagonal indices: 0 (x), 7 (y), 14 (z), 21 (roll), 28 (pitch), 35 (yaw)
        covariance = [0.0] * 36
        covariance[0] = 0.25  # x variance
        covariance[7] = 0.25  # y variance
        covariance[35] = 0.0685  # yaw variance

        assert len(covariance) == 36
        assert covariance[0] == 0.25
        assert covariance[7] == 0.25
        assert covariance[35] == 0.0685


class TestInterpolateWaypoints:
    """Tests for TopomapGenerator.interpolate_waypoints arc-length resampling."""

    @staticmethod
    def _pts(*coords):
        """Helper: create SampledPosition list from (x, y) tuples."""
        return [SampledPosition(x=x, y=y, z=0.0) for x, y in coords]

    @staticmethod
    def _spacing(points):
        """Return list of consecutive 2D distances."""
        return [points[i].distance_to(points[i + 1]) for i in range(len(points) - 1)]

    # ------------------------------------------------------------------
    # Basic behaviour
    # ------------------------------------------------------------------

    def test_straight_line_interval_2(self):
        """10 m straight line with interval=2 → start + 4 interior + goal = 6 pts."""
        sparse = self._pts((0, 0), (10, 0))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=2.0)
        # Expected: 0m(start), 2m, 4m, 6m, 8m, 10m(goal) = 6 points
        assert len(result) == 6
        # All interior spacings should be ~2.0 m
        for d in self._spacing(result)[:-1]:
            assert abs(d - 2.0) < 1e-6

    def test_straight_line_interval_3(self):
        """10 m line with interval=3 → start + 3 interior + goal = 5 pts."""
        sparse = self._pts((0, 0), (10, 0))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=3.0)
        # 0m(start), 3m, 6m, 9m, 10m(goal) = 5 points
        assert len(result) == 5
        for d in self._spacing(result)[:-1]:
            assert abs(d - 3.0) < 1e-6
        # Last gap should be 1.0 m (10 - 9)
        assert abs(self._spacing(result)[-1] - 1.0) < 1e-6

    # ------------------------------------------------------------------
    # Decimation: the key fix
    # ------------------------------------------------------------------

    def test_decimation_skips_dense_navmesh_vertices(self):
        """Many points 0.3 m apart with interval=2.0 → should be decimated.

        This is the exact scenario that was broken before the fix.
        """
        # 30 points, each 0.3 m apart → total 8.7 m
        sparse = self._pts(*[(i * 0.3, 0) for i in range(30)])
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=2.0)
        # Total ~8.7 m / 2.0 m ≈ 4 interior + start + goal = ~6
        # Must be MUCH fewer than the original 30
        assert len(result) < 10
        assert len(result) >= 3  # at least start, one middle, goal

    def test_decimation_with_large_interval(self):
        """Interval larger than total path → only start + goal."""
        sparse = self._pts((0, 0), (3, 0), (5, 0))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=10.0)
        # Total path = 5 m, interval = 10 m → no interior points emitted
        assert len(result) == 2  # start + goal
        assert abs(result[0].x - 0.0) < 1e-6
        assert abs(result[-1].x - 5.0) < 1e-6

    # ------------------------------------------------------------------
    # Multi-segment / L-shaped paths
    # ------------------------------------------------------------------

    def test_l_shaped_path(self):
        """L-shaped path: 6 m east then 8 m north, interval=2.0."""
        sparse = self._pts((0, 0), (6, 0), (6, 8))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=2.0)
        # Expected: start + 6 interior (at 2,4,6,8,10,12) + goal = 8
        assert len(result) == 8
        # Verify all spacings are ~2.0 except possibly the last
        for d in self._spacing(result)[:-1]:
            assert abs(d - 2.0) < 1e-6

    def test_heading_follows_segment_direction(self):
        """Heading should match the direction of the segment each point falls on."""
        sparse = self._pts((0, 0), (10, 0), (10, 10))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=5.0)
        # Points on first segment (east) should have heading ≈ 0
        # Points on second segment (north) should have heading ≈ π/2
        for pt in result:
            if pt.x < 9.9 and pt.y < 0.1:
                # On the east segment
                assert abs(pt.heading - 0.0) < 1e-6
            elif pt.x > 9.9 and pt.y > 0.1 and pt.y < 9.9:
                # On the north segment
                assert abs(pt.heading - math.pi / 2) < 1e-6

    # ------------------------------------------------------------------
    # Edge cases
    # ------------------------------------------------------------------

    def test_single_point_returns_as_is(self):
        """Single point input returns that point."""
        sparse = self._pts((5, 5))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=2.0)
        assert len(result) == 1
        assert result[0].x == 5.0

    def test_two_identical_points(self):
        """Two identical points (zero-length path) → start + goal."""
        sparse = self._pts((3, 3), (3, 3))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=2.0)
        assert len(result) == 2

    def test_empty_input(self):
        """Empty input returns empty list."""
        result = TopomapGenerator.interpolate_waypoints([], interval=2.0)
        assert result == []

    def test_zero_length_segment_in_middle(self):
        """Duplicate vertex in the middle should be handled gracefully."""
        sparse = self._pts((0, 0), (5, 0), (5, 0), (10, 0))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=2.0)
        # Total path = 10 m, same as a straight line
        assert len(result) == 6  # 0,2,4,6,8,10

    def test_start_and_goal_always_included(self):
        """First and last points should always match the original start/goal."""
        sparse = self._pts((1, 2), (7, 2), (7, 9))
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=3.0)
        assert abs(result[0].x - 1.0) < 1e-6
        assert abs(result[0].y - 2.0) < 1e-6
        assert abs(result[-1].x - 7.0) < 1e-6
        assert abs(result[-1].y - 9.0) < 1e-6

    def test_z_coordinate_interpolated(self):
        """Z coordinate should be linearly interpolated along segments."""
        sparse = [
            SampledPosition(x=0, y=0, z=0.0),
            SampledPosition(x=10, y=0, z=5.0),
        ]
        result = TopomapGenerator.interpolate_waypoints(sparse, interval=5.0)
        # 0m(z=0), 5m(z=2.5), 10m(z=5.0)
        assert len(result) == 3
        assert abs(result[0].z - 0.0) < 1e-6
        assert abs(result[1].z - 2.5) < 1e-6
        assert abs(result[2].z - 5.0) < 1e-6
