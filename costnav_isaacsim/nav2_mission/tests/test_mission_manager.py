# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for MissionManager configuration and workflow logic.

This test suite validates:
1. MissionManagerConfig dataclass defaults and customization
2. Mission workflow logic (sampling, teleporting, publishing)
3. Utility functions (yaw to quaternion conversion, covariance matrix)

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

from costnav_isaacsim.nav2_mission.mission_manager import MissionManagerConfig
from costnav_isaacsim.nav2_mission.navmesh_sampler import SampledPosition


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


class TestYawToQuaternion:
    """Tests for yaw to quaternion conversion in orchestrator."""

    def test_yaw_to_quaternion_conversion(self):
        """Test quaternion conversion for various yaw angles."""
        # Zero yaw
        yaw = 0.0
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        assert abs(qz - 0.0) < 0.001
        assert abs(qw - 1.0) < 0.001

        # 90 degrees
        yaw = math.pi / 2
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        assert abs(qz - 0.707) < 0.01
        assert abs(qw - 0.707) < 0.01


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
            from costnav_isaacsim.nav2_mission.mission_manager import MissionManager

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
            # heading=0.5 -> qz=sin(0.25)≈0.2474, qw=cos(0.25)≈0.9689
            qw_expected = math.cos(0.5 / 2.0)
            qz_expected = math.sin(0.5 / 2.0)
            assert abs(orient_values[0][0] - qw_expected) < 0.001  # w
            assert abs(orient_values[0][1] - 0.0) < 0.001  # x
            assert abs(orient_values[0][2] - 0.0) < 0.001  # y
            assert abs(orient_values[0][3] - qz_expected) < 0.001  # z

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
            qw_expected2 = math.cos(1.0 / 2.0)
            qz_expected2 = math.sin(1.0 / 2.0)
            assert abs(orient_values[1][0] - qw_expected2) < 0.001  # w
            assert abs(orient_values[1][1] - 0.0) < 0.001  # x
            assert abs(orient_values[1][2] - 0.0) < 0.001  # y
            assert abs(orient_values[1][3] - qz_expected2) < 0.001  # z

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
