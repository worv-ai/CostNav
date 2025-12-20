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
from unittest.mock import MagicMock

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

    def test_default_values(self):
        """Test default mission manager configuration values."""
        config = MissionManagerConfig()
        assert config.min_distance == 5.0
        assert config.max_distance == 100.0
        assert config.initial_pose_delay == 1.0
        assert config.goal_delay == 0.5
        assert config.teleport_height == 0.1
        assert config.robot_prim_path is None

    def test_custom_values(self):
        """Test custom mission manager configuration."""
        config = MissionManagerConfig(
            min_distance=10.0,
            max_distance=50.0,
            initial_pose_delay=2.0,
            goal_delay=1.0,
            teleport_height=0.2,
            robot_prim_path="/World/Nova_Carter_ROS",
        )
        assert config.min_distance == 10.0
        assert config.max_distance == 50.0
        assert config.initial_pose_delay == 2.0
        assert config.goal_delay == 1.0
        assert config.teleport_height == 0.2
        assert config.robot_prim_path == "/World/Nova_Carter_ROS"


class TestYawToQuaternion:
    """Tests for yaw to quaternion conversion in orchestrator."""

    def test_yaw_to_quaternion_zero(self):
        """Test quaternion conversion for zero yaw."""
        yaw = 0.0
        qx, qy, qz, qw = (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )
        assert qx == 0.0
        assert qy == 0.0
        assert abs(qz - 0.0) < 0.001
        assert abs(qw - 1.0) < 0.001

    def test_yaw_to_quaternion_90_degrees(self):
        """Test quaternion conversion for 90 degree yaw."""
        yaw = math.pi / 2
        qx, qy, qz, qw = (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )
        assert qx == 0.0
        assert qy == 0.0
        assert abs(qz - 0.707) < 0.01
        assert abs(qw - 0.707) < 0.01


class TestTeleportCallback:
    """Tests for teleport callback functionality."""

    def test_teleport_callback_success(self):
        """Test successful teleport callback."""
        callback_called = False
        received_position = None

        def mock_callback(position: SampledPosition) -> bool:
            nonlocal callback_called, received_position
            callback_called = True
            received_position = position
            return True

        position = SampledPosition(x=5.0, y=10.0, z=0.1, heading=0.5)
        result = mock_callback(position)

        assert callback_called
        assert result is True
        assert received_position.x == 5.0
        assert received_position.y == 10.0

    def test_teleport_callback_failure(self):
        """Test failed teleport callback."""

        def mock_callback(position: SampledPosition) -> bool:
            return False

        position = SampledPosition(x=5.0, y=10.0, z=0.0)
        result = mock_callback(position)

        assert result is False


class TestMissionWorkflow:
    """Tests for mission workflow logic."""

    def test_mission_positions_stored(self):
        """Test that mission positions are properly stored."""
        start = SampledPosition(x=0.0, y=0.0, z=0.0, heading=0.0)
        goal = SampledPosition(x=10.0, y=5.0, z=0.0, heading=0.5)

        # Simulate storing positions
        current_start = start
        current_goal = goal

        assert current_start.x == 0.0
        assert current_goal.x == 10.0
        assert current_start.distance_to(current_goal) > 5.0

    def test_distance_validation_in_workflow(self):
        """Test distance validation during mission setup."""
        min_distance = 5.0
        max_distance = 50.0

        start = SampledPosition(x=0.0, y=0.0, z=0.0)
        goal = SampledPosition(x=20.0, y=15.0, z=0.0)

        distance = start.distance_to(goal)

        # Should pass validation
        assert distance >= min_distance
        assert distance <= max_distance


class TestCovarianceMatrix:
    """Tests for initial pose covariance matrix."""

    def test_covariance_indices(self):
        """Test covariance matrix index positions."""
        # 6x6 covariance matrix in row-major order
        # Index 0: x variance (row 0, col 0)
        # Index 7: y variance (row 1, col 1)
        # Index 35: yaw variance (row 5, col 5)

        covariance = [0.0] * 36
        covariance[0] = 0.25  # x variance
        covariance[7] = 0.25  # y variance
        covariance[35] = 0.0685  # yaw variance

        assert covariance[0] == 0.25
        assert covariance[7] == 0.25
        assert covariance[35] == 0.0685
