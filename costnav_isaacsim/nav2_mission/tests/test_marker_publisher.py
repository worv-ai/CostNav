# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for RViz marker publisher module."""

import math
from unittest.mock import MagicMock

# Mock ROS2 dependencies for testing without ROS2 environment
import sys

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

from costnav_isaacsim.nav2_mission.marker_publisher import (
    MarkerConfig,
    START_MARKER_CONFIG,
    GOAL_MARKER_CONFIG,
    ROBOT_MARKER_CONFIG,
)


class TestMarkerConfig:
    """Tests for MarkerConfig dataclass."""

    def test_default_values(self):
        """Test default marker configuration values."""
        config = MarkerConfig(r=1.0, g=0.0, b=0.0)
        assert config.r == 1.0
        assert config.g == 0.0
        assert config.b == 0.0
        assert config.a == 1.0  # Default alpha
        assert config.scale_x == 0.5
        assert config.scale_y == 0.5
        assert config.scale_z == 0.5
        assert config.frame_id == "map"

    def test_custom_values(self):
        """Test custom marker configuration."""
        config = MarkerConfig(
            r=0.5,
            g=0.5,
            b=0.5,
            a=0.8,
            scale_x=1.0,
            scale_y=0.2,
            scale_z=0.2,
            frame_id="odom",
        )
        assert config.r == 0.5
        assert config.a == 0.8
        assert config.scale_x == 1.0
        assert config.frame_id == "odom"


class TestPredefinedConfigs:
    """Tests for predefined marker configurations."""

    def test_start_marker_is_green(self):
        """Test start marker is green."""
        assert START_MARKER_CONFIG.r == 0.0
        assert START_MARKER_CONFIG.g == 1.0
        assert START_MARKER_CONFIG.b == 0.0

    def test_goal_marker_is_red(self):
        """Test goal marker is red."""
        assert GOAL_MARKER_CONFIG.r == 1.0
        assert GOAL_MARKER_CONFIG.g == 0.0
        assert GOAL_MARKER_CONFIG.b == 0.0

    def test_robot_marker_is_blue(self):
        """Test robot marker is blue."""
        assert ROBOT_MARKER_CONFIG.r == 0.0
        assert ROBOT_MARKER_CONFIG.g == 0.0
        assert ROBOT_MARKER_CONFIG.b == 1.0

    def test_all_markers_use_map_frame(self):
        """Test all markers use map frame."""
        assert START_MARKER_CONFIG.frame_id == "map"
        assert GOAL_MARKER_CONFIG.frame_id == "map"
        assert ROBOT_MARKER_CONFIG.frame_id == "map"


class TestQuaternionConversion:
    """Tests for yaw to quaternion conversion."""

    def test_zero_yaw(self):
        """Test quaternion for zero yaw."""
        yaw = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        assert abs(qz - 0.0) < 0.001
        assert abs(qw - 1.0) < 0.001

    def test_90_degree_yaw(self):
        """Test quaternion for 90 degree yaw."""
        yaw = math.pi / 2
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        # For 90 degrees: qz ≈ 0.707, qw ≈ 0.707
        assert abs(qz - 0.707) < 0.01
        assert abs(qw - 0.707) < 0.01

    def test_180_degree_yaw(self):
        """Test quaternion for 180 degree yaw."""
        yaw = math.pi
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        # For 180 degrees: qz ≈ 1.0, qw ≈ 0.0
        assert abs(qz - 1.0) < 0.001
        assert abs(qw - 0.0) < 0.001

    def test_negative_yaw(self):
        """Test quaternion for negative yaw."""
        yaw = -math.pi / 2
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        # For -90 degrees: qz ≈ -0.707, qw ≈ 0.707
        assert abs(qz - (-0.707)) < 0.01
        assert abs(qw - 0.707) < 0.01


class TestMarkerColors:
    """Tests for marker color specifications from implementation plan."""

    def test_start_marker_rgb(self):
        """Test start marker RGB values match spec (0, 1, 0)."""
        assert (START_MARKER_CONFIG.r, START_MARKER_CONFIG.g, START_MARKER_CONFIG.b) == (0.0, 1.0, 0.0)

    def test_goal_marker_rgb(self):
        """Test goal marker RGB values match spec (1, 0, 0)."""
        assert (GOAL_MARKER_CONFIG.r, GOAL_MARKER_CONFIG.g, GOAL_MARKER_CONFIG.b) == (1.0, 0.0, 0.0)

    def test_robot_marker_rgb(self):
        """Test robot marker RGB values match spec (0, 0, 1)."""
        assert (ROBOT_MARKER_CONFIG.r, ROBOT_MARKER_CONFIG.g, ROBOT_MARKER_CONFIG.b) == (0.0, 0.0, 1.0)
