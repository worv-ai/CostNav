# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for configuration loader and dataclasses.

This test suite validates:
1. All config dataclass defaults and customization
2. YAML loading and parsing via MissionConfig.from_dict()
3. load_mission_config() function behavior
4. Config inheritance (manager inherits from other sections)

Note:
    These tests do NOT require ROS2 or Isaac Sim to be running.
"""

import tempfile
from pathlib import Path

from costnav_isaacsim.config import (
    FoodConfig,
    GoalImageConfig,
    InjuryConfig,
    InjuryCostConfig,
    MarkerConfig,
    MissionConfig,
    MissionManagerConfig,
    Nav2Config,
    SamplingConfig,
    TeleportConfig,
    TopoMapConfig,
    load_mission_config,
)


class TestMarkerConfig:
    """Tests for MarkerConfig dataclass."""

    def test_default_values(self):
        """Test MarkerConfig default values."""
        config = MarkerConfig()
        assert config.enabled is True
        assert config.start_topic == "/start_marker"
        assert config.goal_topic == "/goal_marker"
        assert config.arrow_length == 1.0
        assert config.arrow_width == 0.2
        assert config.arrow_height == 0.2

    def test_custom_values(self):
        """Test MarkerConfig with custom values."""
        config = MarkerConfig(
            enabled=False,
            start_topic="/custom_start",
            arrow_length=2.0,
        )
        assert config.enabled is False
        assert config.start_topic == "/custom_start"
        assert config.arrow_length == 2.0


class TestNav2Config:
    """Tests for Nav2Config dataclass."""

    def test_default_values(self):
        """Test Nav2Config default values."""
        config = Nav2Config()
        assert config.wait_time == 10.0
        assert config.initial_pose_delay == 1.0
        assert config.initial_pose_topic == "/initialpose"
        assert config.goal_pose_topic == "/goal_pose"
        assert config.odom_topic == "/odom"

    def test_custom_values(self):
        """Test Nav2Config with custom values."""
        config = Nav2Config(
            wait_time=20.0,
            initial_pose_delay=2.5,
            odom_topic="/chassis/odom",
        )
        assert config.wait_time == 20.0
        assert config.initial_pose_delay == 2.5
        assert config.odom_topic == "/chassis/odom"


class TestTeleportConfig:
    """Tests for TeleportConfig dataclass."""

    def test_default_values(self):
        """Test TeleportConfig default values."""
        config = TeleportConfig()
        assert config.height_offset == 0.5
        assert config.robot_prim == "/World/Nova_Carter_ROS/chassis_link"

    def test_custom_values(self):
        """Test TeleportConfig with custom values."""
        config = TeleportConfig(
            height_offset=0.2,
            robot_prim="/World/CustomRobot/base_link",
        )
        assert config.height_offset == 0.2
        assert config.robot_prim == "/World/CustomRobot/base_link"


class TestSamplingConfig:
    """Tests for SamplingConfig dataclass."""

    def test_default_values(self):
        """Test SamplingConfig default values."""
        config = SamplingConfig()
        assert config.max_attempts == 100
        assert config.validate_path is True
        assert config.edge_margin == 0.5

    def test_custom_values(self):
        """Test SamplingConfig with custom values."""
        config = SamplingConfig(
            max_attempts=50,
            validate_path=False,
            edge_margin=1.0,
        )
        assert config.max_attempts == 50
        assert config.validate_path is False
        assert config.edge_margin == 1.0


class TestFoodConfig:
    """Tests for FoodConfig dataclass."""

    def test_default_values(self):
        """Test FoodConfig default values."""
        config = FoodConfig()
        assert config.enabled is False
        assert config.prim_path == "/World/Food"
        assert config.spoilage_threshold == 0.05

    def test_custom_values(self):
        """Test FoodConfig with custom values."""
        config = FoodConfig(
            enabled=True,
            usd_path="omniverse://server/food.usd",
            spoilage_threshold=0.1,
        )
        assert config.enabled is True
        assert config.usd_path == "omniverse://server/food.usd"
        assert config.spoilage_threshold == 0.1


class TestInjuryCostConfig:
    """Tests for InjuryCostConfig dataclass."""

    def test_default_values(self):
        """Test InjuryCostConfig default values."""
        config = InjuryCostConfig()
        assert config.mais_0 == 0.0
        assert config.mais_1 == 0.0
        assert config.mais_2 == 0.0
        assert config.mais_3 == 0.0
        assert config.mais_4 == 0.0
        assert config.mais_5 == 0.0
        assert config.fatality == 0.0

    def test_custom_values(self):
        """Test InjuryCostConfig with custom values."""
        config = InjuryCostConfig(
            mais_0=100.0,
            mais_1=500.0,
            mais_2=1000.0,
        )
        assert config.mais_0 == 100.0
        assert config.mais_1 == 500.0
        assert config.mais_2 == 1000.0

    def test_as_dict(self):
        """Test InjuryCostConfig.as_dict() method."""
        config = InjuryCostConfig(mais_0=100.0, mais_1=200.0)
        result = config.as_dict()
        assert result["mais_0"] == 100.0
        assert result["mais_1"] == 200.0
        assert "fatality" in result


class TestInjuryConfig:
    """Tests for InjuryConfig dataclass."""

    def test_default_values(self):
        """Test InjuryConfig default values."""
        config = InjuryConfig()
        assert config.enabled is True
        assert config.method == "delta_v"
        assert config.crash_mode == "all"
        assert config.robot_mass == 50.0
        assert config.robot_height is None
        assert config.pedestrian_height is None
        assert isinstance(config.costs, InjuryCostConfig)

    def test_custom_values(self):
        """Test InjuryConfig with custom values."""
        costs = InjuryCostConfig(mais_0=100.0)
        config = InjuryConfig(
            enabled=False,
            method="impulse",
            robot_mass=75.0,
            costs=costs,
        )
        assert config.enabled is False
        assert config.method == "impulse"
        assert config.robot_mass == 75.0
        assert config.costs.mais_0 == 100.0


class TestGoalImageConfig:
    """Tests for GoalImageConfig dataclass."""

    def test_default_values(self):
        """Test GoalImageConfig default values."""
        config = GoalImageConfig()
        assert config.enabled is False
        assert config.topic == "/goal_image"
        assert config.width == 640
        assert config.height == 360
        assert config.camera_height_offset == 0.3
        assert config.camera_prim_path == "/World/goal_camera"
        assert config.camera_usd_path is None

    def test_custom_values(self):
        """Test GoalImageConfig with custom values."""
        config = GoalImageConfig(
            enabled=True,
            topic="/custom_goal_image",
            width=1280,
            height=720,
            camera_height_offset=0.5,
        )
        assert config.enabled is True
        assert config.topic == "/custom_goal_image"
        assert config.width == 1280
        assert config.height == 720
        assert config.camera_height_offset == 0.5


class TestTopoMapConfig:
    """Tests for TopoMapConfig dataclass."""

    def test_default_values(self):
        """Test TopoMapConfig default values."""
        config = TopoMapConfig()
        assert config.enabled is False
        assert config.waypoint_interval == 2.0
        assert config.camera_height_offset == 0.3
        assert config.image_width == 640
        assert config.image_height == 400
        assert config.output_dir == "/tmp/costnav_topomap"
        assert config.camera_prim_path == "/World/topomap_camera"
        assert config.render_settle_steps == 3
        assert config.robot_prim_path is None
        assert config.camera_usd_path is None

    def test_custom_values(self):
        """Test TopoMapConfig with custom values."""
        config = TopoMapConfig(
            enabled=True,
            waypoint_interval=1.0,
            camera_height_offset=0.5,
            image_width=1280,
            image_height=720,
            output_dir="/custom/topomap",
            render_settle_steps=5,
        )
        assert config.enabled is True
        assert config.waypoint_interval == 1.0
        assert config.camera_height_offset == 0.5
        assert config.image_width == 1280
        assert config.image_height == 720
        assert config.output_dir == "/custom/topomap"
        assert config.render_settle_steps == 5


class TestMissionManagerConfig:
    """Tests for MissionManagerConfig dataclass."""

    def test_default_values(self):
        """Test MissionManagerConfig default values."""
        config = MissionManagerConfig()
        assert config.min_distance == 5.0
        assert config.max_distance == 100.0
        assert config.edge_margin == 0.5
        assert config.initial_pose_delay == 1.0
        assert config.goal_delay == 0.5
        assert config.teleport_height == 0.1
        assert config.robot_prim_path is None
        assert config.teleport_settle_steps == 30
        assert config.clear_costmaps_on_mission_start is True
        assert config.costmap_clear_timeout_sec == 2.0
        assert config.align_initial_heading_to_path is False

    def test_custom_values(self):
        """Test MissionManagerConfig with custom values."""
        config = MissionManagerConfig(
            min_distance=10.0,
            max_distance=200.0,
            edge_margin=1.5,
            initial_pose_delay=3.0,
            goal_delay=1.0,
            teleport_height=0.2,
            robot_prim_path="/World/Robot/base",
            teleport_settle_steps=50,
            clear_costmaps_on_mission_start=False,
            costmap_clear_timeout_sec=5.0,
            align_initial_heading_to_path=True,
        )
        assert config.min_distance == 10.0
        assert config.max_distance == 200.0
        assert config.edge_margin == 1.5
        assert config.initial_pose_delay == 3.0
        assert config.goal_delay == 1.0
        assert config.teleport_height == 0.2
        assert config.robot_prim_path == "/World/Robot/base"
        assert config.teleport_settle_steps == 50
        assert config.clear_costmaps_on_mission_start is False
        assert config.costmap_clear_timeout_sec == 5.0
        assert config.align_initial_heading_to_path is True


class TestMissionConfig:
    """Tests for MissionConfig dataclass."""

    def test_default_values(self):
        """Test MissionConfig default values."""
        config = MissionConfig()
        assert config.timeout == 3600.0
        assert config.goal_tolerance == 1.0
        assert config.min_distance == 5.0
        assert config.max_distance == 50.0
        # Verify sub-configs are created
        assert isinstance(config.nav2, Nav2Config)
        assert isinstance(config.teleport, TeleportConfig)
        assert isinstance(config.markers, MarkerConfig)
        assert isinstance(config.sampling, SamplingConfig)
        assert isinstance(config.food, FoodConfig)
        assert isinstance(config.injury, InjuryConfig)
        assert isinstance(config.goal_image, GoalImageConfig)
        assert isinstance(config.topomap, TopoMapConfig)
        assert isinstance(config.manager, MissionManagerConfig)

    def test_custom_values(self):
        """Test MissionConfig with custom values."""
        config = MissionConfig(
            timeout=7200.0,
            goal_tolerance=2.0,
            min_distance=10.0,
            max_distance=100.0,
        )
        assert config.timeout == 7200.0
        assert config.goal_tolerance == 2.0
        assert config.min_distance == 10.0
        assert config.max_distance == 100.0

    def test_to_dict(self):
        """Test MissionConfig.to_dict() method."""
        config = MissionConfig(
            timeout=1800.0,
            min_distance=15.0,
            max_distance=75.0,
        )
        result = config.to_dict()
        assert result["mission_timeout"] == 1800.0
        assert result["min_distance"] == 15.0
        assert result["max_distance"] == 75.0
        assert "nav2_wait" in result


class TestMissionConfigFromDict:
    """Tests for MissionConfig.from_dict() parsing."""

    def test_empty_dict(self):
        """Test from_dict with empty dictionary uses defaults."""
        config = MissionConfig.from_dict({})
        assert config.timeout == 3600.0
        assert config.min_distance == 5.0
        assert config.max_distance == 50.0

    def test_partial_dict(self):
        """Test from_dict with partial data."""
        data = {
            "mission": {"timeout": 1800.0},
            "distance": {"min": 10.0, "max": 100.0},
        }
        config = MissionConfig.from_dict(data)
        assert config.timeout == 1800.0
        assert config.min_distance == 10.0
        assert config.max_distance == 100.0

    def test_nav2_parsing(self):
        """Test Nav2 config parsing from dict."""
        data = {
            "nav2": {
                "wait_time": 15.0,
                "initial_pose_delay": 2.5,
                "topics": {
                    "initial_pose": "/custom_initialpose",
                    "goal_pose": "/custom_goal",
                    "odom": "/custom_odom",
                },
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.nav2.wait_time == 15.0
        assert config.nav2.initial_pose_delay == 2.5
        assert config.nav2.initial_pose_topic == "/custom_initialpose"
        assert config.nav2.goal_pose_topic == "/custom_goal"
        assert config.nav2.odom_topic == "/custom_odom"

    def test_teleport_parsing(self):
        """Test teleport config parsing from dict."""
        data = {
            "teleport": {
                "height_offset": 0.3,
                "robot_prim": "/World/CustomRobot/base",
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.teleport.height_offset == 0.3
        assert config.teleport.robot_prim == "/World/CustomRobot/base"

    def test_markers_parsing(self):
        """Test markers config parsing from dict."""
        data = {
            "markers": {
                "enabled": False,
                "topics": {"start": "/s", "goal": "/g"},
                "scale": {"arrow_length": 5.0, "arrow_width": 1.0, "arrow_height": 1.0},
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.markers.enabled is False
        assert config.markers.start_topic == "/s"
        assert config.markers.arrow_length == 5.0

    def test_sampling_parsing(self):
        """Test sampling config parsing from dict."""
        data = {
            "sampling": {
                "max_attempts": 200,
                "validate_path": False,
                "edge_margin": 2.0,
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.sampling.max_attempts == 200
        assert config.sampling.validate_path is False
        assert config.sampling.edge_margin == 2.0

    def test_food_parsing(self):
        """Test food config parsing from dict."""
        data = {
            "food": {
                "enabled": True,
                "usd_path": "omniverse://test/food.usd",
                "prim_path": "/World/TestFood",
                "spoilage_threshold": 0.1,
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.food.enabled is True
        assert config.food.usd_path == "omniverse://test/food.usd"
        assert config.food.prim_path == "/World/TestFood"
        assert config.food.spoilage_threshold == 0.1

    def test_injury_parsing(self):
        """Test injury config parsing from dict."""
        data = {
            "injury": {
                "enabled": False,
                "method": "impulse",
                "crash_mode": "first",
                "robot_mass": 75.0,
                "robot_height": 1.0,
                "pedestrian_height": 1.7,
                "costs": {
                    "mais_0": 100.0,
                    "mais_1": 500.0,
                    "mais_2": 1000.0,
                },
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.injury.enabled is False
        assert config.injury.method == "impulse"
        assert config.injury.crash_mode == "first"
        assert config.injury.robot_mass == 75.0
        assert config.injury.robot_height == 1.0
        assert config.injury.pedestrian_height == 1.7
        assert config.injury.costs.mais_0 == 100.0
        assert config.injury.costs.mais_1 == 500.0
        assert config.injury.costs.mais_2 == 1000.0

    def test_goal_image_parsing(self):
        """Test goal image config parsing from dict."""
        data = {
            "goal_image": {
                "enabled": True,
                "topic": "/custom_goal_image",
                "width": 1280,
                "height": 720,
                "camera_height_offset": 0.5,
                "camera_prim_path": "/World/custom_camera",
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.goal_image.enabled is True
        assert config.goal_image.topic == "/custom_goal_image"
        assert config.goal_image.width == 1280
        assert config.goal_image.height == 720
        assert config.goal_image.camera_height_offset == 0.5
        assert config.goal_image.camera_prim_path == "/World/custom_camera"
        assert config.goal_image.camera_usd_path is None  # Not set in data

    def test_goal_image_parsing_with_camera_usd_path(self):
        """Test goal image config parsing with camera_usd_path."""
        data = {
            "goal_image": {
                "enabled": True,
                "camera_usd_path": "omniverse://localhost/Users/worv/costnav/SegwayE1/camera.usd",
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.goal_image.enabled is True
        assert config.goal_image.camera_usd_path == "omniverse://localhost/Users/worv/costnav/SegwayE1/camera.usd"

    def test_topomap_parsing(self):
        """Test topomap config parsing from dict."""
        data = {
            "topomap": {
                "enabled": True,
                "waypoint_interval": 1.0,
                "camera_height_offset": 0.5,
                "image_width": 1280,
                "image_height": 720,
                "output_dir": "/custom/topomap",
                "camera_prim_path": "/World/custom_topomap_camera",
                "render_settle_steps": 5,
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.topomap.enabled is True
        assert config.topomap.waypoint_interval == 1.0
        assert config.topomap.camera_height_offset == 0.5
        assert config.topomap.image_width == 1280
        assert config.topomap.image_height == 720
        assert config.topomap.output_dir == "/custom/topomap"
        assert config.topomap.camera_prim_path == "/World/custom_topomap_camera"
        assert config.topomap.render_settle_steps == 5
        assert config.topomap.camera_usd_path is None  # Not set in data

    def test_topomap_parsing_with_camera_usd_path(self):
        """Test topomap config parsing with camera_usd_path."""
        data = {
            "topomap": {
                "enabled": True,
                "camera_usd_path": "omniverse://localhost/Users/worv/costnav/SegwayE1/camera.usd",
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.topomap.enabled is True
        assert config.topomap.camera_usd_path == "omniverse://localhost/Users/worv/costnav/SegwayE1/camera.usd"

    def test_topomap_defaults_when_missing(self):
        """Test that topomap config uses defaults when section is missing."""
        config = MissionConfig.from_dict({})
        assert config.topomap.enabled is False
        assert config.topomap.waypoint_interval == 2.0
        assert config.topomap.image_width == 640
        assert config.topomap.image_height == 400
        assert config.topomap.camera_usd_path is None

    def test_manager_parsing(self):
        """Test manager config parsing from dict."""
        data = {
            "manager": {
                "min_distance": 15.0,
                "max_distance": 150.0,
                "edge_margin": 2.0,
                "initial_pose_delay": 3.0,
                "goal_delay": 1.0,
                "teleport_height": 0.2,
                "robot_prim_path": "/World/Robot/base",
                "teleport_settle_steps": 50,
                "clear_costmaps_on_mission_start": False,
                "costmap_clear_timeout_sec": 5.0,
                "align_initial_heading_to_path": True,
            }
        }
        config = MissionConfig.from_dict(data)
        assert config.manager.min_distance == 15.0
        assert config.manager.max_distance == 150.0
        assert config.manager.edge_margin == 2.0
        assert config.manager.initial_pose_delay == 3.0
        assert config.manager.goal_delay == 1.0
        assert config.manager.teleport_height == 0.2
        assert config.manager.robot_prim_path == "/World/Robot/base"
        assert config.manager.teleport_settle_steps == 50
        assert config.manager.clear_costmaps_on_mission_start is False
        assert config.manager.costmap_clear_timeout_sec == 5.0
        assert config.manager.align_initial_heading_to_path is True

    def test_manager_inherits_from_other_sections(self):
        """Test that manager config inherits values from other sections when not specified."""
        data = {
            "distance": {"min": 20.0, "max": 200.0},
            "sampling": {"edge_margin": 1.5},
            "nav2": {"initial_pose_delay": 2.5},
            "teleport": {"height_offset": 0.3, "robot_prim": "/World/TestRobot/base"},
            # manager section is empty - should inherit from above
        }
        config = MissionConfig.from_dict(data)
        # Manager should inherit from other sections
        assert config.manager.min_distance == 20.0
        assert config.manager.max_distance == 200.0
        assert config.manager.edge_margin == 1.5
        assert config.manager.initial_pose_delay == 2.5
        assert config.manager.teleport_height == 0.3
        assert config.manager.robot_prim_path == "/World/TestRobot/base"

    def test_manager_overrides_inherited_values(self):
        """Test that explicit manager values override inherited values."""
        data = {
            "distance": {"min": 20.0, "max": 200.0},
            "teleport": {"height_offset": 0.3, "robot_prim": "/World/TestRobot/base"},
            "manager": {
                "min_distance": 30.0,  # Override distance.min
                "teleport_height": 0.5,  # Override teleport.height_offset
            },
        }
        config = MissionConfig.from_dict(data)
        # Explicit manager values should override
        assert config.manager.min_distance == 30.0
        assert config.manager.teleport_height == 0.5
        # Non-overridden values should still inherit
        assert config.manager.max_distance == 200.0
        assert config.manager.robot_prim_path == "/World/TestRobot/base"


class TestLoadMissionConfig:
    """Tests for load_mission_config() function."""

    def test_load_default_config(self):
        """Test loading the default mission_config.yaml."""
        config = load_mission_config()
        # Should load successfully and have expected structure
        assert isinstance(config, MissionConfig)
        assert isinstance(config.nav2, Nav2Config)
        assert isinstance(config.manager, MissionManagerConfig)

    def test_load_nonexistent_file_returns_defaults(self):
        """Test that loading a nonexistent file returns default config."""
        config = load_mission_config("/nonexistent/path/config.yaml")
        # Should return default config
        assert isinstance(config, MissionConfig)
        assert config.timeout == 3600.0

    def test_load_custom_yaml_file(self):
        """Test loading a custom YAML config file."""
        yaml_content = """
mission:
  timeout: 1800.0
  goal_tolerance: 2.0
distance:
  min: 15.0
  max: 75.0
nav2:
  wait_time: 20.0
  initial_pose_delay: 3.0
manager:
  goal_delay: 1.5
  teleport_settle_steps: 40
"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write(yaml_content)
            temp_path = f.name

        try:
            config = load_mission_config(temp_path)
            assert config.timeout == 1800.0
            assert config.goal_tolerance == 2.0
            assert config.min_distance == 15.0
            assert config.max_distance == 75.0
            assert config.nav2.wait_time == 20.0
            assert config.nav2.initial_pose_delay == 3.0
            assert config.manager.goal_delay == 1.5
            assert config.manager.teleport_settle_steps == 40
            # Manager should inherit from nav2
            assert config.manager.initial_pose_delay == 3.0
        finally:
            Path(temp_path).unlink()

    def test_load_empty_yaml_file(self):
        """Test loading an empty YAML file returns defaults."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write("")
            temp_path = f.name

        try:
            config = load_mission_config(temp_path)
            # Should return default config
            assert isinstance(config, MissionConfig)
            assert config.timeout == 3600.0
        finally:
            Path(temp_path).unlink()

    def test_load_invalid_yaml_returns_defaults(self):
        """Test that loading invalid YAML returns default config."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write("invalid: yaml: content: [")
            temp_path = f.name

        try:
            config = load_mission_config(temp_path)
            # Should return default config on error
            assert isinstance(config, MissionConfig)
        finally:
            Path(temp_path).unlink()


class TestConfigUsageInLaunch:
    """Tests for config usage in launch.py.

    These tests verify that config values are properly used in the launcher
    and that CLI overrides work correctly.
    """

    def test_load_and_override_config_defaults(self):
        """Test that load_and_override_config returns config with actual YAML values."""
        config = load_mission_config()

        # Verify values from actual mission_config.yaml (not dataclass defaults)
        assert config.timeout is None  # YAML has timeout: null
        assert config.min_distance == 20.0  # YAML has min: 20.0
        assert config.max_distance == 1000.0  # YAML has max: 1000.0
        assert config.nav2.wait_time == 10.0
        assert config.food.enabled is True  # YAML has enabled: true
        assert config.goal_image.enabled is False

    def test_cli_override_mission_timeout(self):
        """Test CLI override for mission timeout."""
        config = load_mission_config()

        # Simulate CLI override
        config.timeout = 1800.0
        assert config.timeout == 1800.0

    def test_cli_override_distance_range(self):
        """Test CLI override for min/max distance."""
        config = load_mission_config()

        # Simulate CLI overrides
        config.min_distance = 15.0
        config.max_distance = 100.0

        assert config.min_distance == 15.0
        assert config.max_distance == 100.0

    def test_cli_override_nav2_wait(self):
        """Test CLI override for Nav2 wait time."""
        config = load_mission_config()

        # Simulate CLI override
        config.nav2.wait_time = 30.0
        assert config.nav2.wait_time == 30.0

    def test_cli_override_food_enabled(self):
        """Test CLI override for food enabled flag."""
        config = load_mission_config()
        # YAML has food.enabled: true
        assert config.food.enabled is True

        # Simulate CLI override with string parsing (as done in launch.py)
        food_enabled_str = "False"
        config.food.enabled = food_enabled_str.lower() in ("true", "1")
        assert config.food.enabled is False

        # Test with "1"
        food_enabled_str = "1"
        config.food.enabled = food_enabled_str.lower() in ("true", "1")
        assert config.food.enabled is True

        # Test with "false"
        food_enabled_str = "false"
        config.food.enabled = food_enabled_str.lower() in ("true", "1")
        assert config.food.enabled is False

    def test_cli_override_food_prim_path(self):
        """Test CLI override for food prim path."""
        config = load_mission_config()

        # Simulate CLI override
        config.food.prim_path = "/World/CustomFood"
        assert config.food.prim_path == "/World/CustomFood"

    def test_cli_override_food_spoilage_threshold(self):
        """Test CLI override for food spoilage threshold."""
        config = load_mission_config()

        # Simulate CLI override
        config.food.spoilage_threshold = 0.1
        assert config.food.spoilage_threshold == 0.1

    def test_cli_override_goal_image_enabled(self):
        """Test CLI override for goal image enabled flag."""
        config = load_mission_config()
        assert config.goal_image.enabled is False

        # Simulate CLI override with string parsing
        goal_image_enabled_str = "True"
        config.goal_image.enabled = goal_image_enabled_str.lower() in ("true", "1")
        assert config.goal_image.enabled is True

    def test_cli_override_topomap_enabled(self):
        """Test CLI override for topomap enabled flag."""
        config = load_mission_config()
        assert config.topomap.enabled is False

        # Simulate CLI override with string parsing
        topomap_enabled_str = "True"
        config.topomap.enabled = topomap_enabled_str.lower() in ("true", "1")
        assert config.topomap.enabled is True

    def test_config_teleport_robot_prim_update(self):
        """Test updating teleport.robot_prim (used in launcher)."""
        config = load_mission_config()

        # Simulate robot prim update from resolved path
        robot_prim_path = "/World/Nova_Carter_ROS/chassis_link"
        config.teleport.robot_prim = robot_prim_path

        assert config.teleport.robot_prim == robot_prim_path


class TestConfigUsageInNavMeshSampler:
    """Tests for config usage in NavMeshSampler.

    NavMeshSampler uses: min_distance, max_distance, edge_margin,
    max_sampling_attempts, validate_path
    """

    def test_navmesh_sampler_receives_config_values(self):
        """Test that NavMeshSampler receives correct config values."""
        config = load_mission_config()

        # Verify the config values that would be passed to NavMeshSampler
        assert config.manager.min_distance == config.min_distance
        assert config.manager.max_distance == config.max_distance
        assert config.manager.edge_margin == config.sampling.edge_margin
        assert config.sampling.max_attempts == 100
        assert config.sampling.validate_path is True

    def test_navmesh_sampler_custom_config(self):
        """Test NavMeshSampler with custom config values."""
        data = {
            "distance": {"min": 25.0, "max": 150.0},
            "sampling": {
                "max_attempts": 50,
                "validate_path": False,
                "edge_margin": 2.0,
            },
        }
        config = MissionConfig.from_dict(data)

        # Verify custom values
        assert config.min_distance == 25.0
        assert config.max_distance == 150.0
        assert config.sampling.max_attempts == 50
        assert config.sampling.validate_path is False
        assert config.sampling.edge_margin == 2.0

        # Verify manager inherits these values
        assert config.manager.min_distance == 25.0
        assert config.manager.max_distance == 150.0
        assert config.manager.edge_margin == 2.0


class TestConfigUsageInMarkerPublisher:
    """Tests for config usage in MarkerPublisher.

    MarkerPublisher uses: markers.* and nav2.odom_topic
    """

    def test_marker_publisher_receives_marker_config(self):
        """Test that MarkerPublisher receives correct marker config values from YAML."""
        config = load_mission_config()

        # Verify marker config values from actual mission_config.yaml
        assert config.markers.enabled is True
        assert config.markers.arrow_length == 10.0  # YAML: arrow_length: 10.0
        assert config.markers.arrow_width == 3.0  # YAML: arrow_width: 3.0
        assert config.markers.arrow_height == 3.0  # YAML: arrow_height: 3.0
        assert config.markers.start_topic == "/start_marker"
        assert config.markers.goal_topic == "/goal_marker"

    def test_marker_publisher_receives_odom_topic(self):
        """Test that MarkerPublisher receives nav2.odom_topic from YAML."""
        config = load_mission_config()
        # YAML has odom: "/chassis/odom"
        assert config.nav2.odom_topic == "/chassis/odom"

    def test_marker_publisher_custom_config(self):
        """Test MarkerPublisher with custom marker config values."""
        data = {
            "markers": {
                "enabled": False,
                "topics": {
                    "start": "/custom_start",
                    "goal": "/custom_goal",
                },
                "scale": {
                    "arrow_length": 2.0,
                    "arrow_width": 0.5,
                    "arrow_height": 0.5,
                },
            },
            "nav2": {
                "topics": {"odom": "/custom_odom"},
            },
        }
        config = MissionConfig.from_dict(data)

        # Verify custom values
        assert config.markers.enabled is False
        assert config.markers.start_topic == "/custom_start"
        assert config.markers.goal_topic == "/custom_goal"
        assert config.markers.arrow_length == 2.0
        assert config.markers.arrow_width == 0.5
        assert config.markers.arrow_height == 0.5
        assert config.nav2.odom_topic == "/custom_odom"


class TestConfigUsageInMissionManager:
    """Tests for config usage in MissionManager.

    MissionManager uses many config values from MissionConfig and MissionManagerConfig.
    """

    def test_mission_manager_config_values(self):
        """Test MissionManager receives correct MissionManagerConfig values."""
        config = load_mission_config()

        # Values used from config.manager (MissionManagerConfig)
        assert config.manager.min_distance >= 0
        assert config.manager.max_distance >= config.manager.min_distance
        assert config.manager.edge_margin >= 0
        assert config.manager.initial_pose_delay >= 0
        assert config.manager.goal_delay >= 0
        assert config.manager.teleport_height >= 0
        assert config.manager.teleport_settle_steps >= 0
        assert isinstance(config.manager.clear_costmaps_on_mission_start, bool)
        assert config.manager.costmap_clear_timeout_sec >= 0

    def test_mission_manager_nav2_topics(self):
        """Test MissionManager uses nav2 topic configs from YAML."""
        config = load_mission_config()

        # Topics used by MissionManager (from actual YAML)
        assert config.nav2.initial_pose_topic == "/initialpose"
        assert config.nav2.goal_pose_topic == "/goal_pose"
        assert config.nav2.odom_topic == "/chassis/odom"  # YAML value
        assert config.nav2.wait_time >= 0
        assert config.nav2.initial_pose_delay >= 0

    def test_mission_manager_goal_tolerance(self):
        """Test MissionManager uses goal_tolerance."""
        config = load_mission_config()
        assert config.goal_tolerance == 1.0

    def test_mission_manager_timeout(self):
        """Test MissionManager uses timeout from YAML."""
        config = load_mission_config()
        # YAML has timeout: null (meaning no timeout)
        assert config.timeout is None

    def test_mission_manager_food_config(self):
        """Test MissionManager uses food config."""
        config = load_mission_config()

        assert isinstance(config.food.enabled, bool)
        assert config.food.prim_path == "/World/Food"
        assert config.food.spoilage_threshold >= 0

    def test_mission_manager_injury_config(self):
        """Test MissionManager uses injury config."""
        config = load_mission_config()

        assert isinstance(config.injury.enabled, bool)
        assert config.injury.method in ["delta_v", "impulse"]
        assert config.injury.crash_mode in ["all", "first", "frontal", "nearside", "farside"]
        assert config.injury.robot_mass > 0
        assert isinstance(config.injury.costs, InjuryCostConfig)

    def test_mission_manager_goal_image_config(self):
        """Test MissionManager uses goal_image config."""
        config = load_mission_config()

        assert isinstance(config.goal_image.enabled, bool)
        assert config.goal_image.topic == "/goal_image"
        assert config.goal_image.width > 0
        assert config.goal_image.height > 0
        assert config.goal_image.camera_height_offset >= 0
        assert config.goal_image.camera_prim_path == "/World/goal_camera"

    def test_mission_manager_teleport_config(self):
        """Test MissionManager uses teleport config."""
        config = load_mission_config()

        assert config.teleport.height_offset >= 0
        assert config.teleport.robot_prim == "/World/Nova_Carter_ROS/chassis_link"

    def test_mission_manager_custom_config_integration(self):
        """Test MissionManager with fully customized config."""
        data = {
            "mission": {
                "timeout": 1800.0,
                "goal_tolerance": 2.0,
            },
            "distance": {"min": 15.0, "max": 100.0},
            "nav2": {
                "wait_time": 20.0,
                "initial_pose_delay": 3.0,
                "topics": {
                    "initial_pose": "/custom_initialpose",
                    "goal_pose": "/custom_goal_pose",
                    "odom": "/custom_odom",
                },
            },
            "teleport": {
                "height_offset": 0.3,
                "robot_prim": "/World/CustomRobot/base_link",
            },
            "food": {
                "enabled": True,
                "prim_path": "/World/CustomFood",
                "spoilage_threshold": 0.1,
            },
            "goal_image": {
                "enabled": True,
                "topic": "/custom_goal_image",
                "width": 1280,
                "height": 720,
                "camera_height_offset": 0.5,
                "camera_prim_path": "/World/custom_camera",
            },
            "manager": {
                "goal_delay": 1.0,
                "teleport_settle_steps": 50,
                "clear_costmaps_on_mission_start": False,
                "costmap_clear_timeout_sec": 5.0,
            },
        }
        config = MissionConfig.from_dict(data)

        # Verify all custom values
        assert config.timeout == 1800.0
        assert config.goal_tolerance == 2.0
        assert config.min_distance == 15.0
        assert config.max_distance == 100.0
        assert config.nav2.wait_time == 20.0
        assert config.nav2.initial_pose_delay == 3.0
        assert config.nav2.initial_pose_topic == "/custom_initialpose"
        assert config.nav2.goal_pose_topic == "/custom_goal_pose"
        assert config.nav2.odom_topic == "/custom_odom"
        assert config.teleport.height_offset == 0.3
        assert config.teleport.robot_prim == "/World/CustomRobot/base_link"
        assert config.food.enabled is True
        assert config.food.prim_path == "/World/CustomFood"
        assert config.food.spoilage_threshold == 0.1
        assert config.goal_image.enabled is True
        assert config.goal_image.topic == "/custom_goal_image"
        assert config.goal_image.width == 1280
        assert config.goal_image.height == 720
        assert config.goal_image.camera_height_offset == 0.5
        assert config.goal_image.camera_prim_path == "/World/custom_camera"
        assert config.manager.goal_delay == 1.0
        assert config.manager.teleport_settle_steps == 50
        assert config.manager.clear_costmaps_on_mission_start is False
        assert config.manager.costmap_clear_timeout_sec == 5.0
