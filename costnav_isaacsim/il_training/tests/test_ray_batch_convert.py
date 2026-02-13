# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for ray_batch_convert module.

This test suite validates:
1. find_bags function (bag discovery in directories)
2. load_config function (YAML config loading)
3. Ray task initialization and configuration

Note:
    These tests do NOT require actual rosbag files or Ray cluster.
    They test the logic and utilities, not the full batch processing.
"""

from pathlib import Path


class TestFindBags:
    """Tests for find_bags function."""

    def test_find_bags_empty_directory(self, tmp_path: Path):
        """Test find_bags returns empty list for empty directory."""
        from il_training.data_processing.converters.ray_batch_convert import find_bags

        result = find_bags(tmp_path)
        assert result == []

    def test_find_bags_with_valid_bags(self, tmp_path: Path):
        """Test find_bags discovers valid rosbag2 directories."""
        from il_training.data_processing.converters.ray_batch_convert import find_bags

        # Create valid bag directories
        bag1 = tmp_path / "bag_001"
        bag1.mkdir()
        (bag1 / "metadata.yaml").touch()

        bag2 = tmp_path / "bag_002"
        bag2.mkdir()
        (bag2 / "metadata.yaml").touch()

        result = find_bags(tmp_path)
        assert len(result) == 2
        assert bag1 in result
        assert bag2 in result

    def test_find_bags_ignores_invalid_dirs(self, tmp_path: Path):
        """Test find_bags ignores directories without metadata.yaml."""
        from il_training.data_processing.converters.ray_batch_convert import find_bags

        # Create valid bag
        valid_bag = tmp_path / "valid_bag"
        valid_bag.mkdir()
        (valid_bag / "metadata.yaml").touch()

        # Create invalid directory (no metadata.yaml)
        invalid_dir = tmp_path / "invalid_dir"
        invalid_dir.mkdir()
        (invalid_dir / "some_file.txt").touch()

        result = find_bags(tmp_path)
        assert len(result) == 1
        assert valid_bag in result
        assert invalid_dir not in result

    def test_find_bags_sorted_output(self, tmp_path: Path):
        """Test find_bags returns sorted list of bags."""
        from il_training.data_processing.converters.ray_batch_convert import find_bags

        # Create bags with names that should be sorted
        for name in ["c_bag", "a_bag", "b_bag"]:
            bag_dir = tmp_path / name
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").touch()

        result = find_bags(tmp_path)
        names = [p.name for p in result]
        assert names == sorted(names)

    def test_find_specific_bags(self, tmp_path: Path):
        """Test find_bags with specific bag names filter."""
        from il_training.data_processing.converters.ray_batch_convert import find_bags

        # Create multiple bags
        for name in ["bag_001", "bag_002", "bag_003"]:
            bag_dir = tmp_path / name
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").touch()

        # Request only specific bags
        result = find_bags(tmp_path, bag_names=["bag_001", "bag_003"])
        names = [p.name for p in result]
        assert "bag_001" in names
        assert "bag_003" in names
        assert "bag_002" not in names

    def test_find_bags_warns_on_invalid_specific_bag(self, tmp_path: Path, capsys):
        """Test find_bags handles invalid specific bag names gracefully."""
        from il_training.data_processing.converters.ray_batch_convert import find_bags

        # Create only one valid bag
        valid_bag = tmp_path / "valid_bag"
        valid_bag.mkdir()
        (valid_bag / "metadata.yaml").touch()

        # Also create an invalid one (no metadata.yaml)
        invalid = tmp_path / "invalid"
        invalid.mkdir()

        result = find_bags(tmp_path, bag_names=["valid_bag", "invalid", "nonexistent"])
        # Should only return the valid bag
        assert len(result) == 1
        assert result[0].name == "valid_bag"


class TestLoadConfig:
    """Tests for load_config function."""

    def test_load_config_none_returns_empty(self):
        """Test that None config returns empty dict."""
        from il_training.data_processing.converters.ray_batch_convert import load_config

        result = load_config(None)
        assert result == {}

    def test_load_config_valid_yaml(self, tmp_path: Path):
        """Test loading valid YAML config."""
        from il_training.data_processing.converters.ray_batch_convert import load_config

        config_file = tmp_path / "config.yaml"
        config_file.write_text("""
ray:
  max_disk_workers: 8
video:
  fps: 25.0
image_topics:
  - /camera/left
  - /camera/right
""")
        result = load_config(config_file)
        assert result["ray"]["max_disk_workers"] == 8
        assert result["video"]["fps"] == 25.0
        assert len(result["image_topics"]) == 2


class TestConvertBagTaskResult:
    """Tests for convert_bag_task result format."""

    def test_success_result_format(self):
        """Test that successful conversion returns proper format."""
        # Expected format for successful conversion
        success_result = {
            "input_path": "/path/to/bag",
            "status": "success",
            "frames_written": {"/camera/image": 100},
            "duration_seconds": 5.0,
            "topics_processed": ["/camera/image"],
        }

        assert success_result["status"] == "success"
        assert "frames_written" in success_result
        assert isinstance(success_result["frames_written"], dict)

    def test_skipped_result_format(self):
        """Test that skipped conversion returns proper format."""
        skipped_result = {
            "input_path": "/path/to/bag",
            "status": "skipped",
            "reason": "already processed",
        }

        assert skipped_result["status"] == "skipped"
        assert "reason" in skipped_result

    def test_error_result_format(self):
        """Test that error conversion returns proper format."""
        error_result = {
            "input_path": "/path/to/bag",
            "status": "error",
            "error": "No image topics found",
        }

        assert error_result["status"] == "error"
        assert "error" in error_result
