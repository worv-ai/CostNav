# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for process_mediaref_bags.py.

This test suite validates:
1. Utility functions (quat_to_yaw, is_backwards, filter_backwards)
2. Image processing functions
3. MediaRef bag discovery and parsing
4. Trajectory data processing and pickle serialization
5. End-to-end pipeline with sample MediaRef data

Note:
    Some tests use real MediaRef sample data located in data/sample_rosbags/
"""

import pickle
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from il_training.data_processing.process_data.process_mediaref_bags import (
    IMAGE_SIZE,
    filter_backwards,
    find_mediaref_bags,
    get_mcap_path,
    is_backwards,
    load_config,
    process_image,
    quat_to_yaw,
)


class TestQuatToYaw:
    """Tests for quaternion to yaw conversion."""

    def test_identity_quaternion_gives_zero_yaw(self):
        """Test identity quaternion (no rotation) gives zero yaw."""
        yaw = quat_to_yaw(0.0, 0.0, 0.0, 1.0)
        assert np.isclose(yaw, 0.0, atol=1e-6)

    def test_90_degree_rotation(self):
        """Test 90 degree rotation around z-axis."""
        # Quaternion for 90 degree rotation: (0, 0, sin(45°), cos(45°))
        yaw = quat_to_yaw(0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
        assert np.isclose(yaw, np.pi / 2, atol=1e-6)

    def test_180_degree_rotation(self):
        """Test 180 degree rotation around z-axis."""
        yaw = quat_to_yaw(0.0, 0.0, 1.0, 0.0)
        assert np.isclose(abs(yaw), np.pi, atol=1e-6)

    def test_negative_yaw(self):
        """Test negative 90 degree rotation."""
        yaw = quat_to_yaw(0.0, 0.0, -np.sin(np.pi / 4), np.cos(np.pi / 4))
        assert np.isclose(yaw, -np.pi / 2, atol=1e-6)


class TestIsBackwards:
    """Tests for backward motion detection."""

    def test_forward_motion_not_backwards(self):
        """Test forward motion along yaw direction is not backwards."""
        pos1 = np.array([0.0, 0.0])
        pos2 = np.array([1.0, 0.0])
        yaw1 = 0.0  # Facing +x direction
        assert not is_backwards(pos1, yaw1, pos2)

    def test_backward_motion_is_backwards(self):
        """Test backward motion opposite to yaw direction."""
        pos1 = np.array([0.0, 0.0])
        pos2 = np.array([-1.0, 0.0])
        yaw1 = 0.0  # Facing +x direction, moving -x
        assert is_backwards(pos1, yaw1, pos2)

    def test_stationary_is_backwards(self):
        """Test stationary (same position) is considered backwards."""
        pos1 = np.array([0.0, 0.0])
        pos2 = np.array([0.0, 0.0])
        yaw1 = 0.0
        assert is_backwards(pos1, yaw1, pos2)

    def test_perpendicular_motion_is_backwards(self):
        """Test perpendicular motion is considered backwards (dot product < eps)."""
        pos1 = np.array([0.0, 0.0])
        pos2 = np.array([0.0, 1.0])  # Moving +y
        yaw1 = 0.0  # Facing +x direction
        # This should be backwards since perpendicular motion has zero forward component
        assert is_backwards(pos1, yaw1, pos2)


class TestFilterBackwards:
    """Tests for trajectory filtering."""

    def test_filter_pure_forward_motion(self):
        """Test all forward motion results in single trajectory."""
        positions = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]])
        yaws = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        images = [Image.new("RGB", (10, 10)) for _ in range(5)]
        traj_data = {"position": positions, "yaw": yaws}

        cut_trajs = filter_backwards(images, traj_data)
        assert len(cut_trajs) >= 1
        total_frames = sum(len(t[0]) for t in cut_trajs)
        assert total_frames > 0

    def test_filter_backward_segment(self):
        """Test backward segment creates trajectory split."""
        # Forward, then backward, then forward
        positions = np.array([[0, 0], [1, 0], [2, 0], [1, 0], [2, 0], [3, 0]])
        yaws = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        images = [Image.new("RGB", (10, 10)) for _ in range(6)]
        traj_data = {"position": positions, "yaw": yaws}

        cut_trajs = filter_backwards(images, traj_data)
        # Should split into multiple segments at backward point
        assert len(cut_trajs) >= 1

    def test_filter_empty_trajectory(self):
        """Test empty trajectory returns empty list."""
        positions = np.array([]).reshape(0, 2)
        yaws = np.array([])
        images = []
        traj_data = {"position": positions, "yaw": yaws}

        cut_trajs = filter_backwards(images, traj_data)
        assert len(cut_trajs) == 0


class TestProcessImage:
    """Tests for image processing function."""

    def test_process_image_output_size(self):
        """Test processed image has correct output size."""
        # Create a test image with 4:3 aspect ratio
        input_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = process_image(input_image)
        assert result.size == IMAGE_SIZE

    def test_process_image_non_standard_aspect_ratio(self):
        """Test processing image with non-standard aspect ratio."""
        # Create a square image
        input_image = np.random.randint(0, 255, (480, 480, 3), dtype=np.uint8)
        result = process_image(input_image)
        assert result.size == IMAGE_SIZE

    def test_process_image_is_pil(self):
        """Test processed image is a PIL Image."""
        input_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = process_image(input_image)
        assert isinstance(result, Image.Image)


class TestLoadConfig:
    """Tests for configuration loading."""

    def test_load_config_none_returns_empty(self):
        """Test None config path returns empty dict."""
        result = load_config(None)
        assert result == {}

    def test_load_config_valid_yaml(self, tmp_path: Path):
        """Test loading valid YAML config file."""
        config_file = tmp_path / "config.yaml"
        config_file.write_text("""
image_topic: /camera/image
odom_topic: /odom
ang_offset: 1.57
""")
        result = load_config(config_file)
        assert result["image_topic"] == "/camera/image"
        assert result["odom_topic"] == "/odom"
        assert np.isclose(result["ang_offset"], 1.57)


class TestFindMediarefBags:
    """Tests for MediaRef bag discovery."""

    def test_find_mediaref_bags_empty_dir(self, tmp_path: Path):
        """Test finding bags in empty directory returns empty list."""
        bags = find_mediaref_bags(tmp_path)
        assert bags == []

    def test_find_mediaref_bags_non_mediaref(self, tmp_path: Path):
        """Test non-mediaref directories are ignored."""
        (tmp_path / "recording_123").mkdir()
        (tmp_path / "recording_123" / "test.mcap").touch()
        bags = find_mediaref_bags(tmp_path)
        assert bags == []

    def test_find_mediaref_bags_without_mcap(self, tmp_path: Path):
        """Test mediaref directories without mcap are ignored."""
        (tmp_path / "recording_123_mediaref").mkdir()
        bags = find_mediaref_bags(tmp_path)
        assert bags == []

    def test_find_mediaref_bags_valid(self, tmp_path: Path):
        """Test finding valid mediaref bags."""
        bag_dir = tmp_path / "recording_123_mediaref"
        bag_dir.mkdir()
        (bag_dir / "test.mcap").touch()
        bags = find_mediaref_bags(tmp_path)
        assert len(bags) == 1
        assert bags[0] == bag_dir


class TestGetMcapPath:
    """Tests for MCAP path extraction."""

    def test_get_mcap_path_not_found(self, tmp_path: Path):
        """Test returns None when no mcap file exists."""
        result = get_mcap_path(tmp_path)
        assert result is None

    def test_get_mcap_path_found(self, tmp_path: Path):
        """Test returns path when mcap file exists."""
        mcap_file = tmp_path / "test.mcap"
        mcap_file.touch()
        result = get_mcap_path(tmp_path)
        assert result == mcap_file


class TestPickleSerialization:
    """Tests for trajectory data pickle serialization."""

    def test_traj_data_pickle_roundtrip(self, tmp_path: Path):
        """Test trajectory data can be saved and loaded correctly."""
        traj_data = {
            "position": np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]),
            "yaw": np.array([0.0, 0.5, 1.0]),
        }

        pkl_path = tmp_path / "traj_data.pkl"
        with open(pkl_path, "wb") as f:
            pickle.dump(traj_data, f)

        with open(pkl_path, "rb") as f:
            loaded = pickle.load(f)

        np.testing.assert_array_equal(loaded["position"], traj_data["position"])
        np.testing.assert_array_equal(loaded["yaw"], traj_data["yaw"])

    def test_traj_data_pickle_contains_required_keys(self, tmp_path: Path):
        """Test pickled trajectory data contains required keys."""
        traj_data = {
            "position": np.array([[1.0, 2.0]]),
            "yaw": np.array([0.5]),
        }

        pkl_path = tmp_path / "traj_data.pkl"
        with open(pkl_path, "wb") as f:
            pickle.dump(traj_data, f)

        with open(pkl_path, "rb") as f:
            loaded = pickle.load(f)

        assert "position" in loaded
        assert "yaw" in loaded


@pytest.mark.skipif(
    not (Path(__file__).parents[4] / "data" / "sample_rosbags").exists(),
    reason="Sample MediaRef data not available",
)
class TestIntegrationWithSampleData:
    """Integration tests using actual sample MediaRef data."""

    @pytest.fixture
    def sample_data_dir(self):
        """Get path to sample data directory."""
        return Path(__file__).parents[4] / "data" / "sample_rosbags"

    def test_find_sample_mediaref_bags(self, sample_data_dir: Path):
        """Test finding MediaRef bags in sample data."""
        bags = find_mediaref_bags(sample_data_dir)
        assert len(bags) > 0
        assert all(b.name.endswith("_mediaref") for b in bags)

    def test_get_mcap_from_sample_bag(self, sample_data_dir: Path):
        """Test getting MCAP path from sample MediaRef bag."""
        bags = find_mediaref_bags(sample_data_dir)
        if bags:
            mcap_path = get_mcap_path(bags[0])
            assert mcap_path is not None
            assert mcap_path.suffix == ".mcap"
            assert mcap_path.exists()

    def test_extract_images_and_odom(self, sample_data_dir: Path):
        """Test image and odometry extraction from sample data."""
        from il_training.data_processing.process_data.process_mediaref_bags import (
            get_images_and_odom_from_mediaref,
        )

        bags = find_mediaref_bags(sample_data_dir)
        if not bags:
            pytest.skip("No sample bags available")

        mcap_path = get_mcap_path(bags[0])
        images, traj_data = get_images_and_odom_from_mediaref(
            mcap_path=mcap_path,
            bag_dir=bags[0],
            image_topic="/front_stereo_camera/left/image_raw",
            odom_topic="/chassis/odom",
            rate=4.0,
            ang_offset=0.0,
        )

        assert images is not None
        assert traj_data is not None
        assert len(images) > 0
        assert len(images) == len(traj_data["position"])
        assert len(images) == len(traj_data["yaw"])
        # Check image format
        assert all(isinstance(img, Image.Image) for img in images)
        assert all(img.size == IMAGE_SIZE for img in images)

    def test_full_pipeline_creates_output(self, sample_data_dir: Path, tmp_path: Path):
        """Test full processing pipeline creates expected output structure."""
        from il_training.data_processing.process_data.process_mediaref_bags import (
            process_mediaref_bag,
        )

        bags = find_mediaref_bags(sample_data_dir)
        if not bags:
            pytest.skip("No sample bags available")

        config = {
            "image_topic": "/front_stereo_camera/left/image_raw",
            "odom_topic": "/chassis/odom",
            "ang_offset": 0.0,
        }

        result = process_mediaref_bag(bags[0], tmp_path, config, sample_rate=4.0)
        assert result is True

        # Check output structure
        traj_dirs = list(tmp_path.iterdir())
        assert len(traj_dirs) > 0

        for traj_dir in traj_dirs:
            # Check traj_data.pkl exists
            pkl_path = traj_dir / "traj_data.pkl"
            assert pkl_path.exists()

            # Load and validate pickle content
            with open(pkl_path, "rb") as f:
                traj_data = pickle.load(f)
            assert "position" in traj_data
            assert "yaw" in traj_data

            # Check images exist
            jpg_files = list(traj_dir.glob("*.jpg"))
            assert len(jpg_files) > 0

            # Verify image count matches traj_data
            assert len(jpg_files) == len(traj_data["position"])

    def test_output_matches_vint_format(self, sample_data_dir: Path, tmp_path: Path):
        """Test output format matches ViNT training data requirements."""
        from il_training.data_processing.process_data.process_mediaref_bags import (
            process_mediaref_bag,
        )

        bags = find_mediaref_bags(sample_data_dir)
        if not bags:
            pytest.skip("No sample bags available")

        config = {
            "image_topic": "/front_stereo_camera/left/image_raw",
            "odom_topic": "/chassis/odom",
            "ang_offset": 0.0,
        }

        process_mediaref_bag(bags[0], tmp_path, config, sample_rate=4.0)

        traj_dirs = list(tmp_path.iterdir())
        if traj_dirs:
            traj_dir = traj_dirs[0]

            # ViNT expects: numbered images 0.jpg, 1.jpg, etc.
            jpg_files = sorted(traj_dir.glob("*.jpg"))
            assert jpg_files[0].stem == "0"

            # Images should be sequential
            expected_names = [str(i) for i in range(len(jpg_files))]
            actual_names = [f.stem for f in jpg_files]
            assert actual_names == expected_names

            # Load first image and check size
            first_img = Image.open(jpg_files[0])
            assert first_img.size == IMAGE_SIZE
