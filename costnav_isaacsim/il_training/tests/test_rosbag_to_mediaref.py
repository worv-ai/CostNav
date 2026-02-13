# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for rosbag_to_mediaref converter.

This test suite validates:
1. VideoWriter class functionality (frame encoding, lossless mode)
2. detect_format function (format detection for different bag types)
3. decode_image_msg function (ROS image message decoding)
4. load_config function (YAML config loading)
5. Path handling utilities

Note:
    These tests do NOT require actual rosbag files or ROS2 to be running.
    They test the logic and utilities, not the full conversion pipeline.
"""

from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest


class TestDetectFormat:
    """Tests for detect_format validation function."""

    def test_detect_mcap_file(self, tmp_path: Path):
        """Test validation passes for MCAP file format."""
        from il_training.data_processing.converters.rosbag_to_mediaref import detect_format

        mcap_file = tmp_path / "test.mcap"
        mcap_file.touch()
        detect_format(mcap_file)  # Should not raise

    def test_detect_rosbag2_directory(self, tmp_path: Path):
        """Test validation passes for rosbag2 directory format."""
        from il_training.data_processing.converters.rosbag_to_mediaref import detect_format

        bag_dir = tmp_path / "test_bag"
        bag_dir.mkdir()
        (bag_dir / "metadata.yaml").touch()
        detect_format(bag_dir)  # Should not raise

    def test_detect_unknown_format(self, tmp_path: Path):
        """Test that unknown format raises ValueError."""
        from il_training.data_processing.converters.rosbag_to_mediaref import detect_format

        unknown_dir = tmp_path / "unknown"
        unknown_dir.mkdir()
        with pytest.raises(ValueError, match="Unknown or unsupported"):
            detect_format(unknown_dir)

    def test_detect_invalid_file_extension(self, tmp_path: Path):
        """Test that non-MCAP files are rejected."""
        from il_training.data_processing.converters.rosbag_to_mediaref import detect_format

        invalid_file = tmp_path / "test.txt"
        invalid_file.touch()
        with pytest.raises(ValueError):
            detect_format(invalid_file)


class TestDecodeImageMsg:
    """Tests for decode_image_msg function.

    All decode functions now return RGB format for video encoding.
    """

    def test_decode_compressed_image_returns_rgb(self):
        """Test decoding of CompressedImage messages returns RGB."""
        import cv2

        from il_training.data_processing.converters.rosbag_to_mediaref import decode_image_msg

        # Create a simple BGR test image and encode it (OpenCV uses BGR)
        # Red pixel in BGR is (0, 0, 255)
        bgr_image = np.zeros((100, 100, 3), dtype=np.uint8)
        bgr_image[0, 0] = [0, 0, 255]  # BGR red
        _, encoded = cv2.imencode(".png", bgr_image)

        mock_msg = MagicMock()
        mock_msg.format = "png compressed bgr8"  # CompressedImage has 'format' attribute
        mock_msg.data = encoded.tobytes()

        result = decode_image_msg(mock_msg)
        assert result is not None
        assert result.shape[:2] == (100, 100)
        # After conversion to RGB, red pixel should be (255, 0, 0)
        assert result[0, 0, 0] == 255  # R channel
        assert result[0, 0, 2] == 0  # B channel

    def test_decode_raw_rgb8_image_unchanged(self):
        """Test decoding of raw RGB8 Image messages (already RGB)."""
        from il_training.data_processing.converters.rosbag_to_mediaref import decode_image_msg

        h, w = 100, 150
        # Create RGB image with known red pixel
        test_data = np.zeros((h, w, 3), dtype=np.uint8)
        test_data[0, 0] = [255, 0, 0]  # RGB red

        mock_msg = MagicMock(spec=["encoding", "height", "width", "data", "is_bigendian"])
        mock_msg.encoding = "rgb8"
        mock_msg.height = h
        mock_msg.width = w
        mock_msg.data = test_data.tobytes()
        mock_msg.is_bigendian = False

        result = decode_image_msg(mock_msg)
        assert result is not None
        assert result.shape == (h, w, 3)
        # RGB8 should pass through unchanged
        assert result[0, 0, 0] == 255  # R
        assert result[0, 0, 1] == 0  # G
        assert result[0, 0, 2] == 0  # B

    def test_decode_raw_bgr8_image_converted_to_rgb(self):
        """Test decoding of raw BGR8 Image messages converts to RGB."""
        from il_training.data_processing.converters.rosbag_to_mediaref import decode_image_msg

        h, w = 80, 120
        # Create BGR image with known red pixel (BGR order: B=0, G=0, R=255)
        test_data = np.zeros((h, w, 3), dtype=np.uint8)
        test_data[0, 0] = [0, 0, 255]  # BGR red

        mock_msg = MagicMock(spec=["encoding", "height", "width", "data", "is_bigendian"])
        mock_msg.encoding = "bgr8"
        mock_msg.height = h
        mock_msg.width = w
        mock_msg.data = test_data.tobytes()
        mock_msg.is_bigendian = False

        result = decode_image_msg(mock_msg)
        assert result is not None
        assert result.shape == (h, w, 3)
        # BGR should be converted to RGB
        assert result[0, 0, 0] == 255  # R (was B in BGR)
        assert result[0, 0, 2] == 0  # B (was R in BGR)

    def test_decode_mono8_image(self):
        """Test decoding of mono8 grayscale Image messages."""
        from il_training.data_processing.converters.rosbag_to_mediaref import decode_image_msg

        h, w = 50, 60
        test_data = np.full((h, w), 128, dtype=np.uint8)

        mock_msg = MagicMock(spec=["encoding", "height", "width", "data", "is_bigendian"])
        mock_msg.encoding = "mono8"
        mock_msg.height = h
        mock_msg.width = w
        mock_msg.data = test_data.tobytes()
        mock_msg.is_bigendian = False

        result = decode_image_msg(mock_msg)
        assert result is not None
        # mono8 converted to RGB has 3 channels
        assert result.shape == (h, w, 3)

    def test_decode_rgba8_image(self):
        """Test decoding of RGBA8 Image messages (alpha channel stripped)."""
        from il_training.data_processing.converters.rosbag_to_mediaref import decode_image_msg

        h, w = 40, 50
        test_data = np.zeros((h, w, 4), dtype=np.uint8)
        test_data[:, :, 0] = 200  # R
        test_data[:, :, 3] = 255  # Alpha

        mock_msg = MagicMock(spec=["encoding", "height", "width", "data", "is_bigendian"])
        mock_msg.encoding = "rgba8"
        mock_msg.height = h
        mock_msg.width = w
        mock_msg.data = test_data.tobytes()
        mock_msg.is_bigendian = False

        result = decode_image_msg(mock_msg)
        assert result is not None
        # Alpha channel should be stripped
        assert result.shape == (h, w, 3)

    def test_decode_unknown_encoding_raises_error(self):
        """Test that unknown image encoding raises ImageFormatError."""
        import pytest
        from rosbags.image import ImageFormatError

        from il_training.data_processing.converters.rosbag_to_mediaref import decode_image_msg

        mock_msg = MagicMock(spec=["encoding", "height", "width", "data", "is_bigendian"])
        mock_msg.encoding = "unknown_format"
        mock_msg.height = 100
        mock_msg.width = 100
        mock_msg.data = b"\x00" * 30000
        mock_msg.is_bigendian = False

        with pytest.raises(ImageFormatError):
            decode_image_msg(mock_msg)


class TestLoadConfig:
    """Tests for load_config function."""

    def test_load_config_none_returns_empty(self):
        """Test that None config returns empty dict."""
        from il_training.data_processing.converters.rosbag_to_mediaref import load_config

        result = load_config(None)
        assert result == {}

    def test_load_config_valid_yaml(self, tmp_path: Path):
        """Test loading a valid YAML config file."""
        from il_training.data_processing.converters.rosbag_to_mediaref import load_config

        config_file = tmp_path / "config.yaml"
        config_file.write_text("""
video:
  fps: 20.0
image_topics:
  - /camera/image_raw
topics_to_remove:
  - /bond
""")
        result = load_config(config_file)
        assert result["video"]["fps"] == 20.0
        assert "/camera/image_raw" in result["image_topics"]
        assert "/bond" in result["topics_to_remove"]


class TestVideoWriter:
    """Tests for VideoWriter class."""

    def test_video_writer_initialization(self, tmp_path: Path):
        """Test VideoWriter initialization."""
        from il_training.data_processing.converters.rosbag_to_mediaref import VideoWriter

        output_path = tmp_path / "test.mp4"
        writer = VideoWriter(output_path, fps=30.0)

        assert writer.output_path == output_path
        assert writer.fps == 30.0
        assert writer.frame_count == 0
        assert writer.container is None
        assert not writer._closed

    def test_video_writer_lossless_mode(self, tmp_path: Path):
        """Test VideoWriter lossless mode initialization."""
        from il_training.data_processing.converters.rosbag_to_mediaref import VideoWriter

        output_path = tmp_path / "test_lossless.mp4"
        writer = VideoWriter(output_path, fps=30.0, lossless=True)

        assert writer.lossless is True

    def test_video_writer_add_frame_none_returns_false(self, tmp_path: Path):
        """Test that adding None frame returns False."""
        from il_training.data_processing.converters.rosbag_to_mediaref import VideoWriter

        output_path = tmp_path / "test.mp4"
        writer = VideoWriter(output_path)

        result = writer.add_frame(None)
        assert result is False

    def test_video_writer_add_frame_empty_returns_false(self, tmp_path: Path):
        """Test that adding empty frame returns False."""
        from il_training.data_processing.converters.rosbag_to_mediaref import VideoWriter

        output_path = tmp_path / "test.mp4"
        writer = VideoWriter(output_path)

        empty_frame = np.array([])
        result = writer.add_frame(empty_frame)
        assert result is False

    def test_video_writer_add_valid_frame(self, tmp_path: Path):
        """Test adding a valid frame to VideoWriter."""
        from il_training.data_processing.converters.rosbag_to_mediaref import VideoWriter

        output_path = tmp_path / "output" / "test.mp4"
        writer = VideoWriter(output_path, fps=30.0)

        # Create a valid test frame (BGR format)
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = writer.add_frame(frame)

        assert result is True
        assert writer.frame_count == 1
        writer.close()

    def test_video_writer_close_idempotent(self, tmp_path: Path):
        """Test that closing VideoWriter multiple times is safe."""
        from il_training.data_processing.converters.rosbag_to_mediaref import VideoWriter

        output_path = tmp_path / "test.mp4"
        writer = VideoWriter(output_path)

        # Close multiple times should not raise
        writer.close()
        writer.close()
        writer.close()
        assert writer._closed is True
