#!/usr/bin/env python3
"""Convert CostNav ROS2 MCAP bags to MediaRef format.

This converter extracts image topics to video files and creates MediaRef
references in the output bag for efficient lazy-loading during training.

Adapted from sketchdrive bag_to_mediaref.py for CostNav data format.

Example:
    python rosbag_to_mediaref.py \
        --input data/sample_rosbags/recording_20260109_061808 \
        --output data/processed/recording_20260109_061808
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from typing import Optional

import av
import cv2
import numpy as np
from mediaref import MediaRef
from rosbags.image import image_to_cvimage
from rosbags.image.image import convert_color
from rosbags.rosbag2 import Reader as Reader2
from rosbags.rosbag2 import Writer as Writer2
from rosbags.rosbag2.writer import StoragePlugin
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.base import TypesysError
from tqdm import tqdm


class VideoWriter:
    """Video encoder using PyAV with CFR and precise timestamps."""

    def __init__(
        self,
        output_path: Path,
        *,
        fps: float = 30.0,
        lossless: bool = False,
    ):
        self.output_path = output_path
        self.fps = fps
        self.container = None
        self.stream = None
        self._closed = False
        self.frame_count = 0
        self.lossless = lossless

    def add_frame(self, image_data: np.ndarray) -> bool:
        """Add RGB image frame to video.

        Args:
            image_data: RGB image array (H, W, 3). Must be in RGB format,
                       as PyAV expects RGB for encoding.

        Returns:
            True if frame was added successfully
        """
        if image_data is None or image_data.size == 0:
            return False

        frame = image_data
        h, w = frame.shape[:2]

        if self.container is None:
            # Initialize with even dimensions for yuv420p
            init_w = w if self.lossless else (w & ~1)
            init_h = h if self.lossless else (h & ~1)
            self._init_writer(init_w, init_h)

        if not self.lossless:
            # Ensure frame matches encoder resolution
            target_w, target_h = self.stream.width, self.stream.height
            even_w, even_h = w & ~1, h & ~1
            if even_w != w or even_h != h:
                frame = frame[:even_h, :even_w]
            if frame.shape[1] != target_w or frame.shape[0] != target_h:
                frame = frame[:target_h, :target_w]

        video_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = self.frame_count
        self.frame_count += 1

        for packet in self.stream.encode(video_frame):
            self.container.mux(packet)

        return True

    def _init_writer(self, width: int, height: int):
        """Initialize video encoder with CFR."""
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        gop_size = 30

        self.container = av.open(str(self.output_path), mode="w")

        if self.lossless:
            self.stream = self.container.add_stream("libx264rgb", rate=int(self.fps))
            self.stream.pix_fmt = "rgb24"
            self.stream.options = {"crf": "0", "preset": "ultrafast"}
        else:
            width = int(width) & ~1
            height = int(height) & ~1
            self.stream = self.container.add_stream("h264", rate=int(self.fps))
            self.stream.pix_fmt = "yuv420p"

        self.stream.width = width
        self.stream.height = height
        self.stream.codec_context.gop_size = gop_size
        self.stream.codec_context.options = {
            "g": str(gop_size),
            "sc_threshold": "0",
            "bf": "0",
        }

    def close(self):
        """Finalize video file."""
        if self._closed:
            return
        if self.stream:
            for packet in self.stream.encode():
                self.container.mux(packet)
        if self.container:
            self.container.close()
        self._closed = True


def detect_format(path: Path) -> None:
    """Validate that path is a supported ROS bag format.

    Raises:
        ValueError: If path is not a valid ROS2 bag (MCAP file or directory with metadata.yaml)
    """
    if path.is_file() and path.suffix == ".mcap":
        return
    if path.is_dir() and (path / "metadata.yaml").exists():
        return
    raise ValueError(f"Unknown or unsupported format: {path}")


def _compressed_image_to_cvimage(msg, color_space: str | None = None) -> np.ndarray:
    """Convert sensor_msgs/msg/CompressedImage to OpenCV image.

    This overrides rosbags.image.compressed_image_to_cvimage to properly handle
    the format field from CompressedImage messages instead of assuming BGR.

    CompressedImage format field examples:
        - "jpeg" (legacy, assume bgr8)
        - "png" (legacy, assume bgr8)
        - "jpeg compressed bgr8"
        - "jpeg compressed rgb8"

    Args:
        msg: CompressedImage message with 'format' and 'data' fields.
        color_space: Target color space (e.g., 'rgb8', 'bgr8'). If None, returns as-is.

    Returns:
        OpenCV image in the requested color space.

    References:
        - https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html
    """
    # Parse the format field to determine source color space
    # Format can be: "jpeg", "png", "jpeg compressed bgr8", "jpeg compressed rgb8", etc.
    format_parts = msg.format.split() if msg.format else []
    if len(format_parts) >= 3 and format_parts[-1] in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
        src_color_space = format_parts[-1]
    else:
        # Legacy format without color space info, assume bgr8
        src_color_space = "bgr8"

    # Decode the compressed image
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_ANYCOLOR)

    if color_space:
        return convert_color(img, src_color_space, color_space)
    return img


def decode_image_msg(msg) -> Optional[np.ndarray]:
    """Decode ROS image message to RGB numpy array.

    This function handles various ROS image message types and encodings,
    converting them to RGB format suitable for video encoding.

    Args:
        msg: ROS message object (Image or CompressedImage)

    Returns:
        RGB numpy array (H, W, 3) or None if decoding fails

    References:
        - sensor_msgs/msg/Image: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
        - sensor_msgs/msg/CompressedImage: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html
    """
    if hasattr(msg, "format"):
        # CompressedImage: use our custom decoder that properly handles format field
        return _compressed_image_to_cvimage(msg, color_space="rgb8")
    else:
        # Image: use rosbags library with color_space conversion
        return image_to_cvimage(msg, color_space="rgb8")


def convert_bag(
    input_path: Path,
    output_path: Path,
    media_dir: Path,
    fps: float = 30.0,
    image_topics: Optional[list[str]] = None,
    topics_to_remove: Optional[list[str]] = None,
) -> dict:
    """Convert CostNav ROS2 bag to MediaRef format.

    Args:
        input_path: Input bag directory path
        output_path: Output bag file/directory path
        media_dir: Directory for media files
        fps: Video frames per second
        image_topics: List of topics to extract as video (auto-detect if None)
        topics_to_remove: List of topics to exclude from output

    Returns:
        Dict with conversion statistics
    """
    detect_format(input_path)
    typestore = get_typestore(Stores.ROS2_JAZZY)
    deserialize = typestore.deserialize_cdr
    serialize = typestore.serialize_cdr

    video_writers: dict[str, VideoWriter] = {}
    media_dir_name = media_dir.name
    topics_to_remove = set(topics_to_remove or [])
    stats = {
        "input_path": str(input_path),
        "frames_written": {},
        "topics_processed": [],
        "topics_removed": list(topics_to_remove),
    }

    # Get bag info
    with Reader2(input_path) as reader:
        bag_start_time_ns = reader.start_time
        duration_ns = reader.duration
        stats["duration_seconds"] = duration_ns / 1e9

        # Identify image topics (from config or auto-detect)
        for conn in reader.connections:
            is_image_topic = False
            if image_topics:
                is_image_topic = conn.topic in image_topics
            else:
                is_image_topic = "Image" in conn.msgtype

            if is_image_topic and conn.topic not in topics_to_remove:
                topic_name = conn.topic.strip("/").replace("/", "_")
                video_path = media_dir / f"{topic_name}.mp4"
                video_writers[conn.topic] = VideoWriter(video_path, fps=fps)
                stats["topics_processed"].append(conn.topic)

    if not video_writers:
        raise ValueError("No image topics found in bag")

    # Pass 1: Extract images to video
    current_frame_indices = {topic: 0 for topic in video_writers}

    with Reader2(input_path) as reader:
        with tqdm(
            total=duration_ns / 1e9,
            desc="Encoding videos",
            unit="s",
            bar_format="{l_bar}{bar}| {n:.2f}/{total:.2f}s [{elapsed}<{remaining}]",
        ) as pbar:
            last_time = bag_start_time_ns
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic not in video_writers:
                    continue

                msg = deserialize(rawdata, connection.msgtype)
                frame = decode_image_msg(msg)

                if frame is not None:
                    video_writers[connection.topic].add_frame(frame)
                    current_frame_indices[connection.topic] += 1

                pbar.update((timestamp - last_time) / 1e9)
                last_time = timestamp

    # Record frame counts
    for topic, writer in video_writers.items():
        stats["frames_written"][topic] = writer.frame_count

    # Pass 2: Create MediaRef bag
    current_frame_indices = {topic: 0 for topic in video_writers}

    with Reader2(input_path) as reader:
        with Writer2(output_path, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
            conn_map: dict[int, object] = {}

            for connection in reader.connections:
                # Skip topics marked for removal
                if connection.topic in topics_to_remove:
                    continue

                msgtype = "std_msgs/msg/String" if connection.topic in video_writers else connection.msgtype
                try:
                    conn = writer.add_connection(connection.topic, msgtype, typestore=typestore)
                    conn_map[connection.id] = conn
                except TypesysError as e:
                    print(f"Warning: Skipping topic '{connection.topic}': {e}", file=sys.stderr)
                    continue

            with tqdm(
                total=duration_ns / 1e9,
                desc="Writing bag",
                unit="s",
                bar_format="{l_bar}{bar}| {n:.2f}/{total:.2f}s [{elapsed}<{remaining}]",
            ) as pbar:
                last_time = bag_start_time_ns
                for connection, timestamp, rawdata in reader.messages():
                    if connection.id not in conn_map:
                        continue

                    if connection.topic in video_writers:
                        msg = deserialize(rawdata, connection.msgtype)
                        frame = decode_image_msg(msg)

                        if frame is not None:
                            frame_idx = current_frame_indices[connection.topic]
                            cfr_pts_ns = int(frame_idx * 1_000_000_000 / fps)
                            current_frame_indices[connection.topic] += 1

                            topic_name = connection.topic.strip("/").replace("/", "_")
                            ref = MediaRef(uri=f"{media_dir_name}/{topic_name}.mp4", pts_ns=cfr_pts_ns)
                            ref_msg = typestore.types["std_msgs/msg/String"](data=ref.model_dump_json())
                            ref_data = serialize(ref_msg, "std_msgs/msg/String")
                            writer.write(conn_map[connection.id], timestamp, ref_data)
                    else:
                        writer.write(conn_map[connection.id], timestamp, rawdata)

                    pbar.update((timestamp - last_time) / 1e9)
                    last_time = timestamp

    # Finalize video writers
    for writer in video_writers.values():
        writer.close()

    # Flatten ROS2 bag directory structure:
    # rosbags creates: output_path/ (e.g., bag_name.mcap/) with metadata.yaml and .mcap file inside
    # The actual MCAP file inside is named like: bag_name.mcap.mcap
    # We want: metadata.yaml and bag_name.mcap directly in the parent directory
    if output_path.is_dir():
        parent_dir = output_path.parent
        bag_name = output_path.stem  # Remove .mcap extension from directory name

        # Move metadata.yaml up one level
        src_metadata = output_path / "metadata.yaml"
        if src_metadata.exists():
            dst_metadata = parent_dir / "metadata.yaml"
            shutil.move(str(src_metadata), str(dst_metadata))

        # Move the actual MCAP file up and rename it
        # The file inside is typically named bag_name.mcap.mcap (double extension)
        # We need to move to temp first because the destination name conflicts with the directory
        mcap_files = list(output_path.glob("*.mcap"))
        dst_mcap = parent_dir / f"{bag_name}.mcap"
        if mcap_files:
            src_mcap = mcap_files[0]
            # Move to temp file first to avoid conflict with directory name
            temp_mcap = parent_dir / f".{bag_name}.mcap.tmp"
            shutil.move(str(src_mcap), str(temp_mcap))

            # Remove the now-empty directory
            output_path.rmdir()

            # Rename temp file to final name
            shutil.move(str(temp_mcap), str(dst_mcap))
        else:
            # No MCAP files, just remove the directory
            output_path.rmdir()

        # Update output_path to point to the actual file for stats
        output_path = dst_mcap

    # Calculate and print statistics
    size_stats = get_size_stats(input_path, output_path, media_dir)
    stats.update(size_stats)
    print_stats(size_stats)
    return stats


def get_size_stats(input_path: Path, output_path: Path, media_dir: Path) -> dict:
    """Calculate file size statistics.

    Returns:
        Dict with input_size, output_size, media_size, total_size (all in bytes)
    """
    if input_path.is_dir():
        input_size = sum(f.stat().st_size for f in input_path.rglob("*") if f.is_file())
    else:
        input_size = input_path.stat().st_size

    if output_path.exists():
        if output_path.is_dir():
            output_size = sum(f.stat().st_size for f in output_path.rglob("*") if f.is_file())
        else:
            output_size = output_path.stat().st_size
    else:
        output_size = 0

    media_size = sum(f.stat().st_size for f in media_dir.glob("*.mp4"))
    total_size = output_size + media_size

    return {
        "input_size": input_size,
        "output_size": output_size,
        "media_size": media_size,
        "total_size": total_size,
        "reduction_percent": (1 - total_size / input_size) * 100 if input_size > 0 else 0,
    }


def print_stats(size_stats: dict):
    """Print file size statistics."""
    mb = 1024 * 1024
    input_size = size_stats["input_size"]
    output_size = size_stats["output_size"]
    media_size = size_stats["media_size"]
    total_size = size_stats["total_size"]

    print(f"\nOriginal: {input_size / mb:.1f} MB")
    print(f"MediaRef bag: {output_size / mb:.1f} MB")
    print(f"Videos: {media_size / mb:.1f} MB")
    print(f"Total: {total_size / mb:.1f} MB ({total_size / input_size * 100:.1f}%)")


def load_config(config_path: Optional[Path]) -> dict:
    """Load processing config from YAML file."""
    if config_path is None:
        return {}

    import yaml

    with open(config_path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(
        description="Convert CostNav ROS2 bags to MediaRef format",
    )
    parser.add_argument("--input", "-i", type=Path, required=True, help="Input bag directory")
    parser.add_argument("--output", "-o", type=Path, help="Output directory (auto if not specified)")
    parser.add_argument("--fps", type=float, default=30.0, help="Video FPS (default: 30.0)")
    parser.add_argument("--config", "-c", type=Path, help="Path to processing config YAML")

    args = parser.parse_args()

    if not args.input.exists():
        print(f"Error: Input not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    # Load config
    config = load_config(args.config)
    fps = config.get("video", {}).get("fps", args.fps)
    image_topics = config.get("image_topics")
    topics_to_remove = config.get("topics_to_remove")

    # Determine output paths - 2-level structure:
    # output_dir/bag_name_mediaref/bag_name.mcap, metadata.yaml, bag_name.media/
    output_dir = args.output or args.input.parent / f"{args.input.name}_mediaref"
    bag_name = args.input.name
    output_path = output_dir / f"{bag_name}.mcap"
    media_dir = output_dir / f"{bag_name}.media"

    if output_path.exists():
        print(f"Error: Output already exists: {output_path}", file=sys.stderr)
        sys.exit(1)

    output_dir.mkdir(parents=True, exist_ok=True)
    media_dir.mkdir(parents=True, exist_ok=True)

    print(f"Input: {args.input}")
    print(f"Output: {output_path}")
    print(f"Media: {media_dir}")
    if image_topics:
        print(f"Image topics: {image_topics}")
    if topics_to_remove:
        print(f"Topics to remove: {topics_to_remove}")

    stats = convert_bag(
        args.input,
        output_path,
        media_dir,
        fps,
        image_topics=image_topics,
        topics_to_remove=topics_to_remove,
    )

    # Write success marker
    success_marker = output_dir / "_SUCCESS"
    success_marker.write_text(f"Conversion completed\nFrames: {stats['frames_written']}\n")
    print(f"\n✓ Conversion complete. Success marker: {success_marker}")


if __name__ == "__main__":
    main()
