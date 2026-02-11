#!/usr/bin/env python3
"""Convert MediaRef format data to ViNT training format.

This script processes MediaRef-formatted ROS2 bags (created by rosbag_to_mediaref.py)
and converts them to the ViNT training format with trajectory folders containing
numbered images and traj_data.pkl files.

Adapted from visualnav-transformer/train/process_bags.py.

Example:
    python process_mediaref_bags.py \
        --input-dir data/sample_rosbags/ \
        --output-dir data/vint_format/ \
        --config il_training/data_processing/configs/vint_processing_config.yaml

Expected Input Structure (MediaRef format):
    input_dir/
    └── recording_*_mediaref/
        ├── metadata.yaml
        ├── recording_*.mcap          # Trajectory data with MediaRef strings
        └── recording_*.media/        # Video files
            └── front_stereo_camera_left_image_raw.mp4

Expected Output Structure (ViNT format):
    output_dir/
    └── trajectory_name_X/
        ├── 0.jpg, 1.jpg, ...        # Extracted images
        └── traj_data.pkl             # Position and yaw data
"""

from __future__ import annotations

import argparse
import pickle
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torchvision.transforms.functional as TF
import yaml
from mediaref import MediaRef, batch_decode
from PIL import Image
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore

# Image processing constants (from visualnav-transformer)
IMAGE_SIZE = (160, 120)
IMAGE_ASPECT_RATIO = 4 / 3


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle (rotation around z in radians)."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(t3, t4)


def is_backwards(pos1: np.ndarray, yaw1: float, pos2: np.ndarray, eps: float = 1e-5) -> bool:
    """Check if trajectory is going backwards given position and yaw of two points."""
    dx, dy = pos2 - pos1
    return dx * np.cos(yaw1) + dy * np.sin(yaw1) < eps


def filter_backwards(
    img_list: List[Image.Image],
    traj_data: Dict[str, np.ndarray],
    start_slack: int = 0,
    end_slack: int = 0,
) -> List[Tuple[List[Image.Image], Dict[str, np.ndarray]]]:
    """Cut out non-positive velocity (backwards) segments of the trajectory.

    Args:
        img_list: List of PIL images
        traj_data: Dictionary with 'position' and 'yaw' arrays
        start_slack: Points to ignore at trajectory start
        end_slack: Points to ignore at trajectory end

    Returns:
        List of (images, traj_data) tuples for forward-moving segments
    """
    traj_pos = traj_data["position"]
    traj_yaws = traj_data["yaw"]
    cut_trajs = []
    start = True

    def process_pair(traj_pair: list) -> Tuple[List, Dict]:
        new_img_list, new_traj_data = zip(*traj_pair)
        new_traj_data = np.array(new_traj_data)
        return (list(new_img_list), {"position": new_traj_data[:, :2], "yaw": new_traj_data[:, 2]})

    for i in range(max(start_slack, 1), len(traj_pos) - end_slack):
        pos1 = traj_pos[i - 1]
        yaw1 = traj_yaws[i - 1]
        pos2 = traj_pos[i]
        if not is_backwards(pos1, yaw1, pos2):
            if start:
                new_traj_pairs = [(img_list[i - 1], [*traj_pos[i - 1], traj_yaws[i - 1]])]
                start = False
            elif i == len(traj_pos) - end_slack - 1:
                cut_trajs.append(process_pair(new_traj_pairs))
            else:
                new_traj_pairs.append((img_list[i - 1], [*traj_pos[i - 1], traj_yaws[i - 1]]))
        elif not start:
            cut_trajs.append(process_pair(new_traj_pairs))
            start = True
    return cut_trajs


def process_image(image_array: np.ndarray) -> Image.Image:
    """Process image array to ViNT format (160x120 with 4:3 aspect ratio)."""
    pil_image = Image.fromarray(image_array)
    w, h = pil_image.size
    # Center crop to 4:3 aspect ratio
    pil_image = TF.center_crop(pil_image, (h, int(h * IMAGE_ASPECT_RATIO)))
    # Resize to target size
    pil_image = pil_image.resize(IMAGE_SIZE)
    return pil_image


def load_config(config_path: Optional[Path]) -> dict:
    """Load processing configuration from YAML file."""
    if config_path is None:
        return {}
    with open(config_path) as f:
        return yaml.safe_load(f)


def find_mediaref_bags(input_dir: Path) -> List[Path]:
    """Find all MediaRef-format bag directories in input directory."""
    bags = []
    for item in input_dir.iterdir():
        if item.is_dir() and item.name.endswith("_mediaref"):
            # Look for mcap or db3 files (rosbags supports both)
            mcap_files = list(item.glob("*.mcap"))
            db3_files = list(item.glob("*.db3"))
            if mcap_files or db3_files:
                bags.append(item)
    return sorted(bags)


def get_images_and_odom_from_mediaref(
    bag_dir: Path,
    image_topic: str,
    odom_topic: str,
    rate: float = 4.0,
    ang_offset: float = 0.0,
) -> Tuple[Optional[List[Image.Image]], Optional[Dict[str, np.ndarray]]]:
    """Extract synchronized images and odometry from MediaRef bag.

    Args:
        bag_dir: Path to MediaRef bag directory (containing .mcap or .db3 files)
        image_topic: Topic name for image data (stored as MediaRef strings)
        odom_topic: Topic name for odometry data
        rate: Sampling rate in Hz
        ang_offset: Angle offset to add to yaw

    Returns:
        Tuple of (list of PIL images, dict with 'position' and 'yaw' arrays)
        Returns (None, None) if required topics are not found
    """
    typestore = get_typestore(Stores.ROS2_JAZZY)
    deserialize = typestore.deserialize_cdr

    # First pass: collect all messages and identify topics
    image_refs: List[Tuple[int, MediaRef]] = []  # (timestamp_ns, MediaRef)
    odom_data: List[Tuple[int, Tuple[float, float], float]] = []  # (ts, (x, y), yaw)

    with Reader2(bag_dir) as reader:
        # Check if required topics exist
        topic_names = {conn.topic for conn in reader.connections}
        if image_topic not in topic_names or odom_topic not in topic_names:
            return None, None

        # Read all messages
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == image_topic:
                # MediaRef stored as std_msgs/String
                msg = deserialize(rawdata, connection.msgtype)
                if hasattr(msg, "data"):
                    try:
                        ref = MediaRef.model_validate_json(msg.data)
                        image_refs.append((timestamp, ref))
                    except Exception:
                        continue
            elif connection.topic == odom_topic:
                msg = deserialize(rawdata, connection.msgtype)
                pos = msg.pose.pose.position
                orient = msg.pose.pose.orientation
                yaw = quat_to_yaw(orient.x, orient.y, orient.z, orient.w) + ang_offset
                odom_data.append((timestamp, (pos.x, pos.y), yaw))

    if not image_refs or not odom_data:
        return None, None

    # Sort by timestamp
    image_refs.sort(key=lambda x: x[0])
    odom_data.sort(key=lambda x: x[0])

    # Synchronize at the specified rate
    sample_interval_ns = int(1e9 / rate)
    start_time = max(image_refs[0][0], odom_data[0][0])
    end_time = min(image_refs[-1][0], odom_data[-1][0])

    synced_refs: List[MediaRef] = []
    synced_odom: List[Tuple[Tuple[float, float], float]] = []

    # Helper to find closest message before timestamp
    def find_closest_before(data_list, target_ts):
        for i in range(len(data_list) - 1, -1, -1):
            if data_list[i][0] <= target_ts:
                return data_list[i]
        return None

    current_time = start_time
    while current_time <= end_time:
        img_match = find_closest_before(image_refs, current_time)
        odom_match = find_closest_before(odom_data, current_time)

        if img_match and odom_match:
            synced_refs.append(img_match[1])
            synced_odom.append((odom_match[1], odom_match[2]))

        current_time += sample_interval_ns

    if not synced_refs:
        return None, None

    # Batch decode images with absolute paths
    absolute_refs = []
    for ref in synced_refs:
        abs_uri = str(bag_dir / ref.uri)
        absolute_refs.append(MediaRef(uri=abs_uri, pts_ns=ref.pts_ns))

    # Decode images
    try:
        frames = batch_decode(absolute_refs)
    except Exception as e:
        print(f"Error decoding images: {e}")
        return None, None

    # Process images to ViNT format
    images = [process_image(frame) for frame in frames]

    # Build trajectory data
    positions = np.array([odom[0] for odom in synced_odom])
    yaws = np.array([odom[1] for odom in synced_odom])
    traj_data = {"position": positions, "yaw": yaws}

    return images, traj_data


def process_mediaref_bag(
    bag_dir: Path,
    output_dir: Path,
    config: dict,
    sample_rate: float = 4.0,
) -> bool:
    """Process a single MediaRef bag to ViNT format.

    Args:
        bag_dir: Path to MediaRef bag directory
        output_dir: Output directory for ViNT format data
        config: Processing configuration
        sample_rate: Sampling rate in Hz

    Returns:
        True if processing succeeded, False otherwise
    """
    # Get config values
    image_topic = config.get("image_topic", "/front_stereo_camera/left/image_raw")
    odom_topic = config.get("odom_topic", "/chassis/odom")
    ang_offset = config.get("ang_offset", 0.0)

    # Generate trajectory name
    traj_name = bag_dir.name.replace("_mediaref", "")

    # Extract images and odometry
    img_data, traj_data = get_images_and_odom_from_mediaref(
        bag_dir=bag_dir,
        image_topic=image_topic,
        odom_topic=odom_topic,
        rate=sample_rate,
        ang_offset=ang_offset,
    )

    if img_data is None or traj_data is None:
        print(f"{bag_dir.name}: Required topics not found. Skipping...")
        return False

    # Filter out backward movements
    cut_trajs = filter_backwards(img_data, traj_data)

    if not cut_trajs:
        print(f"{bag_dir.name}: No forward trajectories found. Skipping...")
        return False

    # Skip segments that are too short for ViNT training
    # ViNT requires: context_size * waypoint_spacing + end_slack + len_traj_pred * waypoint_spacing + 1
    # With defaults (context_size=5, waypoint_spacing=1, end_slack=3, len_traj_pred=5): 5 + 3 + 5 + 1 = 14
    min_segment_length = 14
    saved_count = 0
    for i, (img_data_i, traj_data_i) in enumerate(cut_trajs):
        if len(img_data_i) < min_segment_length:
            continue

        traj_name_i = f"{traj_name}_{saved_count}"
        traj_folder_i = output_dir / traj_name_i

        traj_folder_i.mkdir(parents=True, exist_ok=True)

        # Save trajectory data
        with open(traj_folder_i / "traj_data.pkl", "wb") as f:
            pickle.dump(traj_data_i, f)

        # Save images
        for j, img in enumerate(img_data_i):
            img.save(traj_folder_i / f"{j}.jpg")

        saved_count += 1

    return saved_count > 0


def main(args: argparse.Namespace):
    """Main processing function."""
    # Load config
    config = load_config(args.config)

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find all MediaRef bags
    input_dir = Path(args.input_dir)
    bag_dirs = find_mediaref_bags(input_dir)

    if args.num_trajs >= 0:
        bag_dirs = bag_dirs[: args.num_trajs]

    if not bag_dirs:
        print(f"No MediaRef bags found in {input_dir}")
        return

    print(f"Found {len(bag_dirs)} MediaRef bags to process")

    # Process each bag
    success_count = 0
    total = len(bag_dirs)
    t0 = time.time()
    for i, bag_dir in enumerate(bag_dirs, 1):
        if process_mediaref_bag(bag_dir, output_dir, config, args.sample_rate):
            success_count += 1
        elapsed = time.time() - t0
        avg = elapsed / i
        eta = avg * (total - i)

        def _fmt_duration(secs: float) -> str:
            """Format seconds as human-readable (Xd )HH:MM:SS."""
            s = int(secs)
            days, s = divmod(s, 86400)
            hours, s = divmod(s, 3600)
            minutes, s = divmod(s, 60)
            if days > 0:
                return f"{days}d {hours:02d}:{minutes:02d}:{s:02d}"
            return f"{hours:02d}:{minutes:02d}:{s:02d}"

        print(
            f"  [{i}/{total}] Processed: {bag_dir.name}  (elapsed {_fmt_duration(elapsed)}, ETA {_fmt_duration(eta)})",
            flush=True,
        )

    print(f"\nProcessed {success_count}/{total} bags successfully")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert MediaRef format data to ViNT training format")
    parser.add_argument(
        "--input-dir",
        "-i",
        type=str,
        help="Directory containing MediaRef bags (recording_*_mediaref/) - can be set in config",
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=str,
        help="Output directory for ViNT format data - can be set in config",
    )
    parser.add_argument(
        "--config",
        "-c",
        type=Path,
        help="Path to processing configuration YAML",
    )
    parser.add_argument(
        "--num-trajs",
        "-n",
        type=int,
        default=-1,
        help="Number of bags to process (-1 for all)",
    )
    parser.add_argument(
        "--sample-rate",
        "-s",
        type=float,
        help="Sampling rate in Hz (can be set in config, default: 4.0)",
    )

    args = parser.parse_args()

    # Load config to get paths if not provided via command line
    config = load_config(args.config)
    paths_config = config.get("paths", {})

    # Override with config values if command-line args not provided
    if args.input_dir is None:
        args.input_dir = paths_config.get("input_dir")
    if args.output_dir is None:
        args.output_dir = paths_config.get("output_dir", "./vint_datasets/")
    if args.sample_rate is None:
        args.sample_rate = config.get("processing", {}).get("sample_rate", 4.0)

    # Validate required paths
    if args.input_dir is None:
        print("Error: input_dir must be specified via --input-dir or in config file")
        sys.exit(1)
    if args.output_dir is None:
        print("Error: output_dir must be specified via --output-dir or in config file")
        sys.exit(1)

    print("STARTING PROCESSING MEDIAREF DATASETS")
    main(args)
    print("FINISHED PROCESSING MEDIAREF DATASETS")
