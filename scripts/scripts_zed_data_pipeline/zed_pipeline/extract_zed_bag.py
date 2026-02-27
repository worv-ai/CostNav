#!/usr/bin/env python3
"""
ZED ROS2 Bag에서 3D GRUT 파이프라인용 스테레오 이미지 추출

출력:
- zed_left/*.jpeg (RGB left)
- zed_right/*.jpeg (RGB right)
- frames_meta.json (cuSFM 형식)
- stereo.edex (stereo calibration)

Usage:
    python scripts/extract_zed_bag.py <bag_path> <output_dir> [--start-time SEC] [--end-time SEC]

Examples:
    # 전체 bag 추출
    python scripts/extract_zed_bag.py /path/to/bag /output/dir

    # 10초~60초 구간만, 3프레임마다 추출
    python scripts/extract_zed_bag.py /path/to/bag /output/dir --start-time 10 --end-time 60 --skip 3
"""

import argparse
import json
import os
import sqlite3
import struct
from pathlib import Path

import cv2
import numpy as np


# ZED Camera Parameters (from camera_info)
ZED_INTRINSICS = {
    "width": 960,
    "height": 600,
    "fx": 361.4733581542969,
    "fy": 361.4733581542969,
    "cx": 479.195068359375,
    "cy": 303.70623779296875,
    "baseline": 0.12,  # 43.38 / 361.47
}

# TF Static transforms (from /tf_static)
# zed_camera_center -> zed_left_camera_frame: (x=-0.01, y=0.06, z=0)
# zed_left_camera_frame -> zed_left_camera_optical_frame: rotation only
# Baseline: left y=+0.06, right y=-0.06, so baseline = 0.12m


def parse_compressed_image(data):
    """Parse sensor_msgs/CompressedImage from CDR data"""
    offset = 4  # CDR header

    # Header: stamp
    sec = struct.unpack('<i', data[offset:offset+4])[0]
    nsec = struct.unpack('<I', data[offset+4:offset+8])[0]
    offset += 8
    timestamp_ns = sec * 1_000_000_000 + nsec

    # frame_id (string)
    str_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4 + str_len
    offset = (offset + 3) // 4 * 4  # align to 4

    # format (string)
    str_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    fmt = data[offset:offset+str_len-1].decode('utf-8')
    offset += str_len
    offset = (offset + 3) // 4 * 4

    # data (sequence<uint8>)
    data_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    img_data = data[offset:offset+data_len]

    return timestamp_ns, fmt, img_data


def parse_compressed_depth(data):
    """Parse compressedDepth format (PNG with header)"""
    offset = 4  # CDR header

    # Header: stamp
    sec = struct.unpack('<i', data[offset:offset+4])[0]
    nsec = struct.unpack('<I', data[offset+4:offset+8])[0]
    offset += 8
    timestamp_ns = sec * 1_000_000_000 + nsec

    # frame_id
    str_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4 + str_len
    offset = (offset + 3) // 4 * 4

    # format
    str_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4 + str_len
    offset = (offset + 3) // 4 * 4

    # data
    data_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    img_data = data[offset:offset+data_len]

    # compressedDepth has 12-byte header before PNG
    # header: depth_quantization (float64) + depth_maximum (float32)
    if len(img_data) > 12:
        depth_quant = struct.unpack('<d', img_data[0:8])[0]
        depth_max = struct.unpack('<f', img_data[8:12])[0]
        png_data = img_data[12:]
        return timestamp_ns, png_data, depth_quant, depth_max

    return timestamp_ns, img_data, 0, 0


def decompress_depth(png_data, depth_quant, depth_max):
    """Decompress depth image from compressedDepth format"""
    # Decode PNG
    img = cv2.imdecode(np.frombuffer(png_data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        return None

    # Convert to float depth (meters)
    if img.dtype == np.uint16:
        # 16-bit depth, typically in mm
        depth_m = img.astype(np.float32) / 1000.0
    else:
        # Quantized depth decompression
        depth_m = depth_quant / (img.astype(np.float32) + 1.0)
        depth_m[img == 0] = 0

    return depth_m


def decode_jpeg(img_data):
    """Decode JPEG image"""
    return cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)


def create_stereo_edex(output_path, intrinsics, num_frames):
    """Create stereo.edex file matching Nova Carter format"""
    # Camera 0: left, Camera 1: right
    # transform: 3x4 matrix [[r11,r12,r13,tx],[r21,r22,r23,ty],[r31,r32,r33,tz]]
    # For left camera at origin (optical frame)
    left_transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0]
    ]

    # Right camera: translated by baseline in x direction (optical frame convention)
    right_transform = [
        [1.0, 0.0, 0.0, intrinsics["baseline"]],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0]
    ]

    edex = [
        {
            "cameras": [
                {
                    "id": 0,
                    "intrinsics": {
                        "distortion_model": "pinhole",
                        "distortion_params": [],
                        "focal": [intrinsics["fx"], intrinsics["fy"]],
                        "principal": [intrinsics["cx"], intrinsics["cy"]],
                        "size": [intrinsics["width"], intrinsics["height"]]
                    },
                    "transform": left_transform
                },
                {
                    "id": 1,
                    "intrinsics": {
                        "distortion_model": "pinhole",
                        "distortion_params": [],
                        "focal": [intrinsics["fx"], intrinsics["fy"]],
                        "principal": [intrinsics["cx"], intrinsics["cy"]],
                        "size": [intrinsics["width"], intrinsics["height"]]
                    },
                    "transform": right_transform
                }
            ],
            "frame_end": num_frames,
            "frame_start": 0,
            "version": "0.9"
        },
        {
            "frame_metadata": "frame_metadata.jsonl",
            "sequence": [
                "zed_left/0000000000.jpg",
                "zed_right/0000000000.jpg"
            ]
        }
    ]

    with open(output_path, 'w') as f:
        json.dump(edex, f, indent=4)
    print(f"Created {output_path}")


def create_frames_meta(output_path, frames, intrinsics):
    """Create frames_meta.json matching Nova Carter format for cuSFM"""
    # Camera params matching Nova Carter calibration_parameters format
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    w, h = intrinsics["width"], intrinsics["height"]
    baseline = intrinsics["baseline"]

    # Camera matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_matrix = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    # Identity rectification (already rectified)
    rectification_matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # Projection matrix for left: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
    projection_matrix_left = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    # Projection matrix for right: includes baseline offset
    projection_matrix_right = [fx, 0, cx, -fx * baseline, 0, fy, cy, 0, 0, 0, 1, 0]

    camera_params = {
        "0": {
            "sensor_meta_data": {
                "sensor_id": 0,
                "sensor_type": "CAMERA",
                "sensor_name": "zed_left",
                "frequency": 30,
                "sensor_to_vehicle_transform": {
                    "axis_angle": {"x": 0, "y": 0, "z": 0, "angle_degrees": 0},
                    "translation": {"x": 0, "y": 0, "z": 0}
                }
            },
            "calibration_parameters": {
                "image_width": w,
                "image_height": h,
                "camera_matrix": {"data": camera_matrix, "row_count": 3, "column_count": 3},
                "distortion_coefficients": {"data": [], "row_count": 1, "column_count": 0},
                "rectification_matrix": {"data": rectification_matrix, "row_count": 3, "column_count": 3},
                "projection_matrix": {"data": projection_matrix_left, "row_count": 3, "column_count": 4}
            }
        },
        "1": {
            "sensor_meta_data": {
                "sensor_id": 1,
                "sensor_type": "CAMERA",
                "sensor_name": "zed_right",
                "frequency": 30,
                "sensor_to_vehicle_transform": {
                    "axis_angle": {"x": 0, "y": 0, "z": 0, "angle_degrees": 0},
                    "translation": {"x": baseline, "y": 0, "z": 0}
                }
            },
            "calibration_parameters": {
                "image_width": w,
                "image_height": h,
                "camera_matrix": {"data": camera_matrix, "row_count": 3, "column_count": 3},
                "distortion_coefficients": {"data": [], "row_count": 1, "column_count": 0},
                "rectification_matrix": {"data": rectification_matrix, "row_count": 3, "column_count": 3},
                "projection_matrix": {"data": projection_matrix_right, "row_count": 3, "column_count": 4}
            }
        }
    }

    # Stereo pair configuration (CRITICAL for Foundation Stereo)
    stereo_pair = [
        {
            "left_camera_param_id": "0",
            "right_camera_param_id": "1",
            "baseline_meters": intrinsics["baseline"]
        }
    ]

    meta = {
        "camera_params_id_to_camera_params": camera_params,
        "keyframes_metadata": frames,
        "stereo_pair": stereo_pair
    }

    with open(output_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"Created {output_path} with {len(frames)} frames")


def create_frame_metadata_jsonl(output_path, synced_frames_info):
    """Create frame_metadata.jsonl for stereo frame synchronization"""
    with open(output_path, 'w') as f:
        for frame_info in synced_frames_info:
            line = json.dumps(frame_info, separators=(',', ':'))
            f.write(line + '\n')
    print(f"Created {output_path} with {len(synced_frames_info)} frames")


def extract_bag(bag_path, output_dir, start_time=None, end_time=None, skip_frames=1):
    """Extract stereo images from ROS2 bag (RGB only, no depth)"""
    bag_path = Path(bag_path)
    output_dir = Path(output_dir)

    # Find db3 file
    if bag_path.is_dir():
        db3_files = list(bag_path.glob("*.db3"))
        if not db3_files:
            raise FileNotFoundError(f"No .db3 file found in {bag_path}")
        db3_path = db3_files[0]
    else:
        db3_path = bag_path

    print(f"Reading: {db3_path}")

    # Create output directories
    left_dir = output_dir / "zed_left"
    right_dir = output_dir / "zed_right"

    for d in [left_dir, right_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # Connect to database
    conn = sqlite3.connect(str(db3_path))
    cursor = conn.cursor()

    # Get topic IDs
    cursor.execute("SELECT id, name FROM topics")
    topics = {name: id for id, name in cursor.fetchall()}

    left_topic_id = topics.get('/zed/zed_node/left/image_rect_color/compressed')
    right_topic_id = topics.get('/zed/zed_node/right/image_rect_color/compressed')

    # Get time range
    cursor.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages")
    bag_start, bag_end = cursor.fetchone()
    bag_start_sec = bag_start / 1e9
    bag_end_sec = bag_end / 1e9

    print(f"Bag duration: {bag_end_sec - bag_start_sec:.2f}s")

    # Apply time filters
    if start_time is not None:
        filter_start = int((bag_start_sec + start_time) * 1e9)
    else:
        filter_start = bag_start

    if end_time is not None:
        filter_end = int((bag_start_sec + end_time) * 1e9)
    else:
        filter_end = bag_end

    print(f"Extracting: {(filter_start - bag_start)/1e9:.2f}s - {(filter_end - bag_start)/1e9:.2f}s")

    # Collect messages by timestamp for synchronization
    left_msgs = {}
    right_msgs = {}

    # Query left images
    cursor.execute(f"""
        SELECT data, timestamp FROM messages
        WHERE topic_id = {left_topic_id}
        AND timestamp >= {filter_start} AND timestamp <= {filter_end}
        ORDER BY timestamp
    """)
    for data, ts in cursor.fetchall():
        ts_ns, fmt, img_data = parse_compressed_image(data)
        left_msgs[ts] = img_data

    # Query right images
    cursor.execute(f"""
        SELECT data, timestamp FROM messages
        WHERE topic_id = {right_topic_id}
        AND timestamp >= {filter_start} AND timestamp <= {filter_end}
        ORDER BY timestamp
    """)
    for data, ts in cursor.fetchall():
        ts_ns, fmt, img_data = parse_compressed_image(data)
        right_msgs[ts] = img_data

    conn.close()

    print(f"Found: {len(left_msgs)} left, {len(right_msgs)} right")

    # Find synchronized timestamps (within 50ms)
    left_times = sorted(left_msgs.keys())
    synced_frames = []

    for i, left_ts in enumerate(left_times):
        if i % skip_frames != 0:
            continue

        # Find closest right
        right_ts = min(right_msgs.keys(), key=lambda x: abs(x - left_ts), default=None)

        if right_ts is None:
            continue

        # Check sync (within 50ms)
        if abs(right_ts - left_ts) > 50_000_000:
            continue

        synced_frames.append((left_ts, right_ts))

    print(f"Synchronized frames: {len(synced_frames)}")

    # Extract and save images
    frames_metadata = []
    frame_metadata_jsonl = []

    for idx, (left_ts, right_ts) in enumerate(synced_frames):
        # Decode images
        left_img = decode_jpeg(left_msgs[left_ts])
        right_img = decode_jpeg(right_msgs[right_ts])

        if left_img is None or right_img is None:
            print(f"  Skipping frame {idx}: decode failed")
            continue

        # Save images
        frame_name = f"{left_ts}"

        left_path = left_dir / f"{frame_name}.jpeg"
        right_path = right_dir / f"{frame_name}.jpeg"

        cv2.imwrite(str(left_path), left_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        cv2.imwrite(str(right_path), right_img, [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Add to metadata (cuSFM format - minimal fields matching Nova Carter)
        # synced_sample_id should start from 1 (not 0) to match Nova Carter format
        sample_id = idx + 1

        # Use LEFT timestamp for both cameras (cuVSLAM requires <1ms sync)
        # The actual image files keep their original timestamps for traceability
        synced_ts_us = left_ts // 1000

        # Left camera keyframe
        frames_metadata.append({
            "id": str(len(frames_metadata)),  # unique id for each keyframe
            "camera_params_id": "0",
            "timestamp_microseconds": str(synced_ts_us),
            "image_name": f"zed_left/{frame_name}.jpeg",
            "synced_sample_id": str(sample_id)
        })

        # Right camera keyframe (same timestamp as left for sync)
        frames_metadata.append({
            "id": str(len(frames_metadata)),  # unique id for each keyframe
            "camera_params_id": "1",
            "timestamp_microseconds": str(synced_ts_us),
            "image_name": f"zed_right/{frame_name}.jpeg",
            "synced_sample_id": str(sample_id)
        })

        # Add to frame_metadata.jsonl (synchronized frame info - same timestamp)
        frame_metadata_jsonl.append({
            "cams": [
                {"filename": f"zed_left/{frame_name}.jpeg", "id": 0, "timestamp": synced_ts_us},
                {"filename": f"zed_right/{frame_name}.jpeg", "id": 1, "timestamp": synced_ts_us}
            ],
            "frame_id": idx
        })

        if (idx + 1) % 50 == 0:
            print(f"  Processed {idx + 1}/{len(synced_frames)} frames")

    # Create metadata files
    num_frames = len(synced_frames)
    create_stereo_edex(output_dir / "stereo.edex", ZED_INTRINSICS, num_frames)
    create_frames_meta(output_dir / "frames_meta.json", frames_metadata, ZED_INTRINSICS)
    create_frame_metadata_jsonl(output_dir / "frame_metadata.jsonl", frame_metadata_jsonl)

    print(f"\nDone! Output: {output_dir}")
    print(f"  - {len(synced_frames)} synchronized stereo pairs")
    print(f"  - {len(frames_metadata)} total keyframes (left + right)")


def main():
    parser = argparse.ArgumentParser(description="Extract ZED ROS2 bag for 3D GRUT pipeline")
    parser.add_argument("bag_path", help="Path to ROS2 bag (directory or .db3 file)")
    parser.add_argument("output_dir", help="Output directory for extracted data")
    parser.add_argument("--start-time", type=float, default=None,
                        help="Start time in seconds from bag start")
    parser.add_argument("--end-time", type=float, default=None,
                        help="End time in seconds from bag start")
    parser.add_argument("--skip", type=int, default=1,
                        help="Process every N-th frame (default: 1 = all frames)")

    args = parser.parse_args()

    extract_bag(
        args.bag_path,
        args.output_dir,
        start_time=args.start_time,
        end_time=args.end_time,
        skip_frames=args.skip
    )


if __name__ == "__main__":
    main()

