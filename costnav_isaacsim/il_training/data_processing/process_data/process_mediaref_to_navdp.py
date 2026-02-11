#!/usr/bin/env python3
"""Convert MediaRef-format bags directly to InternNav NavDP dataset structure.

Input (MediaRef format):
  input_dir/
    recording_XXXX_mediaref/
      metadata.yaml
      recording_XXXX.mcap
      recording_XXXX.media/
        front_stereo_camera_left_image_raw.mp4
        ...

Output (NavDP format):
  output_dir/<group>/<scene>/<traj>/
    videos/chunk-000/observation.images.rgb/0.jpg ...
    videos/chunk-000/observation.images.depth/0.png ...
    data/chunk-000/episode_000000.parquet
    data/chunk-000/path.ply
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml
from mediaref import MediaRef, batch_decode
from PIL import Image
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore

try:
    import pandas as pd
except Exception:  # pragma: no cover - optional
    pd = None

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
except Exception:  # pragma: no cover - optional
    pa = None
    pq = None


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle (rotation around z in radians)."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(t3, t4)


def is_backwards(pos1: np.ndarray, yaw1: float, pos2: np.ndarray, eps: float = 1e-5) -> bool:
    """Check if trajectory is going backwards given position and yaw of two points."""
    dx, dy = pos2 - pos1
    return dx * np.cos(yaw1) + dy * np.sin(yaw1) < eps


def segment_forward(
    positions: np.ndarray,
    yaws: np.ndarray,
    start_slack: int = 0,
    end_slack: int = 0,
) -> List[Tuple[int, int]]:
    """Return list of (start_idx, end_idx) for forward-moving segments."""
    segments: List[Tuple[int, int]] = []
    start_idx: Optional[int] = None
    n = len(positions)
    for i in range(max(start_slack, 1), n - end_slack):
        pos1 = positions[i - 1]
        yaw1 = yaws[i - 1]
        pos2 = positions[i]
        if not is_backwards(pos1, yaw1, pos2):
            if start_idx is None:
                start_idx = i - 1
            if i == n - end_slack - 1:
                segments.append((start_idx, i))
        elif start_idx is not None:
            segments.append((start_idx, i - 1))
            start_idx = None
    return segments


def load_config(path: Optional[Path]) -> dict:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit("ERROR: config must be a YAML mapping")
    return data


def find_mediaref_bags(input_dir: Path) -> List[Path]:
    bags = []
    for item in input_dir.iterdir():
        if item.is_dir() and item.name.endswith("_mediaref"):
            mcap_files = list(item.glob("*.mcap"))
            db3_files = list(item.glob("*.db3"))
            if mcap_files or db3_files:
                bags.append(item)
    return sorted(bags)


def _make_abs_ref(bag_dir: Path, ref: MediaRef) -> MediaRef:
    return MediaRef(uri=str(bag_dir / ref.uri), pts_ns=ref.pts_ns)


def _sync_streams(
    image_refs: List[Tuple[int, MediaRef]],
    odom_data: List[Tuple[int, Tuple[float, float], float]],
    depth_refs: Optional[List[Tuple[int, MediaRef]]],
    rate: float,
) -> Tuple[List[MediaRef], List[Tuple[Tuple[float, float], float]], Optional[List[MediaRef]]]:
    image_refs.sort(key=lambda x: x[0])
    odom_data.sort(key=lambda x: x[0])
    if depth_refs is not None:
        depth_refs.sort(key=lambda x: x[0])

    start_time = max(
        image_refs[0][0],
        odom_data[0][0],
        depth_refs[0][0] if depth_refs else image_refs[0][0],
    )
    end_time = min(
        image_refs[-1][0],
        odom_data[-1][0],
        depth_refs[-1][0] if depth_refs else image_refs[-1][0],
    )
    if start_time >= end_time:
        return [], [], [] if depth_refs is not None else None

    sample_interval_ns = int(1e9 / rate)
    img_idx = 0
    odom_idx = 0
    depth_idx = 0

    synced_imgs: List[MediaRef] = []
    synced_odom: List[Tuple[Tuple[float, float], float]] = []
    synced_depths: List[MediaRef] = []

    current_time = start_time
    while current_time <= end_time:
        while img_idx + 1 < len(image_refs) and image_refs[img_idx + 1][0] <= current_time:
            img_idx += 1
        while odom_idx + 1 < len(odom_data) and odom_data[odom_idx + 1][0] <= current_time:
            odom_idx += 1
        if depth_refs is not None:
            while depth_idx + 1 < len(depth_refs) and depth_refs[depth_idx + 1][0] <= current_time:
                depth_idx += 1

        synced_imgs.append(image_refs[img_idx][1])
        synced_odom.append((odom_data[odom_idx][1], odom_data[odom_idx][2]))
        if depth_refs is not None:
            synced_depths.append(depth_refs[depth_idx][1])

        current_time += sample_interval_ns

    if depth_refs is not None:
        return synced_imgs, synced_odom, synced_depths
    return synced_imgs, synced_odom, None


def get_synced_mediaref(
    bag_dir: Path,
    image_topic: str,
    odom_topic: str,
    depth_topic: Optional[str],
    rate: float,
    ang_offset: float,
) -> Tuple[Optional[List[np.ndarray]], Optional[List[np.ndarray]], Optional[np.ndarray], Optional[np.ndarray]]:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    deserialize = typestore.deserialize_cdr

    image_refs: List[Tuple[int, MediaRef]] = []
    depth_refs: List[Tuple[int, MediaRef]] = []
    odom_data: List[Tuple[int, Tuple[float, float], float]] = []

    with Reader2(bag_dir) as reader:
        topic_names = {conn.topic for conn in reader.connections}
        if image_topic not in topic_names or odom_topic not in topic_names:
            return None, None, None, None
        if depth_topic and depth_topic not in topic_names:
            depth_topic = None

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == image_topic:
                msg = deserialize(rawdata, connection.msgtype)
                if hasattr(msg, "data"):
                    try:
                        ref = MediaRef.model_validate_json(msg.data)
                        image_refs.append((timestamp, ref))
                    except Exception:
                        continue
            elif depth_topic and connection.topic == depth_topic:
                msg = deserialize(rawdata, connection.msgtype)
                if hasattr(msg, "data"):
                    try:
                        ref = MediaRef.model_validate_json(msg.data)
                        depth_refs.append((timestamp, ref))
                    except Exception:
                        continue
            elif connection.topic == odom_topic:
                msg = deserialize(rawdata, connection.msgtype)
                pos = msg.pose.pose.position
                orient = msg.pose.pose.orientation
                yaw = quat_to_yaw(orient.x, orient.y, orient.z, orient.w) + ang_offset
                odom_data.append((timestamp, (pos.x, pos.y), yaw))

    if not image_refs or not odom_data:
        return None, None, None, None

    synced_imgs, synced_odom, synced_depths = _sync_streams(image_refs, odom_data, depth_refs or None, rate)
    if not synced_imgs:
        return None, None, None, None

    abs_img_refs = [_make_abs_ref(bag_dir, ref) for ref in synced_imgs]
    frames = batch_decode(abs_img_refs)

    depth_frames = None
    if depth_topic and synced_depths:
        abs_depth_refs = [_make_abs_ref(bag_dir, ref) for ref in synced_depths]
        try:
            depth_frames = batch_decode(abs_depth_refs)
        except Exception:
            depth_frames = None

    positions = np.array([odom[0] for odom in synced_odom], dtype=np.float64)
    yaws = np.array([odom[1] for odom in synced_odom], dtype=np.float64)
    return list(frames), list(depth_frames) if depth_frames is not None else None, positions, yaws


def _make_intrinsic(width: int, height: int, fx: float | None, fy: float | None, cx: float | None, cy: float | None):
    if fx is None:
        fx = float(max(width, height))
    if fy is None:
        fy = float(fx)
    if cx is None:
        cx = float(width) / 2.0
    if cy is None:
        cy = float(height) / 2.0
    return [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]


def _make_extrinsics(positions: np.ndarray, yaws: np.ndarray, camera_height: float) -> List[List[List[float]]]:
    extrinsics: List[List[List[float]]] = []
    for (x, y), yaw in zip(positions, yaws):
        c = math.cos(yaw)
        s = math.sin(yaw)
        extr = [
            [c, -s, 0.0, float(x)],
            [s, c, 0.0, float(y)],
            [0.0, 0.0, 1.0, float(camera_height)],
            [0.0, 0.0, 0.0, 1.0],
        ]
        extrinsics.append(extr)
    return extrinsics


def _write_parquet(path: Path, intrinsic, base_extrinsic, actions) -> None:
    rows = [
        {
            "observation.camera_intrinsic": intrinsic,
            "observation.camera_extrinsic": base_extrinsic,
            "action": actions[i],
        }
        for i in range(len(actions))
    ]
    if pd is not None:
        df = pd.DataFrame(rows)
        df.to_parquet(path, index=False)
        return
    if pa is not None and pq is not None:
        table = pa.Table.from_pylist(rows)
        pq.write_table(table, path)
        return
    raise SystemExit("ERROR: pandas or pyarrow is required to write parquet files.")


def _write_path_ply(path: Path, positions: np.ndarray, z: float, step: int) -> None:
    pts = positions[::step]
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for x, y in pts:
            f.write(f"{float(x)} {float(y)} {float(z)} 0 0 0\n")


def _save_rgb_images(frames: List[np.ndarray], rgb_dir: Path) -> None:
    for i, frame in enumerate(frames):
        img = Image.fromarray(frame)
        img.save(rgb_dir / f"{i}.jpg", quality=95)


def _save_depth_images(
    depth_frames: Optional[List[np.ndarray]],
    depth_dir: Path,
    width: int,
    height: int,
    depth_m: float,
    depth_scale: float,
) -> None:
    if depth_frames is None:
        depth_val = max(depth_m, 0.1)
        depth_u16 = np.full((height, width), int(depth_val * depth_scale), dtype=np.uint16)
        img = Image.fromarray(depth_u16, mode="I;16")
        return img

    imgs = []
    for frame in depth_frames:
        if frame.ndim == 3:
            frame = frame[:, :, 0]
        if frame.dtype != np.uint16:
            depth = frame.astype(np.float32)
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth_u16 = np.clip(depth * depth_scale, 0, np.iinfo(np.uint16).max).astype(np.uint16)
        else:
            depth_u16 = frame
        if depth_u16.shape[:2] != (height, width):
            depth_u16 = np.array(Image.fromarray(depth_u16).resize((width, height), resample=Image.NEAREST))
        imgs.append(Image.fromarray(depth_u16, mode="I;16"))
    return imgs


def process_bag(
    bag_dir: Path,
    output_root: Path,
    image_topic: str,
    odom_topic: str,
    depth_topic: Optional[str],
    sample_rate: float,
    ang_offset: float,
    group_name: str,
    scene_name: str,
    min_length: int,
    camera_height: float,
    depth_m: float,
    depth_scale: float,
    path_step: int,
    fx: float | None,
    fy: float | None,
    cx: float | None,
    cy: float | None,
    filter_backwards: bool,
    overwrite: bool,
) -> int:
    frames, depth_frames, positions, yaws = get_synced_mediaref(
        bag_dir, image_topic, odom_topic, depth_topic, sample_rate, ang_offset
    )
    if frames is None or positions is None or yaws is None:
        print(f"{bag_dir.name}: missing required topics, skipping")
        return 0

    if filter_backwards:
        segments = segment_forward(positions, yaws)
    else:
        segments = [(0, len(positions) - 1)]

    if not segments:
        print(f"{bag_dir.name}: no forward segments, skipping")
        return 0

    saved = 0
    base_name = bag_dir.name.replace("_mediaref", "")
    for seg_idx, (s, e) in enumerate(segments):
        length = e - s + 1
        if length < min_length:
            continue

        seg_frames = frames[s : e + 1]
        seg_depth = depth_frames[s : e + 1] if depth_frames is not None else None
        seg_pos = positions[s : e + 1]
        seg_yaw = yaws[s : e + 1]

        traj_name = f"{base_name}_{seg_idx}"
        traj_out = output_root / group_name / scene_name / traj_name
        if traj_out.exists() and not overwrite:
            continue

        rgb_dir = traj_out / "videos/chunk-000/observation.images.rgb"
        depth_dir = traj_out / "videos/chunk-000/observation.images.depth"
        data_dir = traj_out / "data/chunk-000"
        rgb_dir.mkdir(parents=True, exist_ok=True)
        depth_dir.mkdir(parents=True, exist_ok=True)
        data_dir.mkdir(parents=True, exist_ok=True)

        _save_rgb_images(seg_frames, rgb_dir)

        width, height = Image.open(rgb_dir / "0.jpg").size
        depth_imgs = _save_depth_images(seg_depth, depth_dir, width, height, depth_m, depth_scale)
        if isinstance(depth_imgs, list):
            for i, img in enumerate(depth_imgs):
                img.save(depth_dir / f"{i}.png")
        else:
            # constant depth image for all frames
            for i in range(length):
                depth_imgs.save(depth_dir / f"{i}.png")

        intrinsic = _make_intrinsic(width, height, fx, fy, cx, cy)
        extrinsics = _make_extrinsics(seg_pos, seg_yaw, camera_height)
        _write_parquet(data_dir / "episode_000000.parquet", intrinsic, extrinsics[0], extrinsics)
        _write_path_ply(data_dir / "path.ply", seg_pos, z=0.0, step=path_step)
        saved += 1

    return saved


def main(args: argparse.Namespace) -> None:
    cfg = load_config(args.config)
    paths_cfg = cfg.get("paths", {})
    proc_cfg = cfg.get("processing", {})
    topics_cfg = cfg.get("topics", {})
    cam_cfg = cfg.get("camera", {})

    input_dir = Path(args.input_dir or paths_cfg.get("input_dir", ""))
    output_dir = Path(args.output_dir or paths_cfg.get("output_dir", ""))
    if not input_dir:
        raise SystemExit("ERROR: input_dir is required")
    if not output_dir:
        raise SystemExit("ERROR: output_dir is required")

    image_topic = topics_cfg.get("image_topic", "/front_stereo_camera/left/image_raw")
    odom_topic = topics_cfg.get("odom_topic", "/chassis/odom")
    depth_topic = topics_cfg.get("depth_topic")

    sample_rate = float(proc_cfg.get("sample_rate", 4.0))
    ang_offset = float(proc_cfg.get("ang_offset", 0.0))
    group_name = proc_cfg.get("group_name", "costnav")
    scene_name_default = proc_cfg.get("scene_name", "")
    min_length = int(proc_cfg.get("min_length", 10))
    camera_height = float(proc_cfg.get("camera_height", 1.0))
    depth_m = float(proc_cfg.get("depth_m", 1.0))
    depth_scale = float(proc_cfg.get("depth_scale", 10000.0))
    path_step = int(proc_cfg.get("path_step", 1))
    filter_backwards = bool(proc_cfg.get("filter_backwards", True))
    overwrite = bool(proc_cfg.get("overwrite", False))

    fx = cam_cfg.get("fx")
    fy = cam_cfg.get("fy")
    cx = cam_cfg.get("cx")
    cy = cam_cfg.get("cy")

    output_dir.mkdir(parents=True, exist_ok=True)
    bag_dirs = find_mediaref_bags(input_dir)
    if args.num_trajs >= 0:
        bag_dirs = bag_dirs[: args.num_trajs]

    if not bag_dirs:
        print(f"No MediaRef bags found in {input_dir}")
        return

    total = len(bag_dirs)
    print(f"Found {total} MediaRef bags")
    saved_total = 0
    for i, bag_dir in enumerate(bag_dirs, 1):
        scene_name = scene_name_default or bag_dir.name.replace("_mediaref", "")
        saved = process_bag(
            bag_dir=bag_dir,
            output_root=output_dir,
            image_topic=image_topic,
            odom_topic=odom_topic,
            depth_topic=depth_topic,
            sample_rate=sample_rate,
            ang_offset=ang_offset,
            group_name=group_name,
            scene_name=scene_name,
            min_length=min_length,
            camera_height=camera_height,
            depth_m=depth_m,
            depth_scale=depth_scale,
            path_step=path_step,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            filter_backwards=filter_backwards,
            overwrite=overwrite,
        )
        saved_total += saved
        print(f"  [{i}/{total}] {bag_dir.name} -> {saved} trajs", flush=True)

    print(f"Converted total trajectories: {saved_total}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert MediaRef format data to NavDP dataset structure")
    parser.add_argument("--input-dir", "-i", type=str, help="MediaRef dataset directory")
    parser.add_argument("--output-dir", "-o", type=str, help="NavDP output root directory")
    parser.add_argument("--config", "-c", type=Path, help="Path to navdp processing config YAML")
    parser.add_argument("--num-trajs", "-n", type=int, default=-1, help="Number of bags to process (-1=all)")
    args = parser.parse_args()
    main(args)
