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
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


def _repo_root_from_file() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "third_party" / "InternNav").exists():
            return parent
    raise RuntimeError("Could not locate repo root (third_party/InternNav not found)")


def _find_depth_anything_checkpoint(explicit_path: Optional[str]) -> Optional[Path]:
    if isinstance(explicit_path, str) and explicit_path.strip():
        path = Path(explicit_path).expanduser()
        return path if path.is_absolute() else _repo_root_from_file() / path

    repo_root = _repo_root_from_file()
    candidates = [
        repo_root / "checkpoints" / "depth_anything_v2_metric_hypersim_vits.pth",
        repo_root / "checkpoints" / "depth_anything_v2_vits.pth",
        repo_root / "third_party" / "InternNav" / "checkpoints" / "depth_anything_v2_metric_hypersim_vits.pth",
        repo_root / "third_party" / "InternNav" / "checkpoints" / "depth_anything_v2_vits.pth",
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


class DepthAnythingEstimator:
    def __init__(
        self,
        checkpoint_path: Path,
        encoder: str = "vits",
        input_size: int = 518,
        max_depth: float = 20.0,
    ) -> None:
        import cv2
        import torch

        repo_root = _repo_root_from_file()
        internnav_dir = repo_root / "third_party" / "InternNav"
        depth_anything_root = internnav_dir / "internnav" / "model" / "encoder" / "depth_anything"
        if str(depth_anything_root) not in sys.path:
            sys.path.insert(0, str(depth_anything_root))
        try:
            # Import DepthAnything package directly to avoid importing full
            # internnav.model.encoder (which pulls unrelated heavy deps).
            from depth_anything_v2.dpt import DepthAnythingV2
        except Exception:
            if str(internnav_dir) not in sys.path:
                sys.path.insert(0, str(internnav_dir))
            from internnav.model.encoder.depth_anything.depth_anything_v2.dpt import DepthAnythingV2

        model_configs = {
            "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
            "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
            "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
            "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
        }
        if encoder not in model_configs:
            raise SystemExit(f"ERROR: Unsupported depth_anything_encoder '{encoder}'")
        if not checkpoint_path.exists():
            raise SystemExit(f"ERROR: DepthAnything checkpoint not found: {checkpoint_path}")

        self.cv2 = cv2
        self.input_size = int(input_size)
        self.max_depth = float(max_depth)

        self.model = DepthAnythingV2(max_depth=self.max_depth, **model_configs[encoder])
        state_dict = torch.load(checkpoint_path, map_location="cpu")
        self.model.load_state_dict(state_dict, strict=False)
        self.model.eval()

        if torch.cuda.is_available():
            self.model = self.model.to("cuda")
        elif torch.backends.mps.is_available():
            self.model = self.model.to("mps")
        else:
            self.model = self.model.to("cpu")

    def infer_batch(self, rgb_frames: List[np.ndarray]) -> List[np.ndarray]:
        depth_frames: List[np.ndarray] = []
        for frame in rgb_frames:
            # DepthAnything infer_image expects BGR uint8 image (OpenCV convention).
            bgr = self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2BGR)
            depth = self.model.infer_image(bgr, input_size=self.input_size).astype(np.float32)
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth[depth < 0.0] = 0.0
            depth_frames.append(depth)
        return depth_frames


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle (rotation around z in radians)."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(t3, t4)


def _quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _norm_frame(frame: Optional[str]) -> str:
    if not frame:
        return ""
    frame = frame.strip()
    return frame[1:] if frame.startswith("/") else frame


def _make_transform_matrix(tx: float, ty: float, tz: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[0:3, 0:3] = _quat_to_rot_matrix(qx, qy, qz, qw)
    t[0:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return t


def _lookup_transform_matrix(
    transforms: Dict[Tuple[str, str], np.ndarray],
    parent_frame: str,
    child_frame: str,
) -> Optional[np.ndarray]:
    """Return T_parent_child from transform dict if available (direct or inverse)."""
    parent_frame = _norm_frame(parent_frame)
    child_frame = _norm_frame(child_frame)
    if not parent_frame or not child_frame:
        return None
    if (parent_frame, child_frame) in transforms:
        return transforms[(parent_frame, child_frame)]
    if (child_frame, parent_frame) in transforms:
        try:
            return np.linalg.inv(transforms[(child_frame, parent_frame)])
        except np.linalg.LinAlgError:
            return None
    return None


POINTFIELD_DTYPES = {
    1: ("i1", 1),  # INT8
    2: ("u1", 1),  # UINT8
    3: ("<i2", 2),  # INT16
    4: ("<u2", 2),  # UINT16
    5: ("<i4", 4),  # INT32
    6: ("<u4", 4),  # UINT32
    7: ("<f4", 4),  # FLOAT32
    8: ("<f8", 8),  # FLOAT64
}


def _extract_xyz_from_pointcloud2(msg) -> np.ndarray:
    """Parse sensor_msgs/PointCloud2 and return Nx3 xyz in message frame."""
    width = int(getattr(msg, "width", 0))
    height = int(getattr(msg, "height", 0))
    point_step = int(getattr(msg, "point_step", 0))
    if width <= 0 or height <= 0 or point_step <= 0:
        return np.empty((0, 3), dtype=np.float32)

    fields = {f.name: f for f in getattr(msg, "fields", [])}
    x_field = fields.get("x")
    y_field = fields.get("y")
    z_field = fields.get("z")
    if x_field is None or y_field is None or z_field is None:
        return np.empty((0, 3), dtype=np.float32)

    if (
        x_field.datatype not in POINTFIELD_DTYPES
        or y_field.datatype not in POINTFIELD_DTYPES
        or z_field.datatype not in POINTFIELD_DTYPES
    ):
        return np.empty((0, 3), dtype=np.float32)

    dtype_x, _ = POINTFIELD_DTYPES[x_field.datatype]
    dtype_y, _ = POINTFIELD_DTYPES[y_field.datatype]
    dtype_z, _ = POINTFIELD_DTYPES[z_field.datatype]
    if bool(getattr(msg, "is_bigendian", False)):
        dtype_x = dtype_x.replace("<", ">")
        dtype_y = dtype_y.replace("<", ">")
        dtype_z = dtype_z.replace("<", ">")

    row_step = int(getattr(msg, "row_step", width * point_step))
    data = memoryview(getattr(msg, "data", b""))
    if len(data) < row_step * height:
        return np.empty((0, 3), dtype=np.float32)

    total = width * height
    xyz = np.empty((total, 3), dtype=np.float32)
    out_idx = 0
    for row in range(height):
        row_base = row * row_step
        for col in range(width):
            base = row_base + col * point_step
            x = np.frombuffer(data, dtype=np.dtype(dtype_x), count=1, offset=base + int(x_field.offset))[0]
            y = np.frombuffer(data, dtype=np.dtype(dtype_y), count=1, offset=base + int(y_field.offset))[0]
            z = np.frombuffer(data, dtype=np.dtype(dtype_z), count=1, offset=base + int(z_field.offset))[0]
            xyz[out_idx, 0] = float(x)
            xyz[out_idx, 1] = float(y)
            xyz[out_idx, 2] = float(z)
            out_idx += 1

    xyz = xyz[:out_idx]
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def _read_static_transforms(
    bag_dir: Path,
    tf_topic: Optional[str],
    tf_static_topic: Optional[str],
) -> Dict[Tuple[str, str], np.ndarray]:
    """Read TF/TF_STATIC and keep latest transform for each parent-child pair."""
    transforms: Dict[Tuple[str, str], np.ndarray] = {}
    topics_to_read = [t for t in [tf_static_topic, tf_topic] if isinstance(t, str) and t.strip()]
    if not topics_to_read:
        return transforms

    typestore = get_typestore(Stores.ROS2_JAZZY)
    deserialize = typestore.deserialize_cdr
    with Reader2(bag_dir) as reader:
        topic_names = {conn.topic for conn in reader.connections}
        topics_to_read = [t for t in topics_to_read if t in topic_names]
        if not topics_to_read:
            return transforms

        for connection, _timestamp, rawdata in reader.messages():
            if connection.topic not in topics_to_read:
                continue
            msg = deserialize(rawdata, connection.msgtype)
            for tr in getattr(msg, "transforms", []):
                parent = _norm_frame(getattr(tr.header, "frame_id", ""))
                child = _norm_frame(getattr(tr, "child_frame_id", ""))
                if not parent or not child:
                    continue
                t = tr.transform.translation
                q = tr.transform.rotation
                transforms[(parent, child)] = _make_transform_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    return transforms


def _read_camera_intrinsic_from_topic(
    bag_dir: Path,
    camera_info_topic: Optional[str],
) -> Optional[Tuple[float, float, float, float]]:
    if not isinstance(camera_info_topic, str) or not camera_info_topic.strip():
        return None

    typestore = get_typestore(Stores.ROS2_JAZZY)
    deserialize = typestore.deserialize_cdr
    with Reader2(bag_dir) as reader:
        topic_names = {conn.topic for conn in reader.connections}
        if camera_info_topic not in topic_names:
            return None

        for connection, _timestamp, rawdata in reader.messages():
            if connection.topic != camera_info_topic:
                continue
            msg = deserialize(rawdata, connection.msgtype)
            k = np.array(getattr(msg, "k", []), dtype=np.float64)
            if k.size == 9 and np.isfinite(k).all() and k[0] > 0.0 and k[4] > 0.0:
                return float(k[0]), float(k[4]), float(k[2]), float(k[5])
            p = np.array(getattr(msg, "p", []), dtype=np.float64)
            if p.size >= 12 and np.isfinite(p).all() and p[0] > 0.0 and p[5] > 0.0:
                return float(p[0]), float(p[5]), float(p[2]), float(p[6])
    return None


def _extract_obstacle_points_from_pointcloud(
    bag_dir: Path,
    pointcloud_topic: Optional[str],
    odom_topic: str,
    ang_offset: float,
    base_frame: str,
    transforms: Dict[Tuple[str, str], np.ndarray],
    downsample: int,
    max_points: int,
    min_range: float,
    max_range: float,
) -> np.ndarray:
    """Extract world-frame obstacle points from PointCloud2 + odom."""
    if not isinstance(pointcloud_topic, str) or not pointcloud_topic.strip():
        return np.empty((0, 3), dtype=np.float32)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    deserialize = typestore.deserialize_cdr
    odom_data: List[Tuple[int, float, float, float]] = []
    obstacle_points: List[np.ndarray] = []

    downsample = max(int(downsample), 1)
    max_points = max(int(max_points), 0)
    min_range = max(float(min_range), 0.0)
    max_range = max(float(max_range), min_range + 1e-3)
    base_frame = _norm_frame(base_frame)

    with Reader2(bag_dir) as reader:
        topic_names = {conn.topic for conn in reader.connections}
        if pointcloud_topic not in topic_names or odom_topic not in topic_names:
            return np.empty((0, 3), dtype=np.float32)

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == odom_topic:
                msg = deserialize(rawdata, connection.msgtype)
                pos = msg.pose.pose.position
                orient = msg.pose.pose.orientation
                yaw = quat_to_yaw(orient.x, orient.y, orient.z, orient.w) + ang_offset
                odom_data.append((timestamp, float(pos.x), float(pos.y), float(yaw)))

    if not odom_data:
        return np.empty((0, 3), dtype=np.float32)
    odom_data.sort(key=lambda x: x[0])

    with Reader2(bag_dir) as reader:
        odom_idx = 0
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != pointcloud_topic:
                continue

            while odom_idx + 1 < len(odom_data) and odom_data[odom_idx + 1][0] <= timestamp:
                odom_idx += 1
            _, ox, oy, oyaw = odom_data[odom_idx]

            msg = deserialize(rawdata, connection.msgtype)
            xyz = _extract_xyz_from_pointcloud2(msg)
            if xyz.size == 0:
                continue

            xyz = xyz[::downsample]
            dist = np.linalg.norm(xyz[:, :2], axis=1)
            xyz = xyz[(dist >= min_range) & (dist <= max_range)]
            if xyz.size == 0:
                continue

            cloud_frame = _norm_frame(getattr(getattr(msg, "header", None), "frame_id", ""))
            if cloud_frame and base_frame and cloud_frame != base_frame:
                t_base_cloud = _lookup_transform_matrix(transforms, base_frame, cloud_frame)
                if t_base_cloud is None:
                    continue
                xyz_h = np.concatenate([xyz[:, :3], np.ones((xyz.shape[0], 1), dtype=np.float32)], axis=1)
                xyz = (xyz_h @ t_base_cloud.T)[:, :3]

            c, s = math.cos(oyaw), math.sin(oyaw)
            world_x = ox + c * xyz[:, 0] - s * xyz[:, 1]
            world_y = oy + s * xyz[:, 0] + c * xyz[:, 1]
            world_z = np.zeros_like(world_x, dtype=np.float32)
            obstacle_points.append(np.stack([world_x, world_y, world_z], axis=1))

    if not obstacle_points:
        return np.empty((0, 3), dtype=np.float32)

    merged = np.concatenate(obstacle_points, axis=0).astype(np.float32)
    if max_points > 0 and merged.shape[0] > max_points:
        stride = max(merged.shape[0] // max_points, 1)
        merged = merged[::stride][:max_points]
    return merged


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
        elif item.is_dir() and item.suffix in {".mcap", ".db3"} and (item / "metadata.yaml").exists():
            bags.append(item)
        elif item.is_file() and item.suffix in {".mcap", ".db3"}:
            bags.append(item)
    return sorted(bags)


def _make_abs_ref(bag_dir: Path, ref: MediaRef) -> MediaRef:
    ref_path = Path(ref.uri)
    if ref_path.is_absolute():
        return ref
    if bag_dir.is_dir() and bag_dir.name.endswith("_mediaref"):
        bag_root = bag_dir
    elif bag_dir.suffix in {".mcap", ".db3"}:
        bag_root = bag_dir.parent
    else:
        bag_root = bag_dir.parent
    return MediaRef(uri=str(bag_root / ref_path), pts_ns=ref.pts_ns)


def _bag_name(bag_path: Path) -> str:
    if bag_path.suffix in {".mcap", ".db3"}:
        name = bag_path.stem
    else:
        name = bag_path.name if bag_path.is_dir() else bag_path.stem
    if name.endswith("_mediaref"):
        name = name[: -len("_mediaref")]
    return name


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


def _make_extrinsics(
    positions: np.ndarray,
    yaws: np.ndarray,
    camera_height: float,
    base_to_camera: Optional[np.ndarray] = None,
) -> List[List[List[float]]]:
    extrinsics: List[List[List[float]]] = []
    for (x, y), yaw in zip(positions, yaws):
        c = math.cos(yaw)
        s = math.sin(yaw)
        t_world_base = np.array(
            [
                [c, -s, 0.0, float(x)],
                [s, c, 0.0, float(y)],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        if base_to_camera is not None:
            t_world_cam = t_world_base @ base_to_camera
        else:
            t_world_cam = t_world_base.copy()
            t_world_cam[2, 3] = float(camera_height)
        extrinsics.append(t_world_cam.tolist())
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


def _write_path_ply(
    path: Path,
    positions: np.ndarray,
    z: float,
    step: int,
    obstacle_points: Optional[np.ndarray] = None,
) -> None:
    pts = positions[::step]
    obs = np.empty((0, 3), dtype=np.float32) if obstacle_points is None else obstacle_points
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(pts) + len(obs)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for x, y in pts:
            f.write(f"{float(x)} {float(y)} {float(z)} 0 0 0\n")
        for x, y, z_obs in obs:
            # Loader expects obstacle color close to [0, 0, 0.5] in [0,1] range -> uchar blue 128.
            f.write(f"{float(x)} {float(y)} {float(z_obs)} 0 0 128\n")


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


def _resolve_depth_frames(
    bag_name: str,
    frames: List[np.ndarray],
    depth_frames_from_topic: Optional[List[np.ndarray]],
    depth_source: str,
    depth_estimator: Optional[DepthAnythingEstimator],
) -> Optional[List[np.ndarray]]:
    if depth_source == "topic":
        if depth_frames_from_topic is None:
            print(f"{bag_name}: depth_source=topic but depth topic is missing/invalid, skipping")
            return None
        return depth_frames_from_topic

    if depth_source == "depth_anything":
        if depth_estimator is None:
            raise SystemExit("ERROR: depth_source=depth_anything requires a valid DepthAnything checkpoint.")
        print(f"{bag_name}: estimating depth from RGB using DepthAnything")
        return depth_estimator.infer_batch(frames)

    if depth_source == "auto":
        if depth_frames_from_topic is not None:
            return depth_frames_from_topic
        if depth_estimator is not None:
            print(f"{bag_name}: depth topic not found, estimating depth from RGB using DepthAnything")
            return depth_estimator.infer_batch(frames)
        raise SystemExit(
            f"ERROR: {bag_name}: depth_source=auto but no usable depth source. "
            "Depth topic is missing and DepthAnything checkpoint is not available. "
            "Set costnav.depth_topic or configure processing.depth_anything_checkpoint."
        )

    if depth_source == "constant":
        return None

    raise SystemExit(f"ERROR: Unsupported depth_source '{depth_source}'")


def process_bag(
    bag_dir: Path,
    output_root: Path,
    image_topic: str,
    odom_topic: str,
    depth_topic: Optional[str],
    camera_info_topic: Optional[str],
    pointcloud_topic: Optional[str],
    tf_topic: Optional[str],
    tf_static_topic: Optional[str],
    base_frame: str,
    camera_frame: Optional[str],
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
    depth_source: str,
    depth_estimator: Optional[DepthAnythingEstimator],
    use_camera_info_intrinsic: bool,
    use_tf_extrinsic: bool,
    obstacle_downsample: int,
    obstacle_max_points: int,
    obstacle_min_range: float,
    obstacle_max_range: float,
) -> int:
    frames, depth_frames, positions, yaws = get_synced_mediaref(
        bag_dir, image_topic, odom_topic, depth_topic, sample_rate, ang_offset
    )
    if frames is None or positions is None or yaws is None:
        print(f"{bag_dir.name}: missing required topics, skipping")
        return 0

    transforms = _read_static_transforms(bag_dir, tf_topic=tf_topic, tf_static_topic=tf_static_topic)
    base_to_camera = None
    if use_tf_extrinsic and isinstance(camera_frame, str) and camera_frame.strip():
        base_to_camera = _lookup_transform_matrix(
            transforms,
            parent_frame=base_frame,
            child_frame=camera_frame,
        )
        if base_to_camera is None:
            print(
                f"{bag_dir.name}: TF transform {base_frame} -> {camera_frame} not found, "
                f"falling back to camera_height={camera_height}"
            )

    intrinsic_from_topic = None
    if use_camera_info_intrinsic and (fx is None or fy is None or cx is None or cy is None):
        intrinsic_from_topic = _read_camera_intrinsic_from_topic(bag_dir, camera_info_topic)
        if intrinsic_from_topic is not None:
            print(f"{bag_dir.name}: using CameraInfo intrinsic from topic {camera_info_topic}")
        elif camera_info_topic:
            print(
                f"{bag_dir.name}: camera_info_topic '{camera_info_topic}' not found/invalid, using fallback intrinsic"
            )

    obstacle_points_world = _extract_obstacle_points_from_pointcloud(
        bag_dir=bag_dir,
        pointcloud_topic=pointcloud_topic,
        odom_topic=odom_topic,
        ang_offset=ang_offset,
        base_frame=base_frame,
        transforms=transforms,
        downsample=obstacle_downsample,
        max_points=obstacle_max_points,
        min_range=obstacle_min_range,
        max_range=obstacle_max_range,
    )

    if filter_backwards:
        segments = segment_forward(positions, yaws)
    else:
        segments = [(0, len(positions) - 1)]

    if not segments:
        print(f"{bag_dir.name}: no forward segments, skipping")
        return 0

    depth_frames_resolved = _resolve_depth_frames(
        bag_name=bag_dir.name,
        frames=frames,
        depth_frames_from_topic=depth_frames,
        depth_source=depth_source,
        depth_estimator=depth_estimator,
    )
    if depth_source == "topic" and depth_frames_resolved is None:
        return 0

    saved = 0
    base_name = _bag_name(bag_dir)
    for seg_idx, (s, e) in enumerate(segments):
        length = e - s + 1
        if length < min_length:
            continue

        seg_frames = frames[s : e + 1]
        seg_depth = depth_frames_resolved[s : e + 1] if depth_frames_resolved is not None else None
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

        fx_eff = fx if fx is not None else (intrinsic_from_topic[0] if intrinsic_from_topic is not None else None)
        fy_eff = fy if fy is not None else (intrinsic_from_topic[1] if intrinsic_from_topic is not None else None)
        cx_eff = cx if cx is not None else (intrinsic_from_topic[2] if intrinsic_from_topic is not None else None)
        cy_eff = cy if cy is not None else (intrinsic_from_topic[3] if intrinsic_from_topic is not None else None)
        intrinsic = _make_intrinsic(width, height, fx_eff, fy_eff, cx_eff, cy_eff)
        extrinsics = _make_extrinsics(seg_pos, seg_yaw, camera_height, base_to_camera=base_to_camera)
        _write_parquet(data_dir / "episode_000000.parquet", intrinsic, extrinsics[0], extrinsics)

        seg_obstacles = np.empty((0, 3), dtype=np.float32)
        if obstacle_points_world.size > 0:
            x_min, x_max = float(seg_pos[:, 0].min()) - 2.0, float(seg_pos[:, 0].max()) + 2.0
            y_min, y_max = float(seg_pos[:, 1].min()) - 2.0, float(seg_pos[:, 1].max()) + 2.0
            mask = (
                (obstacle_points_world[:, 0] >= x_min)
                & (obstacle_points_world[:, 0] <= x_max)
                & (obstacle_points_world[:, 1] >= y_min)
                & (obstacle_points_world[:, 1] <= y_max)
            )
            seg_obstacles = obstacle_points_world[mask]

        _write_path_ply(data_dir / "path.ply", seg_pos, z=0.0, step=path_step, obstacle_points=seg_obstacles)
        saved += 1

    return saved


def main(args: argparse.Namespace) -> None:
    cfg = load_config(args.config)
    paths_cfg = cfg.get("paths", {})
    proc_cfg = cfg.get("processing", {})
    topics_cfg = cfg.get("topics", {})
    costnav_cfg = cfg.get("costnav")
    if isinstance(costnav_cfg, dict):
        # Allow ViNT-style configs to override topic settings
        topics_cfg = {**topics_cfg, **costnav_cfg}
    cam_cfg = cfg.get("camera", {})

    input_dir_str = args.input_dir or paths_cfg.get("input_dir")
    output_dir_str = args.output_dir or paths_cfg.get("output_dir")
    if not input_dir_str:
        raise SystemExit("ERROR: input_dir is required")
    if not output_dir_str:
        raise SystemExit("ERROR: output_dir is required")
    input_dir = Path(input_dir_str).expanduser()
    output_dir = Path(output_dir_str).expanduser()
    if not input_dir.exists():
        raise SystemExit(f"ERROR: input_dir does not exist: {input_dir}")

    image_topic = topics_cfg.get("image_topic", "/front_stereo_camera/left/image_raw")
    odom_topic = topics_cfg.get("odom_topic", "/chassis/odom")
    depth_topic = topics_cfg.get("depth_topic")
    camera_info_topic = topics_cfg.get("camera_info_topic")
    pointcloud_topic = topics_cfg.get("pointcloud_topic")
    tf_topic = topics_cfg.get("tf_topic", "/tf")
    tf_static_topic = topics_cfg.get("tf_static_topic", "/tf_static")
    base_frame = topics_cfg.get("base_frame", "base_link")
    camera_frame = topics_cfg.get("camera_frame")

    sample_rate = float(proc_cfg.get("sample_rate", 4.0))
    ang_offset = proc_cfg.get("ang_offset", 0.0)
    if isinstance(costnav_cfg, dict) and "ang_offset" in costnav_cfg:
        ang_offset = costnav_cfg["ang_offset"]
    ang_offset = float(ang_offset)
    group_name = proc_cfg.get("group_name", "costnav")
    scene_name_default = proc_cfg.get("scene_name", "")
    min_length = int(proc_cfg.get("min_length", 10))
    camera_height = float(proc_cfg.get("camera_height", 1.0))
    depth_m = float(proc_cfg.get("depth_m", 1.0))
    depth_scale = float(proc_cfg.get("depth_scale", 10000.0))
    depth_source = str(proc_cfg.get("depth_source", "auto")).strip().lower()
    depth_anything_checkpoint_cfg = proc_cfg.get("depth_anything_checkpoint")
    depth_anything_input_size = int(proc_cfg.get("depth_anything_input_size", 518))
    depth_anything_max_depth = float(proc_cfg.get("depth_anything_max_depth", 20.0))
    depth_anything_encoder = str(proc_cfg.get("depth_anything_encoder", "vits")).strip().lower()
    path_step = int(proc_cfg.get("path_step", 1))
    filter_backwards = bool(proc_cfg.get("filter_backwards", True))
    overwrite = bool(proc_cfg.get("overwrite", False))
    use_camera_info_intrinsic = bool(proc_cfg.get("use_camera_info_intrinsic", True))
    use_tf_extrinsic = bool(proc_cfg.get("use_tf_extrinsic", True))
    obstacle_downsample = int(proc_cfg.get("obstacle_downsample", 8))
    obstacle_max_points = int(proc_cfg.get("obstacle_max_points", 30000))
    obstacle_min_range = float(proc_cfg.get("obstacle_min_range", 0.2))
    obstacle_max_range = float(proc_cfg.get("obstacle_max_range", 30.0))

    fx = cam_cfg.get("fx")
    fy = cam_cfg.get("fy")
    cx = cam_cfg.get("cx")
    cy = cam_cfg.get("cy")

    depth_estimator: Optional[DepthAnythingEstimator] = None
    if depth_source not in {"topic", "depth_anything", "constant", "auto"}:
        raise SystemExit("ERROR: processing.depth_source must be one of: topic, depth_anything, constant, auto")
    print(f"Depth source mode: {depth_source}")

    depth_anything_ckpt = _find_depth_anything_checkpoint(depth_anything_checkpoint_cfg)
    if depth_source in {"depth_anything", "auto"} and depth_anything_ckpt is not None:
        print(f"Initializing DepthAnything from checkpoint: {depth_anything_ckpt}")
        depth_estimator = DepthAnythingEstimator(
            checkpoint_path=depth_anything_ckpt,
            encoder=depth_anything_encoder,
            input_size=depth_anything_input_size,
            max_depth=depth_anything_max_depth,
        )
    elif depth_source == "depth_anything":
        raise SystemExit(
            "ERROR: depth_source=depth_anything but no checkpoint found. "
            "Set processing.depth_anything_checkpoint or place checkpoint in checkpoints/."
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    bag_dirs = find_mediaref_bags(input_dir)

    start_index = int(args.start_index)
    end_index = int(args.end_index)
    if start_index < 0:
        raise SystemExit("ERROR: --start-index must be >= 0")
    if end_index >= 0 and end_index < start_index:
        raise SystemExit("ERROR: --end-index must be >= --start-index (or -1 for no limit)")

    if start_index > 0 or end_index >= 0:
        bag_dirs = bag_dirs[start_index : end_index if end_index >= 0 else None]
        print(f"Resume window enabled: start_index={start_index}, end_index={'end' if end_index < 0 else end_index}")

    shard_id = int(args.shard_id)
    num_shards = int(args.num_shards)
    if num_shards < 1:
        raise SystemExit("ERROR: --num-shards must be >= 1")
    if shard_id < 0 or shard_id >= num_shards:
        raise SystemExit(f"ERROR: --shard-id must be in [0, {num_shards - 1}]")

    if num_shards > 1:
        bag_dirs = bag_dirs[shard_id::num_shards]
        print(f"Shard mode enabled: shard {shard_id + 1}/{num_shards}")

    if args.num_trajs >= 0:
        bag_dirs = bag_dirs[: args.num_trajs]

    if not bag_dirs:
        print(f"No MediaRef bags found in {input_dir}")
        return

    total = len(bag_dirs)
    print(f"Found {total} MediaRef bags")
    saved_total = 0
    for i, bag_dir in enumerate(bag_dirs, 1):
        scene_name = scene_name_default or _bag_name(bag_dir)
        saved = process_bag(
            bag_dir=bag_dir,
            output_root=output_dir,
            image_topic=image_topic,
            odom_topic=odom_topic,
            depth_topic=depth_topic,
            camera_info_topic=camera_info_topic,
            pointcloud_topic=pointcloud_topic,
            tf_topic=tf_topic,
            tf_static_topic=tf_static_topic,
            base_frame=base_frame,
            camera_frame=camera_frame,
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
            depth_source=depth_source,
            depth_estimator=depth_estimator,
            use_camera_info_intrinsic=use_camera_info_intrinsic,
            use_tf_extrinsic=use_tf_extrinsic,
            obstacle_downsample=obstacle_downsample,
            obstacle_max_points=obstacle_max_points,
            obstacle_min_range=obstacle_min_range,
            obstacle_max_range=obstacle_max_range,
        )
        saved_total += saved
        print(f"  [{i}/{total}] {_bag_name(bag_dir)} -> {saved} trajs", flush=True)

    print(f"Converted total trajectories: {saved_total}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert MediaRef format data to NavDP dataset structure")
    parser.add_argument("--input-dir", "-i", type=str, help="MediaRef dataset directory")
    parser.add_argument("--output-dir", "-o", type=str, help="NavDP output root directory")
    parser.add_argument("--config", "-c", type=Path, help="Path to navdp processing config YAML")
    parser.add_argument("--num-trajs", "-n", type=int, default=-1, help="Number of bags to process (-1=all)")
    parser.add_argument("--start-index", type=int, default=0, help="Start index in sorted bag list (for resume)")
    parser.add_argument("--end-index", type=int, default=-1, help="End index (exclusive), -1 means no limit")
    parser.add_argument("--shard-id", type=int, default=0, help="Shard index for parallel conversion")
    parser.add_argument("--num-shards", type=int, default=1, help="Total number of conversion shards")
    args = parser.parse_args()
    main(args)
