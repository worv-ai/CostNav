from typing import Optional

import numpy as np

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2 as pc2
    from std_msgs.msg import Header

    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


def create_pointcloud2_from_array(
    data_array: np.ndarray, frame_id: str = "map", timestamp: Optional["Time"] = None, z_value: float = 0.0
) -> "PointCloud2":
    """Create a PointCloud2 message from a 2D numpy array with shape [N, 2]."""
    if not _ROS_AVAILABLE:
        raise ValueError("ROS is not available. Cannot create PointCloud2.")

    if len(data_array.shape) != 2 or data_array.shape[1] != 2:
        raise ValueError(f"Data array must have shape (N, 2), got {data_array.shape}")

    points_2d = data_array.astype(np.float32)
    num_points = points_2d.shape[0]
    points_3d = np.zeros((num_points, 3), dtype=np.float32)
    points_3d[:, :2] = points_2d
    points_3d[:, 2] = z_value

    point_list = [tuple(point) for point in points_3d]

    header = Header()
    if timestamp is not None:
        header.stamp = timestamp
    else:
        try:
            header.stamp = rclpy.clock.Clock().now().to_msg()
        except Exception:
            header.stamp = Time(sec=0, nanosec=0)
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    return pc2.create_cloud(header, fields, point_list)


def extract_array_from_pointcloud2(cloud_msg: "PointCloud2", include_z: bool = False) -> np.ndarray:
    """Extract a numpy array from a PointCloud2 message."""
    if not _ROS_AVAILABLE:
        raise ValueError("ROS is not available. Cannot extract array.")

    field_names = ("x", "y", "z") if include_z else ("x", "y")
    points = pc2.read_points_list(cloud_msg, field_names=field_names, skip_nans=True)

    if not points:
        return np.array([], dtype=np.float32).reshape(0, len(field_names))

    return np.array(points, dtype=np.float32)
