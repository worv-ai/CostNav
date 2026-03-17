import numpy as np
from numpy.typing import NDArray


def extract_pose_from_msg(msg) -> tuple[NDArray, NDArray]:
    """Extract position and orientation from Odometry message."""
    if hasattr(msg, "twist"):
        pose = msg.pose.pose
    elif hasattr(msg, "pose"):
        pose = msg.pose
    elif hasattr(msg, "position"):
        pose = msg
    else:
        raise ValueError("Invalid message type")

    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return position, orientation


def euler_from_quaternion(quat):
    """Converts quaternion [x, y, z, w] to euler roll, pitch, yaw."""
    quat = np.array(quat)
    quat = quat / np.linalg.norm(quat)
    x, y, z, w = quat

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [*map(float, (roll, pitch, yaw))]
