#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""NavDP ROS2 Policy Node for CostNav (local inference, no server).

Supports point-goal, image-goal, pixel-goal, and no-goal navigation using
NavDP model with RGB-D observations. Publishes trajectory as nav_msgs/Path.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path as FilePath
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from transforms3d.euler import quat2euler, euler2quat

from il_evaluation.agents.navdp_agent import NavDPAgent


def _find_repo_root() -> FilePath:
    here = FilePath(__file__).resolve()
    for parent in here.parents:
        if (parent / "third_party" / "InternNav").exists():
            return parent
    raise RuntimeError("Could not locate repo root (third_party/InternNav not found)")


class DepthAnythingEstimator:
    """Lightweight DepthAnything runtime wrapper for eval-time RGB->depth inference."""

    def __init__(
        self,
        checkpoint: str,
        encoder: str = "vitb",
        input_size: int = 518,
        max_depth: float = 20.0,
        device: str = "cuda:0",
    ) -> None:
        import cv2
        import torch

        checkpoint_path = FilePath(checkpoint).expanduser()
        if not checkpoint_path.exists():
            raise FileNotFoundError(f"DepthAnything checkpoint not found: {checkpoint_path}")

        repo_root = _find_repo_root()
        internnav_dir = repo_root / "third_party" / "InternNav"
        depth_anything_root = internnav_dir / "internnav" / "model" / "encoder" / "depth_anything"
        if str(depth_anything_root) not in sys.path:
            sys.path.insert(0, str(depth_anything_root))
        try:
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
            raise ValueError(f"Unsupported depth_anything encoder: {encoder}")

        self.cv2 = cv2
        self.input_size = int(input_size)

        model = DepthAnythingV2(max_depth=float(max_depth), **model_configs[encoder])
        state_dict = torch.load(str(checkpoint_path), map_location="cpu")
        model.load_state_dict(state_dict, strict=False)
        model.eval()

        # Respect requested device when possible; otherwise fall back gracefully.
        requested = str(device)
        if requested.startswith("cuda") and torch.cuda.is_available():
            model = model.to(requested)
        elif requested.startswith("mps") and hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            model = model.to("mps")
        elif torch.cuda.is_available():
            model = model.to("cuda")
        else:
            model = model.to("cpu")
        self.model = model

    def infer(self, rgb_image: np.ndarray) -> np.ndarray:
        # DepthAnything expects BGR uint8 image (OpenCV convention).
        bgr = self.cv2.cvtColor(rgb_image, self.cv2.COLOR_RGB2BGR)
        depth = self.model.infer_image(bgr, input_size=self.input_size).astype(np.float32)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth[depth < 0.0] = 0.0
        return depth


class NavDPPolicyNode(Node):
    """ROS2 Node for NavDP policy inference."""

    def __init__(
        self,
        checkpoint: str,
        depth_anything_checkpoint: Optional[str] = None,
        inference_rate: float = 10.0,
        image_topic: str = "/front_stereo_camera/left/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        odom_topic: str = "/chassis/odom",
        goal_pose_topic: str = "/goal_pose",
        goal_image_topic: str = "/goal_image",
        pixel_goal_topic: str = "/goal_pixel",
        trajectory_topic: str = "/navdp_trajectory",
        enable_topic: str = "/navdp_enable",
        device: str = "cuda:0",
        image_size: int = 224,
        memory_size: int = 8,
        predict_size: int = 24,
        temporal_depth: int = 16,
        heads: int = 8,
        token_dim: int = 384,
        channels: int = 3,
        dropout: float = 0.1,
        sample_num: int = 32,
        depth_scale: float = 10000.0,
        constant_depth: float = 1.0,
        use_constant_depth: bool = False,  # deprecated compatibility flag
        depth_mode: str = "depth_anything",
        depth_anything_encoder: str = "vitb",
        depth_anything_input_size: int = 518,
        depth_anything_max_depth: float = 20.0,
        goal_mode: str = "point",
    ):
        super().__init__("navdp_policy_node")

        # Initialize agent
        self.agent = NavDPAgent(
            checkpoint=checkpoint,
            depth_anything_checkpoint=depth_anything_checkpoint,
            depth_anything_encoder=depth_anything_encoder,
            image_size=image_size,
            memory_size=memory_size,
            predict_size=predict_size,
            temporal_depth=temporal_depth,
            heads=heads,
            token_dim=token_dim,
            channels=channels,
            dropout=dropout,
            device=device,
            sample_num=sample_num,
            depth_scale=depth_scale,
        )
        self.agent.reset(batch_size=1)

        self.bridge = CvBridge()
        self.inference_rate = inference_rate
        if use_constant_depth:
            depth_mode = "constant"
        self.depth_mode = str(depth_mode)
        self.constant_depth = float(constant_depth)
        self.goal_mode = goal_mode
        self.depth_estimator: Optional[DepthAnythingEstimator] = None

        if self.depth_mode == "depth_anything":
            if not depth_anything_checkpoint:
                raise ValueError("depth_mode=depth_anything requires --depth_anything_checkpoint")
            self.depth_estimator = DepthAnythingEstimator(
                checkpoint=depth_anything_checkpoint,
                encoder=depth_anything_encoder,
                input_size=depth_anything_input_size,
                max_depth=depth_anything_max_depth,
                device=device,
            )
        elif self.depth_mode not in {"constant", "topic"}:
            raise ValueError(f"Unsupported depth_mode: {self.depth_mode}")

        # State
        self.current_image: Optional[np.ndarray] = None
        self.current_depth: Optional[np.ndarray] = None
        self.current_odom: Optional[Odometry] = None
        self.current_goal_pose: Optional[PoseStamped] = None
        self.current_goal_image: Optional[np.ndarray] = None
        self.current_pixel_goal: Optional[PointStamped] = None
        self.enabled = True

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, sensor_qos)
        if self.depth_mode == "topic":
            self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        if self.goal_mode == "point":
            self.goal_sub = self.create_subscription(PoseStamped, goal_pose_topic, self.goal_callback, 10)
        if self.goal_mode == "image":
            self.goal_image_sub = self.create_subscription(Image, goal_image_topic, self.goal_image_callback, 10)
        if self.goal_mode == "pixel":
            self.pixel_goal_sub = self.create_subscription(PointStamped, pixel_goal_topic, self.pixel_goal_callback, 10)
        self.enable_sub = self.create_subscription(Bool, enable_topic, self.enable_callback, 10)

        # Publisher
        self.trajectory_pub = self.create_publisher(NavPath, trajectory_topic, 10)

        # Timer
        timer_period = 1.0 / self.inference_rate
        self.timer = self.create_timer(timer_period, self.inference_callback)

        self.get_logger().info("NavDP policy node started (local inference)")
        self.get_logger().info(f"Depth mode: {self.depth_mode}")
        self.get_logger().info(
            f"Subscribing to: {image_topic}, {depth_topic}, {odom_topic} (goal_mode={self.goal_mode})"
        )
        self.get_logger().info(f"Publishing trajectory to: {trajectory_topic}")

    def image_callback(self, msg: Image) -> None:
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def depth_callback(self, msg: Image) -> None:
        try:
            # passthrough keeps original encoding (e.g., 16UC1 or 32FC1)
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"Depth conversion failed: {exc}")

    def odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        self.current_goal_pose = msg
        # Reset memory queue for new mission
        self.agent.reset(batch_size=1)
        self.get_logger().info("Received new goal. Reset NavDP memory queue.")

    def goal_image_callback(self, msg: Image) -> None:
        try:
            self.current_goal_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.agent.reset(batch_size=1)
            self.get_logger().info("Received new goal image. Reset NavDP memory queue.")
        except Exception as exc:
            self.get_logger().error(f"Goal image conversion failed: {exc}")

    def pixel_goal_callback(self, msg: PointStamped) -> None:
        self.current_pixel_goal = msg
        self.agent.reset(batch_size=1)
        self.get_logger().info("Received new pixel goal. Reset NavDP memory queue.")

    def enable_callback(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def _compute_local_goal(self) -> Optional[np.ndarray]:
        if self.current_goal_pose is None or self.current_odom is None:
            return None

        goal = self.current_goal_pose.pose.position
        odom = self.current_odom.pose.pose.position
        quat = self.current_odom.pose.pose.orientation

        _, _, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])
        dx = goal.x - odom.x
        dy = goal.y - odom.y

        # Transform to robot frame (x forward)
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return np.array([local_x, local_y, 0.0], dtype=np.float32)

    def _get_depth(self) -> Optional[np.ndarray]:
        if self.depth_mode == "constant":
            if self.current_image is None:
                return None
            h, w = self.current_image.shape[:2]
            return np.full((h, w), self.constant_depth, dtype=np.float32)
        if self.depth_mode == "depth_anything":
            if self.current_image is None or self.depth_estimator is None:
                return None
            return self.depth_estimator.infer(self.current_image)
        return self.current_depth

    def inference_callback(self) -> None:
        if not self.enabled:
            return
        if self.current_image is None:
            return
        depth = self._get_depth()
        if depth is None:
            return

        try:
            if self.goal_mode == "point":
                goal = self._compute_local_goal()
                if goal is None:
                    return
                best_traj, _ = self.agent.step_pointgoal(goal, self.current_image, depth)
            elif self.goal_mode == "image":
                if self.current_goal_image is None:
                    return
                best_traj, _ = self.agent.step_imagegoal(self.current_goal_image, self.current_image, depth)
            elif self.goal_mode == "pixel":
                if self.current_pixel_goal is None:
                    return
                pixel = self.current_pixel_goal.point
                best_traj, _ = self.agent.step_pixelgoal((pixel.x, pixel.y), self.current_image, depth)
            elif self.goal_mode == "nogoal":
                best_traj, _ = self.agent.step_nogoal(self.current_image, depth)
            else:
                self.get_logger().error(f"Unknown goal_mode: {self.goal_mode}")
                return
        except Exception as exc:
            self.get_logger().error(f"NavDP inference failed: {exc}")
            return

        # Publish Path in base_link frame (local)
        path_msg = NavPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "base_link"

        for i in range(best_traj.shape[0]):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(best_traj[i, 0])
            pose.pose.position.y = float(best_traj[i, 1])
            theta = float(best_traj[i, 2]) if best_traj.shape[1] >= 3 else 0.0
            q = euler2quat(0.0, 0.0, theta)
            pose.pose.orientation.w = float(q[0])
            pose.pose.orientation.x = float(q[1])
            pose.pose.orientation.y = float(q[2])
            pose.pose.orientation.z = float(q[3])
            path_msg.poses.append(pose)

        self.trajectory_pub.publish(path_msg)


def main():
    parser = argparse.ArgumentParser(description="NavDP ROS2 policy node (local inference)")
    parser.add_argument("--checkpoint", required=True, help="Path to NavDP checkpoint (.ckpt)")
    parser.add_argument(
        "--depth_anything_checkpoint",
        default=None,
        help="Optional DepthAnything checkpoint used by NavDP RGBD backbone",
    )
    parser.add_argument("--device", default="cuda:0", help="Torch device (default: cuda:0)")
    parser.add_argument("--inference_rate", type=float, default=10.0, help="Inference rate in Hz")
    parser.add_argument(
        "--goal_mode",
        type=str,
        default="point",
        choices=["point", "image", "pixel", "nogoal"],
        help="Goal mode: point|image|pixel|nogoal",
    )
    parser.add_argument("--image_topic", default="/front_stereo_camera/left/image_raw", help="RGB image topic")
    parser.add_argument("--depth_topic", default="/camera/depth/image_raw", help="Depth image topic")
    parser.add_argument("--odom_topic", default="/chassis/odom", help="Odometry topic")
    parser.add_argument("--goal_pose_topic", default="/goal_pose", help="Goal pose topic")
    parser.add_argument("--goal_image_topic", default="/goal_image", help="Goal image topic")
    parser.add_argument("--pixel_goal_topic", default="/goal_pixel", help="Pixel goal topic (PointStamped)")
    parser.add_argument("--trajectory_topic", default="/navdp_trajectory", help="Trajectory output topic")
    parser.add_argument("--enable_topic", default="/navdp_enable", help="Enable/disable topic")

    # Model params
    parser.add_argument("--image_size", type=int, default=224)
    parser.add_argument("--memory_size", type=int, default=8)
    parser.add_argument("--predict_size", type=int, default=24)
    parser.add_argument("--temporal_depth", type=int, default=16)
    parser.add_argument("--heads", type=int, default=8)
    parser.add_argument("--token_dim", type=int, default=384)
    parser.add_argument("--channels", type=int, default=3)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--sample_num", type=int, default=32)

    # Depth params
    parser.add_argument("--depth_scale", type=float, default=10000.0, help="Scale for uint16 depth to meters")
    parser.add_argument(
        "--depth_mode",
        type=str,
        default="depth_anything",
        choices=["depth_anything", "topic", "constant"],
        help="Depth input mode (default: depth_anything)",
    )
    parser.add_argument("--use_constant_depth", action="store_true", help="Deprecated: same as --depth_mode constant")
    parser.add_argument("--constant_depth", type=float, default=1.0, help="Constant depth in meters")
    parser.add_argument("--depth_anything_encoder", default="vitb", help="DepthAnything encoder variant")
    parser.add_argument("--depth_anything_input_size", type=int, default=518, help="DepthAnything input size")
    parser.add_argument("--depth_anything_max_depth", type=float, default=20.0, help="DepthAnything max depth")

    args = parser.parse_args()
    depth_mode = "constant" if args.use_constant_depth else args.depth_mode

    rclpy.init()
    node = NavDPPolicyNode(
        checkpoint=args.checkpoint,
        depth_anything_checkpoint=args.depth_anything_checkpoint,
        inference_rate=args.inference_rate,
        image_topic=args.image_topic,
        depth_topic=args.depth_topic,
        odom_topic=args.odom_topic,
        goal_pose_topic=args.goal_pose_topic,
        goal_image_topic=args.goal_image_topic,
        pixel_goal_topic=args.pixel_goal_topic,
        trajectory_topic=args.trajectory_topic,
        enable_topic=args.enable_topic,
        device=args.device,
        image_size=args.image_size,
        memory_size=args.memory_size,
        predict_size=args.predict_size,
        temporal_depth=args.temporal_depth,
        heads=args.heads,
        token_dim=args.token_dim,
        channels=args.channels,
        dropout=args.dropout,
        sample_num=args.sample_num,
        depth_scale=args.depth_scale,
        constant_depth=args.constant_depth,
        use_constant_depth=args.use_constant_depth,
        depth_mode=depth_mode,
        depth_anything_encoder=args.depth_anything_encoder,
        depth_anything_input_size=args.depth_anything_input_size,
        depth_anything_max_depth=args.depth_anything_max_depth,
        goal_mode=args.goal_mode,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
