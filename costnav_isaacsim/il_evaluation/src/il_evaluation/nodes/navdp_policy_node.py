#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""NavDP ROS2 Policy Node for CostNav.

This wrapper follows the shared BasePolicyNode abstraction used by other IL
baselines and keeps NavDP in the default eval mode:
- depth: DepthAnything
- goal: point + image fusion
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from transforms3d.euler import quat2euler

from il_evaluation.agents.navdp_agent import NavDPAgent
from il_evaluation.models.depth_anything_estimator import DepthAnythingEstimator
from il_evaluation.nodes.base_policy_node import BasePolicyNode, build_arg_parser, run_policy_node


class NavDPPolicyNode(BasePolicyNode):
    """ROS2 node for NavDP policy inference via BasePolicyNode hooks."""

    def __init__(
        self,
        checkpoint: str,
        model_config: str,
        robot_config: str,
        goal_type: Optional[str] = None,
        topomap_dir: str = "",
    ) -> None:
        with open(model_config, "r") as f:
            model_cfg = yaml.safe_load(f) or {}

        resolved_goal_type = goal_type or "image_goal"
        if resolved_goal_type != "image_goal":
            raise ValueError(f"NavDP only supports goal_type='image_goal' in this wrapper, got '{resolved_goal_type}'.")

        self.depth_anything_checkpoint = str(
            model_cfg.get("depth_anything_checkpoint", "checkpoints/depth_anything_v2_vits.pth")
        )
        self.depth_anything_encoder = str(model_cfg.get("depth_anything_encoder", "vits"))
        self.depth_anything_input_size = int(model_cfg.get("depth_anything_input_size", 518))
        self.depth_anything_max_depth = float(model_cfg.get("depth_anything_max_depth", 20.0))

        self.point_image_blend_alpha = float(model_cfg.get("point_image_blend_alpha", 0.5))
        self.point_image_blend_alpha = max(0.0, min(1.0, self.point_image_blend_alpha))

        self._agent_kwargs = {
            "depth_anything_checkpoint": self.depth_anything_checkpoint,
            "depth_anything_encoder": self.depth_anything_encoder,
            "image_size": int(model_cfg.get("image_size", 224)),
            "memory_size": int(model_cfg.get("memory_size", 8)),
            "predict_size": int(model_cfg.get("predict_size", 24)),
            "temporal_depth": int(model_cfg.get("temporal_depth", 16)),
            "heads": int(model_cfg.get("heads", 8)),
            "token_dim": int(model_cfg.get("token_dim", 384)),
            "channels": int(model_cfg.get("channels", 3)),
            "dropout": float(model_cfg.get("dropout", 0.1)),
            "sample_num": int(model_cfg.get("sample_num", 32)),
            "depth_scale": float(model_cfg.get("depth_scale", 10000.0)),
        }

        super().__init__(
            checkpoint=checkpoint,
            model_config=model_config,
            robot_config=robot_config,
            goal_type=resolved_goal_type,
            topomap_dir=topomap_dir,
            model_name="NavDP",
            node_name="navdp_policy_node",
            agent_cls=NavDPAgent,
        )

        self.current_odom: Optional[Odometry] = None
        self.current_goal_pose: Optional[PoseStamped] = None

        self.depth_estimator: Optional[DepthAnythingEstimator] = DepthAnythingEstimator(
            checkpoint=self.depth_anything_checkpoint,
            encoder=self.depth_anything_encoder,
            input_size=self.depth_anything_input_size,
            max_depth=self.depth_anything_max_depth,
            device=self.model_cfg.get("device", "cuda:0"),
        )

        self.get_logger().info(
            f"NavDP mode fixed to point_image + depth_anything "
            f"(point_image_blend_alpha={self.point_image_blend_alpha:.2f})"
        )

    def _create_agent(
        self,
        *,
        checkpoint: str,
        model_config_path: str,
        model_cfg: dict,
        device: str,
    ) -> NavDPAgent:
        del model_config_path, model_cfg
        return NavDPAgent(
            checkpoint=checkpoint,
            device=device,
            **self._agent_kwargs,
        )

    def _setup_extra_subscribers(self, robot_cfg: dict, sensor_qos: QoSProfile) -> None:
        del sensor_qos
        topics = robot_cfg.get("topics", {})
        self.odom_topic = topics.get("odom", "/chassis/odom")
        self.goal_pose_topic = str(topics.get("goal_pose", "/goal_pose"))

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_pose_topic, self.goal_callback, 10)

    def _reset_extra_state(self) -> None:
        self.current_goal_pose = None

    def odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        self.current_goal_pose = msg
        self.agent.reset(batch_size=1)
        self.get_logger().info("Received new goal. Reset NavDP memory queue.")

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
        if self.current_image is None or self.depth_estimator is None:
            return None
        return self.depth_estimator.infer(self.current_image)

    def _inference_custom(self) -> bool:
        depth = self._get_depth()
        if depth is None:
            return True

        if self.goal_image is None:
            return True

        goal = self._compute_local_goal()
        if goal is None:
            return True

        try:
            queue_snapshot = [list(q) for q in self.agent.memory_queue]
            point_traj, _ = self.agent.step_pointgoal(goal, self.current_image, depth)
            self.agent.memory_queue = [list(q) for q in queue_snapshot]
            image_traj, _ = self.agent.step_imagegoal(self.goal_image, self.current_image, depth)
            best_traj = self.point_image_blend_alpha * point_traj + (1.0 - self.point_image_blend_alpha) * image_traj
        except Exception as exc:
            self.get_logger().error(f"NavDP inference failed: {exc}")
            return True

        self._publish_trajectory(best_traj)

        if self._goal_image_pub is not None:
            self._publish_debug_image(self._goal_image_pub, self.goal_image, "goal_image")

        return True


def parse_args():
    """Parse command line arguments."""
    return build_arg_parser("NavDP", "navdp_eval.yaml").parse_args()


def main() -> None:
    """Main entry point for the NavDP policy node."""
    args = parse_args()
    run_policy_node(NavDPPolicyNode, args, node_label="NavDP")


if __name__ == "__main__":
    main()
