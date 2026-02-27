#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""NoMaD ROS2 Policy Node for CostNav.

This node subscribes to camera images, runs NoMaD inference,
and publishes trajectory to /model_trajectory for the trajectory follower node.

Supports three navigation modes (selected via ``goal_type``):
1. **no_goal** (default) — exploration without a goal image.
2. **image_goal** — navigate toward a single goal image.
3. **topomap** — follow a topological map generated
   online in simulation by the NavMesh-to-Topomap pipeline
   (see ``TOPOMAP_PIPELINE.md``).

Topomap Navigation
------------------
The topomap is a directory of sequentially numbered PNGs (``0.png``, ``1.png``,
…, ``N.png``) produced by ``TopomapGenerator``.  At each inference step the node:

1. Selects a local window of subgoal images around ``closest_node``
   (controlled by ``--topomap_radius``).
2. Runs batched NoMaD inference to predict distances from the current
   observation to each subgoal in the window.
3. Localises to the closest node (``argmin`` of predicted distances).
4. If the distance to the closest node is below ``--topomap_close_threshold``,
   advances ``closest_node`` by one.
5. Publishes the trajectory toward the chosen subgoal.
6. Publishes ``reached_goal=True`` on ``/model_reached_goal`` when
   ``closest_node == goal_node``.

This mirrors the navigation loop in NoMaD's ``navigate.py`` but runs inside
the CostNav ROS 2 stack.

For ImageGoal mode, the node receives goal images via:
1. /goal_image topic - Published by the mission manager when a new mission starts
2. /set_goal_image service - Service to set the goal image programmatically

Following the NavDP pattern, when a new goal image is received:
- The agent's memory queue is reset (new mission = fresh start)
- The new goal image is stored and used for subsequent inference

Usage:
    # Default (Get goal type from model config)
    python3 nomad_policy_node.py \
        --checkpoint /path/to/model.pth \
        --model_config /path/to/config.yaml \
        --robot_config /path/to/robot.yaml

    # Topomap mode (--goal_type overrides goal_type from nomad_eval.yaml)
    python3 nomad_policy_node.py \
        --checkpoint /path/to/model.pth \
        --model_config /path/to/config.yaml \
        --robot_config /path/to/robot.yaml \
        --goal_type topomap --topomap_dir /tmp/costnav_topomap
"""

from __future__ import annotations

from il_evaluation.agents.nomad_agent import NoMaDAgent
from il_evaluation.nodes.base_policy_node import BasePolicyNode, build_arg_parser, run_policy_node


class NoMaDPolicyNode(BasePolicyNode):
    """ROS2 Node for NoMaD policy inference."""

    def __init__(
        self,
        checkpoint: str,
        model_config: str,
        robot_config: str,
        goal_type: str | None = None,
        topomap_dir: str = "",
    ) -> None:
        super().__init__(
            checkpoint=checkpoint,
            model_config=model_config,
            robot_config=robot_config,
            goal_type=goal_type,
            topomap_dir=topomap_dir,
            model_name="NoMaD",
            node_name="nomad_policy_node",
            agent_cls=NoMaDAgent,
        )


def parse_args():
    """Parse command line arguments."""
    return build_arg_parser("NoMaD", "nomad_eval.yaml").parse_args()


def main() -> None:
    """Main entry point for the NoMaD policy node."""
    args = parse_args()
    run_policy_node(NoMaDPolicyNode, args, node_label="NoMaD")


if __name__ == "__main__":
    main()
