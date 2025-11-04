# Copyright (c) 2025, COCO Navigation Project.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
# Adapted from urban-sim COCO robot configuration

from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
import torch
from isaaclab.actuators import DCMotorCfg, DelayedPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets import Articulation
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.envs.mdp.actions import (
    JointPositionAction,
    JointPositionActionCfg,
    JointVelocityAction,
    JointVelocityActionCfg,
)
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

# Robot instance
COCO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="omniverse://10.50.2.21/Users/worv/coco_one_fix.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.3),
        joint_pos={
            ".*wheel_joint*": 0.0,
            "base_to_front_axle_joint": 0.0,
        },
        joint_vel={
            ".*wheel_joint": 0.0,
            "base_to_front_axle_joint": 0.0,
        },
    ),
    actuators={
        "wheels": DelayedPDActuatorCfg(
            joint_names_expr=[".*wheel_joint"],
            velocity_limit=100.0,
            min_delay=0,
            max_delay=4,
            stiffness={
                ".*_wheel_joint": 0.0,
            },
            damping={".*_wheel_joint": 0.3},
            friction={
                ".*_wheel_joint": 0.0,
            },
            armature={
                ".*_wheel_joint": 0.0,
            },
        ),
        "axle": DCMotorCfg(
            joint_names_expr=["base_to_front_axle_joint"],
            saturation_effort=64.0,
            effort_limit=64.0,
            velocity_limit=20.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        "shock": ImplicitActuatorCfg(
            joint_names_expr=[".*shock_joint"],
            stiffness=0.0,
            damping=0.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

# Action space parameters
DT = 0.1
MAX_W = 2.0
MAX_V = 2.0
ACTION_INTERVAL = 4


class ClassicalCarAction(ActionTerm):
    """Classical car action term for velocity and angular velocity control.

    This action term controls the COCO robot using velocity and steering angle commands.
    The raw actions correspond to [linear_velocity, steering_angle].
    """

    cfg: ClassicalCarActionCfg
    """The configuration of the action term."""

    def __init__(self, cfg: ClassicalCarActionCfg, env: ManagerBasedRLEnv) -> None:
        # initialize the action term
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]
        self._counter = 0
        self.last_wheel_angle = torch.zeros(self.num_envs, 1, device=self.device)

        self.axle_names = ["base_to_front_axle_joint"]
        self.wheel_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
        self.shock_names = [".*shock_joint"]
        self._raw_actions = torch.zeros(self.num_envs, 2, device=self.device)

        # prepare low level actions
        self.acceleration_action: JointVelocityAction = JointVelocityAction(
            JointVelocityActionCfg(
                asset_name="robot",
                joint_names=[".*_wheel_joint"],
                scale=10.0,
                use_default_offset=False,
            ),
            env,
        )
        self.steering_action: JointPositionAction = JointPositionAction(
            JointPositionActionCfg(
                asset_name="robot", joint_names=self.axle_names, scale=1.0, use_default_offset=True
            ),
            env,
        )

    @property
    def action_dim(self) -> int:
        return 2

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self.raw_actions

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions

    def apply_actions(self):
        if self._counter % ACTION_INTERVAL == 0:
            max_wheel_v = 4.0
            wheel_base = 1.5
            radius_rear = 0.3
            max_ang = 40 * torch.pi / 180
            velocity = self.raw_actions[..., :1].clamp(0.0, max_wheel_v) / radius_rear
            angular = self.raw_actions[..., 1:2].clamp(-max_ang, max_ang)
            angular[angular.abs() < 0.05] = torch.zeros_like(angular[angular.abs() < 0.05])
            R = wheel_base / torch.tan(angular)
            left_wheel_angle = torch.arctan(wheel_base / (R - 0.5 * 1.8))
            right_wheel_angle = torch.arctan(wheel_base / (R + 0.5 * 1.8))

            self.steering_action.process_actions(((right_wheel_angle + left_wheel_angle) / 2.0))
            self.acceleration_action.process_actions(
                torch.cat([velocity, velocity, velocity, velocity], dim=1)
            )

        self.steering_action.apply_actions()
        self.acceleration_action.apply_actions()
        self._counter += 1

    def _set_debug_vis_impl(self, _debug_vis: bool):
        pass

    def _debug_vis_callback(self, _event):
        pass


@configclass
class ClassicalCarActionCfg(ActionTermCfg):
    """Configuration for classical car action term.

    See :class:`ClassicalCarAction` for more details.
    """

    class_type: type[ActionTerm] = ClassicalCarAction
    """Class of the action term."""
    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""


@configclass
class COCOVelocityActionsCfg:
    """Action specifications for the COCO robot with velocity control."""

    joint_pos = ClassicalCarActionCfg(asset_name="robot")


# Waypoint-based action space
def clip_angle(angle):
    return (angle + torch.pi) % (2 * torch.pi) - torch.pi


def pd_controller(waypoint):
    dx = waypoint[:, 0]
    dy = waypoint[:, 1]
    v = dx / DT
    w = torch.arctan2(dy, dx) / DT
    v = torch.clip(v, 0, MAX_V)
    w = torch.clip(w, -MAX_W, MAX_W)
    return v, w


class ClassicalCarWaypointAction(ActionTerm):
    """Classical car waypoint action term.

    This action term controls the COCO robot using waypoint commands.
    The raw actions correspond to [dx, dy] waypoint in local frame.
    """

    cfg: ClassicalCarWaypointActionCfg
    """The configuration of the action term."""

    def __init__(self, cfg: ClassicalCarWaypointActionCfg, env: ManagerBasedRLEnv) -> None:
        # initialize the action term
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]
        self._counter = 0
        self.last_wheel_angle = torch.zeros(self.num_envs, 1, device=self.device)

        self.axle_names = ["base_to_front_axle_joint"]
        self.wheel_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
        self.shock_names = [".*shock_joint"]
        self._raw_actions = torch.zeros(self.num_envs, 2, device=self.device)

        # prepare low level actions
        self.acceleration_action: JointVelocityAction = JointVelocityAction(
            JointVelocityActionCfg(
                asset_name="robot",
                joint_names=[".*_wheel_joint"],
                scale=10.0,
                use_default_offset=False,
            ),
            env,
        )
        self.steering_action: JointPositionAction = JointPositionAction(
            JointPositionActionCfg(
                asset_name="robot", joint_names=self.axle_names, scale=1.0, use_default_offset=True
            ),
            env,
        )

    @property
    def action_dim(self) -> int:
        return 2

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self.raw_actions

    def process_actions(self, actions: torch.Tensor):
        actions = actions * 2 / 10
        v, w = pd_controller(actions)
        eps = 1e-6  # Small value to avoid division by zero
        R = torch.where(torch.abs(w) > eps, v / (w + eps), torch.full_like(v, 1e6))
        steering_angle = torch.atan(1.5 / R)
        self._raw_actions[:, 0] = v
        self._raw_actions[:, 1] = steering_angle

    def apply_actions(self):
        if self._counter % ACTION_INTERVAL == 0:
            max_wheel_v = 4.0
            wheel_base = 1.5
            radius_rear = 0.3
            max_ang = 40 * torch.pi / 180
            velocity = self.raw_actions[..., :1].clamp(-max_wheel_v, max_wheel_v) / radius_rear
            angular = self.raw_actions[..., 1:2].clamp(-max_ang, max_ang)
            angular[angular.abs() < 0.05] = torch.zeros_like(angular[angular.abs() < 0.05])
            R = wheel_base / torch.tan(angular)
            left_wheel_angle = torch.arctan(wheel_base / (R - 0.5 * 1.8))
            right_wheel_angle = torch.arctan(wheel_base / (R + 0.5 * 1.8))

            self.steering_action.process_actions(((right_wheel_angle + left_wheel_angle) / 2.0))
            self.acceleration_action.process_actions(
                torch.cat([velocity, velocity, velocity, velocity], dim=1)
            )

        self.steering_action.apply_actions()
        self.acceleration_action.apply_actions()
        self._counter += 1

    def _set_debug_vis_impl(self, _debug_vis: bool):
        pass

    def _debug_vis_callback(self, _event):
        pass


@configclass
class ClassicalCarWaypointActionCfg(ActionTermCfg):
    """Configuration for classical car waypoint action term.

    See :class:`ClassicalCarWaypointAction` for more details.
    """

    class_type: type[ActionTerm] = ClassicalCarWaypointAction
    """Class of the action term."""
    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""


@configclass
class COCOWaypointActionsCfg:
    """Action specifications for the COCO robot with waypoint control."""

    joint_pos = ClassicalCarWaypointActionCfg(asset_name="robot")
