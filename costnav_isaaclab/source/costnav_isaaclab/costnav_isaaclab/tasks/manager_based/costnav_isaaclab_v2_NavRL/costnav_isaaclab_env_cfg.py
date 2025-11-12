# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import isaaclab.sim as sim_utils
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as loc_mdp
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg, TiledCameraCfg
from isaaclab.utils import configclass

from . import mdp
from .coco_robot_cfg import (
    COCO_CFG,
    ClassicalCarActionCfg,
    ClassicalCarWaypointActionCfg,
    OgnCarActionCfg,
    RestrictedCarActionCfg,
)
from .safe_positions_auto_generated import SAFE_POSITIONS

##
# Pre-defined configs
##


##
# Scene definition
##


@configclass
class CostnavIsaaclabSceneCfg(InteractiveSceneCfg):
    """Configuration for COCO robot navigation scene with custom map."""

    # custom map
    # custom_map = AssetBaseCfg(
    #     prim_path="/World/custom_map",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path="omniverse://10.50.2.21/Users/worv/map/Street_sidewalk.usd"
    #     ),
    # )

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot - COCO robot for navigation
    robot: ArticulationCfg = COCO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # sensors
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
    )

    # camera sensor for visual observations
    camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/front_cam",
        update_period=0.1,
        height=1080 // 8,
        width=1920 // 8,
        data_types=["rgb", "distance_to_camera"],
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=[531.0, 0.0, 960.0, 0.0, 531.0, 540.0, 0.0, 0.0, 1.0],
            width=1920,
            height=1080,
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.51, 0.0, 0.015), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"
        ),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    # Use custom command generator that samples from safe positions
    pose_command = mdp.SafePositionPose2dCommandCfg(
        asset_name="robot",
        simple_heading=True,  # Point towards goal for easier learning
        resampling_time_range=(1.0e9, 1.0e9),  # is this the only way to avoid resampling lol
        debug_vis=True,
        safe_positions=SAFE_POSITIONS,  # Use pre-validated safe positions
        min_goal_distance=10.0,  # Minimum distance between robot and goal (in meters)
        ranges=mdp.SafePositionPose2dCommandCfg.Ranges(
            heading=(-math.pi, math.pi),
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # Use OGN car velocity actions (linear velocity + steering angle)
    joint_pos = RestrictedCarActionCfg(asset_name="robot")


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group.

        Note: Observations are concatenated in order: [pose_command, base_lin_vel, base_ang_vel, rgb_flattened]
        - pose_command: [2] - goal position (x, y) in base frame
        - base_lin_vel: [3] - linear velocity (vx, vy, vz) in base frame
        - base_ang_vel: [3] - angular velocity (wx, wy, wz) in base frame
        - rgb_flattened: [72900] - flattened RGB-D image (4 * 135 * 240)
        Total: [72908]

        The mixed input network will split and reshape them internally.
        """

        # observation terms (order preserved - ORDER MATTERS!)
        # 1. Vector observations (goal commands + velocities) - will be processed by MLP branch
        pose_command = ObsTerm(
            func=mdp.advanced_generated_commands,
            params={"command_name": "pose_command", "max_dim": 2, "normalize": True},
        )

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)  # [3] - linear velocity in base frame
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)  # [3] - angular velocity in base frame

        # 2. Visual observations (RGB-D camera) - will be processed by CNN branch
        # Flattened to enable concatenation with vector observations
        rgb = ObsTerm(
            func=mdp.rgbd_processed,
            params={"sensor_cfg": SceneEntityCfg("camera"), "flatten": True},
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True  # Concatenate to single tensor for RL-Games compatibility

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # Print episode rewards on reset (called first to show rewards before reset)
    print_rewards = EventTerm(
        func=mdp.print_episode_rewards,
        mode="reset",
    )

    # Reset robot base position using safe positions
    reset_base = EventTerm(
        func=mdp.reset_root_state_from_safe_positions,
        mode="reset",
        params={
            "safe_positions": SAFE_POSITIONS,  # Use pre-validated safe positions
            "velocity_range": {
                "x": (-0.0, 0.0),
                "y": (-0.0, 0.0),
                "z": (-0.0, 0.0),
                "roll": (-0.0, 0.0),
                "pitch": (-0.0, 0.0),
                "yaw": (-math.pi, math.pi),  # Random yaw orientation
            },
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Arrival reward
    arrived_reward = RewTerm(
        func=loc_mdp.is_terminated_term, weight=200000.0, params={"term_keys": "arrive"}
    )

    # Collision penalty
    collision_penalty = RewTerm(
        func=loc_mdp.is_terminated_term, weight=-200.0, params={"term_keys": "collision"}
    )

    # basic goal based distance reward
    distance_progress = RewTerm(
        func=mdp.distance_to_goal_progress,
        weight=100.0,
        params={"command_name": "pose_command", "slack_penalty": 0.0001},
    )

    # Position tracking reward (coarse) - increased weight and adjusted std for closer goals
    # position_tracking = RewTerm(
    #     func=mdp.position_command_error_tanh,
    #     weight=50.0,  # Increased from 10.0
    #     params={"std": 30.0, "command_name": "pose_command"},  # Reduced from 5.0 for closer goals
    # )

    # Position tracking reward (fine-grained)
    # position_tracking_fine = RewTerm(
    #     func=mdp.position_command_error_tanh,
    #     weight=200.0,  # Increased from 50.0
    #     params={
    #         "std": 10.0,
    #         "command_name": "pose_command",
    #     },  # Reduced from 1.0 for tighter tracking
    # )

    # Reward for moving towards goal - CRITICAL for learning
    # moving_towards_goal = RewTerm(
    #     func=mdp.moving_towards_goal_reward,
    #     weight=100.0,  # Increased from 20.0 to strongly encourage movement
    #     params={"command_name": "pose_command"},
    # )

    # Reward for maintaining target velocity
    # target_vel_rew = RewTerm(
    #     func=mdp.target_vel_reward,
    #     weight=30.0,
    #     params={"command_name": "pose_command"},  # Increased from 10.0
    # )

    # DEBUG: Print rewards every step (weight=0.001 is negligible but ensures it runs)
    debug_print = RewTerm(
        func=mdp.print_rewards,
        weight=0.001,  # Very small weight so it runs but doesn't affect training
        params={"print_every_n_steps": 1},  # Change to higher number to print less frequently
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # Time out
    time_out = DoneTerm(func=loc_mdp.time_out, time_out=True)

    # Collision termination
    collision = DoneTerm(
        func=mdp.illegal_contact,
        time_out=False,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="body_link"),
            "threshold": 1.0,
        },
    )

    # Arrival termination
    arrive = DoneTerm(
        func=mdp.arrive,
        time_out=False,
        params={"threshold": 1.0, "command_name": "pose_command"},
    )


##
# Environment configuration
##


@configclass
class CostnavIsaaclabEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for COCO robot navigation environment with custom map."""

    # Scene settings - using safe positions for spawning, no env_spacing needed
    scene: CostnavIsaaclabSceneCfg = CostnavIsaaclabSceneCfg(num_envs=64, env_spacing=0.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 10  # Control frequency: 50 Hz (500ms / 10)
        self.episode_length_s = 60.0  # second episodes for navigation

        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)

        # simulation settings
        self.sim.dt = 0.005  # 5ms timestep (200 Hz physics)
        self.sim.render_interval = 4  # Render every 4 physics steps
        self.sim.disable_contact_processing = True

        # increase PhysX GPU collision buffers for complex scenes with many collision objects
        self.sim.physx.gpu_collision_stack_size = (
            536870912  # 512 MB to handle complex map collisions
        )
        self.sim.physx.gpu_found_lost_pairs_capacity = (
            8000000  # Increased to handle many collision pairs
        )
        self.sim.physx.gpu_total_aggregate_pairs_capacity = (
            7000000  # Increased to handle aggregate collision pairs
        )

        # Update camera period based on decimation
        if hasattr(self.scene, "camera"):
            self.scene.camera.update_period = self.decimation * self.sim.dt
