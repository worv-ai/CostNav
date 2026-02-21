#!/usr/bin/env python3
# Copyright (c) 2026 CostNav Authors
# Licensed under the MIT License

"""Trajectory Follower ROS2 Node for CostNav.

This node subscribes to trajectory waypoints published by ViNT policy node
and publishes velocity commands at a fixed rate using MPC controller.

Reference:
    Based on the MPC controller implementation from NavDP:
    - Control loop: third_party/NavDP/eval_imagegoal_wheeled.py (lines 220-262)
    - MPC Controller: third_party/NavDP/utils_tasks/tracking_utils.py (MPC_Controller class)

Key features from NavDP:
    - Receives trajectory in world frame
    - Uses robot pose from odometry to track along trajectory
    - Finds nearest point on trajectory and tracks forward
    - MPC with CasADi/IPOPT solver using unicycle dynamics
    - Uses second control output (index 1) for smoother response
    - Cost matrices: Q = diag([10.0, 10.0, 0.0]), R = diag([0.02, 0.15])
    - Default parameters: N=15, desired_v=0.5, v_max=0.5, w_max=0.5, ref_gap=3, dt=0.1
    - When MPC is None, skip control iteration (matching NavDP behavior)

Architecture:
    - ViNT policy node: runs inference at ~4 Hz, publishes full trajectory
    - Trajectory follower node: receives trajectory, publishes cmd_vel at ~20 Hz

MPC reuse:
    The CasADi optimization problem is built once on the first trajectory.
    Subsequent trajectories update the reference path without rebuilding the
    problem, preserving the IPOPT warm-start for faster convergence.

Usage:
    python3 trajectory_follower_node.py \
        --robot_config configs/robot_carter.yaml
"""

import argparse
import threading
from abc import ABC, abstractmethod
from typing import Optional, Tuple

import casadi as ca
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.interpolate import interp1d
from std_msgs.msg import Bool
from transforms3d.euler import quat2euler
from transforms3d.quaternions import quat2mat


class BaseMPCController(ABC):
    """Abstract base class for MPC trajectory tracking controllers.

    Based on third_party/NavDP/utils_tasks/tracking_utils.py MPC_Controller.
    Uses CasADi for optimization with IPOPT solver.

    Subclasses must implement:
        - _get_dynamics(): Return the kinematic model dynamics function
        - _get_cost_matrices(): Return Q and R cost matrices
        - _get_control_bounds(): Return control variable bounds
        - _compute_reference_headings(): Compute heading angles for reference trajectory
    """

    def __init__(
        self,
        trajectory: np.ndarray,
        N: int = 15,
        desired_v: float = 0.5,
        v_max: float = 0.5,
        ref_gap: int = 3,
        dt: float = 0.1,
        lookahead_ratio: float = 0.4,
    ):
        """Initialize MPC controller.

        Args:
            trajectory: Reference trajectory [M, 2] in world frame (x, y).
            N: MPC horizon length.
            desired_v: Desired velocity for reference point spacing.
            v_max: Maximum linear velocity.
            ref_gap: Gap between reference points in horizon.
            dt: Time step for dynamics.
            lookahead_ratio: Fraction of trajectory to skip (0.4 = skip first 40%, matches original ViNT).
        """
        self.N = N
        self.desired_v = desired_v
        self.ref_gap = ref_gap
        self.dt = dt
        self.v_max = v_max
        self.lookahead_ratio = lookahead_ratio

        # Make trajectory denser for better tracking
        self.ref_traj = self._make_ref_denser(trajectory)
        self.ref_traj_len = N // ref_gap + 1

        # Setup MPC problem with CasADi
        self._setup_mpc()

        # Store last solution for warm start
        self.last_opt_x_states: Optional[np.ndarray] = None
        self.last_opt_u_controls: Optional[np.ndarray] = None

    def _make_ref_denser(self, ref_traj: np.ndarray, ratio: int = 50) -> np.ndarray:
        """Interpolate trajectory to make it denser."""
        if len(ref_traj) < 2:
            return ref_traj

        x_orig = np.arange(len(ref_traj))
        new_x = np.linspace(0, len(ref_traj) - 1, num=len(ref_traj) * ratio)
        interp_func_x = interp1d(x_orig, ref_traj[:, 0], kind="linear")
        interp_func_y = interp1d(x_orig, ref_traj[:, 1], kind="linear")
        uniform_x = interp_func_x(new_x)
        uniform_y = interp_func_y(new_x)
        return np.stack((uniform_x, uniform_y), axis=1)

    @abstractmethod
    def _get_dynamics(self):
        """Return dynamics function for the kinematic model.

        Returns:
            Callable that takes (state, control) and returns state derivative.
        """
        pass

    @abstractmethod
    def _get_cost_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return cost matrices Q and R.

        Returns:
            Tuple of (Q, R) where Q is state cost and R is control cost.
        """
        pass

    @abstractmethod
    def _get_control_bounds(self, opti: ca.Opti, opt_controls: ca.MX):
        """Add control bound constraints to the optimization problem.

        Args:
            opti: CasADi Opti instance.
            opt_controls: Control decision variables.
        """
        pass

    @abstractmethod
    def _compute_reference_headings(self, ref_traj: np.ndarray) -> np.ndarray:
        """Compute heading angles for reference trajectory.

        Args:
            ref_traj: Reference trajectory points [N, 2].

        Returns:
            Heading angles [N].
        """
        pass

    def _setup_mpc(self):
        """Setup CasADi MPC optimization problem."""
        opti = ca.Opti()

        # Decision variables
        opt_controls = opti.variable(self.N, 2)  # [v, u2] for each step
        opt_states = opti.variable(self.N + 1, 3)  # [x, y, theta] for each step

        # Parameters
        opt_x0 = opti.parameter(3)  # Initial state
        opt_xs = opti.parameter(3 * self.ref_traj_len)  # Reference states

        # Get model-specific dynamics
        dynamics = self._get_dynamics()

        # Initial condition constraint
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # Dynamics constraints
        for i in range(self.N):
            x_next = opt_states[i, :] + dynamics(opt_states[i, :], opt_controls[i, :]).T * self.dt
            opti.subject_to(opt_states[i + 1, :] == x_next)

        # Get model-specific cost matrices
        Q, R = self._get_cost_matrices()

        # Cost function
        obj = 0
        for i in range(self.N):
            # Control cost
            obj = obj + ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T])
            # State cost at reference points
            if i % self.ref_gap == 0:
                nn = i // self.ref_gap
                ref_state = opt_xs[nn * 3 : nn * 3 + 3]
                state_error = opt_states[i, :] - ref_state.T
                obj = obj + ca.mtimes([state_error, Q, state_error.T])

        opti.minimize(obj)

        # Add model-specific control bounds
        self._get_control_bounds(opti, opt_controls)

        # Solver settings
        opts = {
            "ipopt.max_iter": 100,
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.acceptable_tol": 1e-8,
            "ipopt.acceptable_obj_change_tol": 1e-6,
        }
        opti.solver("ipopt", opts)

        self.opti = opti
        self.opt_xs = opt_xs
        self.opt_x0 = opt_x0
        self.opt_controls = opt_controls
        self.opt_states = opt_states

    def _find_reference_traj(self) -> np.ndarray:
        """Find reference trajectory points starting from lookahead ratio ahead.

        Skip a fraction of the trajectory to track points ahead of the robot.
        This makes the MPC follow curves properly.

        With lookahead_ratio=0.75, we skip the first 75% of trajectory points
        and track the last 25%.
        """
        ref_traj_pts = []

        # Calculate start index based on lookahead ratio
        # lookahead_ratio=0.75 means skip first 75% of points
        start_idx = int(len(self.ref_traj) * self.lookahead_ratio)
        # Ensure we don't go past the last point
        start_idx = min(start_idx, len(self.ref_traj) - 1)

        # Compute cumulative distance along trajectory from start_idx
        if len(self.ref_traj) > 1:
            cum_dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(self.ref_traj, axis=0), axis=1))])
        else:
            cum_dist = np.array([0])

        # Desired arc length between reference points
        desire_arc_length = self.desired_v * self.ref_gap * self.dt

        # Select reference points starting from start_idx
        for i in range(start_idx, len(self.ref_traj)):
            if i == start_idx or (cum_dist[i] - cum_dist[start_idx] >= desire_arc_length * len(ref_traj_pts)):
                ref_traj_pts.append(self.ref_traj[i, :])
                if len(ref_traj_pts) == self.ref_traj_len:
                    break

        # Pad with last point if needed
        while len(ref_traj_pts) < self.ref_traj_len:
            ref_traj_pts.append(self.ref_traj[-1, :])

        return np.array(ref_traj_pts)

    def solve(self, x0: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Solve MPC for current state.

        Args:
            x0: Current robot state [x, y, theta].

        Returns:
            Tuple of (optimal_controls [N, 2], optimal_states [N+1, 3]).
        """
        # Get reference trajectory
        ref_traj = self._find_reference_traj()
        # Compute heading angles (model-specific)
        ref_headings = self._compute_reference_headings(ref_traj)
        ref_traj_with_yaw = np.concatenate((ref_traj, ref_headings.reshape(-1, 1)), axis=1)

        # Set parameters
        self.opti.set_value(self.opt_xs, ref_traj_with_yaw.reshape(-1, 1))
        self.opti.set_value(self.opt_x0, x0)

        # Warm start from previous solution
        if self.last_opt_u_controls is not None:
            self.opti.set_initial(self.opt_controls, self.last_opt_u_controls)
        else:
            self.opti.set_initial(self.opt_controls, np.zeros((self.N, 2)))

        if self.last_opt_x_states is not None:
            self.opti.set_initial(self.opt_states, self.last_opt_x_states)
        else:
            self.opti.set_initial(self.opt_states, np.zeros((self.N + 1, 3)))

        # Solve
        try:
            sol = self.opti.solve()
            self.last_opt_u_controls = sol.value(self.opt_controls)
            self.last_opt_x_states = sol.value(self.opt_states)
        except Exception:
            # If solve fails, return zeros
            self.last_opt_u_controls = np.zeros((self.N, 2))
            self.last_opt_x_states = np.zeros((self.N + 1, 3))

        return self.last_opt_u_controls, self.last_opt_x_states

    def reset(self):
        """Reset controller state."""
        self.last_opt_x_states = None
        self.last_opt_u_controls = None


class UnicycleMPCController(BaseMPCController):
    """MPC Controller using unicycle (differential drive) kinematics.

    For robots like Nova Carter.
    State: [x, y, theta]
    Control: [v, omega] where v is linear velocity and omega is angular velocity

    Unicycle dynamics:
        dx/dt = v * cos(theta)
        dy/dt = v * sin(theta)
        dtheta/dt = omega
    """

    def __init__(
        self,
        trajectory: np.ndarray,
        N: int = 15,
        desired_v: float = 0.5,
        v_max: float = 0.5,
        w_max: float = 0.5,
        ref_gap: int = 3,
        dt: float = 0.1,
        lookahead_ratio: float = 0.4,
    ):
        self.w_max = w_max
        super().__init__(trajectory, N, desired_v, v_max, ref_gap, dt, lookahead_ratio)

    def _get_dynamics(self):
        """Unicycle dynamics: dx = v*cos(theta), dy = v*sin(theta), dtheta = omega."""

        def dynamics(x_, u_):
            return ca.vertcat(u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1])

        return dynamics

    def _get_cost_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Unicycle cost matrices (no heading tracking)."""
        Q = np.diag([10.0, 10.0, 0.0])  # State cost (x, y, theta)
        R = np.diag([0.02, 0.15])  # Control cost (v, omega)
        return Q, R

    def _get_control_bounds(self, opti: ca.Opti, opt_controls: ca.MX):
        """Unicycle control bounds: v in [0, v_max], omega in [-w_max, w_max]."""
        v, w = opt_controls[:, 0], opt_controls[:, 1]
        opti.subject_to(opti.bounded(0.0, v, self.v_max))
        opti.subject_to(opti.bounded(-self.w_max, w, self.w_max))

    def _compute_reference_headings(self, ref_traj: np.ndarray) -> np.ndarray:
        """Unicycle: zero heading (no orientation tracking)."""
        return np.zeros(len(ref_traj))


class BicycleMPCController(BaseMPCController):
    """MPC Controller using bicycle (Ackermann) kinematics.

    For robots with front-wheel steering like Segway E1.
    State: [x, y, theta]
    Control: [v, delta] where v is velocity and delta is steering angle

    Bicycle dynamics:
        dx/dt = v * cos(theta)
        dy/dt = v * sin(theta)
        dtheta/dt = (v / L) * tan(delta)

    where L is the wheelbase (distance between front and rear axles).
    """

    def __init__(
        self,
        trajectory: np.ndarray,
        N: int = 15,
        desired_v: float = 0.5,
        v_max: float = 0.5,
        delta_max: float = 0.5,
        wheelbase: float = 0.53,
        ref_gap: int = 3,
        dt: float = 0.1,
        lookahead_ratio: float = 0.4,
    ):
        self.delta_max = delta_max
        self.wheelbase = wheelbase
        super().__init__(trajectory, N, desired_v, v_max, ref_gap, dt, lookahead_ratio)

    def _get_dynamics(self):
        """Bicycle dynamics: dtheta = (v/L) * tan(delta)."""
        wheelbase = self.wheelbase

        def dynamics(x_, u_):
            return ca.vertcat(
                u_[0] * ca.cos(x_[2]),  # dx = v * cos(theta)
                u_[0] * ca.sin(x_[2]),  # dy = v * sin(theta)
                (u_[0] / wheelbase) * ca.tan(u_[1]),  # dtheta = (v/L) * tan(delta)
            )

        return dynamics

    def _get_cost_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Bicycle cost matrices (with heading tracking)."""
        Q = np.diag([10.0, 10.0, 2.0])  # State cost (x, y, theta)
        R = np.diag([0.02, 0.1])  # Control cost (v, delta)
        return Q, R

    def _get_control_bounds(self, opti: ca.Opti, opt_controls: ca.MX):
        """Bicycle control bounds: v in [0, v_max], delta in [-delta_max, delta_max]."""
        v, delta = opt_controls[:, 0], opt_controls[:, 1]
        opti.subject_to(opti.bounded(0.0, v, self.v_max))
        opti.subject_to(opti.bounded(-self.delta_max, delta, self.delta_max))

    def _compute_reference_headings(self, ref_traj: np.ndarray) -> np.ndarray:
        """Bicycle: compute heading from trajectory direction."""
        headings = np.zeros(len(ref_traj))
        for i in range(len(ref_traj) - 1):
            dx = ref_traj[i + 1, 0] - ref_traj[i, 0]
            dy = ref_traj[i + 1, 1] - ref_traj[i, 1]
            headings[i] = np.arctan2(dy, dx)
        # Last point uses same heading as second-to-last
        if len(ref_traj) > 1:
            headings[-1] = headings[-2]
        return headings


class TrajectoryFollowerNode(Node):
    """ROS2 Node for trajectory following control.

    Subscribes to:
        - /vint_trajectory (nav_msgs/Path) - trajectory from ViNT policy (world frame)
        - /odom (nav_msgs/Odometry) - robot odometry for state feedback
        - /trajectory_follower_enable (std_msgs/Bool) - enable/disable control

    Publishes:
        - /cmd_vel (geometry_msgs/Twist) - velocity commands at control_rate Hz
    """

    def __init__(
        self,
        robot_config: str,
    ):
        super().__init__("trajectory_follower_node")

        # Load robot config — all parameters come from here
        with open(robot_config, "r") as f:
            robot_cfg = yaml.safe_load(f)

        # Get odom topic from config (default: /chassis/odom)
        self.odom_topic = robot_cfg.get("topics", {}).get("odom", "/chassis/odom")

        # Trajectory follower parameters from config
        follower_cfg = robot_cfg.get("trajectory_follower", {})
        self.control_rate = follower_cfg.get("control_rate", 20.0)
        self.max_linear_vel = follower_cfg.get("max_linear_vel", 0.8)
        self.max_angular_vel = follower_cfg.get("max_angular_vel", 0.5)
        self.trajectory_timeout = follower_cfg.get("trajectory_timeout", 0.5)

        # Kinematic model configuration
        self.kinematic_model = follower_cfg.get("kinematic_model", "unicycle")
        self.wheelbase = follower_cfg.get("wheelbase", 0.5)  # For bicycle model
        self.max_steering_angle = follower_cfg.get("max_steering_angle", 0.5)  # For bicycle model
        # MPC lookahead: fraction of trajectory to skip (0.4 = skip first 40%, matches original ViNT)
        self.mpc_lookahead_ratio = follower_cfg.get("mpc_lookahead_ratio", 0.4)

        self.get_logger().info(f"Kinematic model: {self.kinematic_model}")
        if self.kinematic_model == "bicycle":
            self.get_logger().info(f"  Wheelbase: {self.wheelbase}m, Max steering: {self.max_steering_angle}rad")
        self.get_logger().info(
            f"  MPC lookahead ratio: {self.mpc_lookahead_ratio} (skip first {self.mpc_lookahead_ratio * 100:.0f}% of trajectory)"
        )

        # MPC parameters (matching NavDP defaults)
        self.mpc_horizon = 15
        self.mpc_ref_gap = 3
        self.mpc_dt = 0.1

        # State variables
        self.current_trajectory: Optional[np.ndarray] = None
        self.trajectory_timestamp: Optional[float] = None
        self.trajectory_frame_id: str = "base_link"
        self.current_odom: Optional[Odometry] = None
        # Controller will be created based on kinematic model
        self.mpc_controller: Optional[BaseMPCController] = None
        self.enabled = True

        # Thread lock for trajectory updates
        self.trajectory_lock = threading.Lock()

        # Logging counters for interval-based logging
        self._trajectory_receive_count = 0
        self._cmd_vel_publish_count = 0
        self._log_interval = 20  # Log every N events

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.trajectory_sub = self.create_subscription(Path, "/vint_trajectory", self.trajectory_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, sensor_qos)
        self.enable_sub = self.create_subscription(Bool, "/trajectory_follower_enable", self.enable_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Control timer - runs at control_rate Hz
        timer_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(timer_period, self.control_callback)

        self.get_logger().info(f"Trajectory follower node started. Control rate: {self.control_rate} Hz")
        self.get_logger().info("Controller: MPC (NavDP reference)")
        self.get_logger().info(f"Max velocities: linear={self.max_linear_vel}, angular={self.max_angular_vel}")
        self.get_logger().info(f"Subscribing to: /vint_trajectory, {self.odom_topic}")
        self.get_logger().info("Publishing to: /cmd_vel")

    def trajectory_callback(self, msg: Path):
        """Process incoming trajectory from ViNT policy.

        The trajectory can be in either:
        - World frame (frame_id != "base_link")
        - Robot-local frame (frame_id == "base_link"): Will be transformed to world frame
        """
        try:
            # Convert Path message to numpy array
            waypoints = []
            for pose_stamped in msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                waypoints.append([x, y])

            if len(waypoints) == 0:
                return

            trajectory = np.array(waypoints)
            frame_id = msg.header.frame_id

            with self.trajectory_lock:
                self.current_trajectory = trajectory
                self.trajectory_timestamp = self.get_clock().now().nanoseconds / 1e9
                self.trajectory_frame_id = frame_id

                # Create/update MPC controller when new trajectory arrives
                # For MPC, we need world frame trajectory
                # If trajectory is in local frame, transform it
                if frame_id == "base_link" and self.current_odom is not None:
                    world_trajectory = self._transform_to_world_frame(trajectory)
                else:
                    world_trajectory = trajectory

                if self.mpc_controller is None:
                    # First trajectory — build the CasADi problem once
                    if self.kinematic_model == "bicycle":
                        self.mpc_controller = BicycleMPCController(
                            trajectory=world_trajectory,
                            N=self.mpc_horizon,
                            desired_v=self.max_linear_vel,
                            v_max=self.max_linear_vel,
                            delta_max=self.max_steering_angle,
                            wheelbase=self.wheelbase,
                            ref_gap=self.mpc_ref_gap,
                            dt=self.mpc_dt,
                            lookahead_ratio=self.mpc_lookahead_ratio,
                        )
                    else:
                        # Default: unicycle model
                        self.mpc_controller = UnicycleMPCController(
                            trajectory=world_trajectory,
                            N=self.mpc_horizon,
                            desired_v=self.max_linear_vel,
                            v_max=self.max_linear_vel,
                            w_max=self.max_angular_vel,
                            ref_gap=self.mpc_ref_gap,
                            dt=self.mpc_dt,
                            lookahead_ratio=self.mpc_lookahead_ratio,
                        )
                else:
                    # Reuse existing MPC: update ref trajectory, keep warm start
                    self.mpc_controller.ref_traj = self.mpc_controller._make_ref_denser(world_trajectory)

                # Log trajectory receive at interval
                self._trajectory_receive_count += 1
                if self._trajectory_receive_count % self._log_interval == 0:
                    # Calculate trajectory extent
                    traj_start = world_trajectory[0]
                    traj_end = world_trajectory[-1]
                    traj_length = np.sum(np.linalg.norm(np.diff(world_trajectory, axis=0), axis=1))
                    self.get_logger().info(
                        f"[Follower] traj #{self._trajectory_receive_count}: "
                        f"len={traj_length:.2f}m, start=({traj_start[0]:.2f}, {traj_start[1]:.2f}), "
                        f"end=({traj_end[0]:.2f}, {traj_end[1]:.2f})"
                    )

            self.get_logger().debug(f"Received trajectory with {len(waypoints)} waypoints in frame '{frame_id}'")

        except Exception as e:
            self.get_logger().error(f"Failed to process trajectory: {e}")

    def _transform_to_world_frame(self, local_trajectory: np.ndarray) -> np.ndarray:
        """Transform trajectory from robot-local frame to world frame using full 3D rotation.

        Uses the full quaternion-based 3D rotation matrix instead of yaw-only
        2D rotation, so that the transformation remains accurate even when the
        robot has non-zero roll or pitch (e.g. on slopes).

        Args:
            local_trajectory: Trajectory [N, 2] in robot-local frame.

        Returns:
            Trajectory [N, 2] in world frame.
        """
        if self.current_odom is None:
            return local_trajectory

        # Get robot pose from odometry
        pos = self.current_odom.pose.pose.position
        quat = self.current_odom.pose.pose.orientation
        robot_x, robot_y = pos.x, pos.y

        # Get full 3D rotation matrix from quaternion
        # transforms3d quat2mat uses (w, x, y, z) order → 3×3 rotation matrix
        rotation_matrix = quat2mat([quat.w, quat.x, quat.y, quat.z])

        # Transform each point using full 3D rotation
        world_trajectory = np.zeros_like(local_trajectory)
        for i, point in enumerate(local_trajectory):
            # Extend 2D point to 3D (z=0 for ground-plane trajectory)
            point_3d = np.array([point[0], point[1], 0.0])
            # Apply rotation and translation
            rotated = rotation_matrix @ point_3d
            world_trajectory[i, 0] = robot_x + rotated[0]
            world_trajectory[i, 1] = robot_y + rotated[1]

        return world_trajectory

    def _get_robot_state(self) -> Optional[np.ndarray]:
        """Get current robot state [x, y, theta] from odometry."""
        if self.current_odom is None:
            return None

        pos = self.current_odom.pose.pose.position
        quat = self.current_odom.pose.pose.orientation

        # Convert quaternion to yaw using transforms3d
        # transforms3d uses (w, x, y, z) order, returns (roll, pitch, yaw) for 'sxyz' axes
        _, _, theta = quat2euler([quat.w, quat.x, quat.y, quat.z])

        return np.array([pos.x, pos.y, theta])

    def odom_callback(self, msg: Odometry):
        """Process incoming odometry."""
        self.current_odom = msg

    def enable_callback(self, msg: Bool):
        """Enable/disable trajectory following."""
        self.enabled = msg.data
        status = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Trajectory follower {status}")
        if not self.enabled:
            # Stop the robot when disabled
            self.cmd_vel_pub.publish(Twist())
            # Reset MPC controller
            if self.mpc_controller is not None:
                self.mpc_controller.reset()

    def control_callback(self):
        """Execute trajectory following control at fixed rate.

        Similar to the control loop in eval_imagegoal_wheeled.py:
        - Gets current robot state from odometry
        - Uses MPC or pure pursuit to compute velocity command
        - Publishes cmd_vel
        """
        if not self.enabled:
            return

        # Check if we have a valid trajectory
        with self.trajectory_lock:
            if self.current_trajectory is None or self.trajectory_timestamp is None:
                return

            # Check trajectory timeout
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.trajectory_timestamp > self.trajectory_timeout:
                self.get_logger().debug("Trajectory timed out, stopping")
                self.cmd_vel_pub.publish(Twist())
                return

            mpc = self.mpc_controller

        # Skip if MPC is not ready (matching NavDP behavior: if mpc is None: continue)
        if mpc is None:
            return

        # Compute control command using MPC
        cmd_vel = self._compute_mpc_command(mpc)
        self.cmd_vel_pub.publish(cmd_vel)

        # Log cmd_vel publish at interval
        self._cmd_vel_publish_count += 1
        if self._cmd_vel_publish_count % self._log_interval == 0:
            robot_state = self._get_robot_state()
            robot_str = (
                f"robot=({robot_state[0]:.2f}, {robot_state[1]:.2f})" if robot_state is not None else "robot=None"
            )
            self.get_logger().info(
                f"[Follower] cmd_vel #{self._cmd_vel_publish_count}: "
                f"v={cmd_vel.linear.x:.3f} m/s, w={cmd_vel.angular.z:.3f} rad/s, {robot_str}"
            )

    def _compute_mpc_command(self, mpc: BaseMPCController) -> Twist:
        """Compute velocity command using MPC controller.

        Similar to eval_imagegoal_wheeled.py:
            opt_u_controls, opt_x_states = mpc.solve(x0[i,:3])
            v, w = opt_u_controls[1, 0], opt_u_controls[1, 1]

        For unicycle model: control = [v, ω] (angular velocity)
        For bicycle model: control = [v, δ] (steering angle)
            - Convert steering angle to angular velocity: ω = (v/L) * tan(δ)
            - If trajectory points are close, rotate in place instead

        Args:
            mpc: MPC controller instance.

        Returns:
            Twist message with linear and angular velocities.
        """
        twist = Twist()

        # Get current robot state
        robot_state = self._get_robot_state()
        if robot_state is None:
            self.get_logger().debug("No odometry available for MPC")
            return twist

        try:
            # Solve MPC
            opt_u_controls, _ = mpc.solve(robot_state)

            # Use second control output (index 1), same as NavDP
            # This provides a smoother response by looking one step ahead
            v = float(opt_u_controls[1, 0])

            if self.kinematic_model == "bicycle":
                # Bicycle model: control = [v, δ] where δ is steering angle
                # Convert steering angle to angular velocity: ω = (v/L) * tan(δ)
                delta = float(opt_u_controls[1, 1])
                w = (v / self.wheelbase) * np.tan(delta)
            else:
                # Unicycle model: control = [v, ω]
                w = float(opt_u_controls[1, 1])

            twist.linear.x = v
            twist.angular.z = w

        except Exception as e:
            self.get_logger().error(f"MPC solve failed: {e}")

        return twist


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Trajectory Follower ROS2 Node (MPC Controller)")
    parser.add_argument(
        "--robot_config",
        type=str,
        required=True,
        help="Path to robot configuration YAML (contains topics and trajectory_follower params)",
    )
    parser.add_argument(
        "--log_level",
        type=str,
        default="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        help="Log level (default: info)",
    )
    return parser.parse_args()


def main():
    """Main entry point for the trajectory follower node."""
    args = parse_args()

    rclpy.init()

    try:
        node = TrajectoryFollowerNode(
            robot_config=args.robot_config,
        )
        # Set log level
        log_level_map = {
            "debug": rclpy.logging.LoggingSeverity.DEBUG,
            "info": rclpy.logging.LoggingSeverity.INFO,
            "warn": rclpy.logging.LoggingSeverity.WARN,
            "error": rclpy.logging.LoggingSeverity.ERROR,
            "fatal": rclpy.logging.LoggingSeverity.FATAL,
        }
        node.get_logger().set_level(log_level_map[args.log_level])
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error starting trajectory follower node: {e}")
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
