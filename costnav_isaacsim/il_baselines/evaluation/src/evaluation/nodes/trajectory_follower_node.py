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
    - ViNT policy node: runs inference at ~10 Hz, publishes full trajectory
    - Trajectory follower node: receives trajectory, publishes cmd_vel at ~20 Hz

Usage:
    python3 trajectory_follower_node.py \
        --control_rate 20.0 \
        --max_linear_vel 0.5 \
        --max_angular_vel 0.5
"""

import argparse
import threading
from typing import Optional, Tuple

import casadi as ca
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.interpolate import interp1d
from std_msgs.msg import Bool
from transforms3d.euler import quat2euler


class MPCController:
    """MPC Controller for trajectory tracking.

    Based on third_party/NavDP/utils_tasks/tracking_utils.py MPC_Controller.
    Uses CasADi for optimization.
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
    ):
        """Initialize MPC controller.

        Args:
            trajectory: Reference trajectory [M, 2] in world frame (x, y).
            N: MPC horizon length.
            desired_v: Desired velocity for reference point spacing.
            v_max: Maximum linear velocity.
            w_max: Maximum angular velocity.
            ref_gap: Gap between reference points in horizon.
            dt: Time step for dynamics.
        """
        self.N = N
        self.desired_v = desired_v
        self.ref_gap = ref_gap
        self.dt = dt
        self.v_max = v_max
        self.w_max = w_max

        # Make trajectory denser for better tracking
        self.ref_traj = self._make_ref_denser(trajectory)
        self.ref_traj_len = N // ref_gap + 1

        # Setup MPC problem with CasADi
        self._setup_mpc(v_max, w_max)

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

    def _setup_mpc(self, v_max: float, w_max: float):
        """Setup CasADi MPC optimization problem."""
        opti = ca.Opti()

        # Decision variables
        opt_controls = opti.variable(self.N, 2)  # [v, w] for each step
        v, w = opt_controls[:, 0], opt_controls[:, 1]

        opt_states = opti.variable(self.N + 1, 3)  # [x, y, theta] for each step

        # Parameters
        opt_x0 = opti.parameter(3)  # Initial state
        opt_xs = opti.parameter(3 * self.ref_traj_len)  # Reference states

        # Unicycle dynamics: dx = v*cos(theta), dy = v*sin(theta), dtheta = w
        def dynamics(x_, u_):
            return ca.vertcat(u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1])

        # Initial condition constraint
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # Dynamics constraints
        for i in range(self.N):
            x_next = opt_states[i, :] + dynamics(opt_states[i, :], opt_controls[i, :]).T * self.dt
            opti.subject_to(opt_states[i + 1, :] == x_next)

        # Cost function
        Q = np.diag([10.0, 10.0, 0.0])  # State cost (x, y, theta)
        R = np.diag([0.02, 0.15])  # Control cost (v, w)

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

        # Control bounds
        opti.subject_to(opti.bounded(0.0, v, v_max))
        opti.subject_to(opti.bounded(-w_max, w, w_max))

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

    def _find_reference_traj(self, x0: np.ndarray) -> np.ndarray:
        """Find reference trajectory points starting from nearest point to x0."""
        ref_traj_pts = []

        # Find nearest point on trajectory
        distances = np.linalg.norm(self.ref_traj - x0[:2].reshape((1, 2)), axis=1)
        nearest_idx = np.argmin(distances)

        # Compute cumulative distance along trajectory
        if len(self.ref_traj) > 1:
            cum_dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(self.ref_traj, axis=0), axis=1))])
        else:
            cum_dist = np.array([0])

        # Desired arc length between reference points
        desire_arc_length = self.desired_v * self.ref_gap * self.dt

        # Select reference points
        for i in range(nearest_idx, len(self.ref_traj)):
            if i == nearest_idx or (cum_dist[i] - cum_dist[nearest_idx] >= desire_arc_length * len(ref_traj_pts)):
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
        ref_traj = self._find_reference_traj(x0)
        # Add zero yaw angle to reference (we don't track orientation)
        ref_traj_with_yaw = np.concatenate((ref_traj, np.zeros((ref_traj.shape[0], 1))), axis=1)

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
        control_rate: float = 20.0,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 0.5,
        trajectory_timeout: float = 0.5,
    ):
        super().__init__("trajectory_follower_node")

        # Control parameters
        self.control_rate = control_rate
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.trajectory_timeout = trajectory_timeout

        # MPC parameters (matching NavDP defaults)
        self.mpc_horizon = 15
        self.mpc_ref_gap = 3
        self.mpc_dt = 0.1

        # State variables
        self.current_trajectory: Optional[np.ndarray] = None
        self.trajectory_timestamp: Optional[float] = None
        self.trajectory_frame_id: str = "base_link"
        self.current_odom: Optional[Odometry] = None
        self.mpc_controller: Optional[MPCController] = None
        self.enabled = True

        # Thread lock for trajectory updates
        self.trajectory_lock = threading.Lock()

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.trajectory_sub = self.create_subscription(Path, "/vint_trajectory", self.trajectory_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, sensor_qos)
        self.enable_sub = self.create_subscription(Bool, "/trajectory_follower_enable", self.enable_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Control timer - runs at control_rate Hz
        timer_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(timer_period, self.control_callback)

        self.get_logger().info(f"Trajectory follower node started. Control rate: {self.control_rate} Hz")
        self.get_logger().info("Controller: MPC (NavDP reference)")
        self.get_logger().info(f"Max velocities: linear={self.max_linear_vel}, angular={self.max_angular_vel}")
        self.get_logger().info("Subscribing to: /vint_trajectory, /odom")
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

                self.mpc_controller = MPCController(
                    trajectory=world_trajectory,
                    N=self.mpc_horizon,
                    desired_v=self.max_linear_vel,
                    v_max=self.max_linear_vel,
                    w_max=self.max_angular_vel,
                    ref_gap=self.mpc_ref_gap,
                    dt=self.mpc_dt,
                )

            self.get_logger().debug(f"Received trajectory with {len(waypoints)} waypoints in frame '{frame_id}'")

        except Exception as e:
            self.get_logger().error(f"Failed to process trajectory: {e}")

    def _transform_to_world_frame(self, local_trajectory: np.ndarray) -> np.ndarray:
        """Transform trajectory from robot-local frame to world frame.

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

        # Convert quaternion to yaw angle using transforms3d
        # transforms3d uses (w, x, y, z) order, returns (ai, aj, ak) = (roll, pitch, yaw) for 'sxyz' axes
        _, _, robot_theta = quat2euler([quat.w, quat.x, quat.y, quat.z])

        # Rotation matrix
        cos_theta = np.cos(robot_theta)
        sin_theta = np.sin(robot_theta)

        # Transform each point
        world_trajectory = np.zeros_like(local_trajectory)
        for i, point in enumerate(local_trajectory):
            # Rotate and translate
            world_trajectory[i, 0] = robot_x + cos_theta * point[0] - sin_theta * point[1]
            world_trajectory[i, 1] = robot_y + sin_theta * point[0] + cos_theta * point[1]

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

    def _compute_mpc_command(self, mpc: MPCController) -> Twist:
        """Compute velocity command using MPC controller.

        Similar to eval_imagegoal_wheeled.py:
            opt_u_controls, opt_x_states = mpc.solve(x0[i,:3])
            v, w = opt_u_controls[1, 0], opt_u_controls[1, 1]

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
            w = float(opt_u_controls[1, 1])

            twist.linear.x = v
            twist.angular.z = w

        except Exception as e:
            self.get_logger().error(f"MPC solve failed: {e}")

        return twist


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Trajectory Follower ROS2 Node (MPC Controller)")
    parser.add_argument("--control_rate", type=float, default=20.0, help="Control frequency in Hz (default: 20.0)")
    parser.add_argument(
        "--max_linear_vel",
        type=float,
        default=0.5,
        help="Maximum linear velocity m/s (default: 0.5, matching NavDP)",
    )
    parser.add_argument(
        "--max_angular_vel",
        type=float,
        default=0.5,
        help="Maximum angular velocity rad/s (default: 0.5, matching NavDP)",
    )
    parser.add_argument(
        "--trajectory_timeout",
        type=float,
        default=0.5,
        help="Trajectory validity timeout in seconds (default: 0.5)",
    )
    return parser.parse_args()


def main():
    """Main entry point for the trajectory follower node."""
    args = parse_args()

    rclpy.init()

    try:
        node = TrajectoryFollowerNode(
            control_rate=args.control_rate,
            max_linear_vel=args.max_linear_vel,
            max_angular_vel=args.max_angular_vel,
            trajectory_timeout=args.trajectory_timeout,
        )
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
