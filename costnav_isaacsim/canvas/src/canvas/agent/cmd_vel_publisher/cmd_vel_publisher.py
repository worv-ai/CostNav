from dataclasses import dataclass
from pathlib import Path

import message_filters
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, TimeReference
from std_msgs.msg import Bool

from canvas.agent.cmd_vel_publisher.vel_data import VelData
from canvas.agent.utils.pointcloud2_utils import extract_array_from_pointcloud2


@dataclass
class CMDVelPublisherConfig:
    # linear and angular speed limits for the robot
    stop_topic: str = "/stop"
    pred_vel_topic: str = "/vel_predict"
    vel_topic: str = "/cmd_vel"
    model_latency_topic: str = "/model_latency"

    # position
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    vel_timeout: float = 1  # seconds
    rate: float = 20  # Hz

    max_v: float = 15  # 0.2 # m/s
    max_w: float = 10  # 2.8 # rad/s

    # proportional gain for linear velocity and angular velocity
    v_gain: float = 1
    w_gain: float = 1

    # Velocity filtering: if abs(velocity) < threshold, set it to 0
    zero_velocity_threshold_linear: float = 0.01
    zero_velocity_threshold_angular: float = 0.01

    # latency compensation
    enable_latency_compensation: bool = True  # Enable/disable model latency compensation

    # message type: "TwistStamped" or "Twist"
    cmd_vel_msg_type: str = "TwistStamped"

    # Raw (pre-gain) cmd_vel topic for model input feedback
    cmd_vel_model_input_topic: str = "/cmd_vel_model_input"


class CMDVelPublisher(Node):
    def __init__(self, config=CMDVelPublisherConfig()):
        super().__init__("CMDVelPublisher")
        self.config = config

        # init variables
        self.vel = VelData(
            self.get_clock(), self.config.vel_timeout, name="vel", rate=self.config.rate, ref_logger=self.get_logger()
        )
        self.stop = False

        # init ROS
        self._init_ros()

    def _init_ros(self):
        # Set up message filters for synchronization
        self.pred_vel_sub = message_filters.Subscriber(self, PointCloud2, self.config.pred_vel_topic)
        self.model_latency_sub = message_filters.Subscriber(self, TimeReference, self.config.model_latency_topic)

        # Use ApproximateTimeSynchronizer for robust synchronization
        # slop=0.1 allows up to 100ms difference between message timestamps
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.pred_vel_sub, self.model_latency_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Regular subscriber for stop messages (no synchronization needed)
        self.create_subscription(Bool, self.config.stop_topic, self.callback_stop, 1)

        if self.config.cmd_vel_msg_type == "TwistStamped":
            self._msg_cls = TwistStamped
        elif self.config.cmd_vel_msg_type == "Twist":
            self._msg_cls = Twist
        else:
            raise ValueError(
                f"Unsupported cmd_vel_msg_type: '{self.config.cmd_vel_msg_type}'. Must be 'TwistStamped' or 'Twist'."
            )

        self.vel_out = self.create_publisher(self._msg_cls, self.config.vel_topic, 1)
        self.vel_model_input_out = self.create_publisher(self._msg_cls, self.config.cmd_vel_model_input_topic, 1)
        self.create_timer(1.0 / self.config.rate, self._run)
        self.context.on_shutdown(self.shutdown_callback)
        self.get_logger().info(
            f"Registered with master node (msg_type={self.config.cmd_vel_msg_type}). Waiting for synchronized vels..."
        )

    def _create_cmd_vel_msg(self, linear_x=0.0, angular_z=0.0):
        if self._msg_cls is TwistStamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "model"
            msg.twist.linear.x = linear_x
            msg.twist.angular.z = angular_z
        else:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
        return msg

    def shutdown_callback(self):
        self.vel_out.publish(self._create_cmd_vel_msg())
        self.vel_model_input_out.publish(self._create_cmd_vel_msg())

    def _run(self):
        if self.stop:
            # publish zero velocity
            self.vel_out.publish(self._create_cmd_vel_msg())
            self.vel_model_input_out.publish(self._create_cmd_vel_msg())
            return
        if not self.vel.is_valid(verbose=True):
            self.vel_out.publish(self._create_cmd_vel_msg())
            self.vel_model_input_out.publish(self._create_cmd_vel_msg())
            return

        # Get next velocity (latency compensation already applied at ingestion)
        vel_result = self.vel.get()
        if vel_result is None:
            self.vel_out.publish(self._create_cmd_vel_msg())
            self.vel_model_input_out.publish(self._create_cmd_vel_msg())
            return
        v, w = vel_result

        # Publish raw (pre-gain) cmd_vel for model input feedback
        self.vel_model_input_out.publish(self._create_cmd_vel_msg(linear_x=float(v), angular_z=float(w)))

        v = self.config.v_gain * v
        w = self.config.w_gain * w
        v = np.clip(v, -self.config.max_v, self.config.max_v)
        w = np.clip(w, -self.config.max_w, self.config.max_w)

        # Apply zero-velocity threshold filtering (per-axis)
        t_lin = self.config.zero_velocity_threshold_linear
        t_ang = self.config.zero_velocity_threshold_angular
        if t_lin > 0.0 and abs(v) < t_lin:
            v = 0.0
        if t_ang > 0.0 and abs(w) < t_ang:
            w = 0.0

        self.get_logger().debug(f"Publishing velocity: linear={v:.3f}, angular={w:.3f}")
        self.vel_out.publish(self._create_cmd_vel_msg(linear_x=float(v), angular_z=float(w)))

    def synchronized_callback(self, vel_msg: PointCloud2, latency_msg: TimeReference):
        """Synchronized callback for velocity and latency messages using message_filters"""
        try:
            # Extract velocity data from PointCloud2 message
            data = extract_array_from_pointcloud2(vel_msg, include_z=False)

            # Calculate delta_t from TimeReference message
            delta_t = self._calculate_delta_t(latency_msg)

            # Apply latency compensation based on configuration
            # VelData.set() handles all validation internally
            if self.config.enable_latency_compensation:
                success = self.vel.set(data, delta_t)
            else:
                success = self.vel.set(data, delta_t=None)

            # Log at debug level to reduce verbosity in production
            if success:
                vel_ts = self._stamp_to_sec(vel_msg.header.stamp)
                latency_ts = self._stamp_to_sec(latency_msg.header.stamp)
                self.get_logger().debug(
                    f"Synchronized messages processed: vel_ts={vel_ts:.3f}, "
                    f"latency_ts={latency_ts:.3f}, delta_t={delta_t:.3f}s, "
                    f"compensation={'enabled' if self.config.enable_latency_compensation else 'disabled'}"
                )

        except Exception as e:
            self.get_logger().error(f"Error processing synchronized velocity/latency messages: {e}")
            # Don't update velocity data on error to maintain safety

    def callback_stop(self, stop_msg: Bool):
        """Callback function for the reached goal subscriber"""
        # Log stop only once
        if stop_msg.data and not self.stop:
            self.get_logger().info("Stop message received")
        self.stop = stop_msg.data
        # Re-initialize VelData when stop message is received
        self.vel = VelData(
            self.get_clock(), self.config.vel_timeout, name="vel", rate=self.config.rate, ref_logger=self.get_logger()
        )

    def _stamp_to_sec(self, stamp) -> float:
        """Convert a ROS2 Time stamp to seconds as a float."""
        return stamp.sec + stamp.nanosec * 1e-9

    def _calculate_delta_t(self, latency_msg: TimeReference):
        """
        Calculate delta_t from TimeReference message with validation.

        Args:
            latency_msg: TimeReference message containing timing information

        Returns:
            float: Time difference in seconds (t2 - t1)

        Raises:
            ValueError: If timing data is invalid
        """
        try:
            t1 = self._stamp_to_sec(latency_msg.time_ref)  # state extraction time
            t2 = self._stamp_to_sec(latency_msg.header.stamp)  # action publish time
            delta_t = t2 - t1

            # Validate delta_t is reasonable (not negative and not too large)
            if delta_t < 0:
                self.get_logger().warning(f"Negative delta_t calculated: {delta_t:.3f}s, using 0")
                return 0.0
            elif delta_t > 1.0:  # More than 1 second seems unreasonable
                self.get_logger().warning(f"Large delta_t calculated: {delta_t:.3f}s, capping at 1.0s")
                return 1.0

            return delta_t

        except Exception as e:
            self.get_logger().error(f"Error calculating delta_t from TimeReference: {e}")
            return 0.0  # Safe fallback


def main(config_path: Path | None):
    rclpy.init()
    if config_path is None:
        controller = CMDVelPublisher()
    else:
        config_path = Path(config_path)
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        controller = CMDVelPublisher(CMDVelPublisherConfig(**config))

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="PD Controller for the robot")
    parser.add_argument("--config", type=str, help="Path to the configuration file", default=None)
    args = parser.parse_args()

    main(args.config)
