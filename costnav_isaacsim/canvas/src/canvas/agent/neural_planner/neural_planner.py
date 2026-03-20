import logging
import os
import threading
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import copy, deepcopy
from dataclasses import dataclass

import cv2
import numpy as np
import requests

from canvas.agent.neural_planner.scenarios import AnnotatedScenario, Scenario
from canvas.agent.neural_planner.map import SketchMap
from canvas.agent.neural_planner.image_utils import image_to_b64
from canvas.agent.neural_planner.ros_utils import euler_from_quaternion
from canvas.agent.neural_planner.processor import CanvasOnlineProcessor
from canvas.agent.utils.pointcloud2_utils import create_pointcloud2_from_array

# ROS imports
...
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, TimeReference
from std_msgs.msg import Bool, Header, Int32MultiArray, String

MODEL_WORKER_URI = os.getenv("MODEL_WORKER_URI", "http://localhost:8200")


@dataclass
class NeuralPlannerConfig:
    # subscribing topics
    start_pause_topic: str = "/start_pause"
    stop_model_topic: str = "/stop_model"

    exit_topic: str = "/exit_signal"

    instruction_scenario_topic: str = "/instruction_scenario"
    instruction_annotation_topic: str = "/instruction_annotation"

    odom_topic: str = "/odom"
    cmd_vel_topic: str = "/cmd_vel"
    cmd_vel_model_input_topic: str = "/cmd_vel_model_input"  # raw (pre-gain) cmd_vel for model action history
    cmd_vel_msg_type: str = "TwistStamped"  # "TwistStamped" or "Twist"
    rgb_front_topic: str = "/rgb_front/compressed"
    rgb_image_type: str = "Image"  # CompressedImage | Image

    eval_resetting_topic: str = "/eval_resetting"

    # publishing topics
    eval_start_topic: str = "/eval_start_signal"

    action_topic: str = "/vel_predict"
    stop_topic: str = "/stop"

    model_state_topic: str = "/model_state"
    model_latency_topic: str = "/model_latency"

    reached_goal_topic: str = "/reached_goal"
    stopped_before_goal_topic: str = "/stopped_before_goal"

    # local map publishing topics
    image_visualization_topic: str = "/image_visualization"
    local_map_topic: str = "/local_map"
    local_state_topic: str = "/local_map/local_state"
    local_point_stamped_topic: str = "/local_map/point_stamped"

    # map config
    map_yaml: str = "maps/sidewalk.yaml"  # path to the map yaml file

    # other config
    rate: float = 20
    reached_distance: float = 0.7
    visualize: bool = True
    is_eval: bool = False
    num_action: int = 20
    action_dim: int = 2  # Dimension of each action (e.g., 2 for [x, y] velocity)

    stopped_timeout: float = 10  # seconds
    stopped_threshold: float = 0.5  # meters
    stopped_angular_threshold: float = 0.5  # radians

    signal_reached_distance: float = 2.0

    stale_msg_threshold: float = 0.5  # Warn if a sensor message is older than this many seconds


class NeuralPlanner(Node):
    def __init__(self, config: NeuralPlannerConfig):
        super().__init__("neural_planner")
        self.config = config
        self.logger = self.get_logger()

        # initialize subscribers
        self.create_subscription(Bool, config.start_pause_topic, self.start_pause_callback, 1)
        self.create_subscription(Bool, config.stop_model_topic, self.stop_model_callback, 1)
        self.create_subscription(Bool, config.exit_topic, self.exit_callback, 1)

        self.create_subscription(String, config.instruction_scenario_topic, self.instruction_scenario_callback, 1)
        self.create_subscription(
            Int32MultiArray, config.instruction_annotation_topic, self.instruction_annotation_callback, 1
        )

        self._init_states()
        self.initialize_sensors(config)

        self.create_subscription(Bool, config.eval_resetting_topic, self.eval_resetting_callback, 10)

        # initialize publishers
        self.eval_start_publisher = self.create_publisher(Time, config.eval_start_topic, 1)
        self.action_publisher = self.create_publisher(PointCloud2, config.action_topic, 1)
        self.stop_publisher = self.create_publisher(Bool, config.stop_topic, 1)
        self.model_state_publisher = self.create_publisher(String, config.model_state_topic, 1)
        self.model_latency_publisher = self.create_publisher(TimeReference, config.model_latency_topic, 1)

        self.reached_goal_publisher = self.create_publisher(Bool, config.reached_goal_topic, 1)
        self.stopped_before_goal_publisher = self.create_publisher(Bool, config.stopped_before_goal_topic, 1)

        # local map publishers (only if visualize is enabled)
        if config.visualize:
            self.local_map_publisher = self.create_publisher(Image, config.local_map_topic, 1)
            self.local_state_publisher = self.create_publisher(
                CompressedImage, f"{config.local_state_topic}/compressed", 1
            )
            self.local_point_stamped_publisher = self.create_publisher(
                PointStamped, config.local_point_stamped_topic, 1
            )
            self.image_visualization_publisher = self.create_publisher(Image, config.image_visualization_topic, 1)

        self.logger.info("All publishers and subscribers are connected.")

        # initialize scenario
        self.scenario = None
        self.annotation = None

        self._processor_lock = threading.Lock()  # Guards processor access across main + inference threads

        self.processor = None
        self._init_processor()

        self.new_scenario = False
        self.new_annotation = False

        # initialize states
        self.state = "init"
        self.eval_reset_in_progress = False
        self.eval_reset_done = 0
        self.eval_reset_wait_start = None

        # Thread pool for non-blocking model inference (single worker to serialize calls)
        self._inference_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="inference")
        self._inference_in_flight = threading.Event()  # Set while inference is running

        self.create_timer(1.0 / self.config.rate, self.loop)

        self.context.on_shutdown(self.shutdown_callback)

    def _init_states(self):
        self.last_odom = None
        self.odom_queue: deque[Odometry] = deque(maxlen=int(self.config.stopped_timeout * self.config.rate))
        self.last_rgb = None

        self.last_predicted_time = self._cur_stamp_sec()

    def _init_processor(self):
        # TODO: Currently uses a fixed map from map_yaml config.
        # Scenario fields (map_name, sketch_map_name, drive_map_name) are ignored.
        # For future expansion, _init_processor could load the map dynamically from the scenario.
        sketch_map = SketchMap.from_yaml(self.config.map_yaml)
        self.processor = CanvasOnlineProcessor(sketch_map=sketch_map)

    def initialize_sensors(self, config: NeuralPlannerConfig):
        self.create_subscription(Odometry, config.odom_topic, self.odom_callback, 1)

        if config.rgb_image_type == "CompressedImage":
            image_type = CompressedImage
        elif config.rgb_image_type == "Image":
            image_type = Image
        self.create_subscription(image_type, config.rgb_front_topic, self.rgb_callback, 1)

        # cmd_vel subscriber - resolve message type
        if config.cmd_vel_msg_type == "TwistStamped":
            cmd_vel_msg_cls = TwistStamped
        elif config.cmd_vel_msg_type == "Twist":
            cmd_vel_msg_cls = Twist
        else:
            raise ValueError(
                f"Unsupported cmd_vel_msg_type: '{config.cmd_vel_msg_type}'. Must be 'TwistStamped' or 'Twist'."
            )

        # Subscribe to raw (pre-gain) cmd_vel for model action history input
        self.create_subscription(cmd_vel_msg_cls, config.cmd_vel_model_input_topic, self.cmd_vel_callback, 1)

    def get_logger(self, level=logging.INFO) -> logging.Logger:
        logger = logging.getLogger("neural_planner")
        logger.setLevel(level)
        handler = logging.FileHandler("./file.log")
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        # print error to console
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
        return logger

    def _cur_stamp_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_if_stale(self, topic: str, header: Header):
        """Log a warning if the message timestamp is too old compared to the current clock."""
        msg_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
        if msg_sec == 0:
            return  # Timestamp not set
        now_sec = self._cur_stamp_sec()
        age = now_sec - msg_sec
        if age < 0 or age > 1e8:
            return  # Clock domain mismatch (e.g. sim time vs wall clock)
        if age > self.config.stale_msg_threshold:
            self.logger.warning(
                f"Stale message on {topic}: age={age:.2f}s (threshold={self.config.stale_msg_threshold}s)"
            )

    def start_pause_callback(self, msg: Bool):
        if msg.data:  # start
            self.logger.info("Received start signal")
            self.start()
        else:  # pause
            self.logger.info("Received pause signal")
            self.pause()

    def stop_model_callback(self, msg: Bool):
        if msg.data:
            self.logger.info("Received stop signal")
            self.stop()

    def stop(self):
        if self.state not in ["running", "pause"]:
            self.logger.error(f"Stop signal received but state is not running or pause. current state :{self.state}")
            return
        self.state = "stopped"
        self.model_state_publish()
        # Stop velocity commands
        self.stop_publisher.publish(Bool(data=True))

    def pause(self):
        if self.state != "running":
            self.logger.error(f"Pause signal received but state is not running. current state :{self.state}")
            return
        self.state = "pause"
        self.model_state_publish()
        # Stop velocity commands
        self.stop_publisher.publish(Bool(data=True))

    def start(self):
        if self.state not in ["pause", "waiting"]:
            self.logger.error(f"Start signal received but state is not pause or waiting. current state :{self.state}")
            return
        self.state = "running"
        self.model_state_publish()
        # start evaluator and velocity commands
        self.stop_publisher.publish(Bool(data=False))
        self.eval_start_publisher.publish(self.get_clock().now().to_msg())

    def eval_resetting_callback(self, msg: Bool):
        if not self.config.is_eval:
            self.logger.error("Received eval_resetting signal but is_eval is False")
            rclpy.try_shutdown()
        if msg.data:  # start resetting
            self.logger.info("Received eval reset start")
            self.eval_reset_in_progress = True
        else:  # stop resetting
            self.logger.info("Received eval reset end")
            if self.eval_reset_in_progress:
                self.eval_reset_done += 1
            self.eval_reset_in_progress = False

    def model_state_publish(self):
        self.model_state_publisher.publish(String(data=self.state))

    def exit_callback(self, msg: Bool):
        rclpy.try_shutdown()

    def instruction_scenario_callback(self, msg: String):
        self.logger.info("Scenario received")
        scenario = Scenario.model_validate_json(msg.data)
        self.scenario = scenario
        if self.new_annotation:
            self.new_annotation = False
            self.set_scenario()
        else:
            self.new_scenario = True

    def instruction_annotation_callback(self, msg: Int32MultiArray):
        self.logger.info("Annotation received")
        if len(msg.data) == 0:
            annotation = None
        else:
            annotation = np.array(msg.data).reshape((-1, self.config.action_dim))
        self.annotation = annotation
        if self.new_scenario:
            self.new_scenario = False
            self.set_scenario()
        else:
            self.new_annotation = True

    def set_scenario(self):
        self.state = "resetting"
        # Stop velocity commands
        self.stop_publisher.publish(Bool(data=True))
        self.model_state_publish()
        # Reset
        with self._processor_lock:
            self._init_processor()
            annotated_scenario = AnnotatedScenario(scenario=self.scenario, annotation=self.annotation)
            self.processor.reset(annotated_scenario)
        self.logger.info("Annotated scenario set")
        self._init_states()
        # Update state
        if not self.config.is_eval:
            self.state = "waiting"
            self.model_state_publish()
            return

        # start waiting for eval reset and return
        if self.eval_reset_done > 1:
            self.logger.error("Sim Reset count is more than 1, Simulator reset multiple time")
            rclpy.try_shutdown()
            return
        self.eval_reset_wait_start = self.get_clock().now().nanoseconds

    def _is_processor_ready(self) -> bool:
        """Check if the processor exists and has been reset."""
        return self.processor is not None and self.processor._is_reset_called

    def odom_callback(self, msg: Odometry):
        self._warn_if_stale("odom", msg.header)
        msg = copy(msg)
        self.last_odom = msg

        with self._processor_lock:
            if self._is_processor_ready():
                self.processor.append_message(topic="GLOBAL_ODOM", msg=msg)

    def rgb_callback(self, msg: Image | CompressedImage):
        self._warn_if_stale("rgb_front", msg.header)
        msg = copy(msg)
        self.last_rgb = msg

        with self._processor_lock:
            if self._is_processor_ready():
                self.processor.append_message(topic="RGB_FRONT", msg=msg)

    def cmd_vel_callback(self, msg: TwistStamped | Twist):
        """Callback for cmd_vel topic to append to processor"""
        if not self._is_processor_ready():
            return
        try:
            msg = copy(msg)
            # Twist messages don't have a header, so provide the current timestamp
            t = None if hasattr(msg, "header") else self.get_clock().now().nanoseconds
            with self._processor_lock:
                self.processor.append_message(topic="CMD_VEL", msg=msg, t=t)
        except Exception as e:
            self.logger.error(f"Error in cmd_vel_callback: {e}", exc_info=True)

    def has_required_messages(self) -> bool:
        """Check if all required messages (odom and rgb) have been received."""
        if self.last_odom is None:
            self.logger.info("No odom message received")
            return False
        if self.last_rgb is None:
            self.logger.info("No RGB message received")
            return False
        if np.isnan(self.last_odom.pose.pose.position.x):
            self.logger.error("NaN value detected in odom")
            return False
        return True

    def loop(self):
        # waiting for eval reset
        if self.state == "resetting" and self.eval_reset_wait_start is not None:
            if self.eval_reset_done > 0:
                self.logger.info("Simulator reset done")
                self.eval_reset_done = 0
                self.eval_reset_wait_start = None
                self.state = "waiting"
                self.model_state_publish()
                return
            timeout = 180 * 1_000_000_000
            cur_time = self.get_clock().now().nanoseconds
            if cur_time - self.eval_reset_wait_start >= timeout:
                self.logger.error("Timeout reached for waiting for eval reset")
                rclpy.try_shutdown()
                return
            # continue waiting for eval reset
            return

        if self.state != "running":
            return

        if not self.has_required_messages():
            return

        self.odom_queue.append(deepcopy(self.last_odom))

        if self.check_reached(self.last_odom, self.config.signal_reached_distance):
            self.reached_goal_publisher.publish(Bool(data=True))
        elif self.check_stopped():
            self.stopped_before_goal_publisher.publish(Bool(data=True))

        if self.check_reached(self.last_odom, self.config.reached_distance):
            self.logger.info("Reached goal")
            return

        if self.state != "running":
            return
        last_predicted_delta = self._cur_stamp_sec() - self.last_predicted_time
        if last_predicted_delta >= 0.25 and not self._inference_in_flight.is_set():
            self.last_predicted_time = self._cur_stamp_sec()
            self._inference_in_flight.set()
            self._inference_executor.submit(self._inference_model_thread)

    def get_distance_to_goal(self, odom: Odometry):
        if self.annotation is None:
            return float("inf")
        with self._processor_lock:
            goal_x, goal_y = self.processor.coords2pose(self.processor._trajectory_map_editor.human_trajectory[-1])
        return np.sqrt((odom.pose.pose.position.x - goal_x) ** 2 + (odom.pose.pose.position.y - goal_y) ** 2)

    def check_reached(self, odom: Odometry, reached_distance: float):
        distance = self.get_distance_to_goal(odom)
        return distance < reached_distance

    def check_stopped(self):
        if len(self.odom_queue) != self.odom_queue.maxlen:
            return False
        hist_odom, pres_odom = self.odom_queue[0], self.odom_queue[-1]
        hist_position = hist_odom.pose.pose.position
        pres_position = pres_odom.pose.pose.position
        dist = np.sqrt((hist_position.x - pres_position.x) ** 2 + (hist_position.y - pres_position.y) ** 2)
        hist_orientation = hist_odom.pose.pose.orientation
        pres_orientation = pres_odom.pose.pose.orientation
        hist_quat = [hist_orientation.x, hist_orientation.y, hist_orientation.z, hist_orientation.w]
        pres_quat = [pres_orientation.x, pres_orientation.y, pres_orientation.z, pres_orientation.w]
        _, _, hist_yaw = euler_from_quaternion(hist_quat)
        _, _, pres_yaw = euler_from_quaternion(pres_quat)
        yaw_diff = abs((hist_yaw - pres_yaw + np.pi) % (2 * np.pi) - np.pi)
        return dist < self.config.stopped_threshold and yaw_diff < self.config.stopped_angular_threshold

    def _publish_local_map_state(self, state: dict, timestamp):
        try:
            position, _ = state["global_odom"]

            header = Header(frame_id="map", stamp=timestamp)

            if state["map"] is not None:
                map_array = np.array(state["map"])
                map_rgb = map_array[:, :, :3]

                height, width, channels = map_rgb.shape
                image_msg = Image(
                    header=header,
                    height=height,
                    width=width,
                    encoding="rgb8",
                    is_bigendian=0,
                    step=width * channels,
                    data=map_rgb.tobytes(),
                )
                self.local_map_publisher.publish(image_msg)

            if state["local_state"] is not None:
                local_state_array = np.array(state["local_state"])
                local_state_bgra = cv2.cvtColor(local_state_array, cv2.COLOR_RGBA2BGRA)
                _, encoded_local_state = cv2.imencode(".png", local_state_bgra)
                self.local_state_publisher.publish(
                    CompressedImage(
                        header=header, format="bgra8; png compressed bgra8", data=encoded_local_state.tobytes()
                    )
                )

            self.local_point_stamped_publisher.publish(
                PointStamped(header=header, point=Point(x=position[0], y=position[1], z=position[2]))
            )

        except Exception as e:
            self.logger.error(f"Error publishing local map state: {e}", exc_info=True)

    def _publish_camera_visualization(self, state: dict, timestamp):
        try:
            header = Header(frame_id="camera", stamp=timestamp)

            img = state.get("camera_view_front")
            if img is not None:
                img_np = np.array(img)
                height, width, channels = img_np.shape
                image_msg = Image(
                    header=header,
                    height=height,
                    width=width,
                    encoding="rgb8",
                    is_bigendian=0,
                    step=width * channels,
                    data=img_np.tobytes(),
                )
                self.image_visualization_publisher.publish(image_msg)

        except Exception as e:
            self.logger.error(f"Error publishing camera visualization: {e}", exc_info=True)

    def _inference_model_thread(self):
        """Wrapper that runs inference_model in the thread pool and clears the in-flight flag."""
        self.inference_model()
        self._inference_in_flight.clear()

    def inference_model(self):
        try:
            with self._processor_lock:
                state = self.processor.extract_state(detail=self.config.visualize)
            state_extraction_time = self.get_clock().now().to_msg()

            # Front camera + map
            images = [
                image_to_b64(state["camera_view_front"]),
                image_to_b64(state["map"]),
            ]

            # Publish local map state if visualize is enabled
            if self.config.visualize:
                self._publish_local_map_state(state, state_extraction_time)
                self._publish_camera_visualization(state, state_extraction_time)

            # Get raw actions history for tokenization in model worker
            actions_history = state["actions_history"]  # Shape: (history_length, 2)

            messages = [
                {"role": "system", "content": [{"type": "text", "text": state["model_guideline"]}]},
                {
                    "role": "user",
                    "content": [
                        {"type": "image"},
                        {"type": "image"},
                        {"type": "text", "text": ""},
                    ],
                },
            ]

            data = {"messages": [messages], "images": [images], "actions_history": [actions_history.tolist()]}
            response = requests.post(f"{MODEL_WORKER_URI}/inference", json=data)
            try:
                action = response.json()["action"]
                assert len(action) == 1, "Action should be a list with one element"
                action = np.array(action[0])
            except Exception as e:
                self.logger.error(f"Error in model inference: {e}")
                raise e

            if action is None or np.equal(action, None).any() or np.isnan(action).any():
                self.logger.warning("Action is None or contains NaN. Using zeros.")
                chosen_vel = np.zeros((self.config.num_action, self.config.action_dim))
            else:
                chosen_vel = action

            # Build PointCloud2 for velocity prediction
            action_publish_time = self.get_clock().now().to_msg()
            action_msg = create_pointcloud2_from_array(chosen_vel, frame_id="model", timestamp=action_publish_time)

            # Publish the PointCloud2 message
            self.action_publisher.publish(action_msg)

            # Publish model latency information
            latency_msg = TimeReference()
            latency_msg.header.stamp = action_publish_time
            latency_msg.header.frame_id = "model"
            latency_msg.time_ref = state_extraction_time
            latency_msg.source = "state_extraction"
            self.model_latency_publisher.publish(latency_msg)
        except Exception as e:
            self.logger.error(f"Model output error occurred: {e}", exc_info=True)

    def shutdown_callback(self):
        self._inference_executor.shutdown()
        self.stop_publisher.publish(Bool(data=True))


if __name__ == "__main__":
    import argparse

    import yaml

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file", default=None)
    args = parser.parse_args()

    if args.config is None:
        config = NeuralPlannerConfig()
    else:
        with open(args.config, "r") as f:
            cfg = yaml.safe_load(f)
        config = NeuralPlannerConfig(**cfg)
    rclpy.init()
    sg = NeuralPlanner(config)
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(sg)
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        rclpy.try_shutdown()
