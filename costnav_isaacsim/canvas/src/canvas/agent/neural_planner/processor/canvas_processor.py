import warnings
from collections import deque
from typing import Callable, Optional, TypedDict, Union

import cv2
import numpy as np
from numpy.typing import NDArray
from PIL import Image
from rosbags.image import message_to_cvimage
from rosbags.typesys.stores.ros2_jazzy import (
    geometry_msgs__msg__Twist,
    geometry_msgs__msg__TwistStamped,
    sensor_msgs__msg__CompressedImage,
    sensor_msgs__msg__Image,
)

from canvas.agent.neural_planner.scenarios import AnnotatedScenario
from canvas.agent.neural_planner.map import SketchMap
from canvas.agent.neural_planner.ros_utils import extract_pose_from_msg
from canvas.agent.neural_planner.map import TrajectoryMapEditor


def _validate_compressedimage_format(format: str) -> bool:
    parts = format.split()
    if len(parts) != 4 or parts[2] != "compressed" or parts[3] not in ["bgr8", "rgb8"]:
        return False
    return True


def _validate_image_encoding(encoding: str) -> bool:
    return encoding in ["rgb8", "bgr8"]


def process_image(msg: sensor_msgs__msg__CompressedImage | sensor_msgs__msg__Image) -> Image:
    if hasattr(msg, "format"):
        if not _validate_compressedimage_format(msg.format):
            raise ValueError(f"Invalid CompressedImage format: {msg.format}")
        is_rgb = "rgb8" == msg.format.split()[-1]
    elif hasattr(msg, "encoding"):
        if not _validate_image_encoding(msg.encoding):
            raise ValueError(f"Invalid Image encoding: {msg.encoding}")
        is_rgb = "rgb8" == msg.encoding
    image = message_to_cvimage(msg)
    if not is_rgb:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(image)
    if image.height == 0 or image.width == 0:
        raise ValueError("Image has zero height or width.")
    # resize so that width is 640
    image = image.resize((640, int(image.height * 640 / image.width)))
    return image


class CanvasState(TypedDict):
    map: Union[Image.Image, str]
    global_odom: tuple[NDArray, NDArray]  # (position, orientation in quaternion)
    model_guideline: str
    uuid: str
    semantic_uuid: str
    actions_history: NDArray  # (history_length, 2) - 2 seconds of cmd_vel history
    camera_view_front: Optional[Union[Image.Image, str]] = None
    # Local state field (for publishing to ROS topics)
    local_state: Optional[Image.Image] = None


class CanvasOnlineProcessor:
    def __init__(
        self,
        sketch_map: SketchMap,
        action_rate: int = 20,
        history_duration: float = 2.0,
    ):
        self.state: CanvasState = CanvasState()
        self._camera_image: Optional[Image.Image] = None
        self._trajectory_map_editor: TrajectoryMapEditor = None
        self._odom_process_func: Callable = extract_pose_from_msg
        self._img_process_func: Callable = process_image
        self._is_reset_called: bool = False
        self.sketch_map = sketch_map

        self.scenario: AnnotatedScenario = None

        # Action history configuration
        self._action_rate = action_rate
        self._history_duration = history_duration
        self._history_length = int(action_rate * history_duration)  # 2 seconds * 20 Hz = 40 samples
        self._action_history_buffer = deque(maxlen=self._history_length)

        # Zero-hold resampling for cmd_vel (supports both Twist and TwistStamped)
        self._step_ns = int(1e9 / action_rate)  # nanoseconds per action step
        self._last_resampled_time = None
        self._last_cmd_vel = None  # Store last cmd_vel for zero-hold

    def reset(self, scenario: AnnotatedScenario):
        self._trajectory_map_editor = TrajectoryMapEditor(self.sketch_map)
        trajectory = scenario.annotation

        self.scenario = scenario

        if trajectory is not None:
            # convert htl trajectory to drive map's coordinate
            trajectory = self.sketch_map.coords2pose(trajectory)  # pixel coords (y, x) to pose (x, y)
            # interpolate trajectory
            trajectory = self._trajectory_map_editor.interpolate_trajectory(trajectory)
            # convert pose (x, y) to pixel coords (y, x)
            trajectory = np.flip(self.sketch_map.pose2coords(trajectory), axis=1)

            self._trajectory_map_editor.set_human_trajectory(trajectory)

        self._is_reset_called = True
        self.state["model_guideline"] = scenario.scenario.model_guideline
        self.state["uuid"] = scenario.uuid
        self.state["semantic_uuid"] = scenario.scenario.semantic_uuid

        # Clear action history buffer and reset zero-hold state
        self._action_history_buffer.clear()
        self._last_resampled_time = None
        self._last_cmd_vel = None

    def coords2pose(self, coord):
        return self._trajectory_map_editor.canvas.coords2pose(coord)

    def get_start_coords(self):
        return self.coords2pose(self._trajectory_map_editor.human_trajectory[0])

    def _get_stamp_nsec(self, stamp) -> int:
        """Extract nanosecond timestamp from both rosbags Time and rospy Time."""
        if hasattr(stamp, "nsecs"):
            return stamp.to_nsec()
        return stamp.sec * 1000000000 + stamp.nanosec

    def append_message(self, topic: str, msg, t=None) -> None:
        if not self._is_reset_called:
            warnings.warn("reset() should be called before append_message()", UserWarning, stacklevel=2)
            return

        if topic == "GLOBAL_ODOM":
            msg = self._odom_process_func(msg)
            self.state["global_odom"] = msg
            self._append_odom(msg)
        elif topic == "RGB_FRONT":
            self._camera_image = self._img_process_func(msg)
        elif topic == "CMD_VEL":
            self._append_cmd_vel_action(msg, t)

    def _append_cmd_vel_action(
        self, cmd_vel_msg: geometry_msgs__msg__Twist | geometry_msgs__msg__TwistStamped, timestamp_ns: int = None
    ) -> None:
        """
        Zero-hold-resample incoming cmd_vel so that the history buffer
        stores one sample every `self._step_ns` nanoseconds.
        """
        # Handle timestamp extraction
        if hasattr(cmd_vel_msg, "header"):
            timestamp_ns = self._get_stamp_nsec(cmd_vel_msg.header.stamp)
        if timestamp_ns is None:
            raise ValueError(
                "timestamp_ns is None and message doesn't have header. "
                "Either provide timestamp_ns or use TwistStamped message."
            )

        # Handle both Twist and TwistStamped messages
        if hasattr(cmd_vel_msg, "twist"):
            twist = cmd_vel_msg.twist
        else:
            twist = cmd_vel_msg

        cur_cmd = np.array((twist.linear.x, twist.angular.z), dtype=np.float32)

        # First ever sample
        if self._last_resampled_time is None:
            self._last_resampled_time = timestamp_ns
            self._last_cmd_vel = cur_cmd
            self._action_history_buffer.append(cur_cmd)
            return

        dt = timestamp_ns - self._last_resampled_time
        if dt <= 0:  # out-of-order or duplicate message
            self._last_cmd_vel = cur_cmd
            return

        step = self._step_ns
        n_full = dt // step
        remainder = dt % step

        # Zero-hold fill for the previous cmd if at least one full step elapsed
        if n_full:
            hold_copies = n_full if remainder else n_full - 1
            if hold_copies:
                last = self._last_cmd_vel
                self._action_history_buffer.extend(last.copy() for _ in range(hold_copies))

            if remainder == 0:
                self._action_history_buffer.append(cur_cmd)

            self._last_resampled_time += n_full * step

        self._last_cmd_vel = cur_cmd

    def _get_actions_history(self):
        """Get the current action history, padded with zeros if necessary."""
        if not self._action_history_buffer:
            return np.zeros((self._history_length, 2))

        current_history = list(self._action_history_buffer)

        if len(current_history) < self._history_length:
            padding_needed = self._history_length - len(current_history)
            zero_padding = [np.zeros(2) for _ in range(padding_needed)]
            current_history = zero_padding + current_history

        return np.array(current_history)

    def extract_state(self, detail: bool = False) -> CanvasState:
        """
        Extract the current state with front camera view.

        Args:
            detail: If True, include local_state in the state for publishing
        """
        self.state["camera_view_front"] = self._camera_image
        self.state["actions_history"] = self._get_actions_history()

        if detail:
            self.state["map"], self.state["local_state"] = self._get_map(detail=True)
        else:
            self.state["map"] = self._get_map()

        return self.state.copy()

    def _append_odom(self, odom: tuple[NDArray, NDArray]):
        self._trajectory_map_editor.append_odom(odom)

    def _get_map(self, detail: bool = False):
        """Get the local map state."""
        if detail:
            result = self._trajectory_map_editor.get_local_state(detail=True)
            if result is None:
                return None, None

            map_array, local_state_array, _ = result

            resized_map = cv2.resize(map_array, (378, 378))
            resized_map = Image.fromarray(resized_map)

            local_state = Image.fromarray(local_state_array)

            return resized_map, local_state
        else:
            map = self._trajectory_map_editor.get_local_state()
            if map is None:
                return None

            map = cv2.resize(map, (378, 378))
            map = Image.fromarray(map)
            return map
