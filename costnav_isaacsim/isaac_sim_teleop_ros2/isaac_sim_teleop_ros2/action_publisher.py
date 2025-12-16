from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8


class ActionPublisher(ABC):
    @abstractmethod
    def publish(self, data: Joy) -> Tuple[str, str]:
        """
        Publish the action based on the Joy message.
        Returns a tuple of (topic_name, action_state).
        """
        raise NotImplementedError("Subclasses should implement this method.")


class BoolActionPublisher:
    def __init__(
        self,
        topic_name: str,
        axis_num: int = None,
        axis_val: float = None,
        button_num: int = None,
        off_axis_nums=[],
        off_button_nums=[],
        node=None,
    ):
        self.topic_name = topic_name
        self.node = node
        if node is not None:
            self.pub = node.create_publisher(Bool, topic_name, 1)
        else:
            self.pub = None

        self.axis_num = axis_num
        if axis_num is not None:
            assert axis_val is not None and button_num is None
            self.target_vel = axis_val

        self.button_num = button_num
        if button_num is not None:
            assert axis_num is None and axis_val is None
            self.target_vel = 1
        self.off_axis_nums = off_axis_nums
        self.off_button_nums = off_button_nums

        self.previous_val = 0
        self.action_state = False

    def publish(self, data: Joy) -> Tuple[str, str]:
        if self.axis_num is not None:
            current_val = data.axes[self.axis_num]
        elif self.button_num is not None:
            current_val = data.buttons[self.button_num]
        else:
            return self.topic_name, False

        if self.previous_val != current_val:
            if current_val == self.target_vel:
                self.action_state = not self.action_state
                if self.pub is not None:
                    msg = Bool()
                    msg.data = self.action_state
                    self.pub.publish(msg)
            else:
                self._check_off(data)
            self.previous_val = current_val
        elif current_val != self.target_vel:
            self._check_off(data)

        state = "on " if self.action_state else "off"
        return self.topic_name, state

    def _check_off(self, data: Joy) -> None:
        if not self.action_state:
            return
        for off_axis_num in self.off_axis_nums:
            if data.axes[off_axis_num] != 0:
                self.action_state = False
                if self.pub is not None:
                    msg = Bool()
                    msg.data = self.action_state
                    self.pub.publish(msg)
                break

        for off_button_num in self.off_button_nums:
            if data.buttons[off_button_num] != 0:
                self.action_state = False
                if self.pub is not None:
                    msg = Bool()
                    msg.data = self.action_state
                    self.pub.publish(msg)
                break


@dataclass
class IntActionConfig:
    topic_state: str
    topic_value: int
    axis_num: int = None
    axis_val: float = None
    button_num: int = None


class IntActionPublisher:
    def __init__(self, topic_name: str, config_list: List[IntActionConfig], node=None):
        assert len(config_list) > 0, "config_list must not be empty."
        for config in config_list:
            if config.axis_num is not None:
                assert config.axis_val is not None and config.button_num is None
            elif config.button_num is not None:
                assert config.axis_num is None and config.axis_val is None
            else:
                raise ValueError("Either axis_num or button_num must be specified.")

        self.topic_name = topic_name
        self.topic_state = config_list[0].topic_state
        self.topic_value = config_list[0].topic_value
        self.config_list = config_list
        self.node = node
        if node is not None:
            self.pub = node.create_publisher(Int8, topic_name, 1)
        else:
            self.pub = None

    def publish(self, data: Joy) -> Tuple[str, str]:
        for config in self.config_list:
            if (config.axis_num is not None and data.axes[config.axis_num] == config.axis_val) or (
                config.button_num is not None and data.buttons[config.button_num] == 1
            ):
                if config.topic_value != self.topic_value:
                    self.topic_state = config.topic_state
                    self.topic_value = config.topic_value
                    if self.pub is not None:
                        msg = Int8()
                        msg.data = self.topic_value
                        self.pub.publish(msg)
                break
        return self.topic_name, self.topic_state

