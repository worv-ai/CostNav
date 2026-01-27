from abc import ABC, abstractmethod
from typing import Dict, List

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from .action_publisher import ActionPublisher


class Robot(ABC):
    action_publisher_list: List[ActionPublisher] = []

    @abstractmethod
    def print_msg(self) -> str:
        pass

    @abstractmethod
    def set_max_linear_vel(self, linear_vel_level: int) -> None:
        pass

    @abstractmethod
    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        pass

    @abstractmethod
    def get_max_ang_vel(self) -> float:
        pass

    @abstractmethod
    def get_max_lin_vel(self) -> float:
        pass

    @abstractmethod
    def get_max_linear_vel_level(self) -> int:
        pass

    @abstractmethod
    def get_max_linear_vel(self) -> float:
        pass

    def handle_action(self, data: Joy) -> Dict[str, str]:
        result_dict = {}
        for action_publisher in self.action_publisher_list:
            topic_name, action_state = action_publisher.publish(data)
            result_dict[topic_name] = action_state
        return result_dict


class DifferentialRobot(Robot):
    def __init__(self, robot_name, max_lin_vel, max_ang_vel, max_linear_vel_level) -> None:
        super().__init__()
        self.robot_name = robot_name
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.max_linear_vel_level = max_linear_vel_level
        self.set_max_linear_vel(1)

    def print_msg(self) -> str:
        msg = f"""
{self.robot_name}
left stick left/right : increase/decrease angular velocity
    ( rotation: ~ {self.max_ang_vel} rad/s )

right stick up/down : increase/decrease linear velocity
    ( ~ {self.max_lin_vel * 3.6} km/h )

left/right button : increase/decrease max linear velocity
    ( +-{self.max_lin_vel * 3.6 / self.max_linear_vel_level} km/h )
CTRL-C to quit
"""
        return msg

    def get_max_ang_vel(self):
        return self.max_ang_vel

    def get_max_lin_vel(self):
        return self.max_lin_vel

    def set_max_linear_vel(self, linear_vel_level: int):
        self.max_linear_vel = self.max_lin_vel * linear_vel_level / self.max_linear_vel_level

    def get_max_linear_vel_level(self):
        return self.max_linear_vel_level

    def get_max_linear_vel(self):
        return self.max_linear_vel

    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        angular_vel = self.max_ang_vel * angular_rate
        linear_vel = self.max_linear_vel * linear_rate

        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        return twist


class NovaCarter(DifferentialRobot):
    def __init__(self) -> None:
        super().__init__(robot_name=self.__class__.__name__, max_lin_vel=2.0, max_ang_vel=1.2, max_linear_vel_level=1)


class SegwayE1(DifferentialRobot):
    def __init__(self) -> None:
        super().__init__(
            robot_name=self.__class__.__name__,
            max_lin_vel=2.0,
            max_ang_vel=1.2,
            max_linear_vel_level=1,
        )
