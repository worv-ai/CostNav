import math
from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, List

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from .action_publisher import ActionPublisher, BoolActionPublisher, IntActionConfig, IntActionPublisher


class Robot(ABC):
    action_publisher_list: List[ActionPublisher] = []

    @abstractmethod
    def print_msg(self):
        pass

    @abstractmethod
    def set_max_linear_vel(self, linear_vel_level: int):
        pass

    @abstractmethod
    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        pass

    @abstractmethod
    def get_max_ang_vel(self):
        pass

    @abstractmethod
    def get_max_lin_vel(self):
        pass

    @abstractmethod
    def get_max_linear_vel_level(self):
        pass

    @abstractmethod
    def get_max_linear_vel(self):
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
        super().__init__(robot_name=self.__class__.__name__, max_lin_vel=2.5, max_ang_vel=1.2, max_linear_vel_level=18)


class Aiden(DifferentialRobot):
    def __init__(self) -> None:
        super().__init__(
            robot_name=self.__class__.__name__, max_lin_vel=2.5 / 3.6, max_ang_vel=0.7, max_linear_vel_level=5
        )


class GintSS(Robot):
    # vel: km/h
    def __init__(
        self,
        max_forward_vel=3,
        max_backward_vel=2.23,
        min_lin_vel=0.5,
        lin_vel_precision=2,
        wheel_base=1.12,
        max_turn_vel=0.59,
        angular_correction_factor=0.8,
        err=0.01,
        max_linear_vel_level=10,
    ) -> None:
        super().__init__()
        self.max_forward_vel = max_forward_vel
        self.max_backward_vel = max_backward_vel
        self.min_lin_vel = min_lin_vel
        self.lin_vel_precision = lin_vel_precision
        self.err = err
        self.max_linear_vel_level = max_linear_vel_level
        self.wheel_base = wheel_base
        self.max_turn_vel = max_turn_vel
        self.angular_correction_factor = angular_correction_factor
        self.set_max_linear_vel(1)
        self.action_publisher_list = [
            BoolActionPublisher("/action/buzzer", axis_num=7, axis_val=1),
            BoolActionPublisher("/action/spray/left", axis_num=6, axis_val=1),
            BoolActionPublisher("/action/spray/right", axis_num=6, axis_val=-1),
        ]

        self.max_ang_vel = [
            2 * max_turn_vel / 3.6 / wheel_base,
            max_forward_vel / 3.6 / wheel_base * angular_correction_factor,
        ]

    def print_msg(self):
        msg = f"""
GintSS
left stick left/right : increase/decrease angular velocity
    ( rotation: ~ {self.max_ang_vel[0]} rad/s, run: ~ {self.max_ang_vel[1]} rad/s )

right stick up/down : increase/decrease linear velocity
    ( -{self.max_backward_vel} ~ {self.max_forward_vel} km/h )

left/right button : increase/decrease max linear velocity
    ( +-{self.max_forward_vel / self.max_linear_vel_level} km/h )

up : buzzer
left : left spray
right: right spray

CTRL-C to quit
"""
        return msg

    def set_max_linear_vel(self, linear_vel_level: int):
        self.max_linear_vel = self.max_forward_vel * linear_vel_level / self.max_linear_vel_level

    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        if linear_rate == 0 and angular_rate != 0:
            if angular_rate > 0:
                right_vel = self.max_turn_vel
            else:
                right_vel = -self.max_turn_vel
            left_vel = -right_vel
        else:
            right_vel = self.max_linear_vel * linear_rate
            if right_vel < -self.max_backward_vel:
                right_vel = -self.max_backward_vel
            if math.fabs(right_vel) < self.min_lin_vel:
                right_vel = 0

            left_vel = right_vel
            if angular_rate < 0:
                right_vel *= 1 + angular_rate * self.angular_correction_factor
            elif angular_rate > 0:
                left_vel *= 1 - angular_rate * self.angular_correction_factor
            left_vel = round(left_vel, self.lin_vel_precision)
            right_vel = round(right_vel, self.lin_vel_precision)
        linear_vel = (left_vel + right_vel) / 3.6 / 2
        angular_vel = (right_vel - left_vel) / 3.6 / self.wheel_base

        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        return twist

    def get_max_ang_vel(self):
        return max(self.max_ang_vel)

    def get_max_lin_vel(self):
        return self.max_forward_vel / 3.6

    def get_max_linear_vel_level(self):
        return self.max_linear_vel_level

    def get_max_linear_vel(self):
        return self.max_linear_vel / 3.6


class GintATT(Robot):
    def __init__(
        self,
        max_lin_vel=10 / 3.6,
        wheel_base=1.8,
        track_width=1.7,
        max_streering_angle=0.523598776,
        max_linear_vel_level=20,
    ) -> None:
        super().__init__()
        self.max_lin_vel = max_lin_vel
        self.max_linear_vel_level = max_linear_vel_level
        self.wheel_base = wheel_base
        self.set_max_linear_vel(1)
        self.action_publisher_list = [
            IntActionPublisher(
                "/action/drive_mode",
                [
                    IntActionConfig("front_2ws", 0, axis_num=7, axis_val=1),
                    IntActionConfig("rear_2ws ", 1, axis_num=7, axis_val=-1),
                    IntActionConfig("crab     ", 2, axis_num=6, axis_val=1),
                    IntActionConfig("4ws      ", 3, axis_num=6, axis_val=-1),
                ],
            )
        ]

        turning_radius = wheel_base / math.tan(max_streering_angle) + track_width / 2

        self.max_ang_vel = max_lin_vel / turning_radius
        self.max_streering_angle = math.atan(wheel_base / turning_radius)

    def print_msg(self):
        msg = f"""
GintATT
left stick left/right : increase/decrease angular velocity
    ( rotation: ~ {self.max_ang_vel} rad/s )

right stick up/down : increase/decrease linear velocity
    ( ~ {self.max_lin_vel * 3.6} km/h )

left/right button : increase/decrease max linear velocity
    ( +-{self.max_lin_vel * 3.6 / self.max_linear_vel_level} km/h )

up : Front Two-Wheel Steering (front_2ws)
down : Rear Two-Wheel Steering (rear_2ws)
left : Parallel Four-Wheel Steering (crab)
right: Opposed Four-Wheel Steering (4ws)

CTRL-C to quit
"""
        return msg

    def set_max_linear_vel(self, linear_vel_level: int):
        self.max_linear_vel = self.max_lin_vel * linear_vel_level / self.max_linear_vel_level

    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        linear_vel = self.max_linear_vel * linear_rate
        if angular_rate == 0:
            angular_vel = 0
        else:
            turning_radius = self.wheel_base / math.tan(self.max_streering_angle * angular_rate)
            if linear_vel == 0:
                linear_vel = 1e-9
            angular_vel = linear_vel / turning_radius

        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        return twist

    def get_max_ang_vel(self):
        return self.max_ang_vel

    def get_max_lin_vel(self):
        return self.max_lin_vel

    def get_max_linear_vel_level(self):
        return self.max_linear_vel_level

    def get_max_linear_vel(self):
        return self.max_linear_vel


class TestToro(Robot):
    def __init__(
        self,
        max_lin_vel=16 / 3.6,
        backward_vel_reduction_rate=6.4 / 16.1,
        wheel_base=1.64504,
        track_width=1.67749,
        max_streering_angle=0.733038286,
        max_linear_vel_level=32,
    ) -> None:
        super().__init__()
        self.max_lin_vel = max_lin_vel
        self.max_linear_vel_level = max_linear_vel_level
        self.backward_vel_reduction_rate = backward_vel_reduction_rate
        self.wheel_base = wheel_base
        self.set_max_linear_vel(1)
        self.action_publisher_list = [
            IntActionPublisher(
                "/action/reel_mode",
                [
                    IntActionConfig("mowing   ", 0, axis_num=7, axis_val=-1),
                    IntActionConfig("turning  ", 1, axis_num=6, axis_val=1),
                    IntActionConfig("transport", 2, axis_num=7, axis_val=1),
                ],
            )
        ]

        turning_radius = wheel_base / math.tan(max_streering_angle) + track_width / 2

        self.max_ang_vel = max_lin_vel / turning_radius
        self.max_streering_angle = math.atan(wheel_base / turning_radius)

    def print_msg(self):
        msg = f"""
TestToro
left stick left/right : increase/decrease angular velocity
    ( rotation: ~ {self.max_ang_vel} rad/s )

right stick up/down : increase/decrease linear velocity
    ( ~ {self.max_lin_vel * 3.6} km/h )

left/right button : increase/decrease max linear velocity
    ( +-{self.max_lin_vel * 3.6 / self.max_linear_vel_level} km/h )

up : Transport Mode (transport)
left : Turning Mode (turning)
down : Mowing Mode (mowing)

CTRL-C to quit
"""
        return msg

    def set_max_linear_vel(self, linear_vel_level: int):
        self.max_linear_vel = self.max_lin_vel * linear_vel_level / self.max_linear_vel_level

    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        linear_vel = self.max_linear_vel * linear_rate
        if linear_vel < 0:
            linear_vel *= self.backward_vel_reduction_rate
        if angular_rate == 0:
            angular_vel = 0
        else:
            turning_radius = self.wheel_base / math.tan(self.max_streering_angle * angular_rate)
            if linear_vel == 0:
                linear_vel = 1e-9
            angular_vel = linear_vel / turning_radius

        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        return twist

    def get_max_ang_vel(self):
        return self.max_ang_vel

    def get_max_lin_vel(self):
        return self.max_lin_vel

    def get_max_linear_vel_level(self):
        return self.max_linear_vel_level

    def get_max_linear_vel(self):
        return self.max_linear_vel


class IsaacHoundbot(Robot):
    class Status(Enum):
        STOP = 0
        ROTATION = 1
        RUN = 2

    def __init__(
        self,
        max_forward_lin_vel=2.5,
        max_backward_lin_vel=2.5 * 0.25,
        max_ang_vel=1.4,
        max_ang_correction=1.8 / 1.4,
        backward_vel_reduction_rate=0.5,
        err=0.01,
        max_linear_vel_level=18,
    ) -> None:
        super().__init__()
        self.max_forward_lin_vel = max_forward_lin_vel
        self.max_backward_lin_vel = max_backward_lin_vel
        self.max_ang_vel = max_ang_vel
        self.max_ang_correction = max_ang_correction
        self.backward_vel_reduction_rate = backward_vel_reduction_rate
        self.err = err
        self.max_linear_vel_level = max_linear_vel_level
        self.set_max_linear_vel(1)
        self.status = IsaacHoundbot.Status.STOP

    def print_msg(self):
        msg = f"""
Control Your Houndbot
---------------------------
left stick left/right : increase/decrease angular velocity
    ( rotation: ~ {self.max_ang_vel} rad/s, run: ~ {self.max_ang_vel * self.max_ang_correction} rad/s )

right stick up/down : increase/decrease linear velocity
    ( forward: ~ {self.max_forward_lin_vel * 3.6} km/h, backward: ~ {self.max_backward_lin_vel * 3.6} km/h )

left/right button : increase/decrease max linear velocity
    ( +-{self.max_forward_lin_vel * 3.6 / self.max_linear_vel_level} km/h )
CTRL-C to quit
"""
        return msg

    def set_max_linear_vel(self, linear_vel_level: int):
        self.max_linear_vel = self.max_forward_lin_vel * linear_vel_level / self.max_linear_vel_level

    def compute_cmd_vel(self, linear_rate: float, angular_rate: float, current_twist: Twist) -> Twist:
        is_stop = True
        if current_twist is not None:
            is_stop = abs(current_twist.linear.x) < self.err and abs(current_twist.angular.z) < self.err

        if self.status == IsaacHoundbot.Status.STOP:
            if linear_rate != 0:
                self.status = IsaacHoundbot.Status.RUN
            elif angular_rate != 0:
                self.status = IsaacHoundbot.Status.ROTATION
        elif self.status == IsaacHoundbot.Status.ROTATION:
            if is_stop and angular_rate == 0:
                self.status = IsaacHoundbot.Status.STOP
        elif self.status == IsaacHoundbot.Status.RUN:
            if is_stop and linear_rate == 0:
                self.status = IsaacHoundbot.Status.STOP

        angular_vel = self.max_ang_vel * angular_rate
        if self.status == IsaacHoundbot.Status.ROTATION:
            linear_vel = 0
        else:
            angular_vel *= abs(current_twist.linear.x) / self.max_forward_lin_vel * self.max_ang_correction
            linear_vel = self.max_linear_vel * linear_rate
            if linear_vel < 0:
                linear_vel = max(self.backward_vel_reduction_rate * linear_vel, -self.max_backward_lin_vel)

        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        return twist

    def get_max_ang_vel(self):
        return self.max_ang_vel * self.max_ang_correction

    def get_max_lin_vel(self):
        return self.max_forward_lin_vel

    def get_max_linear_vel_level(self):
        return self.max_linear_vel_level

    def get_max_linear_vel(self):
        return self.max_linear_vel


class COCO(DifferentialRobot):
    def __init__(self) -> None:
        super().__init__(
            robot_name=self.__class__.__name__, max_lin_vel=2.5 / 0.9, max_ang_vel=0.7, max_linear_vel_level=10
        )

    def print_msg(self):
        msg = f"""
Tiny little cute COCO
---------------------------
left stick left/right : increase/decrease angular velocity
    ( rotation: ~ {self.max_ang_vel} rad/s )

right stick up/down : increase/decrease linear velocity
    ( ~ {self.max_lin_vel * 3.6} km/h )

left/right button : increase/decrease max linear velocity
    ( +-{self.max_lin_vel * 3.6 / self.max_linear_vel_level} km/h )
CTRL-C to quit
"""
        return msg
