import time
from dataclasses import dataclass

from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header


@dataclass
class ControlState:
    action_state_dict = {}

    curreunt_time = ""
    current_fps = ""
    real_time = time.time()
    real_time_list = []

    previous_btn_lb = 0
    previous_btn_rb = 0

    thread_stop_lb = None
    thread_stop_rb = None

    previous_btn_lsb = 0
    previous_btn_rsb = 0

    previous_btn_a = 0
    previous_btn_b = 0
    previous_btn_x = 0
    previous_btn_y = 0

    previous_axis_rt = 0

    previous_pose = Pose()
    previous_pose.orientation.w = 1

    current_pose = None
    twist = Twist()
    odom_header = Header()

    linear_rate_lock = False
    cml_vel_lock = False

    # model input
    model_input_switch = False
    model_cmd = Twist()

    linear_vel_level = 1

    linear_rate = 0
    angular_rate = 0

    people_pose = None
    is_collision_detected = False

    # control report (for use_control_topic mode)
    control_report_status = ""  # Empty until first report received
    last_control_report_time = 0.0
    control_report_timeout_detected = False
