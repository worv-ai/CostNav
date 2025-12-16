import curses
import sys
from typing import List, Tuple, Optional

from .robot import Robot
from .state import ControlState


class Monitoring:
    def __init__(self, robot: Robot, use_clock: bool, use_control_topic: bool = False, frame_id: str = "teleop"):
        self.use_clock = use_clock
        self.use_control_topic = use_control_topic
        self.frame_id = frame_id
        self.robot = robot
        self.enabled = True
        try:
            self.windows, self.msg_line_cnt = self._make_windows()
        except Exception as e:
            print(f"Warning: Could not initialize terminal UI: {e}", file=sys.stderr)
            print("Terminal UI disabled. The node will continue without visual monitoring.", file=sys.stderr)
            print("Tip: Resize terminal to at least 50x15 characters for UI.", file=sys.stderr)
            self.enabled = False
            self.windows = []
            self.msg_line_cnt = 0

    def _make_windows(self) -> Tuple[List[curses.window], int]:
        stdscr = curses.initscr()

        # Check terminal size - much smaller requirement
        max_y, max_x = stdscr.getmaxyx()
        if max_y < 15 or max_x < 50:
            curses.endwin()
            raise RuntimeError(f"Terminal too small ({max_x}x{max_y}). Need at least 50x15.")

        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)

        # Try to initialize colors, but don't fail if not supported
        try:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_GREEN)
            curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_RED)
            curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
            curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_BLACK)
            curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_BLACK)
            curses.init_pair(6, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
        except:
            pass  # Colors not supported, continue without them

        stdscr.clear()

        msg = self.robot.print_msg()

        try:
            stdscr.addstr(0, 0, msg)
        except curses.error:
            # If we can't write the message, terminal is too small
            curses.endwin()
            raise RuntimeError("Terminal too small to display robot info")

        line_cnt = len(msg.strip().split("\n"))

        try:
            stdscr.addstr(line_cnt + 3, 0, "angular")
            stdscr.addstr(line_cnt + 3, 45, "%")
            stdscr.addstr(line_cnt + 6, 45, "rad/s")

            stdscr.addstr(line_cnt + 9, 1, "linear")
            stdscr.addstr(line_cnt + 9, 45, "%")
            stdscr.addstr(line_cnt + 12, 45, "km/h")

            stdscr.addstr(line_cnt + 15, 0, "max vel")
            stdscr.addstr(line_cnt + 15, 45, "km/h")

            stdscr.addstr(line_cnt + 17, 0, "model cmd_vel")
            stdscr.addstr(line_cnt + 18, 2, "linear:")
            stdscr.addstr(line_cnt + 18, 45, "km/h")
            stdscr.addstr(line_cnt + 19, 2, "angular:")
            stdscr.addstr(line_cnt + 19, 45, "rad/s")

            if self.use_clock:
                stdscr.addstr(line_cnt + 21, 0, "current time:")
                stdscr.addstr(line_cnt + 21, 35, "fps:")

            stdscr.refresh()
        except curses.error:
            curses.endwin()
            raise RuntimeError("Terminal too small for UI layout")

        windows = [stdscr]
        for idx in range(5):
            try:
                bar = curses.newwin(3, 30, line_cnt + 2 + idx * 3, 7)
                bar.border(0)
                windows.append(bar)
            except curses.error:
                # Can't create window, terminal too small
                curses.endwin()
                raise RuntimeError("Terminal too small for progress bars")
        return windows, line_cnt

    def close(self):
        if self.enabled:
            try:
                curses.endwin()
            except:
                pass

    def log(self, control_state: ControlState):
        if not self.enabled:
            return

        try:
            self._log_internal(control_state)
        except curses.error as e:
            # Silently ignore curses errors during logging
            pass

    def _log_internal(self, control_state: ControlState):
        angular_vel = control_state.twist.angular.z
        linear_vel = control_state.twist.linear.x

        stdscr = self.windows[0]

        stdscr.addstr(self.msg_line_cnt + 3, 38, f"{100 * control_state.angular_rate:6.1f}", curses.A_BOLD)
        stdscr.addstr(self.msg_line_cnt + 6, 37, f"{angular_vel:7.4f}", curses.A_BOLD)

        stdscr.addstr(self.msg_line_cnt + 9, 38, f"{100 * control_state.linear_rate:6.1f}", curses.A_BOLD)
        stdscr.addstr(self.msg_line_cnt + 12, 37, f"{linear_vel * 3.6:7.4f}", curses.A_BOLD)

        stdscr.addstr(self.msg_line_cnt + 15, 41, f"{self.robot.get_max_linear_vel() * 3.6:3.1f}", curses.A_BOLD)

        # Display model cmd_vel
        model_linear_vel = control_state.model_cmd.linear.x
        model_angular_vel = control_state.model_cmd.angular.z
        stdscr.addstr(self.msg_line_cnt + 18, 11, f"{model_linear_vel * 3.6:7.4f}", curses.A_BOLD)
        stdscr.addstr(self.msg_line_cnt + 19, 12, f"{model_angular_vel:7.4f}", curses.A_BOLD)

        if self.use_clock:
            stdscr.addstr(self.msg_line_cnt + 21, 14, control_state.curreunt_time, curses.A_BOLD)
            stdscr.addstr(self.msg_line_cnt + 21, 40, f"{control_state.current_fps}", curses.A_BOLD)
            log_position = self.msg_line_cnt + 23
        else:
            log_position = self.msg_line_cnt + 21

        if control_state.cml_vel_lock:
            stdscr.addstr(log_position, 0, "emergency stop", curses.color_pair(3) | curses.A_BOLD)
        else:
            stdscr.addstr(log_position, 0, "              ")

        if control_state.linear_rate_lock:
            stdscr.addstr(log_position, 38, "linear lock", curses.color_pair(3) | curses.A_BOLD)
        else:
            stdscr.addstr(log_position, 38, "           ")

        # Display control status (24 characters: columns 21-44)
        if self.use_control_topic:
            # Check timeout first
            if control_state.control_report_timeout_detected:
                stdscr.addstr(log_position, 21, "TIMEOUT (stopping)      ", curses.color_pair(3) | curses.A_BOLD)
            else:
                # New mode: display control_report_status
                status = control_state.control_report_status
                if status == self.frame_id:
                    stdscr.addstr(log_position, 21, "TELEOP (in control)     ", curses.color_pair(4) | curses.A_BOLD)
                elif status == "model":
                    stdscr.addstr(log_position, 21, "MODEL (autonomous)      ", curses.color_pair(5) | curses.A_BOLD)
                elif status == "":
                    stdscr.addstr(log_position, 21, "WAITING (no report)     ", curses.color_pair(3) | curses.A_BOLD)
                else:
                    # Other teleoperator (truncate status if too long, pad to 24 chars total)
                    status_display = status[:14] if len(status) > 14 else status
                    display_text = f"OTHER ({status_display})"
                    stdscr.addstr(log_position, 21, f"{display_text:24s}", curses.color_pair(6) | curses.A_BOLD)
        else:
            # Legacy mode: display model_input_switch
            if control_state.model_input_switch:
                stdscr.addstr(log_position, 21, "model input switch      ", curses.color_pair(3) | curses.A_BOLD)
            else:
                stdscr.addstr(log_position, 21, "                        ")

        log_position += 1
        for action, state in control_state.action_state_dict.items():
            log_position += 1
            stdscr.addstr(log_position, 0, f"{action}\t{state}", curses.color_pair(3) | curses.A_BOLD)

        stdscr.refresh()

        bar_list = self.windows[1:]
        value_list = [
            (-control_state.angular_rate, 1),
            (-angular_vel, self.robot.get_max_ang_vel()),
            (control_state.linear_rate, 1),
            (linear_vel, self.robot.get_max_lin_vel()),
            (control_state.linear_vel_level, self.robot.get_max_linear_vel_level()),
        ]
        for bar, (val, max_val) in zip(bar_list, value_list):
            color = 1 if 0 < val else 2
            bar_width = 28  # Smaller bar width
            val = min(abs(int(val * bar_width / max_val)), bar_width)
            bar.addstr(1, 1, " " * val, curses.color_pair(color))
            bar.addstr(1, 1 + val, " " * (bar_width - val))
            bar.refresh()

