import glob
import os
import re
import shlex
import shutil
import subprocess

import carb


def build_clean_ros_env() -> dict:
    keep_keys = [
        "HOME",
        "USER",
        "LOGNAME",
        "SHELL",
        "TERM",
        "DISPLAY",
        "XAUTHORITY",
        "XDG_RUNTIME_DIR",
        "DBUS_SESSION_BUS_ADDRESS",
        "LANG",
        "LC_ALL",
        "ROS_DOMAIN_ID",
        "ROS_LOCALHOST_ONLY",
        "RMW_IMPLEMENTATION",
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "CYCLONEDDS_URI",
    ]
    env = {k: v for k, v in os.environ.items() if k in keep_keys}
    clean_path = os.getenv("PIPELINE_CLEAN_PATH")
    env["PATH"] = clean_path if clean_path else os.getenv("PATH", "")
    return env


def _ros_setup_script() -> str | None:
    return os.getenv("ROS_SETUP_SCRIPT") or os.getenv("ROS_SETUP_BASH")


def _extract_joystick_id(name: str) -> str:
    tokens = re.findall(r"[A-Fa-f0-9]{8,}", name or "")
    if tokens:
        return max(tokens, key=len)
    return name


def detect_joystick_device(xbox_id: str | None = None):
    by_id_dir = os.getenv("JOYSTICK_BY_ID_DIR")
    candidates = []
    if by_id_dir and os.path.isdir(by_id_dir):
        for name in sorted(os.listdir(by_id_dir)):
            if "joystick" in name.lower():
                candidates.append(os.path.join(by_id_dir, name))
    elif by_id_dir:
        carb.log_warn(f"[Pipeline] JOYSTICK_BY_ID_DIR not found: {by_id_dir}")

    prefer = [p for p in candidates if re.search(r"xbox|gamepad|controller|joy", os.path.basename(p), re.I)]
    search_list = prefer or candidates

    chosen = None
    if xbox_id:
        for p in search_list:
            if xbox_id in p or xbox_id in os.path.basename(p):
                chosen = p
                break

    if not chosen and search_list:
        chosen = search_list[0]

    if not chosen:
        dev_glob = os.getenv("JOYSTICK_DEV_GLOB")
        if dev_glob:
            js_devices = sorted(glob.glob(dev_glob))
            if js_devices:
                chosen = js_devices[0]
        else:
            carb.log_warn("[Pipeline] JOYSTICK_DEV_GLOB not set. Skipping /dev/input/js* fallback.")

    if not chosen:
        carb.log_warn("[Pipeline] No joystick device found. Check JOYSTICK_BY_ID_DIR or JOYSTICK_DEV_GLOB.")
        return None, None

    detected_id = _extract_joystick_id(os.path.basename(chosen))
    return chosen, detected_id


def build_joystick_teleop_cmd(config_path: str, device: str | None = None) -> str:
    cfg = shlex.quote(os.path.abspath(config_path))
    joy_cmd = "ros2 run joy joy_node"
    if device:
        dev = shlex.quote(device)
        joy_cmd += f" --ros-args -p dev:={dev}"
    return f"{joy_cmd} & ros2 run teleop_twist_joy teleop_node --ros-args --params-file {cfg}"


def launch_terminal_teleop(command: str, label: str) -> bool:
    if not command:
        return False

    ros_setup = _ros_setup_script()
    if ros_setup:
        shell_cmd = f"source {shlex.quote(ros_setup)} >/dev/null 2>&1; {command}"
    else:
        shell_cmd = command
        carb.log_warn("[Pipeline] ROS_SETUP_SCRIPT not set. Running teleop without ROS sourcing.")
    env = build_clean_ros_env()
    terminal_candidates = [
        ["gnome-terminal", "--", "bash", "-c", shell_cmd],
        ["x-terminal-emulator", "-e", "bash", "-c", shell_cmd],
        ["xterm", "-e", "bash", "-c", shell_cmd],
    ]

    for candidate in terminal_candidates:
        if shutil.which(candidate[0]):
            subprocess.Popen(candidate, env=env)
            carb.log_info(f"[Pipeline] Launched {label} teleop terminal.")
            return True

    carb.log_warn(f"[Pipeline] No terminal emulator found. Run {label} teleop manually.")
    carb.log_warn(f"[Pipeline] Command: {command}")
    return False


def launch_background_teleop(command: str, label: str):
    if not command:
        return None

    ros_setup = _ros_setup_script()
    if ros_setup:
        shell_cmd = f"source {shlex.quote(ros_setup)} >/dev/null 2>&1; {command}"
    else:
        shell_cmd = command
        carb.log_warn("[Pipeline] ROS_SETUP_SCRIPT not set. Running teleop without ROS sourcing.")
    env = build_clean_ros_env()
    try:
        proc = subprocess.Popen(["bash", "-c", shell_cmd], env=env, preexec_fn=os.setsid)
        carb.log_info(f"[Pipeline] Started {label} teleop in background.")
        return proc
    except Exception as exc:
        carb.log_error(f"[Pipeline] Failed to start {label} teleop: {exc}")
        return None
