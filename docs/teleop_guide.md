# :joystick: Teleoperation Guide

ROS2 joystick-based teleoperation for Isaac Sim robots. This guide covers Docker-based and manual setup, joystick controls, the curses-based terminal UI, and all available launch arguments.

---

## :rocket: Quick Start

### Docker (Recommended)

```bash
# Default: nova_carter robot, no extra people
make run-teleop

# Choose a different robot
make run-teleop SIM_ROBOT=segway_e1

# Spawn pedestrians in the scene
make run-teleop NUM_PEOPLE=20

# Combine options
make run-teleop SIM_ROBOT=segway_e1 NUM_PEOPLE=20 FOOD=True
```

!!! tip "Stopping"
    Press ++ctrl+c++ once to stop the teleop node. The Makefile will automatically tear down the Docker profile.

### Manual ROS2 Setup

If you prefer to run outside Docker:

```bash
# 1. Prerequisites
sudo apt install ros-${ROS_DISTRO}-joy

# 2. Build the workspace
cd costnav_isaacsim
colcon build --packages-select isaac_sim_teleop_ros2

# 3. Source the workspace
source install/setup.bash

# 4. Set the robot type
export SIM_ROBOT=nova_carter   # or segway_e1

# 5. Launch
ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py
```

---

## :robot: Supported Robots

Both robots use differential drive kinematics with the same control interface.

| Robot | Max Linear Vel | Max Angular Vel | Vel Levels |
|:------|:--------------:|:---------------:|:----------:|
| **nova_carter** | 2.0 m/s (7.2 km/h) | 1.2 rad/s | 10 |
| **segway_e1** | 2.0 m/s (7.2 km/h) | 1.2 rad/s | 10 |

Set the robot via the `SIM_ROBOT` environment variable:

```bash
export SIM_ROBOT=nova_carter   # or segway_e1
```

---

## :video_game: Joystick Controls (Xbox Layout)

### Movement

| Input | Action |
|:------|:-------|
| **Left Stick** (horizontal) | Angular velocity (left/right rotation, up to 1.2 rad/s) |
| **Right Stick** (vertical) | Linear velocity (forward/backward, up to current max) |

### Speed Control

| Button | Action |
|:-------|:-------|
| **LB** (hold) | Decrease max linear velocity (steps of 0.72 km/h) |
| **RB** (hold) | Increase max linear velocity (steps of 0.72 km/h) |

!!! info "Speed Levels"
    Max velocity is divided into 10 levels. Holding LB/RB continuously changes the level with a short initial delay, then rapid stepping.

### Safety

| Button | Action |
|:-------|:-------|
| **LSB** (Left Stick Button) | Toggle emergency stop -- zeroes all velocity |
| **RSB** (Right Stick Button) | Toggle linear rate lock -- holds current linear input |

### Advanced

| Input | Action |
|:------|:-------|
| **RT** (Right Trigger) | Toggle model input switch -- hands control to `/cmd_vel_model` |
| **X Button** | Teleport to previous saved pose |
| **Y Button** | Teleport to initial pose (origin) |

!!! warning "Teleport"
    Teleport requires `use_teleport:=true` (default). The X/Y buttons publish to `/robot_pose` which the simulator consumes.

---

## :tv: Terminal UI

The teleop node includes a curses-based terminal UI that displays real-time robot state.

### Running the UI

=== "Method 1: Launch File (auto-opens xterm)"

    ```bash
    # In the launch file, set prefix to 'xterm -e' for a separate terminal:
    # prefix='xterm -e',
    ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py
    ```

    !!! note
        The Docker setup runs without xterm (`prefix=None`) since xterm is unavailable in containers.

=== "Method 2: Run Directly"

    ```bash
    # Resize your terminal to at least 50x15 characters, then:
    export SIM_ROBOT=nova_carter
    ros2 run isaac_sim_teleop_ros2 isaac_sim_teleop_node
    ```

=== "Method 3: Single Terminal (joy in background)"

    ```bash
    # Start the joy node in the background
    ros2 run joy joy_node &

    # Run teleop in the foreground (UI visible)
    export SIM_ROBOT=nova_carter
    ros2 run isaac_sim_teleop_ros2 isaac_sim_teleop_node
    ```

### What the UI Shows

The terminal UI renders the following information with color-coded progress bars:

| Section | Details |
|:--------|:--------|
| **Robot info** | Robot name, max velocities, control hints |
| **Angular velocity** | Joystick input (%) and actual velocity (rad/s) with progress bar |
| **Linear velocity** | Joystick input (%) and actual velocity (km/h) with progress bar |
| **Max velocity** | Current max linear velocity level (km/h) with progress bar |
| **Model cmd_vel** | Linear (km/h) and angular (rad/s) from the autonomous model |
| **Sim time / FPS** | Current simulation time and real-time FPS (when `use_clock:=true`) |
| **Status flags** | Emergency stop, linear lock, model input switch indicators |

---

## :satellite: ROS2 Topics

### Published Topics

| Topic | Type | Description |
|:------|:-----|:------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command (from joystick or model) |
| `/is_model` | `std_msgs/Bool` | `True` when model is in control, `False` for teleop |
| `/robot_pose` | `geometry_msgs/Pose` | Teleport target (when `use_teleport:=true`) |
| `/control_request` | `std_msgs/Bool` | Request/release control (when `use_control_topic:=true`) |
| `<odom_topic>/latency` | `sensor_msgs/TimeReference` | Odometry latency measurement |

### Subscribed Topics

| Topic | Type | Description |
|:------|:-----|:------------|
| `/joy` | `sensor_msgs/Joy` | Joystick input from `joy_node` |
| `/odom` | `nav_msgs/Odometry` | Robot odometry (pose and twist) |
| `/cmd_vel_model` | `geometry_msgs/Twist` | Model velocity command (autonomous policy) |
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock (when `use_clock:=true`) |
| `/people_pose` | `geometry_msgs/PoseArray` | Pedestrian poses (when `use_people_pose:=true`) |
| `/wheel_odom` | `nav_msgs/Odometry` | Wheel odometry (when `use_wheel_odom:=true`) |
| `/collision_event` | `geometry_msgs/PointStamped` | Collision detection (when `auto_restart_on_collision:=true`) |
| `/start_record` | `std_msgs/Bool` | Saves current pose as "previous pose" for teleport |
| `/control_report` | `std_msgs/String` | Control status report (when `use_control_topic:=true`) |

---

## :gear: Launch Arguments

All arguments are passed to the launch file:

```bash
ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py \
    joy_deadzone:=0.15 \
    use_clock:=true \
    odom_topic:=/wheel_odom
```

| Argument | Default | Description |
|:---------|:--------|:------------|
| `joy_node_name` | `joy_node` | Name of the joy node |
| `joy_deadzone` | `0.12` | Deadzone threshold for joystick axes |
| `use_teleport` | `true` | Enable teleport via X/Y buttons (publishes `/robot_pose`) |
| `use_clock` | `false` | Subscribe to `/clock` for sim time and FPS display |
| `use_people_pose` | `true` | Subscribe to `/people_pose` for pedestrian tracking |
| `use_wheel_odom` | `false` | Use `/wheel_odom` for twist instead of `/odom` |
| `auto_restart_on_collision` | `true` | Auto-teleport to previous pose on collision |
| `use_control_topic` | `false` | Use `/control_request` + `/control_report` for arbitrated control |
| `frame_id` | `teleop` | Frame ID used for latency tracking and control arbitration |
| `odom_topic` | `/odom` | Odometry topic name |
| `img_list` | _(empty)_ | Comma-separated list of image topics for latency tracking |

---

## :hammer_and_wrench: Troubleshooting

### No joystick detected

??? solution "Solutions"
    - Verify the joystick is connected: `ls /dev/input/js*`
    - Check permissions: `sudo chmod 666 /dev/input/js0`
    - Test with `jstest /dev/input/js0`
    - In Docker, ensure the device is mounted (the Makefile handles this automatically)
    - Install the joy package: `sudo apt install ros-${ROS_DISTRO}-joy`

### Robot not moving

??? solution "Solutions"
    - Check that `SIM_ROBOT` is set: `echo $SIM_ROBOT`
    - Verify Isaac Sim is running and healthy
    - Check emergency stop is not active (LSB toggles it) -- the UI shows a red "emergency stop" label
    - Verify `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
    - Ensure the joystick deadzone is not too high (default `0.12`)

### Terminal UI issues ("addwstr ERR")

??? solution "Solutions"
    - Resize your terminal to at least **50 columns x 15 rows**
    - The curses UI degrades gracefully -- if the terminal is too small, the node continues without the UI
    - In Docker, the UI runs without xterm; ensure the container is attached to an interactive terminal (`docker run -it`)
    - If colors are broken, your terminal may not support 256 colors

### Build errors

??? solution "Solutions"
    - Source ROS2 first: `source /opt/ros/${ROS_DISTRO}/setup.bash`
    - Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
    - Clean and rebuild: `rm -rf build/ install/ log/ && colcon build --packages-select isaac_sim_teleop_ros2`
