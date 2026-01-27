# Isaac Sim Teleop ROS2

ROS2 port of the Isaac Sim teleoperation package. This package provides joystick-based teleoperation for Isaac Sim robots, publishing to `/cmd_vel`.

## Overview

This is a minimal port from ROS1 to ROS2, focusing on the core functionality of using a joystick to publish velocity commands to `/cmd_vel`.

## Key Changes from ROS1

- **ROS2 API**: Migrated from `rospy` to `rclpy`
- **Node Structure**: Converted to ROS2 node class-based architecture
- **Launch Files**: Converted from XML to Python launch files
- **QoS Profiles**: Added ROS2 QoS profile support
- **Time API**: Updated to use ROS2 time API (`get_clock().now()`)
- **Publishers/Subscribers**: Updated to ROS2 API

## Dependencies

- ROS2 (Humble or later recommended)
- Python 3
- `joy` package for ROS2
- Standard ROS2 message packages:
  - `geometry_msgs`
  - `sensor_msgs`
  - `std_msgs`
  - `nav_msgs`

## Building

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select isaac_sim_teleop_ros2
source install/setup.bash
```

## Usage

### Basic Usage

Set the robot type via environment variable and launch:

```bash
export SIM_ROBOT=nova_carter  # or segway_e1
ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py
```

### Topics

**Published:**
- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands for the robot
- `/is_model` (`std_msgs/msg/Bool`) - Whether model control is active

**Subscribed:**
- `/joy` (`sensor_msgs/msg/Joy`) - Joystick input
- `/cmd_vel_model` (`geometry_msgs/msg/Twist`) - Model velocity commands (optional)
- `/odom` (`nav_msgs/msg/Odometry`) - Odometry feedback (optional)

### Supported Robots

- `nova_carter` - Nova Carter robot
- `segway_e1` - Segway E1 robot

### Launch Arguments

- `joy_node_name` (default: 'joy_node') - Name of the joy node
- `joy_deadzone` (default: '0.12') - Deadzone for joystick
- `use_teleport` (default: 'true') - Enable teleport functionality
- `use_clock` (default: 'false') - Use clock topic
- `use_people_pose` (default: 'true') - Use people pose topic
- `use_wheel_odom` (default: 'false') - Use wheel odometry
- `auto_restart_on_collision` (default: 'true') - Auto restart on collision
- `use_control_topic` (default: 'false') - Use control topic mode
- `frame_id` (default: 'teleop') - Frame ID for teleop
- `img_list` (default: '') - Comma-separated list of image topics

### Joystick Controls

- **Left Stick (Left/Right)**: Angular velocity control
- **Right Stick (Up/Down)**: Linear velocity control
- **Left Bumper (LB)**: Decrease max linear velocity
- **Right Bumper (RB)**: Increase max linear velocity
- **Left Stick Button (LSB)**: Emergency stop toggle
- **Right Stick Button (RSB)**: Linear rate lock toggle
- **Right Trigger (RT)**: Model input switch
- **X Button**: Teleport to previous pose (if teleport enabled)
- **Y Button**: Teleport to initial pose (if teleport enabled)

## Topics

### Published

- `/cmd_vel` (geometry_msgs/TwistStamped) - Velocity commands
- `/is_model` (std_msgs/Bool) - Model control status
- `/robot_pose` (geometry_msgs/Pose) - Robot pose for teleport (optional)
- `/control_request` (std_msgs/Bool) - Control request (optional)

### Subscribed

- `/joy` (sensor_msgs/Joy) - Joystick input
- `/odom` (nav_msgs/Odometry) - Odometry
- `/cmd_vel_model` (geometry_msgs/TwistStamped) - Model velocity commands
- `/clock` (rosgraph_msgs/Clock) - Simulation clock (optional)
- `/people_pose` (geometry_msgs/PoseArray) - People poses (optional)
- `/wheel_odom` (nav_msgs/Odometry) - Wheel odometry (optional)
- `/collision_event` (geometry_msgs/PointStamped) - Collision events (optional)
- `/control_report` (std_msgs/String) - Control status report (optional)

## Troubleshooting

### Terminal UI Issues

If you see `Error: addwstr() returned ERR`, this means the terminal UI cannot be initialized. The node will continue to work without the visual monitoring. Common causes:

1. **Terminal too small**: Resize your terminal to at least **50x15 characters** (width x height)
2. **Running in non-interactive environment**: The terminal UI requires a proper terminal
3. **Locale issues**: Make sure your locale supports UTF-8

The node will automatically disable the terminal UI and continue publishing `/cmd_vel` commands.

To check your terminal size: `echo "Terminal: $(tput cols)x$(tput lines)"`

### No Joystick Detected

```bash
# Check if joystick is connected
ls -l /dev/input/js0

# Test joystick with joy node
ros2 run joy joy_node
ros2 topic echo /joy
```

### Robot Not Moving

1. Verify `SIM_ROBOT` environment variable is set
2. Check `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
3. Ensure emergency stop is not engaged (press Left Stick Button to toggle)
4. Check joystick deadzone setting

## License

BSD

