# Quick Start Guide

## Prerequisites

- ROS2 installed (Humble, Iron, or Jazzy recommended)
- Joy package for ROS2: `sudo apt install ros-<distro>-joy`
- A joystick/gamepad connected to `/dev/input/js0`

## Installation

1. **Copy to ROS2 workspace:**

   ```bash
   cd ~/ros2_ws/src  # or your ROS2 workspace
   cp -r /workspace/costnav_isaacsim/isaac_sim_teleop_ros2 .
   ```

2. **Build the package:**

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select isaac_sim_teleop_ros2
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Running

### Basic Usage

```bash
# Set the robot type (required)
export SIM_ROBOT=nova_carter

# Launch the teleop node
ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py
```

### Available Robot Types

- `nova_carter` - Nova Carter differential drive robot
- `segway_e1` - Segway E1 robot

### Launch with Custom Parameters

```bash
ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py \
    use_teleport:=false \
    use_clock:=true \
    frame_id:=my_robot
```

## Joystick Controls (Xbox Controller Layout)

### Movement

- **Left Stick (Horizontal)**: Rotate left/right (angular velocity)
- **Right Stick (Vertical)**: Move forward/backward (linear velocity)

### Speed Control

- **LB (Left Bumper)**: Decrease max linear velocity
- **RB (Right Bumper)**: Increase max linear velocity

### Safety

- **LSB (Left Stick Button)**: Emergency stop toggle
- **RSB (Right Stick Button)**: Linear rate lock toggle

### Advanced (if enabled)

- **RT (Right Trigger)**: Switch between manual/model control
- **X Button**: Teleport to previous pose
- **Y Button**: Teleport to initial pose (origin)

## Topics

### Published

- `/cmd_vel` - Velocity commands (geometry_msgs/TwistStamped)
- `/is_model` - Model control status (std_msgs/Bool)

### Subscribed

- `/joy` - Joystick input (sensor_msgs/Joy)
- `/odom` - Robot odometry (nav_msgs/Odometry)

## Troubleshooting

### No joystick detected

```bash
# Check if joystick is connected
ls -l /dev/input/js0

# Test joystick
ros2 run joy joy_node
ros2 topic echo /joy
```

### Robot not moving

1. Check that `SIM_ROBOT` environment variable is set
2. Verify `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
3. Check emergency stop is not engaged (press LSB to toggle)
4. Verify joystick deadzone is not too high

### Build errors

```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select isaac_sim_teleop_ros2
```

## Minimal Example

For the absolute minimal setup to get joystick → /cmd_vel working:

```bash
# Terminal 1: Launch joy node
ros2 run joy joy_node

# Terminal 2: Launch teleop (in separate terminal)
export SIM_ROBOT=nova_carter
ros2 run isaac_sim_teleop_ros2 isaac_sim_teleop_node

# Terminal 3: Monitor output
ros2 topic echo /cmd_vel
```

## Next Steps

- See `README.md` for detailed documentation
- See `PORTING_NOTES.md` for technical details about the ROS1→ROS2 port
- Customize robot parameters in `isaac_sim_teleop_ros2/robot.py`
