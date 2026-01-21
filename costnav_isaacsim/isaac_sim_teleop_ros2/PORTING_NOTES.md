# Porting Notes: ROS1 to ROS2

## Summary

This document describes the minimal port of `isaac_sim_ros_integration/src/isaac_sim_teleop` from ROS1 to ROS2, focusing on joystick to `/cmd_vel` functionality.

## Files Ported

### Core Python Modules

1. **robot.py** - Robot abstraction classes
   - No changes needed (pure Python logic)
   - Robot types preserved: NovaCarter, SegwayE1

2. **state.py** - Control state management
   - No changes needed (dataclass with message types)

3. **action_publisher.py** - Action publishing for robot-specific actions
   - Updated to accept ROS2 node reference
   - Changed publisher creation to use `node.create_publisher()`
   - Updated message publishing to use ROS2 API

4. **monitoring.py** - Terminal UI for teleoperation monitoring
   - No changes needed (uses curses, independent of ROS)

5. **img_latency_manager.py** - Image latency tracking
   - Updated to accept ROS2 node reference
   - Changed to use `node.create_subscription()` and `node.create_publisher()`
   - Updated time API to use `node.get_clock().now().to_msg()`

### Main Node

**nodes/isaac_sim_teleop_node** - Main teleoperation node
- Converted from rospy script to ROS2 Node class
- Changed from `rospy.init_node()` to class-based `Node` initialization
- Updated all publishers/subscribers to use ROS2 API:
  - `rospy.Publisher()` → `self.create_publisher()`
  - `rospy.Subscriber()` → `self.create_subscription()`
- Changed parameters from `rospy.get_param()` to `self.declare_parameter()` and `self.get_parameter()`
- Replaced `rospy.Rate()` with `self.create_timer()`
- Updated time API:
  - `rospy.Time.now()` → `self.get_clock().now().to_msg()`
- Changed main loop from `while not rospy.is_shutdown()` to timer callback
- Updated cleanup to use ROS2 node lifecycle

### Launch Files

**launch/teleop_isaac_sim.launch.py** - ROS2 Python launch file
- Converted from XML to Python launch file
- Changed from `<arg>` tags to `DeclareLaunchArgument`
- Changed from `<node>` tags to `Node` actions
- Updated parameter passing to use ROS2 launch API
- Preserved all original launch arguments

### Package Configuration

1. **package.xml**
   - Updated to format 3
   - Changed `<buildtool_depend>catkin</buildtool_depend>` to `<buildtool_depend>ament_cmake</buildtool_depend>`
   - Added `<buildtool_depend>ament_cmake_python</buildtool_depend>`
   - Added `rosgraph_msgs` dependency
   - Changed to `<export><build_type>ament_cmake</build_type></export>`

2. **CMakeLists.txt**
   - Converted from catkin to ament_cmake
   - Changed `catkin_python_setup()` to `ament_python_install_package()`
   - Changed `catkin_install_python()` to standard `install(PROGRAMS ...)`
   - Updated install paths to use ROS2 conventions

3. **setup.py**
   - Converted from catkin setup to standard setuptools
   - Removed catkin-specific imports
   - Updated to ROS2 package structure

## Key Differences from ROS1

### API Changes

| ROS1 | ROS2 |
|------|------|
| `rospy.init_node()` | `rclpy.init()` + `Node` class |
| `rospy.Publisher()` | `node.create_publisher()` |
| `rospy.Subscriber()` | `node.create_subscription()` |
| `rospy.get_param()` | `node.declare_parameter()` + `node.get_parameter()` |
| `rospy.Time.now()` | `node.get_clock().now()` |
| `rospy.Rate()` | `node.create_timer()` |
| `rospy.spin()` | `rclpy.spin()` |
| XML launch files | Python launch files |

### Message Publishing

ROS1:
```python
pub.publish(Bool(data=True))
```

ROS2:
```python
msg = Bool()
msg.data = True
pub.publish(msg)
```

### Time API

ROS1:
```python
stamp = rospy.Time.now()
```

ROS2:
```python
stamp = self.get_clock().now().to_msg()
```

## What Was NOT Changed

- Core robot control logic
- Joystick button/axis mappings
- Velocity computation algorithms
- Terminal UI (curses-based monitoring)
- Control state management
- All robot-specific configurations

## Testing Recommendations

1. Test with each robot type (nova_carter, segway_e1) by setting `SIM_ROBOT`
2. Verify joystick controls work correctly
3. Test teleport functionality (if enabled)
4. Verify `/cmd_vel` publishing
5. Test emergency stop and velocity locking features
6. Verify model input switching (if used)

## Dependencies

Ensure these ROS2 packages are installed:
- `joy` (ROS2 version)
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `nav_msgs`
- `rosgraph_msgs`

