#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# Isaac Sim ROS2 packages
if [ -f /ros2_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash ]; then
    source /workspace/build_ws/install/local_setup.bash
fi

exec "$@"

