#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# Isaac Sim ROS2 packages
if [ -f /opt/ros_ws/install/local_setup.bash ]; then
    source /opt/ros_ws/install/local_setup.bash
fi

exec "$@"
