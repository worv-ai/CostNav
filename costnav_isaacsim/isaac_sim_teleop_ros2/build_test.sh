#!/bin/bash

# Quick build test script for isaac_sim_teleop_ros2
# This script helps verify the package can be built

echo "========================================="
echo "Isaac Sim Teleop ROS2 - Build Test"
echo "========================================="
echo ""

# Check if we're in a ROS2 workspace
if [ ! -f "package.xml" ]; then
    echo "Error: This script should be run from the package directory"
    echo "Current directory: $(pwd)"
    exit 1
fi

echo "Package directory: $(pwd)"
echo ""

# Check for required files
echo "Checking package structure..."
required_files=(
    "package.xml"
    "CMakeLists.txt"
    "setup.py"
    "nodes/isaac_sim_teleop_node"
    "launch/teleop_isaac_sim.launch.py"
    "isaac_sim_teleop_ros2/__init__.py"
    "isaac_sim_teleop_ros2/robot.py"
    "isaac_sim_teleop_ros2/state.py"
    "isaac_sim_teleop_ros2/action_publisher.py"
    "isaac_sim_teleop_ros2/monitoring.py"
    "isaac_sim_teleop_ros2/img_latency_manager.py"
)

all_files_exist=true
for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (MISSING)"
        all_files_exist=false
    fi
done

echo ""

if [ "$all_files_exist" = false ]; then
    echo "Error: Some required files are missing!"
    exit 1
fi

echo "All required files present!"
echo ""

# Check if node is executable
if [ -x "nodes/isaac_sim_teleop_node" ]; then
    echo "✓ Main node is executable"
else
    echo "✗ Main node is not executable"
    echo "  Run: chmod +x nodes/isaac_sim_teleop_node"
    exit 1
fi

echo ""
echo "========================================="
echo "Package structure verification complete!"
echo "========================================="
echo ""
echo "To build this package:"
echo "  1. Copy this directory to your ROS2 workspace src folder"
echo "  2. Run: colcon build --packages-select isaac_sim_teleop_ros2"
echo "  3. Source: source install/setup.bash"
echo ""
echo "To run:"
echo "  export SIM_ROBOT=nova_carter  # or segway_e1"
echo "  ros2 launch isaac_sim_teleop_ros2 teleop_isaac_sim.launch.py"
echo ""

