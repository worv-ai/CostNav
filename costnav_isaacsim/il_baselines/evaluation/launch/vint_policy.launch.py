"""Launch file for ViNT policy node.

Usage:
    ros2 launch costnav_il_baselines vint_policy.launch.py \
        checkpoint:=/path/to/model.pth

Or with all options:
    ros2 launch costnav_il_baselines vint_policy.launch.py \
        checkpoint:=/path/to/model.pth \
        model_config:=/path/to/vint_eval.yaml \
        robot_config:=/path/to/robot_carter.yaml \
        inference_rate:=10.0 \
        use_imagegoal:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory("costnav_il_baselines")

    # Default config paths
    default_model_config = os.path.join(pkg_share, "configs", "vint_eval.yaml")
    default_robot_config = os.path.join(pkg_share, "configs", "robot_carter.yaml")

    # Declare launch arguments
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint",
        default_value="",
        description="Path to trained ViNT model weights (required)",
    )

    model_config_arg = DeclareLaunchArgument(
        "model_config",
        default_value=default_model_config,
        description="Path to model configuration YAML",
    )

    robot_config_arg = DeclareLaunchArgument(
        "robot_config",
        default_value=default_robot_config,
        description="Path to robot configuration YAML",
    )

    inference_rate_arg = DeclareLaunchArgument(
        "inference_rate",
        default_value="10.0",
        description="Inference frequency in Hz",
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/front_stereo_camera/left/image_raw",
        description="Camera image topic to subscribe to",
    )

    use_imagegoal_arg = DeclareLaunchArgument(
        "use_imagegoal",
        default_value="false",
        description="Whether to use image goal navigation",
    )

    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="PyTorch device for inference",
    )

    # ViNT policy node
    vint_node = Node(
        package="costnav_il_baselines",
        executable="vint_policy_node",
        name="vint_policy_node",
        output="screen",
        parameters=[
            {
                "checkpoint": LaunchConfiguration("checkpoint"),
                "model_config": LaunchConfiguration("model_config"),
                "robot_config": LaunchConfiguration("robot_config"),
                "inference_rate": LaunchConfiguration("inference_rate"),
                "image_topic": LaunchConfiguration("image_topic"),
                "use_imagegoal": LaunchConfiguration("use_imagegoal"),
                "device": LaunchConfiguration("device"),
            }
        ],
    )

    return LaunchDescription(
        [
            checkpoint_arg,
            model_config_arg,
            robot_config_arg,
            inference_rate_arg,
            image_topic_arg,
            use_imagegoal_arg,
            device_arg,
            vint_node,
        ]
    )
