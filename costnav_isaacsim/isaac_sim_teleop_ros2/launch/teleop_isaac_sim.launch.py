#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    joy_node_name_arg = DeclareLaunchArgument(
        "joy_node_name", default_value="joy_node", description="Name of the joy node"
    )

    joy_deadzone_arg = DeclareLaunchArgument("joy_deadzone", default_value="0.12", description="Deadzone for joystick")

    use_teleport_arg = DeclareLaunchArgument(
        "use_teleport", default_value="true", description="Enable teleport functionality"
    )

    use_clock_arg = DeclareLaunchArgument("use_clock", default_value="false", description="Use clock topic")

    use_people_pose_arg = DeclareLaunchArgument(
        "use_people_pose", default_value="true", description="Use people pose topic"
    )

    use_wheel_odom_arg = DeclareLaunchArgument(
        "use_wheel_odom", default_value="false", description="Use wheel odometry"
    )

    auto_restart_on_collision_arg = DeclareLaunchArgument(
        "auto_restart_on_collision", default_value="true", description="Auto restart on collision"
    )

    use_control_topic_arg = DeclareLaunchArgument(
        "use_control_topic", default_value="false", description="Use control topic mode"
    )

    frame_id_arg = DeclareLaunchArgument("frame_id", default_value="teleop", description="Frame ID for teleop")

    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/odom", description="Odometry topic name")

    img_list_arg = DeclareLaunchArgument(
        "img_list", default_value="", description="Comma-separated list of image topics"
    )

    # Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name=LaunchConfiguration("joy_node_name"),
        output="screen",
        parameters=[
            {
                "dev": "/dev/input/js0",
                "deadzone": LaunchConfiguration("joy_deadzone"),
                "autorepeat_rate": 0.0,
            }
        ],
        remappings=[
            ("/joy", "/joy"),
        ],
    )

    # Isaac Sim teleop node
    # Note: prefix with 'xterm -e' to run in separate terminal for UI
    # Change prefix to None to run without separate terminal
    teleop_node = Node(
        package="isaac_sim_teleop_ros2",
        executable="isaac_sim_teleop_node",
        name="isaac_sim_teleop_node",
        output="screen",
        prefix=None,  # Run without separate terminal (xterm not available in Docker)
        parameters=[
            {
                "use_teleport": LaunchConfiguration("use_teleport"),
                "use_clock": LaunchConfiguration("use_clock"),
                "use_people_pose": LaunchConfiguration("use_people_pose"),
                "use_wheel_odom": LaunchConfiguration("use_wheel_odom"),
                "auto_restart_on_collision": LaunchConfiguration("auto_restart_on_collision"),
                "use_control_topic": LaunchConfiguration("use_control_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "img_list": LaunchConfiguration("img_list"),
            }
        ],
        remappings=[
            ("/cmd_vel", "/cmd_vel"),
            ("/joy", "/joy"),
        ],
    )

    return LaunchDescription(
        [
            joy_node_name_arg,
            joy_deadzone_arg,
            use_teleport_arg,
            use_clock_arg,
            use_people_pose_arg,
            use_wheel_odom_arg,
            auto_restart_on_collision_arg,
            use_control_topic_arg,
            frame_id_arg,
            odom_topic_arg,
            img_list_arg,
            joy_node,
            teleop_node,
        ]
    )
