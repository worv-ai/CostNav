# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""RViz launch file with localization (map server + AMCL) and lidar conversion.

This launch file includes:
- RViz2 visualization
- Localization (map_server + AMCL via localization_launch.py)
- Pointcloud to laserscan conversion
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # Get directory of this launch file
    this_dir = os.path.dirname(os.path.abspath(__file__))

    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(this_dir, "carter_navigation.rviz"),
    )

    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(this_dir, "carter_sidewalk.yaml"),
    )

    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(this_dir, "carter_navigation_params.yaml"),
    )

    autostart = LaunchConfiguration("autostart", default="True")

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Omniverse Isaac Sim) clock if True",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config,
                description="Full path to RViz config file to load",
            ),
            DeclareLaunchArgument("map", default_value=map_yaml_file, description="Full path to map yaml file"),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Full path to the ROS2 parameters file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="True",
                description="Automatically startup the localization stack",
            ),
            # Localization (map_server + AMCL)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/localization_launch.py"]),
                launch_arguments={
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                }.items(),
            ),
            # Pointcloud to laserscan conversion
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[("cloud_in", ["/front_3d_lidar/lidar_points"]), ("scan", ["/scan"])],
                parameters=[
                    {
                        "target_frame": "front_3d_lidar",
                        "transform_tolerance": 0.01,
                        "min_height": -0.4,
                        "max_height": 1.5,
                        "angle_min": -1.5708,  # -M_PI/2
                        "angle_max": 1.5708,  # M_PI/2
                        "angle_increment": 0.0087,  # M_PI/360.0
                        "scan_time": 0.3333,
                        "range_min": 0.05,
                        "range_max": 100.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "use_sim_time": True,
                    }
                ],
                name="pointcloud_to_laserscan",
            ),
            # RViz2 visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
