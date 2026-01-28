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

"""Nav2 navigation launch file (navigation stack only).

This launch file only includes Nav2 navigation stack via navigation_launch.py.
Localization + RViz + lidar conversion are launched separately:
- nova_carter: rviz.launch.py (map_server + AMCL)
- segway_e1:  rviz_segway_e1.launch.py (map_server + static map->odom)

Based on nav2_bringup/launch/bringup_launch.py structure:
- rviz.launch.py: localization_launch.py + RViz + pointcloud_to_laserscan
- nav2.launch.py: navigation_launch.py (this file)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get directory of this launch file
    this_dir = os.path.dirname(os.path.abspath(__file__))

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    autostart = LaunchConfiguration("autostart", default="True")

    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(this_dir, "carter_navigation_params.yaml"),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=params_file, description="Full path to the ROS2 parameters file"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="True", description="Use simulation (Omniverse Isaac Sim) clock if True"
            ),
            DeclareLaunchArgument(
                "autostart", default_value="True", description="Automatically startup the nav2 stack"
            ),
            # Navigation stack only (no localization - handled by rviz.launch.py)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/navigation_launch.py"]),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                }.items(),
            ),
        ]
    )
