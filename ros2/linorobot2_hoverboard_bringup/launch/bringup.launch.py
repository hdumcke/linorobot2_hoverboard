# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_hoverboard_description'), 'launch', 'description.launch.py']
    )

    hoverboard_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_hoverboard_control'), 'launch', 'linorobot2_hoverboard_interface.launch.py']
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_hoverboard_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy',
            default_value='false',
            description='Use Joystick'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hoverboard_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
        )
    ])
