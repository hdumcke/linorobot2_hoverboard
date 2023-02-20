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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('horo_description'), 'launch', 'description.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('horo_bringup'), 'launch', 'joy_teleop.launch.py'],
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("horo_description"), "urdf", "robots/urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy',
            default_value='true',
            description='Use Joystick'
        ),

        Node(
            package="horo_controller",
            executable="horo_controller",
        ),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(description_launch_path)
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
        )
    ])
