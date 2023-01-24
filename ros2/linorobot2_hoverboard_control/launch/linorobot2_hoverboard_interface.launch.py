from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linorobot2_hoverboard_control',
            executable='hoverboard_interface',
            name='hoverboard_interface',
            output='screen'
        )
    ])
