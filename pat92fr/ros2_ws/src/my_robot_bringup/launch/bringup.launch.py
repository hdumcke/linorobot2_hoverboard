import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

import yaml

def generate_launch_description():

    config_filepath = os.path.join(
        get_package_share_directory('my_robot_bringup'), 
        'config'
    )

    joy_config_filepath = os.path.join(
        config_filepath,
        "joy.config.yaml"
    )

    teleop_config_filepath = os.path.join(
        config_filepath,
        "teleop.config.yaml"
    )

    hoverboard_driver_config_filepath = os.path.join(
        config_filepath,
        "hoverboard-driver.config.yaml"
    )

    #with open(hoverboard_driver_config_filepath, 'r') as f:
    #    params = yaml.safe_load(f)['hoverboard_driver_node']['ros__parameters']
    #print(params)


    return launch.LaunchDescription(
        [
		# Start URG_NODE2 (LIDAR) using urg_node2 launch file
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				[
					PathJoinSubstitution(
						[
							FindPackageShare('urg_node2'),
							"launch/"
							'urg_node2.launch.py'
						]
					)
				]
			),
			launch_arguments={}.items()
		),
				
		launch_ros.actions.Node(
			package='joy', 
			executable='joy_node', 
			name='joy_node',
			parameters=[joy_config_filepath]
		),

		launch_ros.actions.Node(
			package='teleop_twist_joy', 
			executable='teleop_node',
			name='teleop_twist_joy_node',
			parameters=[teleop_config_filepath]
		),

		launch_ros.actions.Node(
			package='hoverboard-driver-pkg',
			executable='hoverboard-driver',
			name='hoverboard_driver_node',
			parameters=[hoverboard_driver_config_filepath]
		),            
        ]
    ) # return LD
