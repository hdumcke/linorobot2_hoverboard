import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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

    robot_localization_config_filepath = os.path.join(
        config_filepath,
        "robot_localization.config.yaml"
    )

    robot_description_filepath = os.path.join(
        get_package_share_directory('my_robot_description'), 
        'urdf',
        'robots/urdf.xacro'
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py']
    )

    robot_description = ParameterValue(Command(['xacro ', robot_description_filepath]),value_type=str)

    #with open(hoverboard_driver_config_filepath, 'r') as f:
    #    params = yaml.safe_load(f)['hoverboard_driver_node']['ros__parameters']
    #print(params)


    return launch.LaunchDescription(
        [

			launch_ros.actions.Node(
				package='hoverboard-driver-pkg',
				executable='hoverboard-driver',
				name='hoverboard_driver_node',
				parameters=[hoverboard_driver_config_filepath]
			),  
					
			launch_ros.actions.Node(
				package='joy_linux', 
				executable='joy_linux_node', 
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
				package='robot_localization',
				executable='ekf_node',
				name='ekf_filter_node',
				output='screen',
				parameters=[robot_localization_config_filepath],
                remappings=[("imu", "imu/raw_data")]
			),
            
    		launch_ros.actions.Node(
        		package='joint_state_publisher',
        		executable='joint_state_publisher'
        	),

			launch_ros.actions.Node(
        		package='robot_state_publisher',
        		executable='robot_state_publisher',
        		parameters=[{'robot_description': robot_description}]
    		),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch_path),
            )

		]
    ) # return LD
