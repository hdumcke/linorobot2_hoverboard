from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    this_pkg_path = get_package_share_path('my_robot_description')
    default_model_path = this_pkg_path / 'urdf/robots/urdf.xacro'
    default_rviz_config_path = this_pkg_path / 'rviz/description.rviz'

    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot xacro file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='false',
        description='Run rviz'
    )

    use_joint_states_publisher = DeclareLaunchArgument(
        name='publish_joints', 
        default_value='true',
        description='Launch joint_states_publisher'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_description = ParameterValue(
        Command( ['xacro ', LaunchConfiguration('model')] )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description
            }
        ]
    )

    publish_joints = LaunchConfiguration("publish_joints")
    gui = LaunchConfiguration("gui")

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(
            PythonExpression(
                ["'", publish_joints, "' == 'true' and '", gui, "' == 'false'"]
            )
        )
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        use_sim_time_arg,
        use_joint_states_publisher,
        use_rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
