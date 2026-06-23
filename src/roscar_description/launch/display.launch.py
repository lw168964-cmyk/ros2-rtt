import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('roscar_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'roscar.urdf.xacro')
    default_rviz_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', model]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot xacro or URDF file.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_path,
            description='Absolute path to RViz2 config file.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
