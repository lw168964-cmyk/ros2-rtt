import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_config = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'config',
        'lslidar_n10p_uart.yaml',
    )

    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='x10',
        parameters=[driver_config],
        output='screen',
    )

    return LaunchDescription([
        driver_node
    ])
