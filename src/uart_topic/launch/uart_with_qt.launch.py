#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyS1',
        description='Serial port device path'
    )
    baud_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate'
    )
    rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publish frequency (Hz)'
    )

    # 串口节点
    uart_node = Node(
        package='uart_topic',
        executable='uart_send_receive',
        name='uart_send_receive',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    odom_tf_node = Node(
        package='uart_topic',
        executable='recive_to_odom_tf',
        name='recive_to_odom_tf',
        output='screen',
        parameters=[{
            'recive_topic': '/recive_data',
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': True,
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    # Qt 界面节点（无需额外参数）
    qt_node = Node(
        package='uart_topic',
        executable='uart_qt',
        name='uart_qt',
        output='screen',
        # Qt GUI 节点不需要参数，可留空
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        rate_arg,
        uart_node,
        odom_tf_node,
        qt_node,
    ])
