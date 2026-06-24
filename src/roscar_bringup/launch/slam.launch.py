import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('roscar_bringup')
    description_share = get_package_share_directory('roscar_description')
    lslidar_share = get_package_share_directory('lslidar_driver')
    uart_share = get_package_share_directory('uart_topic')

    default_slam_params = os.path.join(
        bringup_share, 'config', 'mapper_params_scan_only.yaml')
    default_rviz_config = os.path.join(bringup_share, 'rviz', 'slam.rviz')
    default_lidar_launch = os.path.join(
        lslidar_share, 'launch', 'lsn10p_launch.py')
    description_launch = os.path.join(
        description_share, 'launch', 'description.launch.py')
    default_uart_launch = os.path.join(
        uart_share, 'launch', 'uart_only.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lidar = LaunchConfiguration('use_lidar')
    use_robot_state = LaunchConfiguration('use_robot_state')
    use_rviz = LaunchConfiguration('use_rviz')
    use_uart = LaunchConfiguration('use_uart')
    use_static_odom = LaunchConfiguration('use_static_odom')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    lidar_launch = LaunchConfiguration('lidar_launch')
    uart_launch = LaunchConfiguration('uart_launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on the real robot.',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the lslidar driver and publish /scan.',
        ),
        DeclareLaunchArgument(
            'use_robot_state',
            default_value='true',
            description='Start robot_state_publisher for the robot model.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 for map and scan visualization.',
        ),
        DeclareLaunchArgument(
            'use_uart',
            default_value='true',
            description='Start UART feedback, /odom and dynamic TF.',
        ),
        DeclareLaunchArgument(
            'use_static_odom',
            default_value='false',
            description='Publish a fixed odom -> base_footprint fallback.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to slam_toolbox parameters.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to RViz2 config.',
        ),
        DeclareLaunchArgument(
            'lidar_launch',
            default_value=default_lidar_launch,
            description='Full path to the lslidar launch file.',
        ),
        DeclareLaunchArgument(
            'uart_launch',
            default_value=default_uart_launch,
            description='Full path to the UART odometry launch file.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=IfCondition(use_robot_state),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(use_lidar),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uart_launch),
            condition=IfCondition(use_uart),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(use_robot_state),
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_footprint',
            output='screen',
            condition=IfCondition(use_static_odom),
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'odom', 'base_footprint',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_footprint_to_laser',
            output='screen',
            arguments=[
                '0.075', '0', '0.1425',
                '0', '0', '0',
                'base_footprint', 'laser',
            ],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
