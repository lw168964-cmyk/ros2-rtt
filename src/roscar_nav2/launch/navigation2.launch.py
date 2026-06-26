import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('roscar_nav2')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    description_share = get_package_share_directory('roscar_description')
    lslidar_share = get_package_share_directory('lslidar_driver')
    uart_share = get_package_share_directory('uart_topic')

    default_map = os.path.join(pkg_share, 'maps', 'real_map_02.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_required_params = os.path.join(
        pkg_share, 'config', 'nav2_required_params.yaml')
    if not os.path.exists(default_required_params):
        default_required_params = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'config',
            'nav2_required_params.yaml',
        )
    default_rviz_config = os.path.join(
        nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    description_launch = os.path.join(
        description_share, 'launch', 'description.launch.py')
    default_lidar_launch = os.path.join(
        lslidar_share, 'launch', 'lsn10p_launch.py')
    default_uart_launch = os.path.join(
        uart_share, 'launch', 'uart_only.launch.py')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    odom_topic = LaunchConfiguration('odom_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_lidar = LaunchConfiguration('use_lidar')
    use_uart = LaunchConfiguration('use_uart')
    use_robot_state = LaunchConfiguration('use_robot_state')
    use_rtt_bridge = LaunchConfiguration('use_rtt_bridge')
    lidar_launch = LaunchConfiguration('lidar_launch')
    uart_launch = LaunchConfiguration('uart_launch')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')

    localization_lifecycle_nodes = [
        'map_server',
        'amcl',
    ]

    navigation_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    required_params = ParameterFile(
        RewrittenYaml(
            source_file=default_required_params,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
                'odom_topic': odom_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
                'odom_topic': odom_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    nav2_params = [required_params, configured_params]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map yaml file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo /clock when true. Set false on the real robot.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Respawn Nav2 nodes if they crash.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel_nav_smoothed',
            description='Smoothed Nav2 velocity command topic.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic used by bt_navigator and velocity_smoother.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz2 with map, robot, global costmap, and local costmap displays.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to the RViz2 config file.',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the N10P serial lidar driver.',
        ),
        DeclareLaunchArgument(
            'use_uart',
            default_value='true',
            description='Start UART feedback, /odom and dynamic TF.',
        ),
        DeclareLaunchArgument(
            'use_robot_state',
            default_value='true',
            description='Start robot_state_publisher for the robot model.',
        ),
        DeclareLaunchArgument(
            'use_rtt_bridge',
            default_value='true',
            description='Convert Nav2 cmd_vel to RTT /send_cmd.',
        ),
        DeclareLaunchArgument(
            'lidar_launch',
            default_value=default_lidar_launch,
            description='Full path to the N10P lidar launch file.',
        ),
        DeclareLaunchArgument(
            'uart_launch',
            default_value=default_uart_launch,
            description='Full path to the UART odometry launch file.',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.1',
            description='Absolute RTT target speed limit in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='0.45',
            description='Absolute RTT angular velocity limit in rad/s.',
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
            name='static_base_footprint_to_laser',
            output='screen',
            arguments=[
                '0.075', '0', '0.1425',
                '0', '0', '0',
                'base_footprint', 'laser',
            ],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings,
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav2_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', cmd_vel_out),
            ],
        ),
        Node(
            package='roscar_task',
            executable='cmd_vel_to_rtt',
            name='cmd_vel_to_rtt',
            output='screen',
            condition=IfCondition(use_rtt_bridge),
            parameters=[{
                'cmd_vel_topic': cmd_vel_out,
                'send_topic': '/send_cmd',
                'max_speed': max_linear_speed,
                'max_angular_velocity': max_angular_velocity,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': localization_lifecycle_nodes},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': navigation_lifecycle_nodes},
            ],
        ),
    ])
