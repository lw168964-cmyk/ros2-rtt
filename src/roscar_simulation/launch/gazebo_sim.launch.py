import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory('roscar_description')
    simulation_pkg = get_package_share_directory('roscar_simulation')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(
        simulation_pkg,
        'worlds',
        'custom_room.world'
    )
    default_model = os.path.join(
        simulation_pkg,
        'urdf',
        'roscar_gazebo_sim.xacro'
    )

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    entity = LaunchConfiguration('entity')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ground_truth_odom = LaunchConfiguration('use_ground_truth_odom')
    verbose = LaunchConfiguration('verbose')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'verbose': verbose,
        }.items()
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg,
                'launch',
                'description.launch.py'
            )
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': use_sim_time,
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_roscar',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', entity,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ]
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_diff_drive_controller',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )

    ground_truth_odom = TimerAction(
        condition=IfCondition(use_ground_truth_odom),
        period=3.0,
        actions=[Node(
            package='roscar_simulation',
            executable='gazebo_ground_truth_odom.py',
            name='gazebo_ground_truth_odom',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_name': entity,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'relative_to_start': True,
                'publish_tf': True,
            }],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the robot xacro or URDF file.',
        ),
        DeclareLaunchArgument(
            'entity',
            default_value='roscar',
            description='Entity name used when spawning the robot in Gazebo.',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x position.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn y position.'),
        DeclareLaunchArgument('z', default_value='0.0', description='Robot spawn z position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw angle.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'use_ground_truth_odom',
            default_value='true',
            description='Publish relative Gazebo odom and TF for stable simulation navigation.',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Start Gazebo with verbose logging.',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                os.path.join(simulation_pkg, 'worlds'),
                ':',
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        gazebo_launch,
        description_launch,
        delayed_spawn,
        delayed_controllers,
        ground_truth_odom
    ])
