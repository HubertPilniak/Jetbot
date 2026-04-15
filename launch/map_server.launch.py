from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': [EnvironmentVariable('HOME'), "/", map_dir, "/", map_name, ".yaml"],
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('map_dir', default_value='maps'),
        DeclareLaunchArgument('map_name', default_value='jetbot_map'),
        map_server,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])