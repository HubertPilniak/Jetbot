from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    world_to_map_merge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_merge_static_transform',
        arguments=['0','0','0','0','0','0','world','map_merge'],
        parameters=[{'use_sim_time': True}]
    )

    map_merge = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[{
            'robot_map_topic': 'map',
            'robot_namespace': 'jetbot',
            'merged_map_topic': 'merged_map',
            'world_frame': 'world',
            'known_init_poses': False,
            'use_sim_time': True,
            'estimation_confidence': 0.7,
            'merging_rate': 0.2
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        world_to_map_merge,
        map_merge
    ])
