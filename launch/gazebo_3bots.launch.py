import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    search_type = LaunchConfiguration('search_type')

    world = LaunchConfiguration('world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                        get_package_share_directory('my_package'), 'worlds', world
                    ]),
            'extra_gazebo_args': '--reset'
        }.items()
    )
    
    jetbots = []
    for i, (name, y, slam_enable) in enumerate([("jetbot_1", "0.0", "true"), ("jetbot_2", "1.0", "false"), ("jetbot_3", "-1.0", "false")]):
        jetbot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('my_package'), 'launch', 'jetbot_sim.launch.py')
            ]),
            launch_arguments={'robot_name': name, 'y': y, 'slam_enable': slam_enable, 'search_type': search_type}.items()
        )

        jetbots.append(TimerAction(period=i * 5.0, actions=[jetbot]))

    return LaunchDescription([DeclareLaunchArgument('search_type', default_value='random'),
                              DeclareLaunchArgument('world', default_value='setup_1'),
                              gazebo
                              ] + jetbots)