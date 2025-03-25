import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(get_package_share_directory('my_package'), 'worlds', 'first_with_objects'),
            'extra_gazebo_args': '--reset'
        }.items()
    )
    
    jetbots = []
    for i, (name, y) in enumerate([("jetbot_1", "0.0"), ("jetbot_2", "1.0"), ("jetbot_3", "-1.0")]):
        jetbot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('my_package'), 'launch', 'jetbot.launch.py')
            ]),
            launch_arguments={'robot_name': name, 'y': y}.items()
        )

        jetbots.append(TimerAction(period=i * 5.0, actions=[jetbot]))

    return LaunchDescription([gazebo] + jetbots)