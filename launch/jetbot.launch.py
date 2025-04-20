from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    search_type = LaunchConfiguration('search_type')

    detector_start = Node(
        package='my_package',
        executable='jetbot_detect'
    )

    search_start = Node(
        package='my_package',
        executable=search_type
    )

    pather_start = Node(
        package='my_package',
        executable='jetbot_pather'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('search_type', default_value='jetbot_search_random'),

        detector_start,
        search_start,
        pather_start
    ])
