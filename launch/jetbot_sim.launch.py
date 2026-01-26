import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import xacro

def modify_yaml_first_line(input_file, new_first_line):
    with open(input_file, "r") as f:
        lines = f.readlines()

    lines[0] = new_first_line + ':\n'

    with open(input_file, "w") as f:
        f.writelines(lines)

def update_yaml(context):
    robot_name_value = LaunchConfiguration('robot_name').perform(context)  
    controllers_file = os.path.join(get_package_share_directory('my_package'), 'config', 'controllers.yaml'
    )
    
    modify_yaml_first_line(controllers_file, robot_name_value)

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_name = LaunchConfiguration('robot_name')
    
    search_type = LaunchConfiguration('search_type')

    y= LaunchConfiguration('y')

    pkg_path = os.path.join(get_package_share_directory('my_package'))
    xacro_file = os.path.join(pkg_path,'models', 'Jetbot_v1', 'model.urdf.xacro')


    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file, ' ',
            'robot_name:=', robot_name
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', 
        namespace=robot_name,
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=robot_name,
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', ['/', robot_name, '/robot_description'],
                    '-entity', robot_name,
                    '-y', y,
                    '-robot_namespace', robot_name],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=["diff_controller",
                   ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=["joint_broadcaster",
                   ],
    )

    detector_start = Node(
        package='my_package',
        executable='jetbot_detect',
        namespace=robot_name
    )

    search_start = Node(
        package='my_package',
        executable=['jetbot_search_', search_type],
        namespace=robot_name
    )

    pather_start = Node(
        package='my_package',
        executable='jetbot_pather',
        namespace=robot_name
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'base_frame': [robot_name, '/base_link'],
            'odom_frame': [robot_name, '/odom'],
            'map_frame':  [robot_name, '/map'],
            'scan_topic': 'scan',
            'use_sim_time': use_sim_time,
            'map_update_interval': 1.0,      # map rewfresh
            'resolution': 0.1,             #  resolution of the map in cm
            'minimum_time_interval': 0.2,    # time between scans
            'transform_timeout': 0.2,
            'min_laser_range': 0.1,
            'max_laser_range': 10.0, 
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('robot_name', default_value='jetbot'),
        DeclareLaunchArgument('search_type', default_value='random'),
        DeclareLaunchArgument('y', default_value='0.0'),

        # OpaqueFunction(function=update_yaml),
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        detector_start,
        search_start,
        pather_start,
        slam
    ])
