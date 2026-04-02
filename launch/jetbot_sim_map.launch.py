import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    x= LaunchConfiguration('x')
    y= LaunchConfiguration('y')
    maps_dir = LaunchConfiguration('maps_dir')

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
                    '-x', x,
                    '-y', y,
                    '-robot_namespace', robot_name],
        output='screen'
    )

    pather = Node(
        package='my_package',
        executable='jetbot_pather',
        namespace=robot_name
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'mode': 'mapping',
            'map_frame': '/map',
            'base_frame': [robot_name, '/base_link'],
            'odom_frame': [robot_name, '/odom'],
            'scan_topic': 'scan',
            'use_sim_time': use_sim_time,
            'map_update_interval': 1.0,
            'resolution': 0.05,
            'min_laser_range': 0.08,
            'max_laser_range': 9.0,
            'ceres_thread_count': 6
        }],
        remappings=[
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata')
        ]
    )

    map_manager = Node(
        package='my_package',
        executable='jetbot_mapper',
        name='mapper',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'maps_dir': maps_dir,
            'image_format': 'pgm',
            'map_mode': 'trinary',
            'free_thresh': 0.25,
            'occupied_thresh': 0.65,
        }]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': robot_name,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(get_package_share_directory('my_package'), 'config', 'navigation.yaml'),
        }.items(),
    )

    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_base_frame': [robot_name, '/base_link'],
            'costmap_topic': '/map',
            'costmap_updates_topic': '/map_updates',
            'visualize': True,
            'planner_frequency': 0.5,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.4,
            'return_to_init': False,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('robot_name', default_value='jetbot'),
        DeclareLaunchArgument('search_type', default_value='random'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('maps_dir', default_value='~/maps', description='Directory to save map'),

        # OpaqueFunction(function=update_yaml),
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        pather,
        slam,
        map_manager,
        #navigation,
        #explore
    ])
