import os

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import ReplaceString

import xacro

def modify_yaml_first_line(input_file, new_first_line):
    with open(input_file, "r") as f:
        lines = f.readlines()

    lines[0] = new_first_line + ':\n'

    with open(input_file, "w") as f:
        f.writelines(lines)

def update_yaml(context):
    robot_name_value = LaunchConfiguration('robot_name').perform(context)  
    controllers_file = os.path.join(get_package_share_directory('my_package'), 'config', 'controllers.yaml')
    
    modify_yaml_first_line(controllers_file, robot_name_value)

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')    
    search_type = LaunchConfiguration('search_type')
    x= LaunchConfiguration('x')
    y= LaunchConfiguration('y')
    maps_dir = LaunchConfiguration('maps_dir')
    map_file_name = LaunchConfiguration('map_file_name')

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

    detector = Node(
        package='my_package',
        executable='jetbot_detect',
        namespace=robot_name
    )

    search = Node(
        package='my_package',
        executable=['jetbot_search_', search_type],
        namespace=robot_name
    )

    pather = Node(
        package='my_package',
        executable='jetbot_pather',
        namespace=robot_name
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'base_frame_id': [robot_name, '/base_link'],
            'odom_frame_id': [robot_name, '/odom'],
            'global_frame_id': 'map',
            'scan_topic': 'scan',
            'map_topic': '/map',
            'use_sim_time': use_sim_time,
            'laser_min_range': 0.8,
            'laser_max_range': 10.0,
            'tf_broadcast': True,
            'set_initial_pose': True,
            'initial_pose': {
                'x': x,
                'y': y,
                'z': 0.0,
                'yaw': 0.0
            },
            #'transform_tolerance': 1.0,
            #'max_beams': 60,
            #'update_min_d': 0.25,
            #'update_min_a': 0.2
        }]
    )

    lifecycle_manager_for_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
        }]
    )

    namespaced_params = ReplaceString(
        source_file=os.path.join(
            get_package_share_directory('my_package'), 'config', 'navigation.yaml'),
        replacements={
            '<robot_namespace>': robot_name
        }
    )

    navigation = GroupAction([
        PushRosNamespace(robot_name),
        SetRemap(src='/tf', dst='/tf'),
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='/tf_static', dst='/tf_static'),
        SetRemap(src='tf_static', dst='/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': robot_name,
                'use_sim_time': use_sim_time,
                'params_file': namespaced_params,
            }.items(),
        )
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('robot_name', default_value='jetbot'),
        DeclareLaunchArgument('search_type', default_value='random'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('maps_dir', default_value='~/maps', description='Directory to maps folder'),
        DeclareLaunchArgument('map_file_name', default_value='jetbot_map', description='Name of map file'),

        # OpaqueFunction(function=update_yaml),
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        detector,
        search,
        pather,
        amcl_node,
        TimerAction(period=2.0, actions=[lifecycle_manager_for_amcl]),
        navigation
    ])
