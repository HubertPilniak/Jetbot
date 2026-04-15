import os

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import ReplaceString


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')    
    search_type = LaunchConfiguration('search_type')
    x= LaunchConfiguration('x')
    y= LaunchConfiguration('y')
    yaw= LaunchConfiguration('yaw')
    spawn= LaunchConfiguration('spawn')

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
        condition=IfCondition(spawn),
        package='robot_state_publisher', executable='robot_state_publisher', 
        namespace=robot_name,
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    joint_state_publisher = Node(
        condition=IfCondition(spawn),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=robot_name,
        output='screen'
    )

    spawn_entity = Node(
        condition=IfCondition(spawn),
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', ['/', robot_name, '/robot_description'],
                    '-entity', robot_name,
                    '-x', x,
                    '-y', y,
                    '-robot_namespace', robot_name],
        output='screen'
    )

    detector = Node(
        package='my_package',
        executable='jetbot_detect',
        namespace=robot_name
    )

    search = Node(
        package='my_package',
        executable=['jetbot_search_', search_type],
        namespace=robot_name,
         parameters=[{
                'start_x': x,
                'start_y': y,
                'start_yaw': yaw
            }]
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
            'min_particles': 100,
            'max_particles': 300,
            'max_beams': 30,
            'transform_tolerance': 1.5,
            'tf_broadcast': True,
            'set_initial_pose': True,
            'initial_pose': {
                'x': x,
                'y': y,
                'z': 0.0,
                'yaw': yaw
            }
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
            '<robot_namespace>': robot_name,
            '<use_sim_time>': use_sim_time
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
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use sim time if true'),
        DeclareLaunchArgument('robot_name', default_value='jetbot'),
        DeclareLaunchArgument('search_type', default_value='points'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn', default_value='True', description='Spawn if true'),

        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        detector,
        pather,
        amcl_node,
        TimerAction(period=3.0, actions=[lifecycle_manager_for_amcl]),
        TimerAction(period=5.0, actions=[navigation]),
        TimerAction(period=7.0, actions=[search])
    ])
