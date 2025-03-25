from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    name = DeclareLaunchArgument('name', default_value='jetbot')
    model = DeclareLaunchArgument('model', default_value='Jetbot_v1')
    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    z = DeclareLaunchArgument('z', default_value='0.0')

    spawn = Node(package='my_package', executable='jetbot_spawn',
                        parameters=[
                            {'name': LaunchConfiguration('name')},
                            {'model': LaunchConfiguration('model')},
                            {'x': LaunchConfiguration('x')},
                            {'y': LaunchConfiguration('y')},
                            {'z': LaunchConfiguration('z')},
                        ],
                           output='screen')           
       
    return LaunchDescription([
        name,
        model,   
        x,
        y,
        z,
        spawn
    ])
