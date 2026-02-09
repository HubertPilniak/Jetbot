#!/bin/bash

ROS_SOURCE="source /opt/ros/humble/setup.bash"
WS_SOURCE="source ~/ros2_ws/install/setup.bash"
GAZEBO_WORLD="ros2_ws/src/my_package/worlds/setup_1"


gnome-terminal --window --title="Gazebo" -- bash -c "$ROS_SOURCE; $WS_SOURCE; ros2 launch gazebo_ros gazebo.launch.py world:=$GAZEBO_WORLD; exec bash" 
gnome-terminal --tab --title="Head"   -- bash -c "$ROS_SOURCE; $WS_SOURCE; ros2 run my_package head; exec bash" 
gnome-terminal --tab --title="Map Merge" -- bash -c "$ROS_SOURCE; $WS_SOURCE; ros2 launch my_package map_merge.launch.py; exec bash"

exit 0

