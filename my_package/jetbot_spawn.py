import os
import rclpy

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node("jetbot_spawner", namespace="tools")

    node.declare_parameter('name', 'jetbot')
    node.declare_parameter('model', 'Jetbot_v1')
    node.declare_parameter('x', 0.0)
    node.declare_parameter('y', 0.0)
    node.declare_parameter('z', 0.0)
    
    robot_name = node.get_parameter('name').value
    robot_model = node.get_parameter('model').value

    model_file = os.path.join(get_package_share_directory("my_package"), "models",
        robot_model, "model.urdf.xacro")
    

    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("Client ready")

    
    request = SpawnEntity.Request()
    request.name = robot_name
    request.xml = open(model_file, 'r').read()
    request.robot_namespace = robot_name
    request.initial_pose.position.x = float(node.get_parameter('x').value)
    request.initial_pose.position.y = float(node.get_parameter('y').value)
    request.initial_pose.position.z = float(node.get_parameter('z').value)


    future = client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print(future.result())
    else:
        print(future.exception())
    

    node.destroy_node()
    rclpy.shutdown()

    


if __name__ == "__main__":
    main()
