import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from nav_msgs.msg import Odometry
import csv
import os
import sys

class PathLogger(Node):
    def __init__(self):
        super().__init__('path_logger')
        self.subscription = self.create_subscription(
            Odometry,
            f'{self.get_namespace()}/diff_controller/odom',
            self.odom_callback,
            10
        )
        self.head_sub = self.create_subscription(
            String, 
            '/signal', 
            self.head_signal, 
            10
        )

        self.running = False
        self.file_path = os.path.expanduser(f'~{self.get_namespace()}_path.csv')
        with open(self.file_path, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z', 'timestamp']) 

    def odom_callback(self, msg):
        if not self.running:  
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        timestamp = self.get_clock().now().to_msg().sec

        with open(self.file_path, 'a') as file:
            writer = csv.writer(file)
            writer.writerow([x, y, z, timestamp])


    def head_signal(self, msg):
        if msg.data == "Start":
            self.get_logger().info("Start!")
            self.running = True

        elif msg.data == "Stop":
            self.get_logger().info("Stop!")
            self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = PathLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
