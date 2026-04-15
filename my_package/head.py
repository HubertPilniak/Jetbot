import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import threading
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

class Head(Node):
    def __init__(self):
        super().__init__('head')

        self.detect_sub = self.create_subscription(
            String,
            '/detect',
            self.detector_callback,
            10
        )
        self.robot_command_pub = self.create_publisher(
            String,
            '/robot_command',
            10
        )
        self.work_report_sub = self.create_subscription(
            String,
            '/work_report',
            self.work_report_callback,
            10
        )
        
        self.bridge = CvBridge()

        self.keyboard_thread = threading.Thread(target=self.keyboard_input, daemon=True)
        self.keyboard_thread.start()

        self.image_saved = False

        self.robots_positions = {}

        self.robot_name = ""

        self.blocked_robot = None
     
    def work_report_callback(self, msg):
        robot_namespace, points_count, visited_count, failed_count = msg.data.split("|")
        self.get_logger().info("--------------")
        self.get_logger().info(f"Robot: {robot_namespace}:")
        self.get_logger().info(f"points: {points_count}, visited: {visited_count}, failed: {failed_count}")
        self.check_time()
        self.get_logger().info("--------------")

    def command_stop(self):
        msg = String()
        msg.data = "Stop"
        self.robot_command_pub.publish(msg)
        self.get_logger().info("Sent Stop!")

    def command_start(self):
        msg = String()
        msg.data = "Start"
        self.robot_command_pub.publish(msg)
        self.start_time = self.get_clock().now()
        self.get_logger().info("Sent Start!")

        self.image_saved = False

    def command_save_map(self):
        msg = String()
        msg.data = "SaveMap"
        self.robot_command_pub.publish(msg)
        self.get_logger().info("Sent SaveMap!")

    def detector_callback(self, msg):
        self.robot_name, status = msg.data.split("|")

        if status == "Color detected!":
            self.get_logger().info(f'Object finded!')
            self.command_stop()
            self.get_logger().info(self.robot_name + " " + status)

            self.image_sub = self.create_subscription(
                Image,
                f'{self.robot_name}/camera/image_raw',
                self.save_image_once,
                1
            )
            msg = String()
            msg.data = "Publish work"
            self.robot_command_pub.publish(msg)
    
    def keyboard_input(self):
        while True:
            user_input = input().strip().split()
            
            command = user_input[0].lower()
            args = user_input[1:] 

            if command.lower() == "start":
                self.command_start()
                
            elif command.lower() == "stop":
                self.command_stop()

            elif command.lower() == "savemap":
                self.command_save_map()
    
    def save_image_once(self, msg):
        if self.image_saved:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = os.path.expanduser(f'~{self.robot_name}_color_detected.png')
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Camera image saved as {self.robot_name}_color_detected.png')

        self.image_saved = True

        self.destroy_subscription(self.image_sub)

    def check_time(self):
        duration = self.get_clock().now() - self.start_time
        total_seconds = duration.nanoseconds / 1e9

        minutes = int(total_seconds // 60)
        seconds = total_seconds % 60

        self.get_logger().info(f"Searching time: {minutes:02d}:{seconds:06.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = Head() 
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()