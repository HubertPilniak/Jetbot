import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import random

class RoomSearch(Node):
    def __init__(self):
        super().__init__('room_search')
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            f'{self.get_namespace()}/diff_controller/cmd_vel_unstamped', 
            1
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 
            f'{self.get_namespace()}/scan', 
            self.laser_callback, 
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            f'{self.get_namespace()}/diff_controller/odom',
            self.odom_callback,
            10
        )
        self.pos_sub = self.create_subscription(
            PoseStamped, 
            '/robot_positions', 
            self.position_callback, 
            10
        )
        self.pos_pub = self.create_publisher(
            PoseStamped, 
            '/robot_positions', 
            10
        )
        self.head_sub = self.create_subscription(
            String, 
            '/signal', 
            self.head_signal, 
            10
        )

        self.x = 0.0
        self.y = 0.0

        self.timer = self.create_timer(0.5, self.search)

        self.twist = Twist()
        self.known_positions = set()
        self.other_robot_positions = []
        self.obstacle_distance = float('inf')
        self.running = False

    def laser_callback(self, msg):
        front_ranges = np.array(msg.ranges[100:260]) 
        left_ranges = np.array(msg.ranges[260:360])
        right_ranges = np.array(msg.ranges[0:100]) 

        self.obstacle_distance = np.min(front_ranges)
        self.left_distance = np.min(left_ranges)
        self.right_distance = np.min(right_ranges)

    def odom_callback(self, msg):
        if not self.running:  
            return
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def position_callback(self, msg):
        pos = (msg.pose.position.x, msg.pose.position.y)
        if pos not in self.other_robot_positions:
            self.other_robot_positions.append(pos)

    def search(self):
        if not self.running:  
            return

        my_pose = PoseStamped()
        my_pose.pose.position.x = self.x
        my_pose.pose.position.y = self.y
        self.pos_pub.publish(my_pose)

        my_pos = (my_pose.pose.position.x, my_pose.pose.position.y)
        if my_pos in self.known_positions:
            self.twist.angular.z = random.choice([-0.5, 0.5])
        else:
            self.known_positions.add(my_pos)


        if self.obstacle_distance < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def head_signal(self, msg):
        if msg.data == "Start":
            self.get_logger().info("Start!")
            self.running = True 

        elif msg.data == "Stop":
            self.get_logger().info("Stop!")
            self.running = False
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = RoomSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
