import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import random

class RoomSearch(Node):
    def __init__(self):
        super().__init__('room_search')
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            f'{self.get_namespace()}/cmd_vel', 
            0
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 
            f'{self.get_namespace()}/scan', 
            self.laser_callback, 
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.get_namespace()}/odom',
            self.odom_callback,
            1
        )
        self.robot_position_pub = self.create_publisher(
            String, 
            '/robot_position', 
            10
        )
        self.head_sub = self.create_subscription(
            String, 
            '/robot_command', 
            self.head_signal, 
            10
        )
        self.grid_sub = self.create_subscription(
            OccupancyGrid, 
            '/grid', 
            self.grid_callback, 
            1
        )
        self.obstacle_position_pub = self.create_publisher(
            Point, 
            '/obstacle_position', 
            1
        )

        self.timer = self.create_timer(0.1, self.search)

        self.twist = Twist()

        self.ranges = np.array([])
        self.front_ranges = np.array([])
        self.left_ranges = np.array([])
        self.right_ranges = np.array([])
        self.rear_ranges = np.array([])

        self.my_pose = Point()
        self.my_orientation = Quaternion()

        self.initial_orientation = Quaternion()

        self.grid = OccupancyGrid()

        self.critical_distance = 0.20 # 20 cm distance is minimal safe, around 15 cm between two lidars

        self.obstacle_distance_min = 0.3

        self.obstacle_distance_mid = 0.4

        self.running = False

        self.facing_direction = (1, 0)

    def grid_callback(self, msg):
        self.grid = msg

    def laser_callback(self, msg):
        self.ranges = np.array(msg.ranges)

        if np.min(self.ranges) < self.critical_distance:
            self.running = False
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            self.get_logger().info("Something too close!")

        self.front_ranges = np.array(msg.ranges[161:200])  # 

    def odom_callback(self, msg):
        if not self.running:  
            return
        
        self.my_pose = msg.pose.pose.position
        self.my_orientation = msg.pose.pose.orientation

        msg_to_send = String()
        msg_to_send.data = f'{self.get_namespace()}|{self.my_pose.x}|{self.my_pose.y}'
        self.robot_position_pub.publish(msg_to_send)

    def search(self):
        if not self.running:
            return

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        front_min = np.min(self.front_ranges)
        
        if front_min > self.obstacle_distance_mid:
            self.twist.linear.x = 0.2

        elif front_min > self.obstacle_distance_min:
            self.twist.linear.x = 0.1
        else:
            self.cmd_vel_pub.publish(self.twist)

            obstacle_pose = Point()

            obstacle_pose.x = self.my_pose.x + self.facing_direction[0] * self.obstacle_distance_min
            obstacle_pose.y = self.my_pose.y + self.facing_direction[1] * self.obstacle_distance_min

            self.obstacle_position_pub.publish(obstacle_pose)

            self.start_changing_direction()

            return

        self.cmd_vel_pub.publish(self.twist) 

    def start_changing_direction(self):

        self.initial_orientation = self.my_orientation
            
        self.timer.cancel()                                              

        angle_to_rotate = 90
            
        self.get_logger().info(f'Rotating by {angle_to_rotate} degree')

        turn = 0.2

        angle_to_rotate = angle_to_rotate * np.pi / 180 # angle to radians,

        self.timer = self.create_timer(0.1, lambda: self.change_direction(turn, angle_to_rotate))

    def change_direction(self, turn, angle):
        if not self.running:  
            return

        if self.quaternion_angle(self.my_orientation, self.initial_orientation) >= angle*0.98 : # when close to expected angle slow down
            self.twist.linear.x = 0.0
            self.twist.angular.z = turn / 5.0

            if self.quaternion_angle(self.my_orientation, self.initial_orientation) >= angle: # we assume that we end rotation when we are equal or above expected angle
                self.timer.cancel()
                self.timer = self.create_timer(0.1, self.search)
        
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = turn

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

    def quaternion_angle(self, q1, q2):
        angle = 0.0
        angle += q1.x * q2.x
        angle += q1.y * q2.y
        angle += q1.z * q2.z
        angle += q1.w * q2.w
        angle = np.clip(angle, -1.0, 1.0)  # clip -> sets the number to a given value if it exceeds the range; make sure it doesn't exceed angle value
        angle = 2 * np.arccos(abs(angle))
        return angle  # in radians
            
    def grid_callback(self, msg):
        self.grid = msg
        
         

def main(args=None):
    rclpy.init(args=args)
    node = RoomSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
