import numpy as np
from example_interfaces.msg import String
import ast

class Callbacks:
    def __init__(self, node):
        self.node = node

    def head_callback(self, msg):

        if msg.data == "Start":
            self.node.get_logger().info("Start!")
            self.node.running = True

        elif msg.data == "Stop":
            self.node.get_logger().info("Stop!")
            self.node.running = False
            self.node.twist.linear.x = 0.0
            self.node.twist.angular.z = 0.0
            self.node.cmd_vel_pub.publish(self.node.twist)
        #
        if msg.data.__contains__("|"):
            robot_name, cell_coordinates = msg.data.split("|")

            if robot_name == self.node.get_namespace():
                self.node.get_logger().info(f"Dostalem koordynaty {cell_coordinates}")
                self.node.cell_coordinates = ast.literal_eval(cell_coordinates)
                self.node.t_d = True
        #

    def laser_callback(self, msg):

        self.node.ranges = np.array(msg.ranges)

        if np.min(self.node.ranges) < self.node.critical_distance:
            self.node.running = False
            self.node.twist.linear.x = 0.0
            self.node.twist.angular.z = 0.0
            self.node.cmd_vel_pub.publish(self.node.twist)

            self.node.get_logger().info("Something too close!")

        self.node.front_ranges = np.array(msg.ranges[135:226]) 
        self.node.left_ranges = np.array(msg.ranges[226:276])
        self.node.right_ranges = np.array(msg.ranges[90:135])
        self.node.rear_ranges = np.concatenate((msg.ranges[315:], msg.ranges[:46]))
        

    def odom_callback(self, msg):  

        self.node.my_pose = msg.pose.pose.position
        self.node.my_orientation = msg.pose.pose.orientation

        msg_to_send = String()
        msg_to_send.data = f'{self.node.get_namespace()}|{self.node.my_pose.x}|{self.node.my_pose.y}'
        self.node.robot_position_pub.publish(msg_to_send)

    def __del__(self):
        print("JetbotMethods DESTROYED")
    