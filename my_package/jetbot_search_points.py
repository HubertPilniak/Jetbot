import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class RoomSearchPoints(Node):
    def __init__(self):
        super().__init__('room_search_points')

        self.robot_namespace = self.get_namespace().strip("/")

        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            f'{self.get_namespace()}/cmd_vel', 
            10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 
            f'{self.get_namespace()}/scan', 
            self.laser_callback, 
            10
        )
        self.head_sub = self.create_subscription(
            String, 
            '/robot_command', 
            self.head_callback,
            10
        )
        self.points_sub = self.create_subscription(
            String,
            '/assigned_points',
            self.points_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.get_namespace()}/odom',
            self.odom_callback,
            1
        )
        self.work_report_pub = self.create_publisher(
            String,
            '/work_report',
            10
        )

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg()

        init_pose.pose.position.x = self.start_x
        init_pose.pose.position.y = self.start_y

        init_pose.pose.orientation.z = math.sin(self.start_yaw / 2.0)
        init_pose.pose.orientation.w = math.cos(self.start_yaw / 2.0)

        self.navigator = BasicNavigator(namespace=self.robot_namespace)

        self.navigator.setInitialPose(init_pose)

        self.odom_received = False

        self.current_yaw = None

        self.prev_yaw = None

        self.critical_distance_robots = 0.20 # 20cm, radius of the robot is 10.6625cm, lidar radius is 3.25cm

        self.critical_distance = 0.12 # 12cm, radius of the robot is 10.6625cm

        self.point_received = False
        self.running = False

        self.failed_points = []

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active")

        self.get_logger().info("Waiting for points...")

        self.points = []
        self.current_point_index = 0

        self.running = False
        self.goal_sent = False
        self.is_spinning = False
        self.spin_target = 2.0 * math.pi - 0.05
        self.spin_accumulated_rotation = 0.0
        self.wait_for_space = False
        self.already_published_work = False

        self.mode = "navigate"
        self.timer = self.create_timer(1, self.control_loop)

    def publish_work(self):

        msg = String()
        visited_count= self.current_point_index - len(self.failed_points)
        msg.data = f"{self.robot_namespace}|{len(self.points)}|{visited_count}|{len(self.failed_points)}"
        self.work_report_pub.publish(msg)
        self.get_logger().info(f"Published work report: {msg.data}")

    def control_loop(self):
        if self.mode == "navigate":
            self.execute_points()
        elif self.mode == "spin":
            self.spin_360()

    def head_callback(self, msg):

        if msg.data == "Start":
            self.get_logger().info("Start!")
            self.running = True

        elif msg.data == "Stop":
            self.get_logger().info("Stop!")
            self.running = False
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif msg.data == self.robot_namespace:
            self.get_logger().info("Received command to unlock")
            self.running = True
            self.unblocking = True
            self.unblocked_timer = self.create_timer(5.0, self.is_unblocked)

        elif msg.data == "Publish work":
            self.get_logger().info("Received command to publish work")
            if self.already_published_work:
                self.get_logger().info("Already published work")
                return
            self.publish_work()
            
    def laser_callback(self, msg):
        if not self.running:
            return

        self.ranges = np.array(msg.ranges)

        if np.min(self.ranges) < self.critical_distance:
            self.running = False
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f"{self.robot_namespace }: Something critically close!")
            self.publish_work()
            self.timer.cancel()
            self.navigator.cancelTask()
            self.destroy_node()
            rclpy.shutdown()
            
    def odom_callback(self, msg):
        
        q = msg.pose.pose.orientation

        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def points_callback(self, msg):
        if self.point_received:
            return

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            return

        routes = data.get("routes", [])

        my_route = None
        for route in routes:
            if route.get("robot_namespace") == self.robot_namespace:
                my_route = route
                break

        if my_route is None:
            self.get_logger().info(f"'{self.robot_namespace}': No points found ")
            return

        points = my_route.get("world_points", [])
        
        if not points:
            self.get_logger().warn(f"'{self.robot_namespace}': Received no points")
            self.point_received = True
            return

        self.points = points

        self.point_received = True
        self.get_logger().info(f"'{self.robot_namespace}': Received {len(points)} points ")

    def make_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        return pose

    def execute_points(self):
        if self.is_spinning:
            return

        if not self.running:
            if self.goal_sent:
                self.get_logger().info(
                    f"Pause requested during navigation to point {self.current_point_index}, canceling current goal..."
                )
                self.navigator.cancelTask()
                self.goal_sent = False
            return

        if self.current_point_index >= len(self.points):
            self.get_logger().info("All points visited!")
            self.running = False
            self.goal_sent = False
            self.publish_work()
            return

        if not self.goal_sent:
            pt = self.points[self.current_point_index]
            x = pt["x"]
            y = pt["y"]

            self.get_logger().info(
                f"Going to point {self.current_point_index}: x={x:.3f}, y={y:.3f}"
            )

            goal_pose = self.make_pose(x, y)
            self.navigator.goToPose(goal_pose)

            self.goal_sent = True
            return

        if not self.navigator.isTaskComplete():
            return

        pt = self.points[self.current_point_index]
        x = pt["x"]
        y = pt["y"]

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(
                f"Reached point {self.current_point_index} ({x:.3f}, {y:.3f})"
            )

            self.mode = "spin"

            self.current_point_index += 1
            self.goal_sent = False
            return

        if result == TaskResult.CANCELED:
            self.get_logger().warn(
                f"Navigation to point {self.current_point_index} ({x:.3f}, {y:.3f}) was canceled"
            )

            self.goal_sent = False
            return

        self.get_logger().error(
            f"Failed to reach point {self.current_point_index} ({x:.3f}, {y:.3f})"
        )

        self.failed_points.append(self.current_point_index)
        self.current_point_index += 1
        self.goal_sent = False

    def spin_360(self, angular_speed=0.8):
        if self.current_yaw == None:
            return

        if not self.is_spinning and self.current_yaw != None:
            self.is_spinning = True
            self.prev_yaw = self.current_yaw
            
            self.get_logger().info("Starting 360-degree rotation")
            return          

        if not self.running:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            return 

        delta = self.normalize_angle(self.current_yaw - self.prev_yaw)
        self.spin_accumulated_rotation += abs(delta)
        self.prev_yaw = self.current_yaw

        if self.spin_accumulated_rotation >= self.spin_target:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

            self.is_spinning = False
            self.spin_accumulated_rotation = 0.0
            self.prev_yaw = None

            self.get_logger().info("Rotation complete")
            self.mode = "navigate"
            return 

        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

        return 

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)

    node = RoomSearchPoints()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()