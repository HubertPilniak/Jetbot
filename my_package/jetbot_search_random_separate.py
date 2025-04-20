import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math


class RoomSearch(Node):
    def __init__(self):
        super().__init__('room_search')
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            f'{self.get_namespace()}/cmd_vel', 
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
            f'{self.get_namespace()}/odom',
            self.odom_callback,
            1
        )
        self.head_sub = self.create_subscription(
            String, 
            '/robot_command', 
            self.head_signal, 
            10
        )
        self.robot_position_pub = self.create_publisher(
            String, 
            '/robot_position', 
            10
        )
        self.robot_position_sub = self.create_subscription(
            String,
            '/robot_position',
            self.update_robots_positions_and_distances,
            10
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

        self.critical_distance = 0.20 # 20 cm distance is minimal safe, around 15 cm between two lidars

        self.obstacle_distance_min = 0.35

        self.obstacle_distance_mid = 0.40

        self.side_obstacle_distance_min = 0.25

        self.angles_distance_min = 0.5

        self.running = False

        self.waiting_for_safe_angle = False

        self.robots_positions = {}

        self.robots_distances = {}

        self.robot_distance_min = 2.0

    def laser_callback(self, msg):
        self.ranges = np.array(msg.ranges)

        if np.min(self.ranges) < self.critical_distance:
            self.running = False
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            self.get_logger().info("Something too close!")

        self.front_ranges = np.array(msg.ranges[135:226]) 
        self.left_ranges = np.array(msg.ranges[226:276])
        self.right_ranges = np.array(msg.ranges[90:135])
        self.rear_ranges = np.concatenate((msg.ranges[315:], msg.ranges[:46]))

    def odom_callback(self, msg):
        
        self.my_pose = msg.pose.pose.position
        self.my_orientation = msg.pose.pose.orientation

        msg_to_send = String()
        msg_to_send.data = f'{self.get_namespace()}|{self.my_pose.x}|{self.my_pose.y}'
        self.robot_position_pub.publish(msg_to_send)

    def search(self):
        if not self.running:
            return

        front_center_min = np.min(self.front_ranges[15:76]) # ranges[15:76] -> front scan from lidar; 60 degrees;


        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        angle = self.check_distances() # check distances

        if angle:                       # if check_distance returns angle then it means that we need to rotate robot
            self.start_changing_direction(angle)
            return


        if front_center_min > self.obstacle_distance_mid:
            self.twist.linear.x = 0.2

            self.avoid_side_collisions()

        elif front_center_min > self.obstacle_distance_min:
            self.twist.linear.x = 0.05

            self.avoid_front_side_collisions()

            self.avoid_side_collisions()
            
        else:
            self.cmd_vel_pub.publish(self.twist)

            self.start_changing_direction(None)

            return
        
        self.cmd_vel_pub.publish(self.twist)

    def change_direction(self, turn, angle):
        if not self.running:  
            return

        if self.quaternion_angle(self.my_orientation, self.initial_orientation) >= angle*0.99 : # when close to expected angle slow down
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
        angle = np.clip(angle, -1.0, 1.0)  # clip -> sets the number to a given value if it exceeds the range; make sure it doesn't exceed cos value
        angle = 2 * np.arccos(abs(angle))
        return angle  # in radians
    
    def find_safe_angles(self, avoid_angle):  # avoid_angle is half of the angle behind the robot that we avoid to not repeat path  
        
        if avoid_angle != 0:
            avoid_angle = int((avoid_angle - 1) / 2.0)

            start_angle = avoid_angle + 1
        else:
            start_angle = avoid_angle
        
        end_angle = avoid_angle

        obstacle_distance = self.angles_distance_min # in what distance to obstacle we don't want to ride;

        safe_angles = np.array([])

        space = 30 # space from obstacle in degree
        start = 0  + avoid_angle

        length_of_array = len(self.ranges)

        got_zero = False

        for i in range(start_angle, length_of_array - end_angle):   
            if self.ranges[i] < obstacle_distance:   # find first range from start index that is less than min distance
                safe_space = i - 1 - space                      

                if safe_space >= start:                         # when we find one check, if there are elements to add to result(after including safe space distance)
                    safe_angles = np.concatenate((safe_angles, np.arange(start, safe_space+1)))

                count = 0

                while(count != space and i < length_of_array  - avoid_angle - 1):  # find a sequence, of distances greater than the min distance, of length 'space' variable
                    i += 1

                    if self.ranges[i] >= obstacle_distance:
                        count += 1

                    else:
                        count = 0

                start = i+1                    

        if start < length_of_array:    
            safe_angles = np.concatenate((safe_angles, np.arange(start, length_of_array - avoid_angle)))
                            
        return safe_angles

    def avoid_side_collisions(self):
        left_min = np.min(self.left_ranges)
        right_min = np.min(self.right_ranges)

    # Jeśli coś jest blisko z lewej strony
        if left_min < self.side_obstacle_distance_min:
            self.twist.linear.x = 0.0
            self.twist.angular.z -= 0.1  # skręć w prawo lekko

    # Jeśli coś jest blisko z prawej strony
        if right_min < self.side_obstacle_distance_min:
            self.twist.linear.x = 0.0
            self.twist.angular.z += 0.1  # skręć w lewo lekko

    def start_changing_direction(self, prefered_angle):


        self.initial_orientation = self.my_orientation

        if self.waiting_for_safe_angle != True:
            safe_angles = self.find_safe_angles(21)  # parameter in find_safe_angle() is a value of angle to avoid behind the robot to not repeat path,
                                                         # if not 0, must be odd number
            
        else:
            safe_angles = self.find_safe_angles(0)  # we set avoid_angle to 0, all angles possible
                                                     
        if (len(safe_angles) == 0):                                      # if there is no safe angles, set waiting_for_safe_angle as True 
            self.get_logger().info(f'Waiting for available safe angle')  # so on next search() we will check all angles possible 
            self.waiting_for_safe_angle = True
            return  
            
        self.timer.cancel()

        self.waiting_for_safe_angle = False 

        if prefered_angle:                                                 
             angle_to_rotate = min(safe_angles, key=lambda angle: abs(angle - prefered_angle)) - 180
        else:
            angle_to_rotate = random.choice(safe_angles) - 180  # safe_angles can have values from 0 to 359, robot always facing 180 degree, 
                                                                # so after substraction of 180 we get angle about which we have to rotate
            
        self.get_logger().info(f'Rotating by {angle_to_rotate} degree')

        if angle_to_rotate < 0:
            angle_to_rotate *= -1
            turn = -0.2
        else:
            turn = 0.2

        angle_to_rotate = angle_to_rotate * np.pi / 180 # angle to radians,

        angle_to_rotate = angle_to_rotate * 0.99  # due to imperfections of calculations of this program and simulation we make a lower tolerance of 1%

        self.timer = self.create_timer(0.1, lambda: self.change_direction(turn, angle_to_rotate))

    def avoid_front_side_collisions(self):  # if obstacle is not in front of robot, turn from it
        front_right_min = np.min(self.front_ranges[:15])
        front_left_min = np.min(self.front_ranges[76:])

        if front_left_min < self.obstacle_distance_min: 
            self.twist.angular.z = -0.1 # turn right

        elif front_right_min < self.obstacle_distance_min:

            self.twist.angular.z = 0.1 # turn left

    def update_robots_positions_and_distances(self, msg):

        robot_name, robot_position_x, robot_position_y = msg.data.split("|")

        self.robots_positions[robot_name] = (float(robot_position_x), float(robot_position_y))
        
        robot_1 = self.get_namespace()

        if robot_1 in self.robots_positions:

            for robot_2, position in self.robots_positions.items():
                if robot_2 != robot_1:
                    self.robots_distances[robot_2] = math.dist(self.robots_positions[robot_1], position)

    def check_distances(self):
        angles = []

        robot_1 = self.get_namespace()

        for robot_2, distance in self.robots_distances.items():
            if distance < self.robot_distance_min:
                dx = self.robots_positions[robot_2][0] - self.robots_positions[robot_1][0]
                dy = self.robots_positions[robot_2][1] - self.robots_positions[robot_1][1]
                angle = math.atan2(dy, dx)
                angles.append(angle)

        if len(angles) == 0:
            return

        if len(angles) > 0:
            avg_angle = math.atan2(
                sum(math.sin(angle) for angle in angles) / len(angles),
                sum(math.cos(angle) for angle in angles) / len(angles)
            )
        else:
            avg_angle = angles[0]


        avg_angle = avg_angle + math.pi

        avg_angle = (avg_angle + math.pi) % (2 * math.pi) - math.pi

        avg_angle = math.degrees(avg_angle)

        self.get_logger().info(f'Kat azymutu {avg_angle}')   
        
        q = self.my_orientation

        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

        yaw = math.degrees(yaw)

        self.get_logger().info(f'Kat robota {yaw}')

        avg_angle = (avg_angle + 180 - yaw) % 360

        if avg_angle < 0:
            avg_angle += 360

        self.get_logger().info(f'Kat obrotu {avg_angle}')

        if avg_angle >= 185 or avg_angle <= 175: # only if rotation is bigger than 5 degree
            return avg_angle
        
        return

def main(args=None):
    rclpy.init(args=args)
    node = RoomSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
