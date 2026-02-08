import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from sensor_msgs.msg import Image
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

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

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

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            '/grid',
            map_qos
        )
        self.robot_position_sub = self.create_subscription(
            String,
            '/robot_position',
            self.grid_update_robot_position,
            10
        )
        self.laser_data_sub = self.create_subscription(
            String,
            '/jetbot/scan',
            self.grid_update_obstacles_positions,
            10
        )


        self.declare_parameter('map_width', 10.0)
        self.declare_parameter('map_height', 10.0)

        map_width = self.get_parameter('map_width').get_parameter_value().double_value
        map_height = self.get_parameter('map_height').get_parameter_value().double_value

        self.grid_create(map_width, map_height)
        
        self.bridge = CvBridge()

        self.keyboard_thread = threading.Thread(target=self.keyboard_input, daemon=True)
        self.keyboard_thread.start()

        self.image_saved = False

        self.robots_positions = {}

        self.robot_name = ""
     

    def command_stop(self):
        msg = String()
        msg.data = "Stop"
        self.robot_command_pub.publish(msg)
        self.get_logger().info("Sent Stop!")
        self.check_time()

        if hasattr(self, "grid"):
            self.grid_save()

    def command_start(self):
        msg = String()
        msg.data = "Start"
        self.robot_command_pub.publish(msg)
        self.start_time = self.get_clock().now()
        self.get_logger().info("Sent Start!")

        self.image_saved = False
    
    def command_take_direction(self, index):
        msg = String()
        msg.data = f'{next(iter(self.robots_positions))}|{self.index_to_world(index)}'
        self.robot_command_pub.publish(msg)
        self.get_logger().info(f"Sent direction to {next(iter(self.robots_positions))}")

    def index_to_world(self, index):
        if 0 <= index < self.grid.info.width * self.grid.info.height:

            x = (index % self.grid.info.width * self.grid.info.resolution) + self.grid.info.origin.position.x + (self.grid.info.resolution / 2.0)
            y = (index // self.grid.info.height * self.grid.info.resolution) + self.grid.info.origin.position.y + (self.grid.info.resolution / 2.0)
            return x, y
        else:
            return self.get_logger().error("index_to_world: Index of cell outside of bonds of the array")   

    def detector_callback(self, msg):
        self.robot_name, status = msg.data.split("|")

        if status == "Color detected!":
            self.command_stop()
            self.get_logger().info(self.robot_name + " " + status)

            self.get_logger().info(f'Object finded!')
            self.check_time()

            self.image_sub = self.create_subscription(
                Image,
                f'{self.robot_name}/camera/image_raw',
                self.save_image_once,
                1
            )

    def keyboard_input(self):
        while True:
            user_input = input().strip().split()
            
            command = user_input[0].lower()
            args = user_input[1:] 

            if command.lower() == "start":
                self.command_start()
                
            elif command.lower() == "stop":
                self.command_stop()

            elif command.lower() == "hop":
                if args:
                    self.command_take_direction(int(args[0]))
                else:
                    print("Error: 'hop' requires a value.")
    

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
        seconds = duration.nanoseconds / 1e9

        self.get_logger().info(f'Searching time: {seconds:.3f} s.')

    def grid_update_robot_position(self, msg):

        self.robot_name, robot_position_x, robot_position_y = msg.data.split("|")

        x, y = (float(robot_position_x), float(robot_position_y))

        resolution = self.grid.info.resolution
        origin_x = self.grid.info.origin.position.x
        origin_y = self.grid.info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        self.robots_positions[self.robot_name] = (grid_x, grid_y)

        if 0 <= grid_x < self.grid.info.width and 0 <= grid_y < self.grid.info.height:
            index = grid_y * self.grid.info.width + grid_x
            
            self.grid.data[index] = 1
            
            self.grid.header.stamp = self.get_clock().now().to_msg()
            self.grid_pub.publish(self.grid)      

    def grid_update_obstacles_positions(self, msg):

        ranges = np.array(msg.ranges)

        # x, y = (float(robot_position_x), float(robot_position_y))

        # resolution = self.grid.info.resolution
        # origin_x = self.grid.info.origin.position.x
        # origin_y = self.grid.info.origin.position.y

        # grid_x = int((x - origin_x) / resolution)
        # grid_y = int((y - origin_y) / resolution)

        # self.robots_positions[self.robot_name] = (grid_x, grid_y)

        # if 0 <= grid_x < self.grid.info.width and 0 <= grid_y < self.grid.info.height:
        #     index = grid_y * self.grid.info.width + grid_x
            
        #     self.grid.data[index] = 1
            
        #     self.grid.header.stamp = self.get_clock().now().to_msg()
        #     self.grid_pub.publish(self.grid)    

    def grid_create(self, width, height):
        if width > 0.0 and height > 0.0:
            resolution = 0.2

            self.grid = OccupancyGrid()
            self.grid.info = MapMetaData()
            self.grid.info.width = int(width / resolution)
            self.grid.info.height = int(height / resolution)
            self.grid.info.resolution = resolution
            self.grid.header.frame_id = "grid"

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'mapmap' # Parent
            t.child_frame_id = 'grid'    # Child 
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            self.tf_static_broadcaster.sendTransform(t)

            self.grid.info.origin = Pose()
            self.grid.info.origin.position.x = - width / 2.0
            self.grid.info.origin.position.y = - height / 2.0

            self.grid.data = [-1] * (self.grid.info.width * self.grid.info.height)

            self.get_logger().info("Grid created.")

    def grid_save(self):
        width = self.grid.info.width
        height = self.grid.info.height
        data = np.array(self.grid.data, dtype=np.int8).reshape((height, width))  # reshape first value number of rows, second number of columns

        display_grid = np.ones((height, width), dtype=np.float32)  # visited (white)
        display_grid[data == -1] = 0.5    # unknown (gray)
        for position in self.robots_positions.values():
            x, y = position
            display_grid[y, x] = 0.0   # robot (black)

        total_cells = width * height
        explored_cells = np.count_nonzero(data == 1)
        unknown_cells = np.count_nonzero(data == -1)

        self.get_logger().info(f"Total cells: {total_cells}")
        self.get_logger().info(f"Explored cells: {explored_cells}")
        self.get_logger().info(f"Unknown cells: {unknown_cells}")

        plt.figure(figsize=(10, 10)) 
        plt.imshow(display_grid, cmap='gray', origin='lower', vmin=0.0, vmax=1.0)
        plt.title("Occupancy Grid")
        plt.xlabel("X (cells)")
        plt.ylabel("Y (cells)")
        plt.grid(False)
        p = plt.gca()
        p.set_xticks(np.arange(0, width, 5))
        p.set_yticks(np.arange(0, height, 5))

        p.set_xticks(np.arange(-.5, width, 1), minor=True)
        p.set_yticks(np.arange(-.5, height, 1), minor=True)

        p.grid(which='minor', color='black', linestyle='-', linewidth=1)

        p.tick_params(which='minor', bottom=False, left=False)

        filename="occupancy_grid.png"
        plt.savefig(filename)
        plt.close()
        print(f"Occupancy grid saved as: {filename}")


def main(args=None):
    rclpy.init(args=args)
    head = Head() 
    rclpy.spin(head)
    head.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()