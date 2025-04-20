import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ColorDetect(Node):
    def __init__(self):
        super().__init__('color_detect')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            f'{self.get_namespace()}/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/detect',
            10
        )
        self.head_sub = self.create_subscription(
            String, 
            '/robot_command', 
            self.head_signal, 
            10
        )

        self.running = False

    def image_callback(self, msg):
        if not self.running:  
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 200, 70])
        upper_red = np.array([5, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        if np.count_nonzero(mask) > 76800:  # 76800 is 25% of 307200 which is number of pixels in 640x480 camera image
            self.send_message()
        
    def send_message(self):
        msg = String()
        msg.data = f"{self.get_namespace()}|Color detected!"
        self.publisher.publish(msg)

    def head_signal(self, msg):
        if msg.data == "Start":
            self.get_logger().info("Start!")
            self.running = True

        elif msg.data == "Stop":
            self.get_logger().info("Stop!")
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetect()  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
