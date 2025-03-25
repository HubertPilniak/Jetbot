import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import threading

class Head(Node):
    def __init__(self):
        super().__init__('head')
        self.subscription = self.create_subscription(
            String,
            '/detect',
            self.detector_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/signal',
            10
        )

        self.keyboard_thread = threading.Thread(target=self.keyboard_input, daemon=True)
        self.keyboard_thread.start()


    def detector_callback(self, msg):
        if "Color detected!" in msg.data:
            self.get_logger().info(msg.data)
            stop_msg = String()
            stop_msg.data = "Stop"
            self.publisher.publish(stop_msg)
            self.get_logger().info("Sent Stop!")

            duration = self.get_clock().now() - self.start_time
            seconds = duration.nanoseconds / 1e9
            self.get_logger().info(f'Object finded! Searching time: {seconds:.3f} s.')

    def keyboard_input(self):
        while True:
            command = input().strip()
            if command.lower() == "start":
                msg = String()
                msg.data = "Start"
                self.publisher.publish(msg)
                self.start_time = self.get_clock().now()
                self.get_logger().info("Sent Start!")
                self.publisher.publish(msg)
            if command.lower() == "stop":
                msg = String()
                msg.data = "Stop"
                self.publisher.publish(msg)
                self.start_time = 0
                self.get_logger().info("Sent Stop!")
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    head = Head() 
    rclpy.spin(head)
    head.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
