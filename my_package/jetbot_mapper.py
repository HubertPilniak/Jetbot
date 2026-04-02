import os
from datetime import datetime

import math
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from slam_toolbox.srv import SaveMap
from nav_msgs.msg import OccupancyGrid, MapMetaData


class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')

        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )

        self.save_map_client = self.create_client(
            SaveMap,
            f'{self.get_namespace()}/slam_toolbox/save_map'
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            f'{self.get_namespace()}/map',
            self.map_callback,
            1
        )

        self.declare_parameter('maps_dir', '~/maps')
        self.declare_parameter('image_format', 'pgm')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('free_thresh', 0.25)
        self.declare_parameter('occupied_thresh', 0.65)

        self.maps_dir = os.path.expanduser(self.get_parameter('maps_dir').value)
        self.image_format = self.get_parameter('image_format').value
        self.map_mode = self.get_parameter('map_mode').value
        self.free_thresh = float(self.get_parameter('free_thresh').value)
        self.occupied_thresh = float(self.get_parameter('occupied_thresh').value)

        self.map_saved = False

        self.get_logger().info(f"maps_dir = {self.maps_dir}")
        os.makedirs(self.maps_dir, exist_ok=True)

        self.get_logger().info('Mapper started')

    def save_map(self):
        if not self.save_map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service slam_toolbox/save_map is not available')
            return

        request = SaveMap.Request()
        request.name.data = f'{self.maps_dir}{self.get_namespace()}_map'

        self.get_logger().info(f'Saving map to: {request.name.data}')

        future = self.save_map_client.call_async(request)

        self.map_saved = True

    def command_callback(self, msg):
        command = msg.data.strip()

        if command == 'Start':
            self.get_logger().info('Mapping started')

        elif command == 'SaveMap':
            self.get_logger().info('SaveMap command received')
            self.save_map()

        elif command == 'Stop':
            self.get_logger().info('Mapping stopped, saving map...')
            self.save_map()

    def map_callback(self, msg):
        if self.map_saved == False:
            return
        if msg == None:
            self.get_logger().info('Map not recieved')
            return
        self.get_logger().info("map_calback(rescale)")
        new_map = self.rescale_grid_data(msg)

        map_name = f'{self.get_namespace()}_rescaled_map'

        self.write_pgm(new_map, f'{self.maps_dir}{map_name}.pgm')
        self.write_yaml(new_map, f'{self.maps_dir}{map_name}.yaml', map_name)

        self.map_saved = False

    def rescale_grid_data(
        self,
        old_map: OccupancyGrid,
        old_resolution: float = 0.05,
        new_resolution: float = 0.2,
        occupied_threshold: int = 50):
        
        scale = new_resolution / old_resolution
        scale_round = round(scale)

        old_width = old_map.info.width
        old_height = old_map.info.height

        block = int(scale_round)
        new_width = math.ceil(old_width / block)
        new_height = math.ceil(old_height / block)

        old_data = list(old_map.data)

        new_map_data = [0] * (new_width * new_height)

        for ny in range(new_height):
            for nx in range(new_width):
                x0 = nx * block
                y0 = ny * block
                x1 = min(x0 + block, old_width)
                y1 = min(y0 + block, old_height)

                has_occupied = False

                for y in range(y0, y1):
                    for x in range(x0, x1):
                        v = old_data[y * old_width + x]

                        if v >= occupied_threshold or v == -1:
                            has_occupied = True
                            break
                    if has_occupied:
                        break

                if has_occupied:
                    out_val = 100
                else:
                    out_val = 0

                new_map_data[ny * new_width + nx] = out_val

        new_map = OccupancyGrid()

        new_map.info = MapMetaData()
        new_map.info.resolution = new_resolution
        new_map.info.width = new_width
        new_map.info.height = new_height
        new_map.info.origin = old_map.info.origin

        new_map.data = new_map_data
        return new_map

    def write_pgm(self, grid, pgm_path):
        width = int(grid.info.width)
        height = int(grid.info.height)
        data = list(grid.data)

        pixels = bytearray()

        for y in range(height):
            for x in range(width):
                v = data[y * width + x]

                if v >= 50 or v == -1:
                    pixels.append(0)
                else:
                    pixels.append(254)

        with open(pgm_path, 'wb') as f:
            header = f'P5\n# CREATOR: coarse_map_publisher\n{width} {height}\n255\n'
            f.write(header.encode('ascii'))
            f.write(pixels)
        self.get_logger().info("write_pgm")

    def write_yaml(self, grid, yaml_path, image_filename):
        origin = grid.info.origin.position

        yaw = 0.0

        content = (
            f'image: {image_filename}.pgm\n'
            f'mode: trinary\n'
            f'resolution: {grid.info.resolution}\n'
            f'origin: [{origin.x}, {origin.y}, {yaw}]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )

        with open(yaml_path, 'w') as f:
            f.write(content)
        self.get_logger().info("write_yaml")


def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()