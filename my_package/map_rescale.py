#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header


class RescaleMap(Node):
    def __init__(self):
        super().__init__('rescale_map')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        try:
            coarse_map = self.rescale_grid_data(
                src=msg,
                target_resolution=self.target_resolution,
                occupied_threshold=self.occupied_threshold,
            )
            self.map_pub.publish(coarse_map)
        except Exception as e:
            self.get_logger().error(f'Failed to rescale map: {e}')

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

        old_data = old_map.data,

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


def main(args=None):
    rclpy.init(args=args)
    node = RescaleMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()