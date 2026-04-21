from pathlib import Path
import os
import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image
from rclpy.node import Node
from nav_msgs.msg import Odometry
import re
from collections import deque
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from std_msgs.msg import String
import json


class PointAllocator(Node):
    def __init__(self):
        super().__init__("point_allocator")

        self.declare_parameter("files_path", "~/maps")
        self.declare_parameter("robots_number", 3)

        self.files_path = os.path.expanduser(self.get_parameter("files_path").value)
        robots_number = int(self.get_parameter("robots_number").value)

        if robots_number < 1 or robots_number > 3:
            self.get_logger().warn(
                f"robots_number={robots_number} is invalid (should be integer number between 1 and 3), using 3"
            )
            robots_number = 3

        all_robot_namespaces = ["jetbot_0", "jetbot_1", "jetbot_2"]
        self.robot_namespaces = all_robot_namespaces[:robots_number]

        self.get_logger().info(f"Using robot namespaces: {self.robot_namespaces}")
        
        self.robot_world_positions = {}
        self.robot_image_positions = {}

        self.scale = 10

        self.timer_period = 5.0

        self.ready_timer = self.create_timer(self.timer_period, self.check_if_ready)

        self.points_pub = self.create_publisher(
            String,
            '/assigned_points',
            10
        )

        self.pose_subscriptions = []

        for ns in self.robot_namespaces:
            topic = f"{ns}/amcl_pose"
            sub = self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                lambda msg, robot_ns=ns: self.robot_pose_callback(robot_ns, msg),
                10,
            )
            self.pose_subscriptions.append(sub)
            self.get_logger().info(f"Subscribing robot amcl topic: {topic}")

        self.get_logger().info(f"Files path: {self.files_path}")

        self.map_info = self.load_map()

        self.get_logger().info(
            f"Map loaded: {self.map_info['width']}x{self.map_info['height']}, "
            f"resolution={self.map_info['resolution']}"
        )

        self.point_cells = (self.load_points_from_file(self.files_path + "/observation_points.txt"))

    def load_map(self):
        yaml_path = os.path.join(self.files_path, "jetbot_rescaled_map.yaml")
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        image_name = Path(meta["image"].lstrip("/"))
        image_path = os.path.join(self.files_path, image_name)

        self.image = Image.open(image_path).convert("L")
        gray = np.array(self.image, dtype=np.uint8)

        resolution = float(meta["resolution"])
        origin = meta["origin"]
        origin_x = float(origin[0])
        origin_y = float(origin[1])

        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.25))

        occ = (255.0 - gray.astype(np.float32)) / 255.0

        free_mask = occ < free_thresh

        return {
            "gray": gray,
            "free_mask": free_mask,
            "resolution": resolution,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "width": gray.shape[1],
            "height": gray.shape[0]
        }

    def load_points_from_file(self, points_txt_path: str):
        point_pattern = re.compile(r"point=\((\d+),\s*(\d+)\)")
        points = []

        with open(points_txt_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                match = point_pattern.search(line)
                if match:
                    x = int(match.group(1))
                    y = int(match.group(2))
                    points.append((x, y))

        return points    

    def robot_pose_callback(self, robot_ns: str, msg: PoseWithCovarianceStamped):
        if robot_ns in self.robot_world_positions:
            return

        wx = msg.pose.pose.position.x
        wy = msg.pose.pose.position.y

        self.robot_world_positions[robot_ns] = (wx, wy)

        image_cell = self.world_to_image_cell(wx, wy)
        self.robot_image_positions[robot_ns] = image_cell

        self.get_logger().info(
            f"{robot_ns}: world=({wx:.2f}, {wy:.2f}) -> image={image_cell}"
        )

    def world_to_image_cell(self, wx: float, wy: float):
        resolution = self.map_info["resolution"]
        origin_x = self.map_info["origin_x"]
        origin_y = self.map_info["origin_y"]
        height = self.map_info["height"]

        map_x = int((wx - origin_x) / resolution)
        map_y = int((wy - origin_y) / resolution)

        image_y = height - 1 - map_y
        return (map_x, image_y)

    def in_bounds(self, x: int, y: int):
        return 0 <= x < self.map_info["width"] and 0 <= y < self.map_info["height"]

    def is_free(self, x: int, y: int):
        return self.in_bounds(x, y) and bool(self.map_info["free_mask"][y, x])

    def check_if_ready(self):
        if len(self.robot_image_positions) != len(self.robot_namespaces):
            missing = [ns for ns in self.robot_namespaces if ns not in self.robot_image_positions]
            self.get_logger().info(f"Waiting for robot poses: {missing}")
            return

        self.get_logger().info("All robot positions received")
        self.get_logger().info(f"Robot image positions: {self.robot_image_positions}")

        self.get_logger().info("All required data received. Building cost matrix...")
        data = self.build_ortools_data_model()
        
        matrix_range = 10
        self.get_logger().info(f"Showing {matrix_range} first rows and colums")
        for i, row in enumerate(data["distance_matrix"][:matrix_range]):
            self.get_logger().info(f"row {i}: {row[:matrix_range]}")
        self.get_logger().info(".....")

        result = self.solve_paths_with_ortools_balanced_cost(data)

        for route in result["routes"]:
            self.get_logger().info(
                f"Robot {route['vehicle_id']} | cost={route['cost']} | nodes={route['nodes']}"
            )

        self.get_logger().info(f"Total cost: {result['total_cost']}")

        self.get_logger().info("Saving map with assigmented points")
        self.save_assignment_image(result)

        self.ready_timer.cancel()

        world_paths = self.convert_routes_to_world_paths(result)
        self.get_logger().info("Converted points to world coordinates:")

        for route in world_paths:
            self.get_logger().info(f"Robot {route['vehicle_id']}:")
            for i, (wx, wy) in enumerate(route["world_points"]):
                self.get_logger().info(f"  point {i}: x={wx:.3f}, y={wy:.3f}")
        
        self.ready_timer = self.create_timer(self.timer_period, lambda: self.publish_world_paths(world_paths))

    def bfs_distances_from_source(self, source, targets=None):

        if not self.is_free(source[0], source[1]):
            return {}

        queue = deque([source])
        visited = {source}
        distances = {source: 0}

        targets_left = None
        if targets is not None:
            targets_left = set(targets)
            if source in targets_left:
                targets_left.remove(source)

        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while queue:
            x, y = queue.popleft()

            if targets_left is not None and not targets_left:
                break

            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                next_cell = (nx, ny)

                if next_cell in visited:
                    continue

                if not self.is_free(nx, ny):
                    continue

                visited.add(next_cell)
                distances[next_cell] = distances[(x, y)] + 1
                queue.append(next_cell)

                if targets_left is not None and next_cell in targets_left:
                    targets_left.remove(next_cell)

        return distances

    def build_cost_matrix(self):

        robot_cells = [self.robot_image_positions[ns] for ns in self.robot_namespaces]
        point_cells = self.point_cells

        real_nodes = robot_cells + point_cells
        real_count = len(real_nodes)
        num_robots = len(robot_cells)

        distance_matrix = [[None for _ in range(real_count)] for _ in range(real_count)]
        unreachable_cost = 10**9

        for i in range(real_count):
            distance_matrix[i][i] = 0

        for i, src in enumerate(real_nodes):
            targets = set(real_nodes[j] for j in range(i + 1, real_count))
            if not targets:
                continue

            dist_map = self.bfs_distances_from_source(src, targets=targets)

            for j in range(i + 1, real_count):
                dst = real_nodes[j]

                path_len_cells = dist_map.get(dst)
                if path_len_cells is None:
                    value = None
                else:
                    path_len_m = path_len_cells * self.map_info["resolution"]
                    value = int(round(path_len_m * self.scale))
                    

                distance_matrix[i][j] = value
                distance_matrix[j][i] = value

        for i in range(num_robots):
            for j in range(i + 1, num_robots):
                distance_matrix[i][j] = None
                distance_matrix[j][i] = None

        for i in range(real_count):
            for j in range(real_count):
                if distance_matrix[i][j] is None:
                    distance_matrix[i][j] = unreachable_cost

        return distance_matrix, robot_cells, point_cells

    def build_ortools_data_model(self, return_to_start=False):
        distance_matrix, robot_cells, point_cells = self.build_cost_matrix()

        num_robots = len(robot_cells)
        starts = list(range(num_robots))
        ends = list(range(num_robots))

        locations = robot_cells + point_cells

        if not return_to_start:
            first_point_node = num_robots
            last_point_node = num_robots + len(point_cells) - 1

            for vehicle_id in range(num_robots):
                end_node = ends[vehicle_id]

                for point_node in range(first_point_node, last_point_node + 1):
                    distance_matrix[point_node][end_node] = 0

        return {
            "distance_matrix": distance_matrix,
            "locations": locations,
            "robot_cells": robot_cells,
            "point_cells": point_cells,
            "starts": starts,
            "ends": ends,
            "num_vehicles": num_robots,
        }

    def solve_paths_with_ortools_balanced_cost(self, data):

        num_vehicles = data["num_vehicles"]
        num_robots = len(data["robot_cells"])
        num_points = len(data["point_cells"])

        first_point_node = num_robots
        last_point_node = num_robots + num_points - 1

        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]),
            num_vehicles,
            data["starts"],
            data["ends"]
        )

        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["distance_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


        for node in range(first_point_node, last_point_node + 1):
            routing.AddDisjunction([manager.NodeToIndex(node)], 10**9)


        routing.AddDimension(
            transit_callback_index,
            0,          
            10**9,    
            True,       
            "Distance"
        )

        distance_dimension = routing.GetDimensionOrDie("Distance")

        distance_dimension.SetGlobalSpanCostCoefficient(100)

        approx_total = 0
        count = 0
        matrix = data["distance_matrix"]

        for i in range(num_robots):
            for j in range(first_point_node, last_point_node + 1):
                if matrix[i][j] < 10**9:
                    approx_total += matrix[i][j]
                    count += 1

        if count > 0:
            avg_start_cost = approx_total // count
            soft_limit = avg_start_cost * max(1, num_points // num_vehicles + 1) * 2

            for vehicle_id in range(num_vehicles):
                end_index = routing.End(vehicle_id)
                distance_dimension.SetCumulVarSoftUpperBound(
                    end_index,
                    soft_limit,
                    100
                )

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
        search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        search_parameters.time_limit.seconds = 15

        solution = routing.SolveWithParameters(search_parameters)

        if solution is None:
            self.get_logger().error("OR-Tools did not find a solution")
            return None

        routes = []
        total_cost = 0

        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)

            route_nodes = []
            route_locations = []
            route_point_nodes = []
            route_point_indices = []
            route_cost = 0

            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route_nodes.append(node)
                route_locations.append(data["locations"][node])

                if first_point_node <= node <= last_point_node:
                    route_point_nodes.append(node)
                    route_point_indices.append(node - first_point_node)

                next_index = solution.Value(routing.NextVar(index))
                next_node = manager.IndexToNode(next_index)

                route_cost += data["distance_matrix"][node][next_node]
                index = next_index

            end_node = manager.IndexToNode(index)
            route_nodes.append(end_node)
            route_locations.append(data["locations"][end_node])

            end_cost = solution.Value(distance_dimension.CumulVar(routing.End(vehicle_id)))

            routes.append({
                "vehicle_id": vehicle_id,
                "nodes": route_nodes,
                "locations": route_locations,
                "point_nodes": route_point_nodes,
                "point_indices": route_point_indices,
                "cost": route_cost,
                "distance_dimension_cost": end_cost
            })

            total_cost += route_cost

        return {
            "routes": routes,
            "total_cost": total_cost,
            "data_model": data
        }

    def save_assignment_image(self, result):
        img = np.array(self.image.convert("RGB"))

        img[(img == [255, 255, 255]).all(axis=2)] = [180, 180, 180]

        img_areas = img.copy()

        img_areas_and_points = img.copy()

        img_areas_and_points_without_robots = img.copy()

        img_red = img.copy()

        robot_cells = [self.robot_image_positions[ns] for ns in self.robot_namespaces]
        point_cells = self.point_cells

        real_nodes = robot_cells + point_cells

        robot_num = len(robot_cells)

        robot_colors = [
            "green", 
            "blue",    
            "yellow"  
        ]
        robot_colors_rgb = [
            (0, 255, 0), 
            (0, 0, 255),    
            (255, 255, 0)   
        ]

        dark_colors_rgb = [
            (0, 120, 0), 
            (0, 0, 120),    
            (120, 120, 0)   
        ]

        visibility_map = self.load_visibility_map()

        for route in result["routes"]:
            robot_id = route['vehicle_id']
            self.get_logger().info(f'Robot {robot_id}: {robot_colors[robot_id]}')
            for point in route['nodes']:
                x, y = real_nodes[point]

                for cell_x, cell_y in visibility_map.get((x, y), []):
                    img_areas[cell_y, cell_x] = dark_colors_rgb[robot_id]
                    img_areas_and_points[cell_y, cell_x] = dark_colors_rgb[robot_id]
                    img_areas_and_points_without_robots[cell_y, cell_x] = dark_colors_rgb[robot_id]

                img[y, x] = robot_colors_rgb[robot_id]
                img_areas_and_points[y, x] = robot_colors_rgb[robot_id]
                img_red[y, x] = robot_colors_rgb[robot_id]
                if point >= robot_num: 
                    img_areas_and_points_without_robots[y, x] = robot_colors_rgb[robot_id]
                    img_red[y, x] = (255, 0, 0)

        for robot_id, (x, y) in enumerate(robot_cells):
            img[y, x] = robot_colors_rgb[robot_id]
            img_areas_and_points[y, x] = robot_colors_rgb[robot_id]
            img_red[y, x] = robot_colors_rgb[robot_id]       

        Image.fromarray(img).save(os.path.join(self.files_path, "jetbot_observation_points_colored.png"))
        Image.fromarray(img_red).save(os.path.join(self.files_path, "jetbot_observation_points_colored_only_robots.png"))
        Image.fromarray(img_areas).save(os.path.join(self.files_path, "jetbot_observation_areas_colored.png"))
        Image.fromarray(img_areas_and_points).save(os.path.join(self.files_path, "jetbot_observation_areas_and_points_colored.png"))
        Image.fromarray(img_areas_and_points_without_robots).save(os.path.join(self.files_path, "jetbot_observation_areas_and_points_without_robots_colored.png"))

    def load_visibility_map(self):
        visibility_path = os.path.join(self.files_path, "visibility.txt")
        visibility_map = {}

        with open(visibility_path, "r", encoding="utf-8") as f:
            for point, line in zip(self.point_cells, f):
                visible = line.split("visible=")[1].strip()
                cells = re.findall(r'\((\d+),\s*(\d+)\)', visible)
                visibility_map[point] = [(int(x), int(y)) for x, y in cells]

        return visibility_map

    def convert_routes_to_world_paths(self, result):
        world_paths = []

        for route in result["routes"]:
            vehicle_id = route["vehicle_id"]
            world_points = []

            for point_idx in route["point_indices"]:
                cell_x, cell_y = self.point_cells[point_idx]
                wx, wy = self.image_cell_to_world(cell_x, cell_y)
                world_points.append((wx, wy))

            world_paths.append({
                "vehicle_id": vehicle_id,
                "world_points": world_points
            })

        return world_paths

    def image_cell_to_world(self, image_x: int, image_y: int):
        resolution = self.map_info["resolution"]
        origin_x = self.map_info["origin_x"]
        origin_y = self.map_info["origin_y"]
        height = self.map_info["height"]

        map_y = height - 1 - image_y

        wx = origin_x + (image_x + 0.5) * resolution
        wy = origin_y + (map_y + 0.5) * resolution

        return (wx, wy)

    def publish_world_paths(self, world_paths):
        msg = String()

        payload = {
            "routes": []
        }

        for route in world_paths:
            vehicle_id = route["vehicle_id"]

            robot_namespace = ""
            if vehicle_id < len(self.robot_namespaces):
                robot_namespace = self.robot_namespaces[vehicle_id]

            payload["routes"].append({
                "vehicle_id": vehicle_id,
                "robot_namespace": robot_namespace,
                "world_points": [
                    {"x": float(x), "y": float(y)}
                    for x, y in route["world_points"]
                ]
            })

        msg.data = json.dumps(payload)
        self.points_pub.publish(msg)

        self.get_logger().info(f"Published routes for {len(payload['routes'])} robots on /assigned_points")

def main(args=None):
    rclpy.init(args=args)
    node = PointAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()