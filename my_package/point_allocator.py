from collections import deque
from pathlib import Path
import re
import yaml

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image
from rclpy.node import Node


class PointAllocator(Node):
    def __init__(self):
        super().__init__("point_allocator")

        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("points_txt_path", "")
        self.declare_parameter("robot_namespaces", [])
        self.declare_parameter("pose_topic_suffix", "/amcl_pose")
        self.declare_parameter("free_threshold_override", -1.0)
        self.declare_parameter("scale", 100)
        self.declare_parameter("return_to_start", False)
        self.declare_parameter("build_once_when_ready", True)

        self.map_yaml_path = self.get_parameter("map_yaml_path").value
        self.points_txt_path = self.get_parameter("points_txt_path").value
        self.robot_namespaces = list(self.get_parameter("robot_namespaces").value)
        self.pose_topic_suffix = self.get_parameter("pose_topic_suffix").value
        self.free_threshold_override = float(self.get_parameter("free_threshold_override").value)
        self.scale = int(self.get_parameter("scale").value)
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.build_once_when_ready = bool(self.get_parameter("build_once_when_ready").value)

        if not self.map_yaml_path:
            raise ValueError("Parametr 'map_yaml_path' jest wymagany")
        if not self.points_txt_path:
            raise ValueError("Parametr 'points_txt_path' jest wymagany")
        if not self.robot_namespaces:
            raise ValueError("Parametr 'robot_namespaces' nie może być pusty")

        self.map_info = self._load_map(self.map_yaml_path)
        self.points_image = self._load_points_from_file(self.points_txt_path)
        self.points_image = [self._snap_to_nearest_free(p) for p in self.points_image]

        self.robot_world_positions = {}
        self.robot_image_positions = {}
        self.subscriptions = []
        self.data_model = None
        self.built = False

        for ns in self.robot_namespaces:
            topic = f"{ns.rstrip('/')}{self.pose_topic_suffix}"
            sub = self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                lambda msg, robot_ns=ns: self._robot_pose_callback(robot_ns, msg),
                10,
            )
            self.subscriptions.append(sub)
            self.get_logger().info(f"Subskrybuję pozycję robota z topicu: {topic}")

        self.timer = self.create_timer(1.0, self._try_build_once)

        self.get_logger().info(
            f"Wczytano mapę: {self.map_info['width']}x{self.map_info['height']}, "
            f"resolution={self.map_info['resolution']}"
        )
        self.get_logger().info(f"Wczytano {len(self.points_image)} punktów z pliku")

    def _load_map(self, map_yaml_path: str):
        map_yaml_path = Path(map_yaml_path).resolve()
        with open(map_yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        image_path = Path(meta["image"])
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path

        image = Image.open(image_path).convert("L")
        gray = np.array(image, dtype=np.uint8)

        resolution = float(meta["resolution"])
        origin = meta["origin"]
        origin_x = float(origin[0])
        origin_y = float(origin[1])

        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))

        # Przeliczenie jak w map_server: piksel -> occupancy probability
        if negate == 0:
            occ = (255.0 - gray.astype(np.float32)) / 255.0
        else:
            occ = gray.astype(np.float32) / 255.0

        if self.free_threshold_override >= 0.0:
            free_mask = occ < self.free_threshold_override
        else:
            free_mask = occ < free_thresh

        return {
            "gray": gray,
            "free_mask": free_mask,
            "resolution": resolution,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "width": gray.shape[1],
            "height": gray.shape[0],
            "occupied_thresh": occupied_thresh,
            "free_thresh": free_thresh,
        }

    def _load_points_from_file(self, points_txt_path: str):
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

    def _robot_pose_callback(self, robot_ns: str, msg: PoseWithCovarianceStamped):
        wx = msg.pose.pose.position.x
        wy = msg.pose.pose.position.y

        self.robot_world_positions[robot_ns] = (wx, wy)

        image_cell = self._world_to_image_cell(wx, wy)
        snapped = self._snap_to_nearest_free(image_cell)

        self.robot_image_positions[robot_ns] = snapped

    def _world_to_image_cell(self, wx: float, wy: float):
        resolution = self.map_info["resolution"]
        origin_x = self.map_info["origin_x"]
        origin_y = self.map_info["origin_y"]
        height = self.map_info["height"]

        map_x = int((wx - origin_x) / resolution)
        map_y = int((wy - origin_y) / resolution)

        # zamiana z układu mapy ROS (0,0 na dole) na układ obrazu (0,0 u góry)
        image_y = height - 1 - map_y
        return (map_x, image_y)

    def _in_bounds(self, x: int, y: int):
        return 0 <= x < self.map_info["width"] and 0 <= y < self.map_info["height"]

    def _is_free(self, x: int, y: int):
        return self._in_bounds(x, y) and bool(self.map_info["free_mask"][y, x])

    def _snap_to_nearest_free(self, cell, max_radius: int = 20):
        cx, cy = cell

        if self._is_free(cx, cy):
            return (cx, cy)

        visited = set()
        q = deque()
        q.append((cx, cy, 0))
        visited.add((cx, cy))

        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while q:
            x, y, dist = q.popleft()

            if dist > max_radius:
                break

            if self._is_free(x, y):
                return (x, y)

            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                if (nx, ny) in visited:
                    continue
                if not self._in_bounds(nx, ny):
                    continue
                visited.add((nx, ny))
                q.append((nx, ny, dist + 1))

        raise ValueError(f"Nie udało się znaleźć wolnej komórki w pobliżu {cell}")

    def _bfs_distances_from_source(self, source, targets=None):
        """
        BFS na gridzie 4-kierunkowym.
        Zwraca słownik: {(x, y): dystans_w_komórkach}
        """
        if not self._is_free(source[0], source[1]):
            return {}

        q = deque([source])
        visited = {source}
        dist = {source: 0}

        targets_left = None
        if targets is not None:
            targets_left = set(targets)
            if source in targets_left:
                targets_left.remove(source)

        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while q:
            x, y = q.popleft()

            if targets_left is not None and not targets_left:
                break

            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                nxt = (nx, ny)

                if nxt in visited:
                    continue
                if not self._is_free(nx, ny):
                    continue

                visited.add(nxt)
                dist[nxt] = dist[(x, y)] + 1
                q.append(nxt)

                if targets_left is not None and nxt in targets_left:
                    targets_left.remove(nxt)

        return dist

    def build_cost_matrix(self):
        """
        Tworzy pełną macierz kosztów:
        - najpierw roboty
        - potem punkty obserwacyjne

        Koszt liczony BFS-em po wolnych komórkach mapy.
        Wynik końcowy w integerach, przeskalowany do OR-Tools.
        """
        if len(self.robot_image_positions) != len(self.robot_namespaces):
            raise RuntimeError("Nie ma jeszcze pozycji wszystkich robotów")

        robot_cells = [self.robot_image_positions[ns] for ns in self.robot_namespaces]
        point_cells = list(self.points_image)

        real_nodes = robot_cells + point_cells
        real_count = len(real_nodes)

        distance_matrix = [[0 for _ in range(real_count)] for _ in range(real_count)]
        unreachable_cost = 10**9

        target_set = set(real_nodes)

        for i, src in enumerate(real_nodes):
            dist_map = self._bfs_distances_from_source(src, targets=target_set)

            for j, dst in enumerate(real_nodes):
                if i == j:
                    distance_matrix[i][j] = 0
                    continue

                path_len_cells = dist_map.get(dst)
                if path_len_cells is None:
                    distance_matrix[i][j] = unreachable_cost
                else:
                    path_len_m = path_len_cells * self.map_info["resolution"]
                    distance_matrix[i][j] = int(round(path_len_m * self.scale))

        return distance_matrix, robot_cells, point_cells

    def build_ortools_data_model(self):
        distance_matrix, robot_cells, point_cells = self.build_cost_matrix()

        num_robots = len(robot_cells)
        starts = list(range(num_robots))

        if self.return_to_start:
            ends = list(range(num_robots))
            locations = robot_cells + point_cells
            return {
                "distance_matrix": distance_matrix,
                "locations": locations,
                "robot_cells": robot_cells,
                "point_cells": point_cells,
                "starts": starts,
                "ends": ends,
                "num_vehicles": num_robots,
            }

        # Trasy otwarte: dodajemy dummy end nodes
        real_count = len(distance_matrix)
        total_count = real_count + num_robots
        unreachable_cost = 10**9

        extended = [[unreachable_cost for _ in range(total_count)] for _ in range(total_count)]

        for i in range(real_count):
            for j in range(real_count):
                extended[i][j] = distance_matrix[i][j]

        dummy_start = real_count

        # z każdego prawdziwego węzła można skończyć trasę w dowolnym dummy end z kosztem 0
        for i in range(real_count):
            for r in range(num_robots):
                end_idx = dummy_start + r
                extended[i][end_idx] = 0

        # dummy end nigdzie dalej nie prowadzą
        for r in range(num_robots):
            end_idx = dummy_start + r
            extended[end_idx][end_idx] = 0

        ends = [dummy_start + r for r in range(num_robots)]
        locations = robot_cells + point_cells + [None] * num_robots

        return {
            "distance_matrix": extended,
            "locations": locations,
            "robot_cells": robot_cells,
            "point_cells": point_cells,
            "starts": starts,
            "ends": ends,
            "num_vehicles": num_robots,
        }

    def _try_build_once(self):
        if self.built:
            return
        if not self.build_once_when_ready:
            return
        if len(self.robot_image_positions) != len(self.robot_namespaces):
            missing = [ns for ns in self.robot_namespaces if ns not in self.robot_image_positions]
            self.get_logger().info(f"Czekam na pozycje robotów: {missing}")
            return

        self.data_model = self.build_ortools_data_model()
        self.built = True

        self.get_logger().info("Macierz kosztów została zbudowana.")
        self.get_logger().info(
            f"Liczba robotów: {self.data_model['num_vehicles']}, "
            f"liczba punktów: {len(self.data_model['point_cells'])}, "
            f"rozmiar macierzy: {len(self.data_model['distance_matrix'])} x {len(self.data_model['distance_matrix'][0])}"
        )

        for i, row in enumerate(self.data_model["distance_matrix"][:5]):
            self.get_logger().info(f"row {i}: {row[:10]}")


def main(args=None):
    rclpy.init(args=args)
    node = PointAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()