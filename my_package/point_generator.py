import math
import cv2
import numpy as np
import argparse
from pathlib import Path

class PointGenerator:

    def __init__(
        self,
        pgm_path: str,
        observation_range_cells: int,
        free_threshold: int = 250,
    ):
        self.pgm_path = str(pgm_path + "/jetbot_rescaled_map.pgm")
        self.observation_range_cells = int(observation_range_cells)
        self.free_threshold = int(free_threshold)

        self.gray = self.load_pgm(self.pgm_path)
        self.height, self.width = self.gray.shape

        # Obstacle map:
        # True  -> occupied cell
        # False -> free cell
        self.static_obstacle_mask = self.gray < self.free_threshold

        # Work grid:
        # 0   -> free not covvered
        # 100 -> obstacle or already covered
        self.working_grid = np.where(self.static_obstacle_mask, 100, 0).astype(np.uint8)

        self.observation_points = []

        self.visible_offsets = self.visible_offsets()
        self.dda_rays = self.dda_rays(self.visible_offsets)

    def visible_offsets(self):
        d = self.observation_range_cells
        d2 = d * d

        offsets = []

        for dy in range(-d, d + 1):
            for dx in range(-d, d + 1):
                if dx * dx + dy * dy <= d2:
                    offsets.append((dx, dy))

        return offsets

    def dda_rays(self, visible_offsets):
        ray_lookup = {}

        for dx, dy in visible_offsets:
            steps = max(abs(dx), abs(dy))

            if steps == 0:
                ray_lookup[(dx, dy)] = [(0, 0)]
                continue

            x_inc = dx / steps
            y_inc = dy / steps

            x = 0.0
            y = 0.0

            ray_cells = []
            seen = set()

            for _ in range(steps + 1):
                xi = int(round(x))
                yi = int(round(y))

                if (xi, yi) not in seen:
                    ray_cells.append((xi, yi))
                    seen.add((xi, yi))

                x += x_inc
                y += y_inc

            ray_lookup[(dx, dy)] = ray_cells

        return ray_lookup

    def load_pgm(self, path: str) -> np.ndarray:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Unable to load PGM file: {path}")
        return img

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free_in_working_grid(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and self.working_grid[y, x] == 0
    
    def is_free_in_obstacle_mask(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and self.static_obstacle_mask[y, x] == False

    def is_static_obstacle(self, x: int, y: int) -> bool:
        return (not self.in_bounds(x, y)) or self.static_obstacle_mask[y, x]

    def find_first_free_cell(self):
        for y in range(self.height):
            for x in range(self.width):
                if self.working_grid[y, x] == 0:
                    return x, y
        return None

    def last_free_on_path(self, cells):
        last_free = None
        for x, y in cells:
            if not self.is_free_in_working_grid(x, y):
                break
            last_free = (x, y)
        return last_free
    
    def last_free_on_diagonal_path(self, cells):
        last_free = None
        left_obstacle = False
        top_obstacle = False
        for x, y in cells:
            if not self.is_free_in_working_grid(x, y):    
                break
            if not self.is_free_in_obstacle_mask(x-1, y):
                if top_obstacle:
                    break
                left_obstacle = True
            if not self.is_free_in_obstacle_mask(x, y-1):
                if left_obstacle:
                    break
                top_obstacle = True

            last_free = (x, y)
        return last_free

    def last_visible_on_diagonal_path(self, cells, sign_x, sign_y):
        last_free = None
        left_obstacle = False
        top_obstacle = False
        for x, y in cells:
            if not self.is_free_in_obstacle_mask(x, y):    
                break
            if not self.is_free_in_obstacle_mask(x-sign_x, y):
                if top_obstacle:
                    break
                left_obstacle = True
            if not self.is_free_in_obstacle_mask(x, y-sign_y):
                if left_obstacle:
                    break
                top_obstacle = True

            last_free = (x, y)
        return last_free

    def generate_candidates(self, x: int, y: int):
        d = self.observation_range_cells

        candidates = []

        # 1) diagonal
        diag_step = int(d / math.sqrt(2))
        diagonal_cells = [(x + k, y + k) for k in range(1, 1 + diag_step)]
        cand_diagonal = self.last_free_on_diagonal_path(diagonal_cells)
        if cand_diagonal is not None:
            candidates.append(cand_diagonal)

        # 2) horizontal
        horizontal_cells = [(xx, y) for xx in range(x + 1, x + 1 + d)]
        cand_horizontal = self.last_free_on_path(horizontal_cells)
        if cand_horizontal is not None:
            candidates.append(cand_horizontal)

        # 3) vertical
        vertical_cells = [(x, yy) for yy in range(y + 1, y + 1 + d)]
        cand_vertical = self.last_free_on_path(vertical_cells)
        if cand_vertical is not None:
            candidates.append(cand_vertical)

        # if candidates empty use starting cell
        if not candidates:
            candidates = [(x, y)]

        return candidates

    def plan(self):
        step = 0

        while True:
            seed = self.find_first_free_cell()
            if seed is None:
                break

            x, y = seed
            candidates = self.generate_candidates(x, y)

            best_candidate = None
            best_visible = []

            for cx, cy in candidates:
                visible = self.get_markable_cells_from_point(cx, cy, self.visible_offsets, self.dda_rays)
                if len(visible) > len(best_visible):
                    best_visible = visible
                    best_candidate = (cx, cy)

            for vx, vy in best_visible:
                self.working_grid[vy, vx] = 100

            self.observation_points.append(
                {
                    "step": step,
                    "seed_cell": (x, y),
                    "observation_point": best_candidate,
                    "visible_count": len(best_visible),
                    "visible_cells": best_visible,
                }
            )

            step += 1

        self.validate_visibility_count()

        return self.observation_points

    def sign_of(self, x):
        if x > 0:
            return 1
        if x < 0:
            return -1

    def get_vision_on_diagonals(self, cx, cy):
        diag_step = int(self.observation_range_cells / math.sqrt(2))
        diagonal_ends = [(-1, -1), (1, -1), (-1, 1), (1, 1)]
        markable_cells_diagonal = []
        for sign_x, sign_y in diagonal_ends:
            diagonal_cells = [(cx + (sign_x * k), cy + (sign_y * k)) for k in range(1, 1 + diag_step)]
            last_markable_diagonal = self.last_visible_on_diagonal_path(diagonal_cells, sign_x, sign_y)
            if last_markable_diagonal is not None:
                diagonal_part = diagonal_cells[:diagonal_cells.index(last_markable_diagonal) + 1]

                for x, y in diagonal_part:
                    if self.is_free_in_working_grid(x, y):
                        markable_cells_diagonal.append((x, y))

        return markable_cells_diagonal

    def get_markable_cells_from_point(self, cx, cy, visible_offsets, dda_rays):
        markable = []

        for dx, dy in visible_offsets:
            tx = cx + dx
            ty = cy + dy

            if not self.is_free_in_working_grid(tx, ty):
                continue

            if abs(dx) == abs(dy) and dx != 0:
                continue

            ray_cells = dda_rays[(dx, dy)]
            blocked = False

            for lx, ly in ray_cells:
                rx = cx + lx
                ry = cy + ly

                if self.static_obstacle_mask[ry, rx]:
                    blocked = True
                    break

            if blocked:
                continue

            markable.append((tx, ty))
            
        markable.extend(self.get_vision_on_diagonals(cx, cy))

        return markable

    def save_debug_image(self, output_path: str):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        img[self.static_obstacle_mask] = (0, 0, 0)

        free_remaining = (~self.static_obstacle_mask) & (self.working_grid == 0)
        img[free_remaining] = (255, 255, 255)

        covered = (~self.static_obstacle_mask) & (self.working_grid == 100)
        img[covered] = (180, 180, 180)

        for item in self.observation_points:
            x, y = item["observation_point"]
            if self.in_bounds(x, y):
                img[y, x] = (0, 0, 255)

        cv2.imwrite(output_path, img)

    def save_points_txt(self, output_path: str):
        with open(output_path, "w", encoding="utf-8") as f:
            for item in self.observation_points:
                f.write(
                    f"step={item['step']}, "
                    f"seed={item['seed_cell']}, "
                    f"point={item['observation_point']}, "
                    f"visible_count={item['visible_count']}\n"
                )

    def save_visibility_txt(self, output_path: str):
        with open(output_path, "w", encoding="utf-8") as f:
            for item in self.observation_points:
                f.write(
                    f"step={item['step']}, "
                    f"point={item['observation_point']}, "
                    f"visible={item['visible_cells']}\n"
                )

    def validate_visibility_count(self):
        total_visible_count = sum(item["visible_count"] for item in self.observation_points)
        total_free_cells = int(np.count_nonzero(~self.static_obstacle_mask))

        if total_visible_count < total_free_cells:
            missing = total_free_cells - total_visible_count
            print(
                f"[WARNING] Not all free cells were seen. "
                f"visible_count={total_visible_count}, free_cells={total_free_cells}, missing={missing}"
            )
            return False

        if total_visible_count > total_free_cells:
            extra = total_visible_count - total_free_cells
            print(
                f"[WARNING] Duplicates detected. "
                f"visible_count={total_visible_count}, free_cells={total_free_cells}, extra={extra}"
            )
            return False

        print(
            f"[OK] Visibility count is correct. "
            f"visible_count={total_visible_count}, free_cells={total_free_cells}"
        )
        return True

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("pgm_path", help="Path to PGM file")
    parser.add_argument("--range-cells", type=int, default=8, help="Range of observation in cells")
    parser.add_argument("--free-threshold", type=int, default=220)
    args = parser.parse_args()

    pgm_path = Path(args.pgm_path).resolve()
    output_dir = pgm_path

    debug_image_path = output_dir / "observation_points_debug.png"
    points_txt_path = output_dir / "observation_points.txt"
    visibility_txt_path = output_dir / "visibility.txt"

    planner = PointGenerator(
        pgm_path=str(pgm_path),
        observation_range_cells=args.range_cells,
        free_threshold=args.free_threshold,
    )

    points = planner.plan()
    planner.save_debug_image(str(debug_image_path))
    planner.save_points_txt(str(points_txt_path))
    planner.save_visibility_txt(str(visibility_txt_path))

    print(f"Observation points: {len(points)}")
    print(f"Saved image: {debug_image_path}")
    print(f"Saved points: {points_txt_path}")

if __name__ == "__main__":
    main()