from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class GridPoint:
    x: float
    y: float


class OccupancyGrid:
    """Small local 2D occupancy grid built from AirSim LiDAR points."""

    def __init__(
        self,
        center_x: float,
        center_y: float,
        size_m: float = 60.0,
        resolution_m: float = 1.5,
        inflation_m: float = 3.0,
    ) -> None:
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.size_m = max(12.0, float(size_m))
        self.resolution_m = max(0.5, float(resolution_m))
        self.inflation_m = max(0.0, float(inflation_m))
        self.width = int(math.ceil(self.size_m / self.resolution_m))
        self.height = self.width
        self.origin_x = self.center_x - self.size_m / 2.0
        self.origin_y = self.center_y - self.size_m / 2.0
        self.blocked: set[tuple[int, int]] = set()
        self.raw_blocked: set[tuple[int, int]] = set()

    def add_world_points(self, points: Iterable[tuple[float, float]]) -> None:
        for x, y in points:
            cell = self.world_to_cell(x, y)
            if cell is not None:
                self.raw_blocked.add(cell)
        self._inflate()

    def block_outside_world_bounds(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        for col in range(self.width):
            for row in range(self.height):
                point = self.cell_to_world((col, row))
                if point.x < x_min or point.x > x_max or point.y < y_min or point.y > y_max:
                    self.blocked.add((col, row))

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((float(x) - self.origin_x) / self.resolution_m)
        row = int((float(y) - self.origin_y) / self.resolution_m)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def cell_to_world(self, cell: tuple[int, int]) -> GridPoint:
        col, row = cell
        return GridPoint(
            x=self.origin_x + (col + 0.5) * self.resolution_m,
            y=self.origin_y + (row + 0.5) * self.resolution_m,
        )

    def is_blocked(self, cell: tuple[int, int]) -> bool:
        return cell in self.blocked

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def nearest_free(self, cell: tuple[int, int], max_radius: int = 8) -> tuple[int, int] | None:
        if self.in_bounds(cell) and not self.is_blocked(cell):
            return cell
        for radius in range(1, max_radius + 1):
            candidates: list[tuple[int, int]] = []
            for dx in range(-radius, radius + 1):
                candidates.append((cell[0] + dx, cell[1] - radius))
                candidates.append((cell[0] + dx, cell[1] + radius))
            for dy in range(-radius + 1, radius):
                candidates.append((cell[0] - radius, cell[1] + dy))
                candidates.append((cell[0] + radius, cell[1] + dy))
            for candidate in candidates:
                if self.in_bounds(candidate) and not self.is_blocked(candidate):
                    return candidate
        return None

    def _inflate(self) -> None:
        inflate_cells = int(math.ceil(self.inflation_m / self.resolution_m))
        self.blocked = set(self.raw_blocked)
        for col, row in self.raw_blocked:
            for dx in range(-inflate_cells, inflate_cells + 1):
                for dy in range(-inflate_cells, inflate_cells + 1):
                    if math.hypot(dx, dy) * self.resolution_m <= self.inflation_m:
                        cell = (col + dx, row + dy)
                        if self.in_bounds(cell):
                            self.blocked.add(cell)

    def to_api(self) -> dict[str, object]:
        return {
            "center": {"x": self.center_x, "y": self.center_y},
            "size_m": self.size_m,
            "resolution_m": self.resolution_m,
            "inflation_m": self.inflation_m,
            "raw_obstacles": len(self.raw_blocked),
            "inflated_obstacles": len(self.blocked),
        }
