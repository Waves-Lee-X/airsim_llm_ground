from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

from core.occupancy_grid import OccupancyGrid
from core.path_planner import Waypoint


@dataclass(frozen=True)
class AvoidancePlan:
    ok: bool
    waypoints: list[Waypoint]
    reason: str
    grid: dict[str, object]

    def to_api(self) -> dict[str, object]:
        return {
            "ok": self.ok,
            "waypoints": [point.to_api() for point in self.waypoints],
            "reason": self.reason,
            "grid": self.grid,
        }


def plan_local_path(
    start: Waypoint,
    goal: Waypoint,
    obstacle_points: list[tuple[float, float]],
    grid_size_m: float = 70.0,
    resolution_m: float = 1.5,
    inflation_m: float = 3.5,
    max_waypoints: int = 18,
    bounds: tuple[float, float, float, float] | None = None,
) -> AvoidancePlan:
    center_x = (start.x + goal.x) / 2.0
    center_y = (start.y + goal.y) / 2.0
    span = max(abs(start.x - goal.x), abs(start.y - goal.y), grid_size_m)
    grid = OccupancyGrid(
        center_x=center_x,
        center_y=center_y,
        size_m=max(grid_size_m, span + 24.0),
        resolution_m=resolution_m,
        inflation_m=inflation_m,
    )
    grid.add_world_points(obstacle_points)
    if bounds is not None:
        grid.block_outside_world_bounds(*bounds)
    start_cell = grid.world_to_cell(start.x, start.y)
    goal_cell = grid.world_to_cell(goal.x, goal.y)
    if start_cell is None or goal_cell is None:
        return AvoidancePlan(False, [], "start or goal outside local grid", grid.to_api())
    start_cell = grid.nearest_free(start_cell) or start_cell
    goal_cell = grid.nearest_free(goal_cell) or goal_cell
    if grid.is_blocked(start_cell) or grid.is_blocked(goal_cell):
        return AvoidancePlan(False, [], "no free start or goal cell", grid.to_api())
    cells = _astar(grid, start_cell, goal_cell)
    if not cells:
        return AvoidancePlan(False, [], "A* could not find a free path", grid.to_api())
    sampled = _sample_cells(cells, max_waypoints=max_waypoints)
    waypoints = [Waypoint(grid.cell_to_world(cell).x, grid.cell_to_world(cell).y, goal.z) for cell in sampled]
    if not waypoints or _distance_2d(waypoints[-1], goal) > resolution_m * 2.0:
        waypoints.append(goal)
    api_grid = grid.to_api()
    if bounds is not None:
        api_grid["bounds"] = {"x_min": bounds[0], "x_max": bounds[1], "y_min": bounds[2], "y_max": bounds[3]}
    return AvoidancePlan(True, waypoints, f"A* path generated with {len(waypoints)} waypoints", api_grid)


def _astar(
    grid: OccupancyGrid,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    visited: set[tuple[int, int]] = set()

    while open_heap:
        _priority, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return _reconstruct(came_from, current)
        for neighbor, cost in _neighbors(current):
            if not grid.in_bounds(neighbor) or grid.is_blocked(neighbor):
                continue
            tentative = g_score[current] + cost
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                priority = tentative + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, neighbor))
    return []


def _neighbors(cell: tuple[int, int]) -> list[tuple[tuple[int, int], float]]:
    col, row = cell
    result: list[tuple[tuple[int, int], float]] = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            result.append(((col + dx, row + dy), math.sqrt(dx * dx + dy * dy)))
    return result


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct(came_from: dict[tuple[int, int], tuple[int, int]], current: tuple[int, int]) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _sample_cells(cells: list[tuple[int, int]], max_waypoints: int) -> list[tuple[int, int]]:
    if len(cells) <= max_waypoints:
        return cells[1:]
    step = max(1, len(cells) // max_waypoints)
    sampled = cells[step::step]
    if cells[-1] not in sampled:
        sampled.append(cells[-1])
    return sampled[:max_waypoints]


def _distance_2d(a: Waypoint, b: Waypoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
