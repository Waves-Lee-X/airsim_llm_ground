from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import TYPE_CHECKING

from core.path_planner import Waypoint

if TYPE_CHECKING:
    from core.temporal_grid import Grid2DSlice


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


def plan_local_path_on_slice(
    start: Waypoint,
    goal: Waypoint,
    grid_slice: "Grid2DSlice",
    max_waypoints: int = 18,
    unknown_cell_cost: float = 0.5,
) -> AvoidancePlan:
    """Run A* on a 2D slice extracted from the 3D temporal voxel grid.

    Unknown cells are not blocked but incur a configurable movement cost
    penalty (default 0.5 extra), so the planner prefers known-free routes
    without blindly avoiding unknown areas.
    """
    start_cell = grid_slice.world_to_cell(start.x, start.y)
    goal_cell = grid_slice.world_to_cell(goal.x, goal.y)
    if start_cell is None or goal_cell is None:
        return AvoidancePlan(False, [], "start or goal outside grid slice", grid_slice.to_api())

    start_cell = grid_slice.nearest_free(start_cell) or start_cell
    goal_cell = grid_slice.nearest_free(goal_cell) or goal_cell
    if grid_slice.is_blocked(start_cell) or grid_slice.is_blocked(goal_cell):
        return AvoidancePlan(False, [], "no free start or goal cell in slice", grid_slice.to_api())

    cells = _astar(grid_slice, start_cell, goal_cell, unknown_cell_cost=unknown_cell_cost)
    if not cells:
        return AvoidancePlan(False, [], "A* could not find a free path on temporal slice", grid_slice.to_api())

    sampled = _sample_cells(cells, max_waypoints=max_waypoints)
    waypoints = []
    for cell in sampled:
        wx, wy = grid_slice.cell_to_world(cell)
        waypoints.append(Waypoint(wx, wy, goal.z))

    if not waypoints or _distance_2d(waypoints[-1], goal) > grid_slice.resolution_m * 2.0:
        waypoints.append(goal)

    return AvoidancePlan(True, waypoints, f"A* on temporal slice: {len(waypoints)} waypoints", grid_slice.to_api())


def _astar(
    grid_slice: "Grid2DSlice",
    start: tuple[int, int],
    goal: tuple[int, int],
    unknown_cell_cost: float = 0.5,
) -> list[tuple[int, int]]:
    """A* on a Grid2DSlice with configurable unknown-cell cost.

    Blocked cells are impassable.  Unknown cells are traversable but
    incur an extra *unknown_cell_cost* penalty per step so the planner
    prefers known-free routes.
    """
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
        for neighbor, base_cost in _neighbors(current):
            if not grid_slice.in_bounds(neighbor) or grid_slice.is_blocked(neighbor):
                continue
            step_cost = base_cost
            if grid_slice.is_unknown(neighbor):
                step_cost += unknown_cell_cost
            tentative = g_score[current] + step_cost
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
