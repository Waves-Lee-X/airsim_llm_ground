"""3D A* path planning on OctoMap occupancy grids.

Operates on dense 3D grids extracted from OctoMapBridge.extract_planning_grid().
Supports 26-neighbor connectivity, configurable occupancy thresholds, and
unknown-cell cost penalties.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import TYPE_CHECKING

import numpy as np

from core.path_planner import Waypoint

if TYPE_CHECKING:
    from core.octomap_adapter import OctoMapBridge

# Occupancy threshold above which a voxel is considered impassable
DEFAULT_OCCUPIED_THRESHOLD = 0.65


def plan_path_3d(
    start: Waypoint,
    goal: Waypoint,
    octomap: "OctoMapBridge",
    resolution: float = 0.5,
    margin_xy_m: float = 8.0,
    margin_z_m: float = 4.0,
    max_waypoints: int = 30,
    unknown_cost: float = 0.5,
    occupied_threshold: float = DEFAULT_OCCUPIED_THRESHOLD,
    max_expansions: int = 200_000,
    timeout_s: float = 0.5,
) -> tuple[list[Waypoint], dict[str, object]]:
    """Plan a 3D path from start to goal through the OctoMap.

    Args:
        start/goal: World-frame waypoints.
        octomap: The OctoMapBridge instance.
        resolution: Planning grid resolution (m).
        margin_xy_m: Extra XY margin around the bounding box (m).
        margin_z_m: Extra Z margin (kept smaller — drone altitude changes slowly).
        max_waypoints: Maximum output waypoints (sub-sampled).
        unknown_cost: Extra cost multiplier for unknown voxels (0..1).
        occupied_threshold: Occupancy probability above which voxels are blocked.
        max_expansions: Maximum A* node expansions (safety limit).
        timeout_s: Time budget for planning.

    Returns:
        (waypoints, diagnostics_dict).  If no path found, waypoints is empty.
    """
    t0 = time.time()

    # -- compute planning bounds ------------------------------------------
    bx_min = min(start.x, goal.x) - margin_xy_m
    bx_max = max(start.x, goal.x) + margin_xy_m
    by_min = min(start.y, goal.y) - margin_xy_m
    by_max = max(start.y, goal.y) + margin_xy_m
    bz_min = min(start.z, goal.z) - margin_z_m
    bz_max = max(start.z, goal.z) + margin_z_m

    # Extract dense occupancy grid (vectorized, fast)
    grid, (ox, oy, oz) = octomap.extract_planning_grid(
        bx_min, bx_max, by_min, by_max, bz_min, bz_max, resolution=resolution,
    )

    gx, gy, gz = grid.shape

    # -- world → grid index helpers (inline for speed) --------------------
    _ox, _oy, _oz = ox, oy, oz
    _res = resolution

    start_idx = (int((start.x - _ox) / _res), int((start.y - _oy) / _res), int((start.z - _oz) / _res))
    goal_idx = (int((goal.x - _ox) / _res), int((goal.y - _oy) / _res), int((goal.z - _oz) / _res))

    # Clamp to grid bounds (safe — grid is sized to include start/goal + margin)
    start_idx = _clamp_idx(start_idx, grid.shape)
    goal_idx = _clamp_idx(goal_idx, grid.shape)

    if grid[start_idx] >= occupied_threshold:
        start_idx = _nearest_free_idx(grid, start_idx, occupied_threshold) or start_idx
    if grid[goal_idx] >= occupied_threshold:
        goal_idx = _nearest_free_idx(grid, goal_idx, occupied_threshold) or goal_idx

    if grid[start_idx] >= occupied_threshold or grid[goal_idx] >= occupied_threshold:
        return [], {"error": "start or goal blocked", "grid_shape": grid.shape}

    # -- A* search ---------------------------------------------------------
    _neighbors = _neighbor_offsets(resolution)
    _occ_th = occupied_threshold
    _u_cost = unknown_cost
    _occ_range = max(0.001, occupied_threshold - 0.5)

    goal_ci, goal_rj, goal_lk = goal_idx

    # Use flat arrays for g_score / visited (much faster than dict+set hashing)
    g_score = np.full(grid.shape, np.inf, dtype=np.float32)
    g_score[start_idx] = 0.0
    closed = np.zeros(grid.shape, dtype=bool)

    open_heap: list[tuple[float, int, int, int, int]] = [
        (_heuristic(start_idx, goal_idx), 0, start_idx[0], start_idx[1], start_idx[2])
    ]
    came_from: dict[tuple[int, int, int], tuple[int, int, int]] = {}
    expansions = 0
    tie_counter = 1
    _gx, _gy, _gz = gx, gy, gz

    while open_heap and expansions < max_expansions:
        if time.time() - t0 > timeout_s:
            return [], {"error": "timeout", "expansions": expansions}

        _, _, ci, rj, lk = heapq.heappop(open_heap)
        if closed[ci, rj, lk]:
            continue
        closed[ci, rj, lk] = True
        expansions += 1

        if ci == goal_ci and rj == goal_rj and lk == goal_lk:
            current = (ci, rj, lk)
            cells = _reconstruct_3d(came_from, current)
            waypoints = _cells_to_waypoints(cells, idx_to_world_3d_factory(_ox, _oy, _oz, _res), max_waypoints)
            return waypoints, {
                "ok": True, "expansions": expansions, "path_cells": len(cells),
                "waypoints": len(waypoints), "time_ms": round((time.time() - t0) * 1000),
                "grid_shape": grid.shape,
            }

        cur_g = float(g_score[ci, rj, lk])

        for di, dj, dk, step_dist in _neighbors:
            ni, nj, nk = ci + di, rj + dj, lk + dk
            if ni < 0 or ni >= _gx or nj < 0 or nj >= _gy or nk < 0 or nk >= _gz:
                continue
            if closed[ni, nj, nk]:
                continue

            occ = float(grid[ni, nj, nk])
            if occ >= _occ_th:
                continue

            # Step cost scales with occupancy probability
            if occ > 0.5:
                cost = step_dist * (1.0 + _u_cost * 2.0 * (occ - 0.5) / _occ_range)
            elif occ == 0.5:
                cost = step_dist * (1.0 + _u_cost)
            else:
                cost = step_dist

            tentative = cur_g + cost
            if tentative >= g_score[ni, nj, nk]:
                continue

            g_score[ni, nj, nk] = tentative
            came_from[(ni, nj, nk)] = (ci, rj, lk)
            h = _heuristic_raw(ni, nj, nk, goal_ci, goal_rj, goal_lk) * _res
            heapq.heappush(open_heap, (tentative + h, tie_counter, ni, nj, nk))
            tie_counter += 1

    return [], {
        "ok": False, "expansions": expansions,
        "time_ms": round((time.time() - t0) * 1000), "grid_shape": grid.shape,
    }


# ── Precomputed 26-neighbor offsets + Euclidean distances ────────────

_NEIGHBOR_CACHE: dict[float, list[tuple[int, int, int, float]]] = {}


def _neighbor_offsets(resolution: float) -> list[tuple[int, int, int, float]]:
    """Return (di, dj, dk, distance_m) for the 26 neighbors, cached by resolution."""
    if resolution in _NEIGHBOR_CACHE:
        return _NEIGHBOR_CACHE[resolution]
    offsets: list[tuple[int, int, int, float]] = []
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            for dk in (-1, 0, 1):
                if di == 0 and dj == 0 and dk == 0:
                    continue
                dist = math.hypot(di, dj, dk) * resolution
                offsets.append((di, dj, dk, dist))
    _NEIGHBOR_CACHE[resolution] = offsets
    return offsets


def _heuristic(a: tuple[int, int, int], b: tuple[int, int, int]) -> float:
    """Euclidean distance between two grid indices (unit = cells, not metres)."""
    return math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _heuristic_raw(ci: int, rj: int, lk: int, gi: int, gj: int, gk: int) -> float:
    """Euclidean distance without resolution multiplier."""
    return math.hypot(ci - gi, rj - gj, lk - gk)


def idx_to_world_3d_factory(ox: float, oy: float, oz: float, resolution: float):
    """Return a callable that converts grid index → world coordinate."""
    def convert(ci: int, rj: int, lk: int) -> tuple[float, float, float]:
        return (ox + (ci + 0.5) * resolution,
                oy + (rj + 0.5) * resolution,
                oz + (lk + 0.5) * resolution)
    return convert


def _reconstruct_3d(
    came_from: dict[tuple[int, int, int], tuple[int, int, int]],
    current: tuple[int, int, int],
) -> list[tuple[int, int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _cells_to_waypoints(
    cells: list[tuple[int, int, int]],
    idx_to_world,
    max_waypoints: int,
) -> list[Waypoint]:
    """Convert grid-index path to world Waypoints, sub-sampling for efficiency.

    The goal (cells[-1]) is always included as the final waypoint.
    """
    if len(cells) <= 2:
        waypoints = []
        for ci, rj, lk in cells:
            wx, wy, wz = idx_to_world(ci, rj, lk)
            waypoints.append(Waypoint(wx, wy, wz))
        return waypoints

    # Reserve the last waypoint slot for the goal
    budget = max_waypoints - 1
    step = max(1, (len(cells) - 1) // budget) if budget > 0 else len(cells)
    waypoints = []
    for i in range(0, len(cells) - 1, step):
        ci, rj, lk = cells[i]
        wx, wy, wz = idx_to_world(ci, rj, lk)
        waypoints.append(Waypoint(wx, wy, wz))
        if len(waypoints) >= budget:
            break

    # Goal is always the last waypoint
    last_ci, last_rj, last_lk = cells[-1]
    waypoints.append(Waypoint(*idx_to_world(last_ci, last_rj, last_lk)))
    return waypoints


def _clamp_idx(
    idx: tuple[int, int, int],
    shape: tuple[int, int, int],
) -> tuple[int, int, int]:
    return (
        max(0, min(idx[0], shape[0] - 1)),
        max(0, min(idx[1], shape[1] - 1)),
        max(0, min(idx[2], shape[2] - 1)),
    )


def _nearest_free_idx(
    grid: np.ndarray,
    idx: tuple[int, int, int],
    threshold: float,
    max_radius: int = 6,
) -> tuple[int, int, int] | None:
    ci, rj, lk = idx
    for r in range(1, max_radius + 1):
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                for dk in range(-r, r + 1):
                    ni, nj, nk = ci + di, rj + dj, lk + dk
                    if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1] and 0 <= nk < grid.shape[2]:
                        if grid[ni, nj, nk] < threshold:
                            return ni, nj, nk
    return None
