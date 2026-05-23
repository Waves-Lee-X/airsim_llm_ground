from __future__ import annotations

import math

from core.obstacle_avoidance import (
    AvoidancePlan,
    _astar,
    _sample_cells,
    _distance_2d,
    _heuristic,
    _reconstruct,
    plan_local_path,
    plan_local_path_on_slice,
)
from core.occupancy_grid import OccupancyGrid
from core.path_planner import Waypoint
from core.temporal_grid import TemporalVoxelGrid


# ── helpers ────────────────────────────────────────────────────

def _free_grid() -> OccupancyGrid:
    return OccupancyGrid(
        center_x=0.0, center_y=0.0, size_m=50.0,
        resolution_m=1.0, inflation_m=0.0,
    )


def _blocked_grid() -> OccupancyGrid:
    g = _free_grid()
    g.add_world_points([(5.0, 0.0), (5.5, 0.5), (5.0, -0.5)])
    return g


# ── A* core ─────────────────────────────────────────────────────

class TestAStar:
    def test_direct_path_in_empty_grid(self):
        g = _free_grid()
        path = _astar(g, (10, 10), (30, 10))
        assert len(path) > 0
        assert path[0] == (10, 10)
        assert path[-1] == (30, 10)

    def test_no_path_when_goal_blocked(self):
        g = OccupancyGrid(center_x=0.0, center_y=0.0, size_m=10.0,
                          resolution_m=0.5, inflation_m=0.0)
        g.add_world_points([(2.0, 0.0)])
        g_cell = g.world_to_cell(2.0, 0.0)
        path = _astar(g, g.world_to_cell(-2.0, 0.0), g_cell)
        assert path == []

    def test_path_avoids_single_obstacle(self):
        g = _free_grid()
        for dy in range(-2, 3):
            g.add_world_points([(5.0, float(dy))])
        path = _astar(g, (10, 10), (30, 10))  # use raw cell coords
        # Convert to world coords for sanity check
        pts = [(g.cell_to_world(c).x, g.cell_to_world(c).y) for c in path]
        mid_path_x = pts[len(pts) // 2][0]
        assert mid_path_x > 6.0 or pts[0][1] != 10.0

    def test_heuristic_is_euclidean(self):
        assert _heuristic((0, 0), (3, 4)) == 5.0

    def test_reconstruct_returns_full_path(self):
        came_from = {(1, 0): (0, 0), (2, 0): (1, 0)}
        path = _reconstruct(came_from, (2, 0))
        assert path == [(0, 0), (1, 0), (2, 0)]


class TestSampleCells:
    def test_returns_all_when_under_limit(self):
        cells = [(i, 0) for i in range(5)]
        assert _sample_cells(cells, 10) == cells[1:]

    def test_subsamples_when_over_limit(self):
        cells = [(i, 0) for i in range(100)]
        sampled = _sample_cells(cells, 10)
        assert len(sampled) <= 10
        assert cells[-1] in sampled


class TestDistance2D:
    def test_hypot_distance(self):
        a = Waypoint(0.0, 0.0, -10.0)
        b = Waypoint(3.0, 4.0, -12.0)
        assert _distance_2d(a, b) == 5.0


# ── plan_local_path ────────────────────────────────────────────

class TestPlanLocalPath:
    def test_direct_path_returns_ok(self):
        plan = plan_local_path(
            Waypoint(0.0, 0.0, -10.0),
            Waypoint(20.0, 0.0, -10.0),
            obstacle_points=[],
            grid_size_m=30.0,
            resolution_m=1.0,
        )
        assert plan.ok
        assert len(plan.waypoints) >= 1

    def test_path_detours_around_obstacles(self):
        plan = plan_local_path(
            Waypoint(0.0, 0.0, -10.0),
            Waypoint(12.0, 0.0, -10.0),
            obstacle_points=[(5.0, float(y)) for y in range(-6, 7)],
            grid_size_m=30.0,
            resolution_m=1.0,
            inflation_m=1.0,
        )
        assert plan.ok
        mid_xs = [wp.x for wp in plan.waypoints[1:-1]]
        assert any(x < 3.0 for x in mid_xs) or any(abs(wp.y) > 3.0 for wp in plan.waypoints), \
            "path should detour around the obstacle wall"

    def test_completely_blocked_returns_not_ok(self):
        plan = plan_local_path(
            Waypoint(-5.0, 0.0, -10.0),
            Waypoint(5.0, 0.0, -10.0),
            obstacle_points=[(0.0, float(y)) for y in range(-10, 11)],
            grid_size_m=20.0,
            resolution_m=1.0,
            inflation_m=1.0,
            bounds=(-10.0, 10.0, -10.0, 10.0),
        )
        assert not plan.ok

    def test_start_equals_goal(self):
        plan = plan_local_path(
            Waypoint(5.0, 5.0, -10.0),
            Waypoint(5.0, 5.0, -10.0),
            obstacle_points=[],
        )
        assert plan.ok


# ── plan_local_path_on_slice ────────────────────────────────────

class TestPlanLocalPathOnSlice:
    @staticmethod
    def _sample_grid_with_blocked_center() -> TemporalVoxelGrid:
        g = TemporalVoxelGrid(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=40.0, size_z_m=12.0, resolution_m=1.0,
            hit_confirm=1, inflation_m=0.5,
        )
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                g.add_points_3d([(float(dx), float(dy), -10.0)])
        return g

    def test_direct_path_on_clean_slice(self):
        g = TemporalVoxelGrid(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=40.0, size_z_m=12.0, resolution_m=1.0,
            hit_confirm=3, inflation_m=0.0,
        )
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        plan = plan_local_path_on_slice(
            Waypoint(-15.0, 0.0, -10.0),
            Waypoint(15.0, 0.0, -10.0),
            sl,
            max_waypoints=20,
        )
        assert plan.ok
        assert len(plan.waypoints) > 0

    def test_path_avoids_blocked_center(self):
        g = self._sample_grid_with_blocked_center()
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        plan = plan_local_path_on_slice(
            Waypoint(-15.0, 0.0, -10.0),
            Waypoint(15.0, 0.0, -10.0),
            sl,
            max_waypoints=24,
        )
        assert plan.ok
        for wp in plan.waypoints:
            dist_to_center = math.hypot(wp.x, wp.y)
            assert dist_to_center > 2.0, f"waypoint {wp} too close to blocked center"

    def test_no_path_when_fully_blocked(self):
        g = TemporalVoxelGrid(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=20.0, size_z_m=8.0, resolution_m=1.0,
            hit_confirm=1, inflation_m=0.0,
        )
        for dx in range(-10, 11):
            for dy in range(-10, 11):
                g.add_points_3d([(float(dx), float(dy), -10.0)])
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        plan = plan_local_path_on_slice(
            Waypoint(-8.0, 0.0, -10.0),
            Waypoint(8.0, 0.0, -10.0),
            sl,
            max_waypoints=24,
        )
        assert not plan.ok

    def test_plan_includes_grid_info_in_reason(self):
        g = TemporalVoxelGrid(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=30.0, size_z_m=10.0, resolution_m=1.0,
            hit_confirm=3, inflation_m=0.0,
        )
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        plan = plan_local_path_on_slice(
            Waypoint(-10.0, 0.0, -10.0),
            Waypoint(10.0, 0.0, -10.0),
            sl,
        )
        assert "temporal" in plan.reason.lower() or "A*" in plan.reason.lower()


# ── AvoidancePlan ───────────────────────────────────────────────

class TestAvoidancePlan:
    def test_to_api_serializes_correctly(self):
        plan = AvoidancePlan(
            True,
            [Waypoint(1.0, 2.0, -8.0)],
            "test reason",
            {"key": "value"},
        )
        api = plan.to_api()
        assert api["ok"] is True
        assert api["reason"] == "test reason"
        assert api["grid"] == {"key": "value"}
        assert len(api["waypoints"]) == 1
        assert api["waypoints"][0]["x"] == 1.0
