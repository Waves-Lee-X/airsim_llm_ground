from __future__ import annotations

import math

from core.temporal_grid import TemporalVoxelGrid, VoxelState


class TestTemporalVoxelGrid:
    """Tests for the 3D temporal voxel grid."""

    @staticmethod
    def _grid(**kw) -> TemporalVoxelGrid:
        defaults = dict(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=40.0, size_z_m=20.0, resolution_m=1.0,
            hit_confirm=2, miss_confirm=3, inflation_m=0.0,
        )
        defaults.update(kw)
        return TemporalVoxelGrid(**defaults)

    # ── init / bounds ──────────────────────────────────────────

    def test_init_sets_origin_and_dimensions(self):
        g = self._grid(size_xy_m=40.0, size_z_m=20.0, resolution_m=1.0)
        assert g.cols == 40
        assert g.rows == 40
        assert g.layers == 20
        assert g.origin_x == -20.0
        assert g.origin_y == -20.0
        assert g.origin_z == -20.0

    def test_world_to_voxel_inside(self):
        g = self._grid()
        idx = g._world_to_voxel(0.0, 0.0, -10.0)
        assert idx is not None
        assert idx == (20, 20, 10)

    def test_world_to_voxel_outside_returns_none(self):
        g = self._grid()
        assert g._world_to_voxel(999.0, 0.0, -10.0) is None

    def test_voxel_center_world(self):
        g = self._grid()
        x, y, z = g.voxel_center_world(20, 20, 10)
        assert abs(x) < 1.0
        assert abs(y) < 1.0
        assert abs(z + 10.0) < 1.0

    # ── point ingestion ────────────────────────────────────────

    def test_add_points_3d_increments_hit_counts(self):
        g = self._grid(hit_confirm=2)
        pt = (2.0, 3.0, -10.0)
        g.add_points_3d([pt])
        g.add_points_3d([pt])
        idx = g._world_to_voxel(*pt)
        assert idx is not None
        assert g.is_confirmed_occupied(*idx)

    def test_single_point_does_not_confirm_under_threshold(self):
        g = self._grid(hit_confirm=3)
        pt = (2.0, 3.0, -10.0)
        g.add_points_3d([pt])
        idx = g._world_to_voxel(*pt)
        assert idx is not None
        assert g.get_state(*idx) == VoxelState.UNKNOWN

    # ── ray casts ──────────────────────────────────────────────

    def test_ray_cast_confirms_hit_and_miss(self):
        g = self._grid(hit_confirm=2, miss_confirm=2)
        origin = (0.0, 0.0, -10.0)
        endpoints = [(10.0, 0.0, -10.0)]
        for _ in range(3):
            g.add_ray_casts(origin, endpoints)
        hit_idx = g._world_to_voxel(10.0, 0.0, -10.0)
        mid_idx = g._world_to_voxel(5.0, 0.0, -10.0)
        assert hit_idx is not None
        assert mid_idx is not None
        assert g.is_confirmed_occupied(*hit_idx)
        assert g.is_confirmed_free(*mid_idx)

    # ── state queries ──────────────────────────────────────────

    def test_unknown_is_not_confirmed(self):
        g = self._grid()
        assert not g.is_confirmed_occupied(10, 10, 5)
        assert not g.is_confirmed_free(10, 10, 5)
        assert g.get_state(10, 10, 5) == VoxelState.UNKNOWN

    def test_get_state_out_of_bounds_returns_unknown(self):
        g = self._grid()
        assert g.get_state(-999, 0, 0) == VoxelState.UNKNOWN

    # ── 2D slice ───────────────────────────────────────────────

    def test_extract_2d_slice_empty_when_no_obstacles(self):
        g = self._grid()
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        assert sl.width == g.cols
        assert sl.height == g.rows
        assert len(sl.blocked) == 0

    def test_extract_2d_slice_blocks_occupied_columns(self):
        g = self._grid(hit_confirm=1, inflation_m=0.0)
        for _ in range(3):
            g.add_points_3d([(5.0, 5.0, -10.0)])
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        assert len(sl.blocked) > 0
        assert sl.is_blocked(sl.world_to_cell(5.0, 5.0))

    def test_slice_cell_to_world_roundtrip(self):
        g = self._grid()
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        cell = sl.world_to_cell(3.0, -2.0)
        assert cell is not None
        wx, wy = sl.cell_to_world(cell)
        assert abs(wx - 3.0) < g.resolution_m
        assert abs(wy + 2.0) < g.resolution_m

    # ── inflation ──────────────────────────────────────────────

    def test_inflation_expands_blocked_cells(self):
        g_no_infl = self._grid(hit_confirm=1, inflation_m=0.0)
        g_infl = self._grid(hit_confirm=1, inflation_m=2.0)
        pt = (5.0, 5.0, -10.0)
        g_no_infl.add_points_3d([pt])
        g_infl.add_points_3d([pt])
        sl_no = g_no_infl.extract_2d_slice(z_center=-10.0, half_height=4.0)
        sl_yes = g_infl.extract_2d_slice(z_center=-10.0, half_height=4.0)
        assert len(sl_yes.blocked) > len(sl_no.blocked)

    # ── recenter ────────────────────────────────────────────────

    def test_recenter_preserves_overlapping_data(self):
        g = self._grid(size_xy_m=20.0, hit_confirm=1)
        g.add_points_3d([(2.0, 2.0, -10.0)])
        g.recenter(new_center_x=5.0, new_center_y=5.0, new_center_z=-10.0)
        idx = g._world_to_voxel(2.0, 2.0, -10.0)
        assert idx is not None
        assert g.is_confirmed_occupied(*idx)

    def test_recenter_drops_data_outside_new_bounds(self):
        g = self._grid(size_xy_m=10.0, hit_confirm=1)
        g.add_points_3d([(0.0, 0.0, -10.0)])
        g.recenter(new_center_x=30.0, new_center_y=30.0, new_center_z=-10.0)
        idx = g._world_to_voxel(0.0, 0.0, -10.0)
        assert idx is None

    # ── should_recenter ─────────────────────────────────────────

    def test_should_recenter_false_near_center(self):
        g = self._grid(size_xy_m=40.0)
        assert not g.should_recenter(2.0, 2.0, -10.0, margin=0.3)

    def test_should_recenter_true_near_boundary(self):
        g = self._grid(size_xy_m=40.0)
        half = 20.0
        assert g.should_recenter(half * 0.8, 0.0, -10.0, margin=0.3)

    def test_should_recenter_true_near_z_boundary(self):
        g = self._grid(size_z_m=20.0)
        assert g.should_recenter(0.0, 0.0, -1.0, margin=0.3)

    # ── is_path_blocked ────────────────────────────────────────

    def test_is_path_blocked_returns_true_for_occupied_column(self):
        g = self._grid(hit_confirm=1)
        g.add_points_3d([(4.0, 4.0, -10.0)])
        assert g.is_path_blocked(4.0, 4.0, -10.0, half_height=4.0)

    def test_is_path_blocked_returns_false_for_free_column(self):
        g = self._grid()
        assert not g.is_path_blocked(4.0, 4.0, -10.0, half_height=4.0)

    # ── occupied_columns_below ──────────────────────────────────

    def test_occupied_columns_below_finds_obstacles(self):
        g = self._grid(hit_confirm=1)
        g.add_points_3d([(0.0, 0.0, -9.0)])
        occupied = g.occupied_columns_below(z_ceiling=-8.0)
        assert len(occupied) > 0

    def test_occupied_columns_below_empty_when_nothing(self):
        g = self._grid()
        occupied = g.occupied_columns_below(z_ceiling=-8.0)
        assert len(occupied) == 0

    # ── clear ──────────────────────────────────────────────────

    def test_clear_resets_all_state(self):
        g = self._grid(hit_confirm=1)
        g.add_points_3d([(0.0, 0.0, -10.0)])
        g.clear()
        assert g._total_points == 0
        assert g.hit_counts.sum() == 0
        assert g.miss_counts.sum() == 0

    # ── stats ──────────────────────────────────────────────────

    def test_stats_returns_expected_keys(self):
        g = self._grid()
        s = g.stats()
        assert "dimensions" in s
        assert "total_voxels" in s
        assert "resolution_m" in s

    # ── world_bounds ───────────────────────────────────────────

    def test_world_bounds_returns_correct_extents(self):
        g = self._grid(size_xy_m=40.0, size_z_m=20.0)
        b = g.world_bounds()
        assert b["x_min"] == -20.0
        assert b["x_max"] == 20.0
        assert b["z_min"] == -20.0
        assert b["z_max"] == 0.0

    # ── max_decay ──────────────────────────────────────────────

    def test_max_decay_caps_counts(self):
        g = self._grid(hit_confirm=1, max_decay=2)
        pt = (0.0, 0.0, -10.0)
        for _ in range(10):
            g.add_points_3d([pt])
        idx = g._world_to_voxel(*pt)
        assert idx is not None
        assert g.hit_counts[idx] <= 2


class TestGrid2DSlice:
    """Tests for the 2D slice used by A*."""

    def _sample_grid(self) -> TemporalVoxelGrid:
        g = TemporalVoxelGrid(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=30.0, size_z_m=12.0, resolution_m=1.0,
            hit_confirm=1, inflation_m=0.0,
        )
        g.add_points_3d([(8.0, 0.0, -10.0)])
        return g

    def test_nearest_free_returns_same_cell_when_free(self):
        g = self._sample_grid()
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        cell = sl.world_to_cell(0.0, 0.0)
        assert sl.nearest_free(cell) == cell

    def test_nearest_free_finds_nearby_cell_when_blocked(self):
        g = self._sample_grid()
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        blocked_cell = sl.world_to_cell(8.0, 0.0)
        free = sl.nearest_free(blocked_cell, max_radius=5)
        assert free is not None
        assert not sl.is_blocked(free)

    def test_nearest_free_returns_none_when_surrounded(self):
        g = TemporalVoxelGrid(
            center_x=0.0, center_y=0.0, center_z=-10.0,
            size_xy_m=6.0, size_z_m=4.0, resolution_m=1.0,
            hit_confirm=1, inflation_m=0.0,
        )
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                g.add_points_3d([(float(dx), float(dy), -10.0)])
        sl = g.extract_2d_slice(z_center=-10.0, half_height=4.0)
        center = sl.world_to_cell(0.0, 0.0)
        assert sl.nearest_free(center, max_radius=2) is None
