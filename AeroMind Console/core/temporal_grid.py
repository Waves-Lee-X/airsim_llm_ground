from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Iterable

import numpy as np


class VoxelState(Enum):
    UNKNOWN = auto()
    OCCUPIED = auto()
    FREE = auto()


@dataclass
class Grid2DSlice:
    """2D slice extracted from the 3D voxel grid for A* path planning."""
    origin_x: float
    origin_y: float
    size_m: float
    resolution_m: float
    width: int
    height: int
    blocked: set[tuple[int, int]]
    unknown: set[tuple[int, int]] = field(default_factory=set)
    z_extract: float = 0.0
    half_height: float = 8.0

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((float(x) - self.origin_x) / self.resolution_m)
        row = int((float(y) - self.origin_y) / self.resolution_m)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        col, row = cell
        return (
            self.origin_x + (col + 0.5) * self.resolution_m,
            self.origin_y + (row + 0.5) * self.resolution_m,
        )

    def is_blocked(self, cell: tuple[int, int]) -> bool:
        return cell in self.blocked

    def is_unknown(self, cell: tuple[int, int]) -> bool:
        return cell in self.unknown

    def is_free(self, cell: tuple[int, int]) -> bool:
        return cell not in self.blocked and cell not in self.unknown

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def nearest_free(self, cell: tuple[int, int], max_radius: int = 8) -> tuple[int, int] | None:
        if self.in_bounds(cell) and not self.is_blocked(cell):
            return cell
        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self.in_bounds(candidate) and not self.is_blocked(candidate):
                        return candidate
        return None

    def to_api(self) -> dict[str, object]:
        return {
            "type": "temporal_2d_slice",
            "z_extract": self.z_extract,
            "half_height": self.half_height,
            "size_m": self.size_m,
            "blocked_cells": len(self.blocked),
            "unknown_cells": len(self.unknown),
        }


class TemporalVoxelGrid:
    """3D voxel grid with temporal filtering and decay.

    Each voxel tracks hit_count and free_count independently.
    A voxel is confirmed OCCUPIED when hit_count >= hit_confirm.
    A voxel is confirmed FREE when free_count >= free_confirm and hit_count < hit_confirm.
    Otherwise the voxel is UNKNOWN.

    Depth camera rays provide both hit (endpoint) and free (ray traversal) information.
    LiDAR points provide only hit information.

    Time decay: calling decay_counts() multiplies all counts by decay_factor,
    so stale obstacles gradually fade out.
    """

    def __init__(
        self,
        center_x: float,
        center_y: float,
        center_z: float,
        size_xy_m: float = 70.0,
        size_z_m: float = 24.0,
        resolution_m: float = 1.0,
        hit_confirm: int = 3,
        free_confirm: int = 5,
        inflation_m: float = 1.5,
        max_decay: int = 20,
    ) -> None:
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.center_z = float(center_z)
        self.size_xy_m = max(12.0, float(size_xy_m))
        self.size_z_m = max(4.0, float(size_z_m))
        self.resolution_m = max(0.5, float(resolution_m))
        self.hit_confirm = max(1, hit_confirm)
        self.free_confirm = max(1, free_confirm)
        self.inflation_m = max(0.0, float(inflation_m))
        self.max_decay = max(1, max_decay)

        self.cols = int(math.ceil(self.size_xy_m / self.resolution_m))
        self.rows = self.cols
        self.layers = int(math.ceil(self.size_z_m / self.resolution_m))

        self.origin_x = self.center_x - self.size_xy_m / 2.0
        self.origin_y = self.center_y - self.size_xy_m / 2.0
        self.origin_z = self.center_z - self.size_z_m / 2.0

        self.hit_counts: np.ndarray = np.zeros((self.cols, self.rows, self.layers), dtype=np.int16)
        self.free_counts: np.ndarray = np.zeros((self.cols, self.rows, self.layers), dtype=np.int16)
        self._total_points: int = 0
        self._last_decay_at: float = 0.0

    def add_points_3d(self, points: Iterable[tuple[float, float, float]]) -> int:
        """Register hit counts for 3D points (e.g., from LiDAR or depth camera endpoints)."""
        added = 0
        for x, y, z in points:
            idx = self._world_to_voxel(x, y, z)
            if idx is not None:
                col, row, layer = idx
                if self.hit_counts[col, row, layer] < self.max_decay:
                    self.hit_counts[col, row, layer] += 1
                added += 1
        self._total_points += added
        return added

    def add_ray_casts(
        self,
        origin: tuple[float, float, float],
        endpoints: Iterable[tuple[float, float, float]],
    ) -> int:
        """Cast rays from origin to each endpoint.

        Voxels along each ray get free_count incremented (free space).
        Voxels at the endpoints get hit_count incremented (occupied).
        """
        ox, oy, oz = float(origin[0]), float(origin[1]), float(origin[2])
        rays = 0
        for ex, ey, ez in endpoints:
            rays += 1
            ray_len = math.hypot(ex - ox, ey - oy, ez - oz)
            if ray_len <= 0.01:
                continue

            steps = max(1, int(ray_len / (self.resolution_m * 0.5)))
            for step_i in range(steps):
                t = step_i / steps
                px = ox + (ex - ox) * t
                py = oy + (ey - oy) * t
                pz = oz + (ez - oz) * t
                idx = self._world_to_voxel(px, py, pz)
                if idx is not None:
                    col, row, layer = idx
                    if self.free_counts[col, row, layer] < self.max_decay:
                        self.free_counts[col, row, layer] += 1

            idx_end = self._world_to_voxel(ex, ey, ez)
            if idx_end is not None:
                col, row, layer = idx_end
                if self.hit_counts[col, row, layer] < self.max_decay:
                    self.hit_counts[col, row, layer] += 1

        return rays

    def decay_counts(self, factor: float = 0.95) -> None:
        """Apply exponential decay to all voxel counts.

        Multiplies hit_counts and free_counts by *factor*, gradually fading
        stale obstacles.  Call periodically (e.g. every 5 s).
        """
        self.hit_counts = (self.hit_counts.astype(np.float32) * factor).astype(np.int16)
        self.free_counts = (self.free_counts.astype(np.float32) * factor).astype(np.int16)
        self._last_decay_at = time_now()

    def get_state(self, col: int, row: int, layer: int) -> VoxelState:
        if not (0 <= col < self.cols and 0 <= row < self.rows and 0 <= layer < self.layers):
            return VoxelState.UNKNOWN
        h = int(self.hit_counts[col, row, layer])
        f = int(self.free_counts[col, row, layer])
        if h >= self.hit_confirm:
            return VoxelState.OCCUPIED
        if f >= self.free_confirm and h < self.hit_confirm:
            return VoxelState.FREE
        return VoxelState.UNKNOWN

    def is_confirmed_occupied(self, col: int, row: int, layer: int) -> bool:
        return self.get_state(col, row, layer) == VoxelState.OCCUPIED

    def is_confirmed_free(self, col: int, row: int, layer: int) -> bool:
        return self.get_state(col, row, layer) == VoxelState.FREE

    def extract_2d_slice(
        self,
        z_center: float,
        half_height: float,
    ) -> Grid2DSlice:
        """Extract a 2D occupancy slice for A* path planning.

        Only considers voxels in the vertical range
        [z_center - half_height, z_center + half_height] — the UAV's
        traversable band.  Voxels outside this band are ignored.

        Returns a slice with three semantic sets:
        - blocked:  any column with a confirmed OCCUPIED voxel in the band
        - unknown:  columns with only UNKNOWN voxels (no confirmed FREE)
        - Cells not in either set are confirmed FREE.
        """
        z_min = z_center - half_height
        z_max = z_center + half_height

        layer_min = max(0, int((z_min - self.origin_z) / self.resolution_m))
        layer_max = min(self.layers - 1, int((z_max - self.origin_z) / self.resolution_m))

        blocked: set[tuple[int, int]] = set()
        unknown: set[tuple[int, int]] = set()

        for col in range(self.cols):
            for row in range(self.rows):
                has_free = False
                for layer in range(layer_min, layer_max + 1):
                    state = self.get_state(col, row, layer)
                    if state == VoxelState.OCCUPIED:
                        blocked.add((col, row))
                        break
                    if state == VoxelState.FREE:
                        has_free = True
                else:
                    # No OCCUPIED voxel found in this column
                    if not has_free:
                        unknown.add((col, row))

        if self.inflation_m > 0:
            inflate_cells = int(math.ceil(self.inflation_m / self.resolution_m))
            inflated_blocked: set[tuple[int, int]] = set(blocked)
            for col, row in blocked:
                for dc in range(-inflate_cells, inflate_cells + 1):
                    for dr in range(-inflate_cells, inflate_cells + 1):
                        if math.hypot(dc, dr) * self.resolution_m <= self.inflation_m:
                            nc, nr = col + dc, row + dr
                            if 0 <= nc < self.cols and 0 <= nr < self.rows:
                                inflated_blocked.add((nc, nr))
            blocked = inflated_blocked

        # Remove unknown cells that got swallowed by inflation
        unknown -= blocked

        return Grid2DSlice(
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            size_m=self.size_xy_m,
            resolution_m=self.resolution_m,
            width=self.cols,
            height=self.rows,
            blocked=blocked,
            unknown=unknown,
            z_extract=z_center,
            half_height=half_height,
        )

    def occupied_columns_below(self, z_ceiling: float) -> set[tuple[int, int]]:
        """Find all (col, row) where any voxel below z_ceiling is confirmed occupied."""
        layer_max = max(0, int((z_ceiling - self.origin_z) / self.resolution_m))
        if layer_max >= self.layers:
            layer_max = self.layers - 1
        occupied: set[tuple[int, int]] = set()
        for col in range(self.cols):
            for row in range(self.rows):
                for layer in range(layer_max + 1):
                    if self.is_confirmed_occupied(col, row, layer):
                        occupied.add((col, row))
                        break
        return occupied

    def is_path_blocked(
        self,
        x: float,
        y: float,
        z: float,
        half_height: float = 6.0,
    ) -> bool:
        """Check if a specific world position is in a blocked column."""
        idx = self._world_to_voxel(x, y, z)
        if idx is None:
            return False
        col, row, _layer = idx
        z_min = z - half_height
        z_max = z + half_height
        layer_min = max(0, int((z_min - self.origin_z) / self.resolution_m))
        layer_max = min(self.layers - 1, int((z_max - self.origin_z) / self.resolution_m))
        for layer in range(layer_min, layer_max + 1):
            if self.is_confirmed_occupied(col, row, layer):
                return True
        return False

    def recenter(self, new_center_x: float, new_center_y: float, new_center_z: float) -> None:
        """Recenter the grid around a new position, preserving overlapping data."""
        old_origin_x = self.origin_x
        old_origin_y = self.origin_y
        old_origin_z = self.origin_z

        self.center_x = float(new_center_x)
        self.center_y = float(new_center_y)
        self.center_z = float(new_center_z)
        new_origin_x = self.center_x - self.size_xy_m / 2.0
        new_origin_y = self.center_y - self.size_xy_m / 2.0
        new_origin_z = self.center_z - self.size_z_m / 2.0

        dc = int(round((new_origin_x - old_origin_x) / self.resolution_m))
        dr = int(round((new_origin_y - old_origin_y) / self.resolution_m))
        dl = int(round((new_origin_z - old_origin_z) / self.resolution_m))

        if dc == 0 and dr == 0 and dl == 0:
            self.origin_x = new_origin_x
            self.origin_y = new_origin_y
            self.origin_z = new_origin_z
            return

        new_hits = np.zeros((self.cols, self.rows, self.layers), dtype=np.int16)
        new_frees = np.zeros((self.cols, self.rows, self.layers), dtype=np.int16)

        src_c0, src_c1 = max(0, dc), min(self.cols, self.cols + dc)
        src_r0, src_r1 = max(0, dr), min(self.rows, self.rows + dr)
        src_l0, src_l1 = max(0, dl), min(self.layers, self.layers + dl)

        dst_c0, dst_c1 = src_c0 - dc, src_c1 - dc
        dst_r0, dst_r1 = src_r0 - dr, src_r1 - dr
        dst_l0, dst_l1 = src_l0 - dl, src_l1 - dl

        if src_c1 > src_c0 and src_r1 > src_r0 and src_l1 > src_l0:
            new_hits[dst_c0:dst_c1, dst_r0:dst_r1, dst_l0:dst_l1] = self.hit_counts[src_c0:src_c1, src_r0:src_r1, src_l0:src_l1]
            new_frees[dst_c0:dst_c1, dst_r0:dst_r1, dst_l0:dst_l1] = self.free_counts[src_c0:src_c1, src_r0:src_r1, src_l0:src_l1]

        self.hit_counts = new_hits
        self.free_counts = new_frees
        self.origin_x = new_origin_x
        self.origin_y = new_origin_y
        self.origin_z = new_origin_z

    def should_recenter(self, x: float, y: float, z: float, margin: float = 0.3) -> bool:
        """Check if (x, y, z) is close to the grid boundary and recenter is needed."""
        half_xy = self.size_xy_m / 2.0
        half_z = self.size_z_m / 2.0
        threshold_xy = half_xy * (1.0 - margin)
        threshold_z = half_z * (1.0 - margin)
        return (
            abs(float(x) - self.center_x) > threshold_xy
            or abs(float(y) - self.center_y) > threshold_xy
            or abs(float(z) - self.center_z) > threshold_z
        )

    def clear(self) -> None:
        self.hit_counts.fill(0)
        self.free_counts.fill(0)
        self._total_points = 0

    def stats(self) -> dict[str, object]:
        occupied_count = 0
        free_count = 0
        unknown_count = 0
        for col in range(min(self.cols, 10)):
            for row in range(min(self.rows, 10)):
                for layer in range(self.layers):
                    s = self.get_state(col, row, layer)
                    if s == VoxelState.OCCUPIED:
                        occupied_count += 1
                    elif s == VoxelState.FREE:
                        free_count += 1
                    else:
                        unknown_count += 1
        total = self.cols * self.rows * self.layers
        return {
            "dimensions": f"{self.cols}x{self.rows}x{self.layers}",
            "total_voxels": total,
            "total_points_ingested": self._total_points,
            "resolution_m": self.resolution_m,
            "hit_confirm": self.hit_confirm,
            "free_confirm": self.free_confirm,
            "inflation_m": self.inflation_m,
            "occupied_voxels_estimate": occupied_count * total // max(1, 10 * 10 * self.layers) if total > 0 else 0,
        }

    def to_api(self) -> dict[str, object]:
        return self.stats()

    def _world_to_voxel(self, x: float, y: float, z: float) -> tuple[int, int, int] | None:
        col = int((float(x) - self.origin_x) / self.resolution_m)
        row = int((float(y) - self.origin_y) / self.resolution_m)
        layer = int((float(z) - self.origin_z) / self.resolution_m)
        if 0 <= col < self.cols and 0 <= row < self.rows and 0 <= layer < self.layers:
            return col, row, layer
        return None

    def voxel_center_world(self, col: int, row: int, layer: int) -> tuple[float, float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution_m,
            self.origin_y + (row + 0.5) * self.resolution_m,
            self.origin_z + (layer + 0.5) * self.resolution_m,
        )

    def world_bounds(self) -> dict[str, float]:
        return {
            "x_min": self.origin_x,
            "x_max": self.origin_x + self.size_xy_m,
            "y_min": self.origin_y,
            "y_max": self.origin_y + self.size_xy_m,
            "z_min": self.origin_z,
            "z_max": self.origin_z + self.size_z_m,
        }


def time_now() -> float:
    import time
    return time.time()
