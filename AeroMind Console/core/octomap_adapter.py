"""Pure-Python OctoMap-compatible 3D occupancy grid.

Uses log-odds for Bayesian updates, supports time decay and clamping.
Compatible API with octomap-python concepts but no C++ dependencies.
"""

from __future__ import annotations

import math
import time
from typing import Iterable

import numpy as np


class OctoMapBridge:
    """3D probabilistic occupancy map with Bayesian updates and time decay.

    Each voxel stores a log-odds value L = log(p/(1-p)).
    - Sensor hit:  L += log(prob_hit/(1-prob_hit))
    - Sensor miss: L -= log((1-prob_miss)/prob_miss)
    - Clamping:    |L| <= log(clamp_thresh/(1-clamp_thresh))
    - Decay:       L *= decay_factor  (pulls toward 0.5 = unknown)

    Occupancy is queried as probability p = 1 - 1/(1+exp(L)).
    """

    def __init__(
        self,
        resolution: float = 0.5,
        prob_hit: float = 0.7,
        prob_miss: float = 0.4,
        clamping_threshold: float = 0.97,
        decay_factor: float = 0.95,
        size_xy_m: float = 100.0,
        size_z_m: float = 30.0,
    ) -> None:
        self.resolution = max(0.1, float(resolution))
        self.prob_hit = float(np.clip(prob_hit, 0.51, 0.99))
        self.prob_miss = float(np.clip(prob_miss, 0.01, 0.49))
        self.clamping_threshold = float(np.clip(clamping_threshold, 0.51, 0.999))
        self.decay_factor = float(np.clip(decay_factor, 0.5, 1.0))

        # Pre-compute log-odds increments
        self._log_odds_hit = math.log(self.prob_hit / (1.0 - self.prob_hit))
        self._log_odds_miss = math.log(self.prob_miss / (1.0 - self.prob_miss))
        self._log_odds_clamp = math.log(self.clamping_threshold / (1.0 - self.clamping_threshold))
        self._log_odds_occupied = math.log(0.6 / 0.4)  # > this → occupied

        self.size_xy_m = max(20.0, float(size_xy_m))
        self.size_z_m = max(10.0, float(size_z_m))

        self.cols = int(math.ceil(self.size_xy_m / self.resolution))
        self.rows = self.cols
        self.layers = int(math.ceil(self.size_z_m / self.resolution))

        # Center at origin initially; caller should recenter
        self.center_x = 0.0
        self.center_y = 0.0
        self.center_z = 0.0
        self.origin_x = self.center_x - self.size_xy_m / 2.0
        self.origin_y = self.center_y - self.size_xy_m / 2.0
        self.origin_z = self.center_z - self.size_z_m / 2.0

        # Log-odds: 0.0 = unknown (p=0.5)
        self._log_odds: np.ndarray = np.zeros(
            (self.cols, self.rows, self.layers), dtype=np.float32
        )
        self._last_decay_at: float = time.time()
        self._total_insertions: int = 0

    # ── world ↔ voxel ──────────────────────────────────────────────

    def _world_to_voxel(self, x: float, y: float, z: float) -> tuple[int, int, int] | None:
        col = int((float(x) - self.origin_x) / self.resolution)
        row = int((float(y) - self.origin_y) / self.resolution)
        layer = int((float(z) - self.origin_z) / self.resolution)
        if 0 <= col < self.cols and 0 <= row < self.rows and 0 <= layer < self.layers:
            return col, row, layer
        return None

    def voxel_center(self, col: int, row: int, layer: int) -> tuple[float, float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
            self.origin_z + (layer + 0.5) * self.resolution,
        )

    # ── insertion ──────────────────────────────────────────────────

    def insert_point_cloud(
        self,
        points_world: list[tuple[float, float, float]],
        sensor_origin: tuple[float, float, float] | None = None,
    ) -> int:
        """Insert a 3D point cloud (world coordinates) into the map.

        If *sensor_origin* is provided, performs ray-casting: voxels along
        each ray get a miss update, and endpoints get a hit update.
        Otherwise only hit updates are applied.

        Returns number of points processed.
        """
        sx, sy, sz = (
            (float(sensor_origin[0]), float(sensor_origin[1]), float(sensor_origin[2]))
            if sensor_origin is not None
            else (None, None, None)
        )

        count = 0
        for px, py, pz in points_world:
            x, y, z = float(px), float(py), float(pz)

            # Ray-cast miss updates
            if sx is not None:
                ray_len = math.hypot(x - sx, y - sy, z - sz)
                if ray_len > 0.01:
                    steps = max(1, int(ray_len / (self.resolution * 0.5)))
                    for step_i in range(steps):
                        t = step_i / steps
                        rx = sx + (x - sx) * t
                        ry = sy + (y - sy) * t
                        rz = sz + (z - sz) * t
                        idx = self._world_to_voxel(rx, ry, rz)
                        if idx is not None:
                            self._apply_miss(*idx)

            # Hit update at endpoint
            idx = self._world_to_voxel(x, y, z)
            if idx is not None:
                self._apply_hit(*idx)
                count += 1

        self._total_insertions += count
        return count

    def insert_lidar_points(
        self,
        points_world: list[tuple[float, float, float]],
        sensor_origin: tuple[float, float, float],
    ) -> int:
        """Insert LiDAR points with ray-casting from sensor origin."""
        return self.insert_point_cloud(points_world, sensor_origin)

    def _apply_hit(self, col: int, row: int, layer: int) -> None:
        """Apply a hit (occupied) update to a voxel."""
        val = float(self._log_odds[col, row, layer]) + self._log_odds_hit
        self._log_odds[col, row, layer] = np.clip(val, -self._log_odds_clamp, self._log_odds_clamp)

    def _apply_miss(self, col: int, row: int, layer: int) -> None:
        """Apply a miss (free) update to a voxel."""
        val = float(self._log_odds[col, row, layer]) + self._log_odds_miss
        self._log_odds[col, row, layer] = np.clip(val, -self._log_odds_clamp, self._log_odds_clamp)

    # ── queries ────────────────────────────────────────────────────

    def get_log_odds(self, x: float, y: float, z: float) -> float:
        """Return raw log-odds at world position (0.0 = unknown)."""
        idx = self._world_to_voxel(x, y, z)
        if idx is None:
            return 0.0
        return float(self._log_odds[idx])

    def get_occupancy(self, x: float, y: float, z: float) -> float:
        """Return occupancy probability (0..1) at world position.
        Unknown voxels return 0.5."""
        L = self.get_log_odds(x, y, z)
        if L == 0.0:
            return 0.5
        return 1.0 - 1.0 / (1.0 + math.exp(L))

    def is_occupied(self, x: float, y: float, z: float) -> bool:
        """True if the voxel at (x,y,z) is confirmed occupied (p >= 0.6)."""
        return self.get_log_odds(x, y, z) >= self._log_odds_occupied

    def is_unknown(self, x: float, y: float, z: float) -> bool:
        """True if the voxel at (x,y,z) has never been observed."""
        idx = self._world_to_voxel(x, y, z)
        if idx is None:
            return True
        return float(self._log_odds[idx]) == 0.0

    def is_free(self, x: float, y: float, z: float) -> bool:
        """True if the voxel at (x,y,z) is confirmed free."""
        L = self.get_log_odds(x, y, z)
        return L <= -self._log_odds_occupied

    # ── bulk extraction for planning ───────────────────────────────

    def extract_planning_grid(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        z_min: float,
        z_max: float,
        resolution: float | None = None,
    ) -> tuple[np.ndarray, tuple[float, float, float]]:
        """Extract a dense 3D occupancy-probability grid for path planning.

        Returns (occupancy_grid_3d, (ox, oy, oz)) where:
        - occupancy_grid_3d[i,j,k] = probability (0..1)
        - (ox, oy, oz) = world origin of grid[0,0,0]

        Unknown voxels → 0.5, Free → ~0.3, Occupied → ~0.7+

        Uses vectorized numpy indexing — O(1) Python calls regardless of grid size.
        """
        res = float(resolution) if resolution is not None else self.resolution
        cols = int(math.ceil((x_max - x_min) / res))
        rows = int(math.ceil((y_max - y_min) / res))
        layers = int(math.ceil((z_max - z_min) / res))

        if cols <= 0 or rows <= 0 or layers <= 0:
            return np.zeros((1, 1, 1), dtype=np.float32), (x_min, y_min, z_min)

        # World coordinates of each planning-grid cell centre
        wx = np.arange(cols, dtype=np.float64) * res + x_min + res * 0.5
        wy = np.arange(rows, dtype=np.float64) * res + y_min + res * 0.5
        wz = np.arange(layers, dtype=np.float64) * res + z_min + res * 0.5

        # Octomap voxel indices (1D per axis → broadcast to 3D)
        oc = ((wx - self.origin_x) / self.resolution).astype(np.int32)
        or_ = ((wy - self.origin_y) / self.resolution).astype(np.int32)
        ol = ((wz - self.origin_z) / self.resolution).astype(np.int32)

        # Broadcast to 3D
        oc_3d = oc[:, None, None]
        or_3d = or_[None, :, None]
        ol_3d = ol[None, None, :]

        valid = (
            (oc_3d >= 0) & (oc_3d < self.cols) &
            (or_3d >= 0) & (or_3d < self.rows) &
            (ol_3d >= 0) & (ol_3d < self.layers)
        )

        oc_safe = np.clip(oc_3d, 0, self.cols - 1)
        or_safe = np.clip(or_3d, 0, self.rows - 1)
        ol_safe = np.clip(ol_3d, 0, self.layers - 1)

        # Vectorised lookup: extract log-odds for all cells at once
        lo = np.where(valid, self._log_odds[oc_safe, or_safe, ol_safe], 0.0)

        # log-odds → occupancy probability;  L==0 → p=0.5 (unknown)
        grid = np.full((cols, rows, layers), 0.5, dtype=np.float32)
        nonzero = lo != 0.0
        grid[nonzero] = 1.0 - 1.0 / (1.0 + np.exp(lo[nonzero].astype(np.float64)))

        return grid, (x_min, y_min, z_min)

    def get_occupied_voxels(
        self,
        x_range: tuple[float, float] | None = None,
        y_range: tuple[float, float] | None = None,
        z_range: tuple[float, float] | None = None,
    ) -> list[tuple[float, float, float]]:
        """Return world coordinates of all occupied voxels, optionally filtered."""
        mask = self._log_odds >= self._log_odds_occupied
        indices = np.argwhere(mask)
        voxels: list[tuple[float, float, float]] = []
        for ci, rj, lk in indices:
            wx = self.origin_x + (int(ci) + 0.5) * self.resolution
            wy = self.origin_y + (int(rj) + 0.5) * self.resolution
            wz = self.origin_z + (int(lk) + 0.5) * self.resolution
            if x_range and not (x_range[0] <= wx <= x_range[1]):
                continue
            if y_range and not (y_range[0] <= wy <= y_range[1]):
                continue
            if z_range and not (z_range[0] <= wz <= z_range[1]):
                continue
            voxels.append((round(wx, 2), round(wy, 2), round(wz, 2)))
        return voxels

    @property
    def occupied_count(self) -> int:
        return int((self._log_odds >= self._log_odds_occupied).sum())

    # ── maintenance ────────────────────────────────────────────────

    def decay_map(self, factor: float | None = None) -> None:
        """Pull all log-odds toward 0 (unknown) by multiplying with *factor*."""
        f = float(factor) if factor is not None else self.decay_factor
        self._log_odds *= f
        self._last_decay_at = time.time()

    def seconds_since_decay(self) -> float:
        return time.time() - self._last_decay_at

    def recenter(self, new_cx: float, new_cy: float, new_cz: float) -> None:
        """Recenter the grid around a new position, preserving overlapping data."""
        old_ox, old_oy, old_oz = self.origin_x, self.origin_y, self.origin_z

        self.center_x = float(new_cx)
        self.center_y = float(new_cy)
        self.center_z = float(new_cz)
        new_ox = self.center_x - self.size_xy_m / 2.0
        new_oy = self.center_y - self.size_xy_m / 2.0
        new_oz = self.center_z - self.size_z_m / 2.0

        dc = int(round((new_ox - old_ox) / self.resolution))
        dr = int(round((new_oy - old_oy) / self.resolution))
        dl = int(round((new_oz - old_oz) / self.resolution))

        if dc == 0 and dr == 0 and dl == 0:
            self.origin_x = new_ox
            self.origin_y = new_oy
            self.origin_z = new_oz
            return

        new_lo = np.zeros((self.cols, self.rows, self.layers), dtype=np.float32)
        sc0, sc1 = max(0, dc), min(self.cols, self.cols + dc)
        sr0, sr1 = max(0, dr), min(self.rows, self.rows + dr)
        sl0, sl1 = max(0, dl), min(self.layers, self.layers + dl)

        dc0, dc1 = sc0 - dc, sc1 - dc
        dr0, dr1 = sr0 - dr, sr1 - dr
        dl0, dl1 = sl0 - dl, sl1 - dl

        if sc1 > sc0 and sr1 > sr0 and sl1 > sl0:
            new_lo[dc0:dc1, dr0:dr1, dl0:dl1] = self._log_odds[sc0:sc1, sr0:sr1, sl0:sl1]

        self._log_odds = new_lo
        self.origin_x = new_ox
        self.origin_y = new_oy
        self.origin_z = new_oz

    def should_recenter(self, x: float, y: float, z: float, margin: float = 0.3) -> bool:
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
        self._log_odds.fill(0.0)
        self._total_insertions = 0

    def stats(self) -> dict[str, object]:
        occ = int((self._log_odds >= self._log_odds_occupied).sum())
        free = int((self._log_odds <= -self._log_odds_occupied).sum())
        unknown = int(self._log_odds.size) - occ - free
        return {
            "dimensions": f"{self.cols}x{self.rows}x{self.layers}",
            "total_voxels": int(self._log_odds.size),
            "resolution_m": self.resolution,
            "occupied_voxels": occ,
            "free_voxels": free,
            "unknown_voxels": unknown,
            "total_insertions": self._total_insertions,
            "prob_hit": self.prob_hit,
            "prob_miss": self.prob_miss,
            "clamping_threshold": self.clamping_threshold,
            "decay_factor": self.decay_factor,
        }

    def to_api(self) -> dict[str, object]:
        return self.stats()
