"""Rolling world-frame voxel map with ESDF-style clearance queries."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field


VoxelKey = tuple[int, int, int]
Point3 = tuple[float, float, float]


@dataclass
class LocalEsdfMap:
    resolution: float = 0.35
    max_points: int = 12000
    rolling_radius: float = 24.0
    decay_sec: float = 4.0
    occupied: set[VoxelKey] = field(default_factory=set)
    _last_seen: dict[VoxelKey, float] = field(default_factory=dict)

    def clear(self):
        self.occupied.clear()
        self._last_seen.clear()

    def insert_points(
        self,
        points: list[Point3],
        *,
        stamp: float | None = None,
        sensor_origin: Point3 | None = None,
    ):
        """Insert a scan in the map frame and clear visible ray interiors."""
        now = time.monotonic() if stamp is None else float(stamp)
        finite = [point for point in points if all(math.isfinite(v) for v in point)]
        stride = max(1, len(finite) // self.max_points) if finite else 1
        selected = finite[::stride]

        if sensor_origin is not None:
            for endpoint in selected:
                self._clear_ray(sensor_origin, endpoint)

        for point in selected:
            key = self._key(*point)
            self.occupied.add(key)
            self._last_seen[key] = now
        self.prune(sensor_origin, stamp=now)

    def prune(self, center: Point3 | None, *, stamp: float | None = None):
        now = time.monotonic() if stamp is None else float(stamp)
        radius_sq = self.rolling_radius * self.rolling_radius
        remove = []
        for key in self.occupied:
            expired = now - self._last_seen.get(key, now) > self.decay_sec
            outside = False
            if center is not None:
                point = self._center(key)
                outside = sum((point[i] - center[i]) ** 2 for i in range(3)) > radius_sq
            if expired or outside:
                remove.append(key)
        for key in remove:
            self.occupied.discard(key)
            self._last_seen.pop(key, None)

    def clearance(self, point: Point3, max_radius: float = 8.0) -> float:
        if not self.occupied:
            return max_radius
        px, py, pz = point
        radius_cells = max(1, math.ceil(max_radius / self.resolution))
        cx, cy, cz = self._key(px, py, pz)
        best_sq = max_radius * max_radius
        for ix in range(cx - radius_cells, cx + radius_cells + 1):
            for iy in range(cy - radius_cells, cy + radius_cells + 1):
                for iz in range(cz - radius_cells, cz + radius_cells + 1):
                    key = (ix, iy, iz)
                    if key not in self.occupied:
                        continue
                    ox, oy, oz = self._center(key)
                    dist_sq = (px - ox) ** 2 + (py - oy) ** 2 + (pz - oz) ** 2
                    if dist_sq < best_sq:
                        best_sq = dist_sq
        return math.sqrt(best_sq)

    def trajectory_clearance(
        self,
        samples: list[Point3],
        max_radius: float = 4.0,
    ) -> float:
        if not samples:
            return 0.0
        return min(self.clearance(sample, max_radius) for sample in samples)

    def nearest_obstacle(self, position: Point3, max_radius: float = 12.0) -> float:
        if not self.occupied:
            return max_radius
        best_sq = max_radius * max_radius
        for key in self.occupied:
            point = self._center(key)
            dist_sq = sum((point[i] - position[i]) ** 2 for i in range(3))
            if dist_sq < best_sq:
                best_sq = dist_sq
        return math.sqrt(best_sq)

    def nearest_ahead(self, max_radius: float = 12.0) -> float:
        """Compatibility query for callers still using a body-local map."""
        return self.nearest_obstacle((0.0, 0.0, 0.0), max_radius)

    def points(self) -> list[Point3]:
        return [self._center(key) for key in self.occupied]

    def _clear_ray(self, origin: Point3, endpoint: Point3):
        distance = math.dist(origin, endpoint)
        if distance <= self.resolution:
            return
        count = max(1, int(distance / self.resolution))
        endpoint_key = self._key(*endpoint)
        for index in range(count):
            ratio = index / count
            point = tuple(
                origin[axis] + (endpoint[axis] - origin[axis]) * ratio
                for axis in range(3)
            )
            key = self._key(*point)
            if key == endpoint_key:
                continue
            self.occupied.discard(key)
            self._last_seen.pop(key, None)

    def _key(self, x: float, y: float, z: float) -> VoxelKey:
        return (
            math.floor(x / self.resolution),
            math.floor(y / self.resolution),
            math.floor(z / self.resolution),
        )

    def _center(self, key: VoxelKey) -> Point3:
        return tuple((value + 0.5) * self.resolution for value in key)
