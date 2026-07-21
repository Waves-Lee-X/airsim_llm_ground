"""Geometry and hysteresis for semantic object-to-path relationships."""

from __future__ import annotations

import math


def point_to_segment_distance(point, start, end):
    point = tuple(float(value) for value in point)
    start = tuple(float(value) for value in start)
    end = tuple(float(value) for value in end)
    direction = tuple(end[index] - start[index] for index in range(3))
    length_squared = sum(value * value for value in direction)
    if length_squared <= 1e-12:
        return math.dist(point, start)
    projection = sum(
        (point[index] - start[index]) * direction[index] for index in range(3)
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = tuple(start[index] + projection * direction[index] for index in range(3))
    return math.dist(point, closest)


def point_to_path_distance(point, path_points):
    points = [tuple(float(value) for value in item) for item in path_points]
    if not points:
        return float("inf")
    if len(points) == 1:
        return math.dist(tuple(point), points[0])
    return min(
        point_to_segment_distance(point, points[index], points[index + 1])
        for index in range(len(points) - 1)
    )


def predicted_path_distance(
    position,
    velocity,
    path_points,
    horizon_sec: float = 2.0,
    sample_period_sec: float = 0.5,
):
    horizon = max(0.0, float(horizon_sec))
    period = max(0.05, float(sample_period_sec))
    sample_count = max(1, int(math.ceil(horizon / period)))
    best_distance = float("inf")
    best_time = 0.0
    for index in range(sample_count + 1):
        time_offset = min(horizon, index * period)
        predicted = tuple(
            float(position[axis]) + float(velocity[axis]) * time_offset
            for axis in range(3)
        )
        distance = point_to_path_distance(predicted, path_points)
        if distance < best_distance:
            best_distance = distance
            best_time = time_offset
    return best_distance, best_time


class PathIntrusionMonitor:
    """Convert noisy per-frame path intersections into entered/cleared edges."""

    def __init__(self, confirm_frames: int = 3, clear_frames: int = 3):
        self.confirm_frames = max(1, int(confirm_frames))
        self.clear_frames = max(1, int(clear_frames))
        self._hits = {}
        self._misses = {}
        self._active = set()

    @property
    def active_ids(self):
        return set(self._active)

    def update(self, intruding_ids):
        observed = set(str(value) for value in intruding_ids)
        entered = []
        cleared = []
        known = set(self._hits) | set(self._misses) | self._active | observed
        for object_id in known:
            if object_id in observed:
                self._hits[object_id] = self._hits.get(object_id, 0) + 1
                self._misses[object_id] = 0
                if (
                    object_id not in self._active
                    and self._hits[object_id] >= self.confirm_frames
                ):
                    self._active.add(object_id)
                    entered.append(object_id)
            else:
                self._hits[object_id] = 0
                self._misses[object_id] = self._misses.get(object_id, 0) + 1
                if (
                    object_id in self._active
                    and self._misses[object_id] >= self.clear_frames
                ):
                    self._active.remove(object_id)
                    cleared.append(object_id)
                if object_id not in self._active and self._misses[object_id] >= self.clear_frames:
                    self._hits.pop(object_id, None)
                    self._misses.pop(object_id, None)
        return {"entered": sorted(entered), "cleared": sorted(cleared), "active": sorted(self._active)}

    def reset(self):
        cleared = sorted(self._active)
        self._hits.clear()
        self._misses.clear()
        self._active.clear()
        return cleared
