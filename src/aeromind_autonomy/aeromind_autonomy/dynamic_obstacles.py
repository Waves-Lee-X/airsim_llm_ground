"""Time-aligned dynamic obstacle predictions for local trajectory scoring."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class DynamicObstacle:
    object_id: str
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    radius: float
    confidence: float
    observed_at: float


@dataclass(frozen=True)
class DynamicClearance:
    clearance: float
    object_id: str = ""
    time_to_closest_sec: float = float("inf")
    confidence: float = 0.0


class DynamicObstacleField:
    """Store a fresh world-model snapshot and score trajectories in space-time."""

    def __init__(
        self,
        *,
        timeout_sec: float = 1.5,
        default_radius: float = 0.45,
        uncertainty_sigma: float = 2.0,
        minimum_confidence: float = 0.45,
        classes: tuple[str, ...] = ("person",),
    ):
        self.timeout_sec = max(0.1, float(timeout_sec))
        self.default_radius = max(0.05, float(default_radius))
        self.uncertainty_sigma = max(0.0, float(uncertainty_sigma))
        self.minimum_confidence = max(0.0, min(1.0, float(minimum_confidence)))
        self.classes = {str(value).strip().lower() for value in classes if str(value).strip()}
        self._obstacles: tuple[DynamicObstacle, ...] = ()
        self._updated_at: float | None = None
        self._lock = threading.RLock()

    def update(self, records: list[dict], *, now: float | None = None) -> None:
        timestamp = time.monotonic() if now is None else float(now)
        obstacles = []
        for record in records:
            if str(record.get("class_name", "")).strip().lower() not in self.classes:
                continue
            confidence = float(record.get("confidence", 0.0))
            if confidence < self.minimum_confidence:
                continue
            position = _finite_vector(record.get("position"))
            velocity = _finite_vector(record.get("velocity"), default=(0.0, 0.0, 0.0))
            if position is None or velocity is None:
                continue
            size = _finite_vector(record.get("size"), default=(0.0, 0.0, 0.0))
            covariance = record.get("position_covariance") or ()
            variance = 0.0
            if len(covariance) >= 9:
                variance = max(
                    0.0,
                    (float(covariance[0]) + float(covariance[4]) + float(covariance[8])) / 3.0,
                )
            measured_radius = max(size or (0.0, 0.0, 0.0)) * 0.5
            radius = max(self.default_radius, measured_radius) + self.uncertainty_sigma * math.sqrt(variance)
            obstacles.append(
                DynamicObstacle(
                    object_id=str(record.get("id") or "dynamic_object"),
                    position=position,
                    velocity=velocity,
                    radius=radius,
                    confidence=confidence,
                    observed_at=timestamp,
                )
            )
        with self._lock:
            self._obstacles = tuple(obstacles)
            self._updated_at = timestamp

    def clear(self) -> None:
        with self._lock:
            self._obstacles = ()
            self._updated_at = None

    def fresh(self, *, now: float | None = None) -> bool:
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            return bool(
                self._updated_at is not None
                and timestamp - self._updated_at <= self.timeout_sec
            )

    def trajectory_clearance(
        self,
        samples: list[dict],
        *,
        trajectory_elapsed: float = 0.0,
        now: float | None = None,
        max_radius: float = 8.0,
    ) -> DynamicClearance:
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            updated_at = self._updated_at
            obstacles = self._obstacles
        if updated_at is None or timestamp - updated_at > self.timeout_sec or not obstacles:
            return DynamicClearance(float(max_radius))

        latency = max(0.0, timestamp - updated_at)
        best = DynamicClearance(float(max_radius))
        for sample in samples:
            position = _finite_vector(sample.get("position"))
            if position is None:
                continue
            sample_time = max(
                0.0,
                float(sample.get("t", 0.0)) - float(trajectory_elapsed),
            )
            prediction_time = latency + sample_time
            for obstacle in obstacles:
                predicted = tuple(
                    obstacle.position[axis] + obstacle.velocity[axis] * prediction_time
                    for axis in range(3)
                )
                clearance = max(0.0, math.dist(position, predicted) - obstacle.radius)
                if clearance < best.clearance:
                    best = DynamicClearance(
                        clearance=clearance,
                        object_id=obstacle.object_id,
                        time_to_closest_sec=sample_time,
                        confidence=obstacle.confidence,
                    )
        return best

    def current_clearance(
        self,
        position: tuple[float, float, float],
        *,
        now: float | None = None,
        max_radius: float = 12.0,
    ) -> DynamicClearance:
        return self.trajectory_clearance(
            [{"t": 0.0, "position": position}],
            now=now,
            max_radius=max_radius,
        )


def _finite_vector(value, default=None):
    if value is None:
        return default
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return default
    if len(vector) != 3 or not all(math.isfinite(item) for item in vector):
        return default
    return vector
