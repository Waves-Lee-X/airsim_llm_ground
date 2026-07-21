"""Constant-velocity 3D tracking primitives for the semantic world model."""

from __future__ import annotations

import math

import numpy as np


class ConstantVelocityTrack:
    """A six-state Kalman track with an explicit observation lifecycle."""

    def __init__(
        self,
        track_id: str,
        class_name: str,
        position,
        confidence: float,
        timestamp: float,
        measurement_variance: float = 0.25,
    ):
        self.id = str(track_id)
        self.class_name = str(class_name)
        self.state = np.zeros(6, dtype=np.float64)
        self.state[:3] = np.asarray(position, dtype=np.float64)
        self.covariance = np.diag(
            [measurement_variance] * 3 + [4.0, 4.0, 4.0]
        ).astype(np.float64)
        self.confidence = float(confidence)
        self.first_seen = float(timestamp)
        self.last_seen = float(timestamp)
        self.last_update = float(timestamp)
        self.hit_count = 1
        self.miss_count = 0
        self.sources = []
        self.depth_m = float("nan")

    def predict(self, timestamp: float, acceleration_variance: float = 1.0):
        dt = max(0.0, min(2.0, float(timestamp) - self.last_update))
        if dt <= 0.0:
            return
        transition = np.eye(6, dtype=np.float64)
        transition[0, 3] = dt
        transition[1, 4] = dt
        transition[2, 5] = dt
        block = np.array(
            [[0.25 * dt**4, 0.5 * dt**3], [0.5 * dt**3, dt**2]],
            dtype=np.float64,
        ) * float(acceleration_variance)
        process = np.zeros((6, 6), dtype=np.float64)
        for axis in range(3):
            process[axis, axis] = block[0, 0]
            process[axis, axis + 3] = block[0, 1]
            process[axis + 3, axis] = block[1, 0]
            process[axis + 3, axis + 3] = block[1, 1]
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        self.last_update = float(timestamp)

    def update(
        self,
        position,
        confidence: float,
        timestamp: float,
        measurement_variance: float = 0.25,
    ):
        self.predict(timestamp)
        observation = np.asarray(position, dtype=np.float64)
        measurement = np.zeros((3, 6), dtype=np.float64)
        measurement[:, :3] = np.eye(3)
        noise = np.eye(3, dtype=np.float64) * float(measurement_variance)
        innovation = observation - measurement @ self.state
        innovation_covariance = measurement @ self.covariance @ measurement.T + noise
        gain = self.covariance @ measurement.T @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        identity = np.eye(6, dtype=np.float64)
        self.covariance = (identity - gain @ measurement) @ self.covariance
        self.confidence = 0.7 * self.confidence + 0.3 * float(confidence)
        self.last_seen = float(timestamp)
        self.hit_count += 1
        self.miss_count = 0

    def mark_missed(self, timestamp: float):
        self.predict(timestamp)
        self.miss_count += 1

    @property
    def position(self):
        return tuple(float(value) for value in self.state[:3])

    @property
    def velocity(self):
        return tuple(float(value) for value in self.state[3:])

    @property
    def position_covariance(self):
        return tuple(float(value) for value in self.covariance[:3, :3].reshape(-1))

    def distance_to(self, position) -> float:
        return math.dist(self.position, tuple(float(value) for value in position))

    def lifecycle(self, minimum_hits: int, lost_after_misses: int) -> str:
        if self.miss_count >= int(lost_after_misses):
            return "lost"
        if self.hit_count >= int(minimum_hits):
            return "confirmed"
        return "tentative"


def greedy_association(observations, tracks, maximum_distance: float):
    """Associate same-class 3D observations to nearest tracks without reuse."""
    candidates = []
    for observation_index, observation in enumerate(observations):
        if not observation.get("position_valid"):
            continue
        for track_id, track in tracks.items():
            if track.class_name != observation.get("class_name"):
                continue
            distance = track.distance_to(observation["position"])
            if distance <= float(maximum_distance):
                candidates.append((distance, observation_index, track_id))
    matches = {}
    used_tracks = set()
    for _distance, observation_index, track_id in sorted(candidates):
        if observation_index in matches or track_id in used_tracks:
            continue
        matches[observation_index] = track_id
        used_tracks.add(track_id)
    return matches
