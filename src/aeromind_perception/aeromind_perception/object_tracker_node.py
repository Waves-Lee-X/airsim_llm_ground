#!/usr/bin/env python3
"""Track fused semantic observations and publish persistent world objects."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node

from aeromind_interfaces.msg import (
    SemanticObject,
    SemanticObjectArray,
    WorldModelHealth,
)
from .object_tracking import ConstantVelocityTrack, greedy_association


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("object_tracker_node")
        self.declare_parameter("observations_topic", "/world_model/observations")
        self.declare_parameter("objects_topic", "/world_model/objects")
        self.declare_parameter("association_distance_m", 2.0)
        self.declare_parameter("minimum_confirmed_hits", 3)
        self.declare_parameter("lost_after_misses", 3)
        self.declare_parameter("track_timeout_sec", 5.0)
        self.declare_parameter("dynamic_speed_mps", 0.35)
        self.declare_parameter("measurement_variance_m2", 0.25)
        self.declare_parameter("acceleration_variance", 1.0)

        self._lock = threading.RLock()
        self._tracks = {}
        self._next_track = 1
        self._frame_id = ""
        self._unlocated_observations = []
        self._last_observation_monotonic = None
        self._fusion_health = None
        self._objects_publisher = self.create_publisher(
            SemanticObjectArray, str(self.get_parameter("objects_topic").value), 10
        )
        self._health_publisher = self.create_publisher(
            WorldModelHealth, "/world_model/health", 10
        )
        self.create_subscription(
            SemanticObjectArray,
            str(self.get_parameter("observations_topic").value),
            self._observations_callback,
            10,
        )
        self.create_subscription(
            WorldModelHealth,
            "/world_model/fusion_health",
            self._fusion_health_callback,
            10,
        )
        self.create_timer(0.5, self._timer_callback)
        self.get_logger().info("三维语义对象跟踪节点已启动")

    def _fusion_health_callback(self, msg: WorldModelHealth):
        with self._lock:
            self._fusion_health = msg

    def _observations_callback(self, msg: SemanticObjectArray):
        timestamp = _stamp_seconds(msg.header.stamp) or time.time()
        fallback_variance = float(
            self.get_parameter("measurement_variance_m2").value
        )
        observations = [
            _observation_dict(item, fallback_variance) for item in msg.objects
        ]
        valid_observations = [item for item in observations if item["position_valid"]]
        with self._lock:
            self._frame_id = msg.header.frame_id
            self._unlocated_observations = [
                item for item in observations if not item["position_valid"]
            ]
            self._last_observation_monotonic = time.monotonic()
            acceleration_variance = float(
                self.get_parameter("acceleration_variance").value
            )
            for track in self._tracks.values():
                track.predict(timestamp, acceleration_variance)
            matches = greedy_association(
                valid_observations,
                self._tracks,
                float(self.get_parameter("association_distance_m").value),
            )
            matched_track_ids = set()
            for index, observation in enumerate(valid_observations):
                track_id = matches.get(index)
                if track_id is None:
                    track_id = self._new_track(observation, timestamp)
                else:
                    track = self._tracks[track_id]
                    track.update(
                        observation["position"],
                        observation["confidence"],
                        timestamp,
                        observation["measurement_variance"],
                    )
                    _copy_evidence(track, observation)
                matched_track_ids.add(track_id)
            for track_id, track in self._tracks.items():
                if track_id not in matched_track_ids:
                    track.mark_missed(timestamp)
            self._remove_expired(timestamp)
        self._publish(timestamp)

    def _new_track(self, observation, timestamp: float):
        track_id = f"{_safe_id(observation['class_name'])}_{self._next_track:04d}"
        self._next_track += 1
        track = ConstantVelocityTrack(
            track_id,
            observation["class_name"],
            observation["position"],
            observation["confidence"],
            timestamp,
            observation["measurement_variance"],
        )
        _copy_evidence(track, observation)
        self._tracks[track_id] = track
        return track_id

    def _remove_expired(self, timestamp: float):
        timeout = float(self.get_parameter("track_timeout_sec").value)
        self._tracks = {
            track_id: track
            for track_id, track in self._tracks.items()
            if timestamp - track.last_seen <= timeout
        }

    def _timer_callback(self):
        with self._lock:
            if self._last_observation_monotonic is None:
                self._publish_health(0, 0)
                return
            age = time.monotonic() - self._last_observation_monotonic
            if age > float(self.get_parameter("track_timeout_sec").value):
                self._tracks.clear()
            track_count = len(self._tracks)
            confirmed = sum(
                self._lifecycle(track) == "confirmed"
                for track in self._tracks.values()
            )
        self._publish_health(track_count, confirmed)

    def _lifecycle(self, track):
        return track.lifecycle(
            int(self.get_parameter("minimum_confirmed_hits").value),
            int(self.get_parameter("lost_after_misses").value),
        )

    def _publish(self, timestamp: float):
        message = SemanticObjectArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        with self._lock:
            tracks = list(self._tracks.values())
            unlocated = list(self._unlocated_observations)
        for track in tracks:
            item = SemanticObject()
            item.id = track.id
            item.class_name = track.class_name
            item.confidence = float(track.confidence)
            item.position_valid = True
            item.position.x, item.position.y, item.position.z = track.position
            item.velocity.x, item.velocity.y, item.velocity.z = track.velocity
            item.position_covariance = list(track.position_covariance)
            item.dynamic = math.sqrt(sum(value * value for value in track.velocity)) >= float(
                self.get_parameter("dynamic_speed_mps").value
            )
            item.depth_m = float(track.depth_m)
            item.age_sec = float(max(0.0, timestamp - track.last_seen))
            item.state = self._lifecycle(track)
            item.evidence_type = "fused_tracked"
            item.sources = list(track.sources)
            item.hit_count = int(track.hit_count)
            item.miss_count = int(track.miss_count)
            message.objects.append(item)
        for index, observation in enumerate(unlocated):
            item = SemanticObject()
            item.id = f"observed_2d_{_safe_id(observation['class_name'])}_{index:04d}"
            item.class_name = observation["class_name"]
            item.confidence = float(observation["confidence"])
            item.position_valid = False
            item.dynamic = False
            item.depth_m = float("nan")
            item.age_sec = 0.0
            item.state = "observed_2d"
            item.evidence_type = "observed_2d"
            item.sources = list(observation["sources"])
            item.hit_count = 1
            item.miss_count = 0
            message.objects.append(item)
        self._objects_publisher.publish(message)
        confirmed = sum(item.state == "confirmed" for item in message.objects)
        self._publish_health(len(message.objects), confirmed)

    def _publish_health(self, track_count: int, confirmed_count: int):
        health = WorldModelHealth()
        health.header.stamp = self.get_clock().now().to_msg()
        health.header.frame_id = self._frame_id
        with self._lock:
            source = self._fusion_health
            observation_age = (
                time.monotonic() - self._last_observation_monotonic
                if self._last_observation_monotonic is not None
                else float("inf")
            )
        if source is not None:
            health.detections_fresh = source.detections_fresh
            health.depth_fresh = source.depth_fresh
            health.camera_info_valid = source.camera_info_valid
            health.tf_available = source.tf_available
            health.sync_delta_sec = source.sync_delta_sec
            health.observation_count = source.observation_count
        health.track_count = int(track_count)
        health.confirmed_track_count = int(confirmed_count)
        health.healthy = bool(source and source.healthy and observation_age <= 2.0)
        health.state = "TRACKING" if health.healthy else "DEGRADED"
        health.message = (
            f"tracks={track_count}, confirmed={confirmed_count}, "
            f"observation_age={observation_age:.2f}s"
        )
        self._health_publisher.publish(health)


def _observation_dict(item, fallback_variance: float):
    return {
        "class_name": item.class_name,
        "confidence": float(item.confidence),
        "position_valid": bool(item.position_valid),
        "position": (float(item.position.x), float(item.position.y), float(item.position.z)),
        "depth_m": float(item.depth_m),
        "sources": list(item.sources),
        "measurement_variance": _measurement_variance(
            item.position_covariance, fallback_variance
        ),
    }


def _copy_evidence(track, observation):
    track.sources = list(observation.get("sources") or [])
    track.depth_m = float(observation.get("depth_m", float("nan")))


def _measurement_variance(covariance, fallback: float):
    values = list(covariance)
    diagonal = [float(values[index]) for index in (0, 4, 8)] if len(values) == 9 else []
    valid = [value for value in diagonal if math.isfinite(value) and value > 0.0]
    return sum(valid) / len(valid) if valid else float(fallback)


def _safe_id(value: str):
    normalized = "".join(char.lower() if char.isalnum() else "_" for char in value)
    return normalized.strip("_") or "object"


def _stamp_seconds(stamp):
    value = float(stamp.sec) + float(stamp.nanosec) / 1e9
    return value if value > 0.0 else None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
