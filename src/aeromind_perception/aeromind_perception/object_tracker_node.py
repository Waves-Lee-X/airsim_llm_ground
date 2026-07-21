#!/usr/bin/env python3
"""Track fused semantic observations and publish persistent world objects."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from aeromind_interfaces.msg import (
    SemanticRelation,
    SemanticRelationArray,
    SemanticObject,
    SemanticObjectArray,
    Trajectory,
    WorldModelEvent,
    WorldModelHealth,
)
from aeromind_interfaces.srv import QueryWorldModel
from .object_tracking import ConstantVelocityTrack, greedy_association
from .semantic_relations import PathIntrusionMonitor, predicted_path_distance
from .semantic_geometry import transform_point
from .world_model_store import WorldModelStore


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
        self.declare_parameter("history_enabled", True)
        self.declare_parameter("history_db_path", "~/.aeromind/world_model.db")
        self.declare_parameter("history_retention_hours", 24.0)
        self.declare_parameter("history_max_rows", 200000)
        self.declare_parameter("history_write_interval_sec", 1.0)
        self.declare_parameter("path_intrusion_classes", ["person"])
        self.declare_parameter("path_intrusion_radius_m", 2.0)
        self.declare_parameter("path_prediction_horizon_sec", 2.0)
        self.declare_parameter("path_prediction_period_sec", 0.5)
        self.declare_parameter("path_uncertainty_sigma", 1.0)
        self.declare_parameter("path_confirm_frames", 3)
        self.declare_parameter("path_clear_frames", 3)
        self.declare_parameter("trajectory_timeout_sec", 3.0)

        self._lock = threading.RLock()
        self._tracks = {}
        self._next_track = 1
        self._frame_id = ""
        self._unlocated_observations = []
        self._last_observation_monotonic = None
        self._fusion_health = None
        self._poses_by_frame = {}
        self._store = self._create_store()
        self._last_history_write_at = None
        self._trajectory = None
        self._intrusion_monitor = PathIntrusionMonitor(
            int(self.get_parameter("path_confirm_frames").value),
            int(self.get_parameter("path_clear_frames").value),
        )
        self._intrusion_evidence = {}
        self._event_sequence = 1
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._objects_publisher = self.create_publisher(
            SemanticObjectArray, str(self.get_parameter("objects_topic").value), 10
        )
        self._health_publisher = self.create_publisher(
            WorldModelHealth, "/world_model/health", 10
        )
        self._relations_publisher = self.create_publisher(
            SemanticRelationArray, "/world_model/relations", 10
        )
        self._events_publisher = self.create_publisher(
            WorldModelEvent, "/world_model/events", 10
        )
        self.create_subscription(
            SemanticObjectArray,
            str(self.get_parameter("observations_topic").value),
            self._observations_callback,
            10,
        )
        self.create_subscription(
            Odometry, "/sensor/odometry", self._odometry_callback, 10
        )
        self.create_subscription(
            Odometry, "/localization/odometry", self._odometry_callback, 10
        )
        self.create_subscription(
            Trajectory, "/autonomy/trajectory", self._trajectory_callback, 10
        )
        self.create_service(
            QueryWorldModel, "/world_model/query", self._query_callback
        )
        self.create_subscription(
            WorldModelHealth,
            "/world_model/fusion_health",
            self._fusion_health_callback,
            10,
        )
        self.create_timer(0.5, self._timer_callback)
        self.get_logger().info("三维语义对象跟踪节点已启动")

    def _create_store(self):
        if not bool(self.get_parameter("history_enabled").value):
            return None
        try:
            store = WorldModelStore(
                str(self.get_parameter("history_db_path").value),
                float(self.get_parameter("history_retention_hours").value),
                int(self.get_parameter("history_max_rows").value),
            )
            self.get_logger().info(f"世界对象历史库: {store.path}")
            return store
        except Exception as exc:
            self.get_logger().error(f"世界对象历史库初始化失败，继续无持久化运行: {exc}")
            return None

    def _odometry_callback(self, msg: Odometry):
        frame_id = str(msg.header.frame_id)
        if not frame_id:
            return
        position = msg.pose.pose.position
        with self._lock:
            self._poses_by_frame[frame_id] = {
                "position": (float(position.x), float(position.y), float(position.z)),
                "received_at": time.monotonic(),
            }

    def _trajectory_callback(self, msg: Trajectory):
        terminal_modes = {
            "safe_hover",
            "goal_reached",
            "hover_no_goal",
            "blocked_hold",
            "sensor_timeout_hold",
        }
        active = bool(
            msg.collision_free
            and len(msg.points) >= 2
            and str(msg.planner_mode) not in terminal_modes
        )
        if active:
            trajectory_id = str(msg.trajectory_id or msg.planner_mode)
            with self._lock:
                previous_id = (
                    self._trajectory.get("trajectory_id")
                    if self._trajectory else None
                )
            if previous_id and previous_id != trajectory_id:
                self._clear_trajectory("trajectory_replaced")
            with self._lock:
                self._trajectory = {
                    "frame_id": str(msg.header.frame_id),
                    "trajectory_id": trajectory_id,
                    "planner_mode": str(msg.planner_mode),
                    "stamp": msg.header.stamp,
                    "points": [
                        (
                            float(point.position.x),
                            float(point.position.y),
                            float(point.position.z),
                        )
                        for point in msg.points
                    ],
                    "received_at": time.monotonic(),
                }
            return
        self._clear_trajectory("trajectory_inactive")

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
        persist = bool(
            self._store is not None
            and (
                self._last_history_write_at is None
                or timestamp < self._last_history_write_at
                or timestamp - self._last_history_write_at
                >= float(self.get_parameter("history_write_interval_sec").value)
            )
        )
        if persist:
            self._last_history_write_at = timestamp
        self._publish(timestamp, persist=persist)

    def _new_track(self, observation, timestamp: float):
        if self._store is not None:
            try:
                track_id = self._store.allocate_track_id(observation["class_name"])
            except Exception as exc:
                self.get_logger().error(
                    f"持久 Track ID 分配失败，使用进程内 ID: {exc}",
                    throttle_duration_sec=5.0,
                )
                track_id = self._local_track_id(observation["class_name"])
        else:
            track_id = self._local_track_id(observation["class_name"])
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

    def _local_track_id(self, class_name: str):
        track_id = f"{_safe_id(class_name)}_{self._next_track:06d}"
        self._next_track += 1
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

    def _publish(self, timestamp: float, persist: bool = False):
        message = SemanticObjectArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        with self._lock:
            tracks = list(self._tracks.values())
            unlocated = list(self._unlocated_observations)
        records = []
        for track in tracks:
            item = self._track_message(track, timestamp)
            message.objects.append(item)
            records.append(_track_record(item))
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
        if persist and self._store is not None:
            try:
                self._store.record(self._frame_id, timestamp, records)
            except Exception as exc:
                self.get_logger().error(
                    f"写入世界对象历史失败: {exc}", throttle_duration_sec=5.0
                )
        confirmed = sum(item.state == "confirmed" for item in message.objects)
        self._publish_health(len(message.objects), confirmed)
        self._evaluate_path_relations(tracks)

    def _track_message(self, track, timestamp: float):
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
        return item

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

    def _evaluate_path_relations(self, tracks):
        with self._lock:
            trajectory = dict(self._trajectory) if self._trajectory else None
        if trajectory is None:
            return
        if (
            time.monotonic() - trajectory["received_at"]
            > float(self.get_parameter("trajectory_timeout_sec").value)
        ):
            self._clear_trajectory("trajectory_timeout")
            return
        path_points, transformed = self._trajectory_points_in_world(trajectory)
        if path_points is None:
            return
        classes = {
            _normalize_class_name(value)
            for value in self.get_parameter("path_intrusion_classes").value
        }
        radius = float(self.get_parameter("path_intrusion_radius_m").value)
        sigma = float(self.get_parameter("path_uncertainty_sigma").value)
        relations = SemanticRelationArray()
        relations.header.stamp = self.get_clock().now().to_msg()
        relations.header.frame_id = self._frame_id
        intruding = []
        evidence = {}
        track_by_id = {track.id: track for track in tracks}
        for track in tracks:
            if (
                self._lifecycle(track) != "confirmed"
                or _normalize_class_name(track.class_name) not in classes
            ):
                continue
            distance, prediction_time = predicted_path_distance(
                track.position,
                track.velocity,
                path_points,
                float(self.get_parameter("path_prediction_horizon_sec").value),
                float(self.get_parameter("path_prediction_period_sec").value),
            )
            covariance = track.position_covariance
            position_std = math.sqrt(
                max(0.0, covariance[0] + covariance[4] + covariance[8]) / 3.0
            )
            threshold = radius + sigma * position_std
            relation = SemanticRelation()
            relation.subject_id = track.id
            relation.predicate = (
                "intersects_path" if distance <= threshold else "near_path"
            )
            relation.object_id = trajectory["trajectory_id"]
            relation.confidence = float(track.confidence)
            relation.distance_m = float(distance)
            relation.evidence_type = "predicted" if prediction_time > 0.0 else "fused"
            relation.sources = [
                "/world_model/objects",
                "/autonomy/trajectory",
            ]
            if transformed:
                relation.sources.append("/tf")
            relations.relations.append(relation)
            if distance <= threshold:
                intruding.append(track.id)
                evidence[track.id] = {
                    "object_id": track.id,
                    "class_name": track.class_name,
                    "trajectory_id": trajectory["trajectory_id"],
                    "distance_m": distance,
                    "threshold_m": threshold,
                    "prediction_time_sec": prediction_time,
                    "position_std_m": position_std,
                    "frame_id": self._frame_id,
                    "trajectory_source_frame": trajectory["frame_id"],
                    "trajectory_transformed": transformed,
                }
        self._relations_publisher.publish(relations)
        unknown_active = {
            object_id
            for object_id in self._intrusion_monitor.active_ids
            if object_id not in track_by_id
            or self._lifecycle(track_by_id[object_id]) != "confirmed"
        }
        transitions = self._intrusion_monitor.update(
            intruding, unknown_ids=unknown_active
        )
        self._intrusion_evidence.update(evidence)
        for object_id in transitions["entered"]:
            self._publish_world_event(
                "person_entered_path",
                "high",
                True,
                [object_id],
                self._intrusion_evidence.get(object_id, {}),
            )
        for object_id in transitions["cleared"]:
            self._publish_world_event(
                "person_cleared_path",
                "info",
                False,
                [object_id],
                self._intrusion_evidence.pop(object_id, {}),
            )

    def _clear_trajectory(self, reason: str):
        with self._lock:
            self._trajectory = None
        relations = SemanticRelationArray()
        relations.header.stamp = self.get_clock().now().to_msg()
        relations.header.frame_id = self._frame_id
        self._relations_publisher.publish(relations)
        cleared = self._intrusion_monitor.reset()
        for object_id in cleared:
            evidence = self._intrusion_evidence.pop(object_id, {})
            evidence["clear_reason"] = reason
            self._publish_world_event(
                "person_cleared_path", "info", False, [object_id], evidence
            )

    def _trajectory_points_in_world(self, trajectory):
        source_frame = trajectory["frame_id"]
        if source_frame == self._frame_id:
            return trajectory["points"], False
        if not source_frame or not self._frame_id:
            return None, False
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                source_frame,
                Time.from_msg(trajectory["stamp"]),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"轨迹语义关系 TF 不可用: {self._frame_id} <- {source_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None, False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points = [
            transform_point(
                point,
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            for point in trajectory["points"]
        ]
        return points, True

    def _publish_world_event(
        self,
        event_type: str,
        severity: str,
        active: bool,
        object_ids,
        evidence,
    ):
        message = WorldModelEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.id = f"world-event-{self._event_sequence:08d}"
        self._event_sequence += 1
        message.event_type = event_type
        message.severity = severity
        message.active = bool(active)
        message.object_ids = [str(value) for value in object_ids]
        message.confidence = float(
            max(
                (
                    self._tracks[object_id].confidence
                    for object_id in object_ids
                    if object_id in self._tracks
                ),
                default=0.0,
            )
        )
        message.evidence_json = json.dumps(evidence, ensure_ascii=False)
        self._events_publisher.publish(message)
        self.get_logger().warning(
            f"世界模型事件: {event_type}, objects={message.object_ids}"
        )

    def _query_callback(self, request, response):
        query_type = str(request.query_type or "current").strip().lower()
        if query_type not in {"current", "nearest", "history", "health"}:
            response.success = False
            response.message = f"不支持的 query_type: {query_type}"
            return response
        response.frame_id = self._frame_id
        limit = max(1, min(int(request.limit or 20), 100))
        if query_type == "health":
            payload = self._health_payload()
            response.success = True
            response.message = "世界模型健康状态已读取"
            response.result_json = json.dumps(payload, ensure_ascii=False)
            return response
        if query_type == "history":
            if self._store is None:
                response.success = False
                response.message = "世界对象历史库未启用或不可用"
                return response
            try:
                history = self._store.query_history(
                    object_id=str(request.object_id).strip(),
                    class_name=_normalize_class_name(request.class_name),
                    fresh_within_sec=max(0.0, float(request.fresh_within_sec)),
                    limit=limit,
                    now=self.get_clock().now().nanoseconds / 1e9,
                )
            except Exception as exc:
                response.success = False
                response.message = f"查询世界对象历史失败: {exc}"
                return response
            response.success = True
            response.message = f"返回 {len(history)} 条历史观测"
            response.result_json = json.dumps(
                {"query_type": "history", "count": len(history), "history": history},
                ensure_ascii=False,
            )
            return response

        now = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            tracks = list(self._tracks.values())
            unlocated = list(self._unlocated_observations)
        items = [self._track_message(track, now) for track in tracks]
        items.extend(_unlocated_messages(unlocated))
        items = _filter_objects(
            items,
            object_id=str(request.object_id).strip(),
            class_name=_normalize_class_name(request.class_name),
            dynamic_only=bool(request.dynamic_only),
            confirmed_only=bool(request.confirmed_only),
            fresh_within_sec=max(0.0, float(request.fresh_within_sec)),
        )
        reference = None
        if request.reference_position_valid:
            reference = (
                float(request.reference_position.x),
                float(request.reference_position.y),
                float(request.reference_position.z),
            )
        elif query_type == "nearest" or float(request.max_distance_m) > 0.0:
            reference = self._reference_position()
            if reference is None:
                response.success = False
                response.message = f"没有与 {self._frame_id or '世界'} 坐标系匹配的新鲜无人机位姿"
                return response
        if reference is not None:
            located = [item for item in items if item.position_valid]
            located.sort(key=lambda item: _object_distance(item, reference))
            maximum = float(request.max_distance_m)
            if maximum > 0.0:
                located = [
                    item for item in located
                    if _object_distance(item, reference) <= maximum
                ]
            items = located
        items = items[:limit]
        response.success = True
        response.message = f"返回 {len(items)} 个世界对象"
        response.objects = items
        response.result_json = json.dumps(
            {
                "query_type": query_type,
                "count": len(items),
                "frame_id": self._frame_id,
                "reference_position_m": (
                    {"x": reference[0], "y": reference[1], "z": reference[2]}
                    if reference is not None else None
                ),
            },
            ensure_ascii=False,
        )
        return response

    def _reference_position(self):
        with self._lock:
            value = self._poses_by_frame.get(self._frame_id)
            if value is None or time.monotonic() - value["received_at"] > 2.0:
                return None
            return tuple(value["position"])

    def _health_payload(self):
        with self._lock:
            source = self._fusion_health
            tracks = list(self._tracks.values())
        return {
            "frame_id": self._frame_id,
            "healthy": bool(source and source.healthy),
            "fusion_state": source.state if source else "UNAVAILABLE",
            "sync_delta_sec": (
                float(source.sync_delta_sec) if source and math.isfinite(source.sync_delta_sec) else None
            ),
            "track_count": len(tracks),
            "confirmed_track_count": sum(
                self._lifecycle(track) == "confirmed" for track in tracks
            ),
            "history_enabled": self._store is not None,
            "history_rows": self._store.count() if self._store is not None else 0,
        }

    def close(self):
        if self._store is not None:
            self._store.close()
            self._store = None


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


def _track_record(item):
    return {
        "id": item.id,
        "class_name": item.class_name,
        "confidence": float(item.confidence),
        "position_valid": bool(item.position_valid),
        "position": (float(item.position.x), float(item.position.y), float(item.position.z)),
        "velocity": (float(item.velocity.x), float(item.velocity.y), float(item.velocity.z)),
        "dynamic": bool(item.dynamic),
        "state": item.state,
        "hit_count": int(item.hit_count),
        "miss_count": int(item.miss_count),
        "position_covariance": list(item.position_covariance),
        "sources": list(item.sources),
    }


def _unlocated_messages(observations):
    values = []
    for index, observation in enumerate(observations):
        item = SemanticObject()
        item.id = f"observed_2d_{_safe_id(observation['class_name'])}_{index:04d}"
        item.class_name = observation["class_name"]
        item.confidence = float(observation["confidence"])
        item.position_valid = False
        item.state = "observed_2d"
        item.evidence_type = "observed_2d"
        item.sources = list(observation["sources"])
        item.hit_count = 1
        values.append(item)
    return values


def _filter_objects(
    items,
    object_id: str = "",
    class_name: str = "",
    dynamic_only: bool = False,
    confirmed_only: bool = False,
    fresh_within_sec: float = 0.0,
):
    expected_class = class_name.lower()
    return [
        item for item in items
        if (not object_id or item.id == object_id)
        and (not expected_class or item.class_name.lower() == expected_class)
        and (not dynamic_only or item.dynamic)
        and (not confirmed_only or item.state == "confirmed")
        and (fresh_within_sec <= 0.0 or item.age_sec <= fresh_within_sec)
    ]


def _object_distance(item, reference):
    return math.dist(
        (float(item.position.x), float(item.position.y), float(item.position.z)),
        reference,
    )


def _safe_id(value: str):
    normalized = "".join(char.lower() if char.isalnum() else "_" for char in value)
    return normalized.strip("_") or "object"


def _normalize_class_name(value: str):
    normalized = str(value or "").strip().lower()
    aliases = {
        "人": "person",
        "人员": "person",
        "行人": "person",
        "汽车": "car",
        "车辆": "car",
        "车": "car",
        "公交车": "bus",
    }
    return aliases.get(normalized, normalized)


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
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
