#!/usr/bin/env python3
"""Fuse 2D detections, depth, camera intrinsics and TF into semantic objects."""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from aeromind_interfaces.msg import (
    DetectionArray,
    SemanticObject,
    SemanticObjectArray,
)
from .semantic_geometry import associate_track, project_pixel, transform_point


class SemanticFusionNode(Node):
    def __init__(self):
        super().__init__("semantic_fusion_node")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("rgb_topic", "/sensor/camera/rgb/front_center")
        self.declare_parameter("depth_topic", "/sensor/camera/depth/front_center")
        self.declare_parameter(
            "camera_info_topic", "/sensor/camera/depth/front_center/camera_info"
        )
        self.declare_parameter("minimum_depth_m", 0.3)
        self.declare_parameter("maximum_depth_m", 80.0)
        self.declare_parameter("track_timeout_sec", 3.0)
        self.declare_parameter("association_distance_m", 2.0)
        self.declare_parameter("dynamic_speed_mps", 0.35)
        self.declare_parameter("depth_window_fraction", 0.35)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._lock = threading.RLock()
        self._depth = None
        self._depth_received = 0.0
        self._camera_info = None
        self._rgb_size = None
        self._detections_received = False
        self._tracks = {}
        self._next_track = 1
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            SemanticObjectArray, "/world_model/objects", 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value), self._rgb_callback, 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._depth_callback, 10
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            DetectionArray,
            str(self.get_parameter("detections_topic").value),
            self._detections_callback,
            10,
        )
        self.create_timer(0.5, self._publish_snapshot)
        self.get_logger().info(
            f"语义世界融合节点已启动: world_frame={self._world_frame}"
        )

    def _rgb_callback(self, msg: Image):
        with self._lock:
            self._rgb_size = (int(msg.width), int(msg.height))

    def _depth_callback(self, msg: Image):
        depth = _depth_image(msg)
        if depth is None:
            self.get_logger().warning(
                f"不支持的深度图编码: {msg.encoding}", throttle_duration_sec=5.0
            )
            return
        with self._lock:
            self._depth = (depth, msg.header.frame_id)
            self._depth_received = time.monotonic()

    def _camera_info_callback(self, msg: CameraInfo):
        if float(msg.k[0]) <= 0.0 or float(msg.k[4]) <= 0.0:
            return
        with self._lock:
            self._camera_info = msg

    def _detections_callback(self, msg: DetectionArray):
        now = time.monotonic()
        with self._lock:
            self._detections_received = True
            if (
                self._depth is None
                or self._camera_info is None
                or now - self._depth_received > 2.0
            ):
                self._update_without_depth(msg, now)
                return
            depth, depth_frame = self._depth
            camera_info = self._camera_info
            rgb_size = self._rgb_size or (depth.shape[1], depth.shape[0])

        transform = self._lookup_transform(depth_frame or camera_info.header.frame_id)
        observations = []
        for detection in msg.detections:
            depth_m, pixel = self._detection_depth(detection, depth, rgb_size)
            observation = {
                "class_name": detection.class_name,
                "confidence": float(detection.confidence),
                "position_valid": False,
                "depth_m": float("nan"),
                "sources": ["/perception/detections"],
            }
            if depth_m is not None and transform is not None:
                optical = project_pixel(pixel[0], pixel[1], depth_m, camera_info.k)
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                observation["position"] = transform_point(
                    optical,
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )
                observation["position_valid"] = True
                observation["depth_m"] = depth_m
                observation["sources"].extend(
                    [
                        str(self.get_parameter("depth_topic").value),
                        str(self.get_parameter("camera_info_topic").value),
                        "/tf",
                    ]
                )
            observations.append(observation)
        with self._lock:
            self._update_tracks(observations, now)
        self._publish_snapshot()

    def _update_without_depth(self, msg: DetectionArray, now: float):
        observations = [
            {
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "position_valid": False,
                "depth_m": float("nan"),
                "sources": ["/perception/detections"],
            }
            for item in msg.detections
        ]
        self._update_tracks(observations, now)

    def _detection_depth(self, detection, depth, rgb_size):
        rgb_width, rgb_height = rgb_size
        scale_x = depth.shape[1] / max(1, rgb_width)
        scale_y = depth.shape[0] / max(1, rgb_height)
        center_x = float(detection.x) * scale_x
        center_y = float(detection.y) * scale_y
        fraction = float(self.get_parameter("depth_window_fraction").value)
        half_width = max(2, int(float(detection.width) * scale_x * fraction * 0.5))
        half_height = max(2, int(float(detection.height) * scale_y * fraction * 0.5))
        x0 = max(0, int(center_x) - half_width)
        x1 = min(depth.shape[1], int(center_x) + half_width + 1)
        y0 = max(0, int(center_y) - half_height)
        y1 = min(depth.shape[0], int(center_y) + half_height + 1)
        values = depth[y0:y1, x0:x1]
        minimum = float(self.get_parameter("minimum_depth_m").value)
        maximum = float(self.get_parameter("maximum_depth_m").value)
        valid = values[np.isfinite(values) & (values >= minimum) & (values <= maximum)]
        if valid.size == 0:
            return None, (center_x, center_y)
        return float(np.median(valid)), (center_x, center_y)

    def _lookup_transform(self, source_frame: str):
        if not source_frame:
            return None
        try:
            return self._tf_buffer.lookup_transform(
                self._world_frame, source_frame, Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"语义目标 TF 不可用: {self._world_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _update_tracks(self, observations, now: float):
        timeout = float(self.get_parameter("track_timeout_sec").value)
        self._tracks = {
            track_id: track
            for track_id, track in self._tracks.items()
            if now - track["last_seen"] <= timeout
        }
        used = set()
        for observation in observations:
            track_id = None
            if observation["position_valid"]:
                candidates = {
                    key: value for key, value in self._tracks.items() if key not in used
                }
                track_id = associate_track(
                    observation["class_name"],
                    observation["position"],
                    candidates,
                    float(self.get_parameter("association_distance_m").value),
                )
            else:
                track_id = next(
                    (
                        key
                        for key, value in self._tracks.items()
                        if key not in used
                        and value["class_name"] == observation["class_name"]
                        and not value["position_valid"]
                    ),
                    None,
                )
            if track_id is None:
                track_id = f"{_safe_id(observation['class_name'])}_{self._next_track:04d}"
                self._next_track += 1
                previous = None
            else:
                previous = self._tracks[track_id]
            used.add(track_id)
            velocity = (0.0, 0.0, 0.0)
            if previous and observation["position_valid"] and previous["position_valid"]:
                delta = max(1e-3, now - previous["last_seen"])
                measured = tuple(
                    (observation["position"][index] - previous["position"][index]) / delta
                    for index in range(3)
                )
                velocity = tuple(
                    0.65 * previous["velocity"][index] + 0.35 * measured[index]
                    for index in range(3)
                )
            self._tracks[track_id] = {
                **observation,
                "id": track_id,
                "velocity": velocity,
                "first_seen": previous["first_seen"] if previous else now,
                "last_seen": now,
            }

    def _publish_snapshot(self):
        now = time.monotonic()
        timeout = float(self.get_parameter("track_timeout_sec").value)
        message = SemanticObjectArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._world_frame
        with self._lock:
            if not self._detections_received:
                return
            tracks = [
                dict(track)
                for track in self._tracks.values()
                if now - track["last_seen"] <= timeout
            ]
        for track in tracks:
            item = SemanticObject()
            item.id = track["id"]
            item.class_name = track["class_name"]
            item.confidence = track["confidence"]
            item.position_valid = bool(track["position_valid"])
            if item.position_valid:
                item.position.x, item.position.y, item.position.z = track["position"]
            item.velocity.x, item.velocity.y, item.velocity.z = track["velocity"]
            item.dynamic = math.sqrt(sum(value * value for value in track["velocity"])) >= float(
                self.get_parameter("dynamic_speed_mps").value
            )
            item.depth_m = float(track["depth_m"])
            item.age_sec = float(now - track["last_seen"])
            item.state = "observed" if item.age_sec <= 1.0 else "stale"
            item.evidence_type = "fused" if item.position_valid else "observed_2d"
            item.sources = track["sources"]
            message.objects.append(item)
        self._publisher.publish(message)


def _depth_image(msg: Image):
    encoding = msg.encoding.lower()
    if encoding in ("32fc1", "32fc"):
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        scale = 1.0
    elif encoding in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        scale = 0.001
    else:
        return None
    row_values = int(msg.step) // dtype.itemsize
    expected = row_values * int(msg.height)
    if row_values < int(msg.width) or len(msg.data) < expected * dtype.itemsize:
        return None
    values = np.frombuffer(bytes(msg.data), dtype=dtype, count=expected)
    if values.size != expected:
        return None
    return values.reshape((int(msg.height), row_values))[:, : int(msg.width)].astype(np.float32) * scale


def _safe_id(value: str):
    normalized = "".join(char.lower() if char.isalnum() else "_" for char in value)
    return normalized.strip("_") or "object"


def main(args=None):
    rclpy.init(args=args)
    node = SemanticFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
