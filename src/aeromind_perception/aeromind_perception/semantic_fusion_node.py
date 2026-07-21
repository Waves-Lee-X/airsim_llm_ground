#!/usr/bin/env python3
"""Fuse 2D detections, depth, camera intrinsics and TF into semantic objects."""

from __future__ import annotations

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
    WorldModelHealth,
)
from .semantic_geometry import camera_intrinsics_valid, project_pixel, transform_point


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
        self.declare_parameter("depth_window_fraction", 0.35)
        self.declare_parameter("maximum_sync_delta_sec", 0.15)
        self.declare_parameter("sensor_timeout_sec", 2.0)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._lock = threading.RLock()
        self._depth = None
        self._depth_received = 0.0
        self._depth_stamp_sec = None
        self._camera_info = None
        self._rgb_size = None
        self._detections_received = False
        self._last_detection_received = 0.0
        self._last_health = {
            "sync_delta_sec": float("nan"),
            "depth_fresh": False,
            "camera_info_valid": False,
            "tf_available": False,
            "observation_count": 0,
            "message": "等待检测数据",
        }
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            SemanticObjectArray, "/world_model/observations", 10
        )
        self._health_publisher = self.create_publisher(
            WorldModelHealth, "/world_model/fusion_health", 10
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
        self.create_timer(0.5, self._publish_health)
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
            self._depth = (depth, msg.header.frame_id, msg.header.stamp)
            self._depth_received = time.monotonic()
            self._depth_stamp_sec = _stamp_seconds(msg.header.stamp)

    def _camera_info_callback(self, msg: CameraInfo):
        if not camera_intrinsics_valid(msg.k, msg.width, msg.height):
            self.get_logger().warning(
                "CameraInfo 内参或分辨率无效", throttle_duration_sec=5.0
            )
            return
        with self._lock:
            self._camera_info = msg

    def _detections_callback(self, msg: DetectionArray):
        now = time.monotonic()
        detection_stamp_sec = _stamp_seconds(msg.header.stamp)
        with self._lock:
            self._detections_received = True
            self._last_detection_received = now
            sensor_timeout = float(self.get_parameter("sensor_timeout_sec").value)
            if (
                self._depth is None
                or self._camera_info is None
                or now - self._depth_received > sensor_timeout
            ):
                reason = "深度数据超时或 CameraInfo 无效"
                self._publish_2d_observations(msg, reason)
                return
            depth, depth_frame, depth_stamp = self._depth
            camera_info = self._camera_info
            rgb_size = self._rgb_size or (depth.shape[1], depth.shape[0])
            depth_stamp_sec = self._depth_stamp_sec

        if not camera_intrinsics_valid(
            camera_info.k,
            camera_info.width,
            camera_info.height,
            depth.shape[1],
            depth.shape[0],
        ):
            self._publish_2d_observations(msg, "CameraInfo 与深度图分辨率不一致")
            return

        sync_delta = (
            abs(detection_stamp_sec - depth_stamp_sec)
            if detection_stamp_sec is not None and depth_stamp_sec is not None
            else float("inf")
        )
        if sync_delta > float(self.get_parameter("maximum_sync_delta_sec").value):
            self._publish_2d_observations(
                msg, f"RGB/Depth 时间差 {sync_delta:.3f}s 超过门限", sync_delta
            )
            return

        transform = self._lookup_transform(
            depth_frame or camera_info.header.frame_id, depth_stamp
        )
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
            self._last_health = {
                "sync_delta_sec": sync_delta,
                "depth_fresh": True,
                "camera_info_valid": True,
                "tf_available": transform is not None,
                "observation_count": len(observations),
                "message": "三维融合正常" if transform is not None else "TF 不可用，降级为二维观测",
            }
        self._publish_observations(msg, observations)

    def _publish_2d_observations(
        self, msg: DetectionArray, reason: str, sync_delta: float = float("nan")
    ):
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
        with self._lock:
            self._last_health = {
                "sync_delta_sec": sync_delta,
                "depth_fresh": False,
                "camera_info_valid": self._camera_info is not None,
                "tf_available": False,
                "observation_count": len(observations),
                "message": reason,
            }
        self._publish_observations(msg, observations)

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

    def _lookup_transform(self, source_frame: str, stamp):
        if not source_frame:
            return None
        try:
            return self._tf_buffer.lookup_transform(
                self._world_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"语义目标 TF 不可用: {self._world_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _publish_observations(self, source: DetectionArray, observations):
        message = SemanticObjectArray()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self._world_frame
        for index, observation in enumerate(observations):
            item = SemanticObject()
            item.id = f"observation_{index:04d}"
            item.class_name = observation["class_name"]
            item.confidence = observation["confidence"]
            item.position_valid = bool(observation["position_valid"])
            if item.position_valid:
                item.position.x, item.position.y, item.position.z = observation["position"]
                variance = max(0.04, 0.01 * float(observation["depth_m"]) ** 2)
                item.position_covariance = [
                    variance, 0.0, 0.0,
                    0.0, variance, 0.0,
                    0.0, 0.0, variance,
                ]
            item.dynamic = False
            item.depth_m = float(observation["depth_m"])
            item.age_sec = 0.0
            item.state = "observation"
            item.evidence_type = "fused" if item.position_valid else "observed_2d"
            item.sources = observation["sources"]
            item.hit_count = 1
            item.miss_count = 0
            message.objects.append(item)
        self._publisher.publish(message)

    def _publish_health(self):
        now = time.monotonic()
        with self._lock:
            values = dict(self._last_health)
            detection_age = (
                now - self._last_detection_received
                if self._last_detection_received > 0.0
                else float("inf")
            )
        message = WorldModelHealth()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._world_frame
        message.detections_fresh = detection_age <= float(
            self.get_parameter("sensor_timeout_sec").value
        )
        message.depth_fresh = bool(values["depth_fresh"])
        message.camera_info_valid = bool(values["camera_info_valid"])
        message.tf_available = bool(values["tf_available"])
        message.sync_delta_sec = float(values["sync_delta_sec"])
        message.observation_count = int(values["observation_count"])
        message.healthy = bool(
            message.detections_fresh
            and message.depth_fresh
            and message.camera_info_valid
            and message.tf_available
            and math_is_finite_within(
                message.sync_delta_sec,
                float(self.get_parameter("maximum_sync_delta_sec").value),
            )
        )
        message.state = "FUSING" if message.healthy else "DEGRADED"
        message.message = str(values["message"])
        self._health_publisher.publish(message)


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


def _stamp_seconds(stamp):
    value = float(stamp.sec) + float(stamp.nanosec) / 1e9
    return value if value > 0.0 else None


def math_is_finite_within(value: float, maximum: float):
    return bool(np.isfinite(value) and 0.0 <= value <= maximum)


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
