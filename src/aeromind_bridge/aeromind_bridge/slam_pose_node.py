#!/usr/bin/env python3
"""Publish map-frame odometry by applying a SLAM map->odom correction."""

from __future__ import annotations

import json
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .slam_pose import compose_pose


class SlamPoseNode(Node):
    def __init__(self):
        super().__init__("slam_pose_node")
        self.declare_parameter("input_odom_topic", "/sensor/odometry")
        self.declare_parameter("output_odom_topic", "/localization/odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("input_timeout_sec", 1.0)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._lock = threading.RLock()
        self._last_input = 0.0
        self._last_output = 0.0
        self._last_error = "等待 RTAB-Map map->odom TF"
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_odom_topic").value), 20
        )
        self._status_pub = self.create_publisher(String, "/localization/status", 10)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_odom_topic").value),
            self._odom_callback,
            20,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"SLAM 位姿桥已启动: {self._map_frame}->{self._odom_frame}, "
            f"输出={self.get_parameter('output_odom_topic').value}"
        )

    def _odom_callback(self, msg: Odometry):
        now = time.monotonic()
        with self._lock:
            self._last_input = now
        source_frame = str(msg.header.frame_id or self._odom_frame).lstrip("/")
        if source_frame != self._odom_frame.lstrip("/"):
            with self._lock:
                self._last_error = (
                    f"里程计 frame_id={source_frame}，期望 {self._odom_frame}"
                )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._odom_frame,
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("transform_timeout_sec").value)
                ),
            )
        except TransformException as exc:
            with self._lock:
                self._last_error = f"map->odom TF 不可用: {exc}"
            self.get_logger().warning(self._last_error, throttle_duration_sec=5.0)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = msg.pose.pose
        position, orientation = compose_pose(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )
        output = Odometry()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self._map_frame
        output.child_frame_id = msg.child_frame_id or self._base_frame
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = position
        (
            output.pose.pose.orientation.x,
            output.pose.pose.orientation.y,
            output.pose.pose.orientation.z,
            output.pose.pose.orientation.w,
        ) = orientation
        output.pose.covariance = msg.pose.covariance
        output.twist = msg.twist
        self._odom_pub.publish(output)
        with self._lock:
            self._last_output = now
            self._last_error = ""

    def _publish_status(self):
        now = time.monotonic()
        input_timeout = float(self.get_parameter("input_timeout_sec").value)
        with self._lock:
            input_age = now - self._last_input if self._last_input else None
            output_age = now - self._last_output if self._last_output else None
            healthy = bool(
                input_age is not None
                and output_age is not None
                and input_age <= input_timeout
                and output_age <= input_timeout
            )
            payload = {
                "backend": "rtabmap_rgbd",
                "healthy": healthy,
                "map_frame": self._map_frame,
                "odom_frame": self._odom_frame,
                "base_frame": self._base_frame,
                "input_age_sec": round(input_age, 3) if input_age is not None else None,
                "output_age_sec": round(output_age, 3) if output_age is not None else None,
                "message": "SLAM map 位姿可用" if healthy else self._last_error,
            }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

