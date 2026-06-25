#!/usr/bin/env python3
"""AeroMind Web 控制台节点。

用一个轻量 HTTP 服务把常用 ROS topic/service 暴露给浏览器：
  - 订阅飞控状态、里程计、RGB 图像、深度图、点云、检测结果
  - 调用 arm/takeoff/land/execute_task 服务
  - 发布 /control/cmd_vel 供 Web 虚拟摇杆使用
"""

import base64
import json
import math
import mimetypes
import os
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from aeromind_interfaces.msg import Detection, DroneState
from aeromind_interfaces.srv import ArmDrone, ExecuteTask, Land, Takeoff


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


class WebConsoleNode(Node):
    """ROS node + embedded HTTP server for the browser console."""

    def __init__(self):
        super().__init__("web_console_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)

        self._lock = threading.Lock()
        self._state = None
        self._odom = None
        self._image = None
        self._depth = None
        self._pointcloud = None
        self._detection = None
        self._events = []

        self.create_subscription(DroneState, "/control/drone_state", self._state_cb, 10)
        self.create_subscription(Odometry, "/sensor/odometry", self._odom_cb, 10)
        self.create_subscription(Image, "/sensor/camera/rgb/front_center", self._image_cb, 10)
        self.create_subscription(Image, "/sensor/camera/depth/front_center", self._depth_cb, 10)
        self.create_subscription(PointCloud2, "/sensor/lidar/points", self._pointcloud_cb, 10)
        self.create_subscription(Detection, "/perception/detection", self._detection_cb, 10)

        self._cmd_vel_pub = self.create_publisher(Twist, "/control/cmd_vel", 10)

        self._arm_client = self.create_client(ArmDrone, "/control/arm")
        self._takeoff_client = self.create_client(Takeoff, "/control/takeoff")
        self._land_client = self.create_client(Land, "/control/land")
        self._task_client = self.create_client(ExecuteTask, "/agent/execute_task")

        self._static_dir = os.path.join(
            get_package_share_directory("aeromind_web"), "static"
        )
        self._server = ThreadingHTTPServer((self._host, self._port), ConsoleRequestHandler)
        self._server.node = self
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="aeromind-web-http",
            daemon=True,
        )
        self._server_thread.start()

        self._add_event("system", f"Web 控制台监听中: http://{self._host}:{self._port}")
        self.get_logger().info(
            f"Web 控制台已启动: http://localhost:{self._port}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: DroneState):
        with self._lock:
            self._state = {
                "armed": bool(msg.armed),
                "mode": msg.mode,
                "battery": float(msg.battery),
                "gps_fix": int(msg.gps_fix),
                "ekf_healthy": bool(msg.ekf_healthy),
            }

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        with self._lock:
            self._odom = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "position": {"x": pos.x, "y": pos.y, "z": pos.z},
                "velocity": {"x": vel.x, "y": vel.y, "z": vel.z},
            }

    def _image_cb(self, msg: Image):
        data = bytes(msg.data)
        with self._lock:
            self._image = {
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "height": int(msg.height),
                "width": int(msg.width),
                "encoding": msg.encoding,
                "step": int(msg.step),
                "data": base64.b64encode(data).decode("ascii"),
            }

    def _depth_cb(self, msg: Image):
        summary = self._summarize_depth(msg)
        with self._lock:
            self._depth = summary

    def _pointcloud_cb(self, msg: PointCloud2):
        summary = self._summarize_pointcloud(msg)
        with self._lock:
            self._pointcloud = summary

    def _detection_cb(self, msg: Detection):
        with self._lock:
            self._detection = {
                "class_name": msg.class_name,
                "confidence": float(msg.confidence),
                "x": float(msg.x),
                "y": float(msg.y),
                "width": float(msg.width),
                "height": float(msg.height),
            }

    # ------------------------------------------------------------------
    # API methods called by HTTP handler
    # ------------------------------------------------------------------

    def snapshot(self):
        with self._lock:
            return {
                "state": self._state,
                "odom": self._odom,
                "depth": self._depth,
                "pointcloud": self._pointcloud_meta(),
                "detection": self._detection,
                "events": list(self._events[-80:]),
                "services": {
                    "arm": self._arm_client.service_is_ready(),
                    "takeoff": self._takeoff_client.service_is_ready(),
                    "land": self._land_client.service_is_ready(),
                    "agent": self._task_client.service_is_ready(),
                },
            }

    def latest_image(self):
        with self._lock:
            return dict(self._image) if self._image is not None else None

    def latest_pointcloud(self):
        with self._lock:
            return dict(self._pointcloud) if self._pointcloud is not None else None

    def arm(self, arm: bool):
        req = ArmDrone.Request()
        req.arm = bool(arm)
        return self._call_service(self._arm_client, req, "解锁" if arm else "加锁")

    def takeoff(self, altitude: float):
        req = Takeoff.Request()
        req.altitude = float(altitude)
        return self._call_service(self._takeoff_client, req, f"起飞到 {altitude:.1f}m")

    def land(self):
        req = Land.Request()
        return self._call_service(self._land_client, req, "降落")

    def execute_task(self, task: str):
        req = ExecuteTask.Request()
        req.task_description = task.strip()
        if not req.task_description:
            return {"success": False, "message": "任务不能为空"}
        return self._call_service(self._task_client, req, f"自然语言任务: {req.task_description}")

    def publish_cmd_vel(self, payload):
        msg = Twist()
        linear = payload.get("linear", {})
        angular = payload.get("angular", {})
        msg.linear.x = float(linear.get("x", 0.0))
        msg.linear.y = float(linear.get("y", 0.0))
        msg.linear.z = float(linear.get("z", 0.0))
        msg.angular.z = float(angular.get("z", 0.0))
        self._cmd_vel_pub.publish(msg)
        return {"success": True, "message": "cmd_vel 已发布"}

    def shutdown_server(self):
        if hasattr(self, "_server"):
            self._server.shutdown()
            self._server.server_close()

    def _call_service(self, client, request, label: str, timeout_sec: float = 8.0):
        if not client.service_is_ready():
            msg = f"{label}: 服务未就绪"
            self._add_event("error", msg)
            return {"success": False, "message": msg}

        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout_sec):
            msg = f"{label}: 服务调用超时"
            self._add_event("error", msg)
            return {"success": False, "message": msg}

        try:
            result = future.result()
        except Exception as exc:
            msg = f"{label}: {exc}"
            self._add_event("error", msg)
            return {"success": False, "message": msg}

        data = {}
        for field in ("success", "message", "result"):
            if hasattr(result, field):
                data[field] = getattr(result, field)
        if "success" not in data:
            data["success"] = True
        if "message" not in data:
            data["message"] = f"{label}: 已完成"

        self._add_event("service", f"{label}: {data.get('message', '')}")
        return data

    def _add_event(self, kind: str, message: str):
        with self._lock:
            self._events.append({"kind": kind, "message": message})
            self._events = self._events[-100:]

    def _pointcloud_meta(self):
        if self._pointcloud is None:
            return None
        return {
            key: value
            for key, value in self._pointcloud.items()
            if key != "points"
        }

    def _summarize_depth(self, msg: Image):
        data = bytes(msg.data)
        width = int(msg.width)
        height = int(msg.height)
        value_step = None
        read_value = None
        to_meters = None
        encoding = msg.encoding.lower()
        summary = {
            "stamp": _stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "width": width,
            "height": height,
            "encoding": msg.encoding,
            "step": int(msg.step),
            "center_m": None,
            "min_m": None,
            "max_m": None,
            "valid_samples": 0,
        }

        if encoding in ("32fc1", "32fc"):
            value_step = 4
            read_value = lambda offset: struct.unpack_from("<f", data, offset)[0]
            to_meters = lambda value: value
        elif encoding in ("16uc1", "mono16"):
            value_step = 2
            read_value = lambda offset: struct.unpack_from("<H", data, offset)[0]
            to_meters = lambda value: float(value) / 1000.0

        if width <= 0 or height <= 0 or value_step is None or not data:
            return summary

        center_offset = (height // 2) * int(msg.step) + (width // 2) * value_step
        if center_offset + value_step <= len(data):
            center = to_meters(read_value(center_offset))
            if math.isfinite(center) and center > 0:
                summary["center_m"] = center

        total = width * height
        stride = max(1, total // 6000)
        min_depth = None
        max_depth = None
        valid = 0
        for idx in range(0, total, stride):
            row = idx // width
            col = idx % width
            offset = row * int(msg.step) + col * value_step
            if offset + value_step > len(data):
                continue
            value = to_meters(read_value(offset))
            if not math.isfinite(value) or value <= 0:
                continue
            min_depth = value if min_depth is None else min(min_depth, value)
            max_depth = value if max_depth is None else max(max_depth, value)
            valid += 1

        summary["min_m"] = min_depth
        summary["max_m"] = max_depth
        summary["valid_samples"] = valid
        return summary

    def _summarize_pointcloud(self, msg: PointCloud2):
        width = int(msg.width)
        height = int(msg.height)
        point_step = int(msg.point_step)
        total_points = width * height
        summary = {
            "stamp": _stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "width": width,
            "height": height,
            "point_step": point_step,
            "row_step": int(msg.row_step),
            "total_points": total_points,
            "sampled_points": 0,
            "points": [],
        }

        fields = {field.name: field for field in msg.fields}
        if point_step <= 0 or not all(name in fields for name in ("x", "y", "z")):
            return summary

        x_field = fields["x"]
        y_field = fields["y"]
        z_field = fields["z"]
        if not all(field.datatype == 7 for field in (x_field, y_field, z_field)):
            return summary

        endian = ">" if msg.is_bigendian else "<"
        stride = max(1, total_points // 1800)
        data = bytes(msg.data)
        points = []

        for idx in range(0, total_points, stride):
            row = idx // width if width else 0
            col = idx % width if width else 0
            base = row * int(msg.row_step) + col * point_step
            max_offset = base + max(x_field.offset, y_field.offset, z_field.offset) + 4
            if max_offset > len(data):
                continue
            x = struct.unpack_from(endian + "f", data, base + x_field.offset)[0]
            y = struct.unpack_from(endian + "f", data, base + y_field.offset)[0]
            z = struct.unpack_from(endian + "f", data, base + z_field.offset)[0]
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            points.append([round(float(x), 3), round(float(y), 3), round(float(z), 3)])

        summary["points"] = points
        summary["sampled_points"] = len(points)
        return summary


class ConsoleRequestHandler(BaseHTTPRequestHandler):
    """HTTP routes for static files and JSON APIs."""

    server_version = "AeroMindWeb/0.1"

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._serve_static("index.html")
        elif path == "/api/status":
            self._json(self.server.node.snapshot())
        elif path == "/api/camera":
            image = self.server.node.latest_image()
            if image is None:
                self._json({"success": False, "message": "暂无相机图像"}, status=404)
            else:
                image["success"] = True
                self._json(image)
        elif path == "/api/pointcloud":
            pointcloud = self.server.node.latest_pointcloud()
            if pointcloud is None:
                self._json({"success": False, "message": "暂无点云数据"}, status=404)
            else:
                pointcloud["success"] = True
                self._json(pointcloud)
        elif path.startswith("/static/"):
            self._serve_static(path.removeprefix("/static/"))
        else:
            self._json({"success": False, "message": "not found"}, status=404)

    def do_POST(self):
        path = urlparse(self.path).path
        payload = self._read_json()
        node = self.server.node

        if path == "/api/control/arm":
            result = node.arm(bool(payload.get("arm", True)))
        elif path == "/api/control/takeoff":
            result = node.takeoff(float(payload.get("altitude", 10.0)))
        elif path == "/api/control/land":
            result = node.land()
        elif path == "/api/agent/task":
            result = node.execute_task(str(payload.get("task", "")))
        elif path == "/api/cmd_vel":
            result = node.publish_cmd_vel(payload)
        else:
            result = {"success": False, "message": "not found"}
            self._json(result, status=404)
            return

        self._json(result, status=200 if result.get("success", False) else 503)

    def log_message(self, fmt, *args):
        # Keep HTTP access logs out of the ROS console unless there is a real error.
        return

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}

    def _json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_static(self, rel_path: str):
        rel_path = rel_path.strip("/") or "index.html"
        static_dir = self.server.node._static_dir
        full_path = os.path.abspath(os.path.join(static_dir, rel_path))
        if not full_path.startswith(os.path.abspath(static_dir)) or not os.path.exists(full_path):
            self._json({"success": False, "message": "not found"}, status=404)
            return

        content_type = mimetypes.guess_type(full_path)[0] or "application/octet-stream"
        with open(full_path, "rb") as f:
            body = f.read()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main(args=None):
    rclpy.init(args=args)
    node = WebConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_server()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
