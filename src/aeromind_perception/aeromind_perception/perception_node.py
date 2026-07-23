#!/usr/bin/env python3
"""Image capture, VLM analysis and unified perception health."""

import base64
import io
import json
import os
import re
import threading
import time
import urllib.error
import urllib.request
from collections import deque

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from aeromind_interfaces.msg import DetectionArray, PerceptionHealth
from aeromind_interfaces.srv import AnalyzeImage, CaptureImage

try:
    from PIL import Image as PilImage
except Exception:
    PilImage = None


def _strip_json_fence(content: str) -> str:
    text = str(content or "").strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text, flags=re.IGNORECASE)
        text = re.sub(r"\s*```$", "", text)
    return text.strip()


def extract_vlm_json(content: str) -> dict:
    text = _strip_json_fence(content)
    candidates = [text]
    start = text.find("{")
    end = text.rfind("}")
    if start >= 0 and end > start:
        candidates.append(text[start : end + 1])
    for candidate in candidates:
        for normalized in (
            candidate,
            re.sub(r",\s*([}\]])", r"\1", candidate),
        ):
            try:
                result = json.loads(normalized)
            except json.JSONDecodeError:
                continue
            if isinstance(result, dict):
                return result
    raise ValueError("VLM 返回内容不是合法 JSON 对象")


def normalize_vlm_content(content: str, fallback: dict) -> dict:
    try:
        result = extract_vlm_json(content)
    except ValueError as exc:
        scene = _strip_json_fence(content)
        if not scene:
            raise ValueError("VLM 返回内容为空") from exc
        return {
            "message": "图像语义分析完成（结构化格式已降级）",
            "scene": scene,
            "risk_level": fallback.get("risk_level", "low"),
            "suggestion": fallback.get("suggestion", "请结合深度和点云复核"),
            "objects": fallback.get("objects", []),
            "format_warning": str(exc),
        }
    risk_level = str(result.get("risk_level", "")).lower()
    if risk_level not in {"low", "medium", "high"}:
        result["risk_level"] = fallback.get("risk_level", "low")
    return result


def classify_signal(age_sec, timeout_sec, enabled=True, error=""):
    """Return a stable health state without confusing empty data with no data."""
    if not enabled:
        return "DISABLED"
    if error:
        return "ERROR"
    if age_sec is None:
        return "MISSING"
    if age_sec > timeout_sec:
        return "STALE"
    return "OK"


class PerceptionNode(Node):
    """感知节点"""

    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("image_topic", "/sensor/camera/rgb/front_center")
        self.declare_parameter("depth_topic", "/sensor/camera/depth/front_center")
        self.declare_parameter("camera_info_topic", "/sensor/camera/depth/front_center/camera_info")
        self.declare_parameter("pointcloud_topic", "/sensor/lidar/points")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("detector_status_topic", "/perception/detector_status")
        self.declare_parameter("yolo_enabled", False)
        self.declare_parameter("sensor_timeout_sec", 2.0)
        self.declare_parameter("vlm_enabled", False)
        self.declare_parameter("vlm_api_url", os.environ.get("AEROMIND_VLM_API_URL", ""))
        self.declare_parameter("vlm_api_key", os.environ.get("AEROMIND_VLM_API_KEY", ""))
        self.declare_parameter("vlm_model", os.environ.get("AEROMIND_VLM_MODEL", ""))
        self.declare_parameter("vlm_timeout_sec", 20.0)
        self.declare_parameter("auto_analyze_enabled", False)
        self.declare_parameter("auto_analyze_interval_sec", 8.0)
        self.declare_parameter("auto_analyze_use_vlm", False)
        self.declare_parameter(
            "capture_dir",
            os.environ.get(
                "AEROMIND_CAPTURE_DIR",
                os.path.expanduser("~/aeromind_ws/missions/captures"),
            ),
        )
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self._detections_topic = str(self.get_parameter("detections_topic").value)
        self._detector_status_topic = str(self.get_parameter("detector_status_topic").value)
        self._yolo_enabled = bool(self.get_parameter("yolo_enabled").value)
        self._sensor_timeout_sec = max(0.5, float(self.get_parameter("sensor_timeout_sec").value))
        self._vlm_enabled = bool(self.get_parameter("vlm_enabled").value)
        self._vlm_api_url = (
            str(self.get_parameter("vlm_api_url").value).strip()
            or os.environ.get("AEROMIND_VLM_API_URL", "").strip()
        )
        self._vlm_api_key = (
            str(self.get_parameter("vlm_api_key").value).strip()
            or os.environ.get("AEROMIND_VLM_API_KEY", "").strip()
            or os.environ.get("DASHSCOPE_API_KEY", "").strip()
        )
        self._vlm_model = (
            str(self.get_parameter("vlm_model").value).strip()
            or os.environ.get("AEROMIND_VLM_MODEL", "").strip()
        )
        self._vlm_timeout_sec = float(self.get_parameter("vlm_timeout_sec").value)
        self._auto_analyze_enabled = bool(self.get_parameter("auto_analyze_enabled").value)
        self._auto_analyze_interval_sec = max(1.0, float(self.get_parameter("auto_analyze_interval_sec").value))
        self._auto_analyze_use_vlm = bool(self.get_parameter("auto_analyze_use_vlm").value)
        self._capture_dir = os.path.expanduser(str(self.get_parameter("capture_dir").value))
        self._data_lock = threading.RLock()
        self._sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self._capture_callback_group = MutuallyExclusiveCallbackGroup()
        self._vlm_callback_group = MutuallyExclusiveCallbackGroup()
        self._latest_image = None
        self._latest_detections = []
        self._latest_detections_at = 0.0
        self._latest_analysis = None
        self._last_auto_analysis_time = 0.0
        self._signal_times = {
            name: deque(maxlen=60)
            for name in ("rgb", "depth", "camera_info", "pointcloud", "detections")
        }
        self._detector_error = ""
        self._detector_message = "等待检测器状态"
        self._vlm_last_at = 0.0
        self._vlm_error = ""
        self._vlm_message = "VLM 已禁用"
        if self._vlm_enabled:
            missing = []
            if not self._vlm_api_url:
                missing.append("API URL")
            if not self._vlm_api_key:
                missing.append("API Key")
            if not self._vlm_model:
                missing.append("模型")
            self._vlm_error = f"缺少 {', '.join(missing)}" if missing else ""
            self._vlm_message = self._vlm_error or "VLM 已配置，等待调用"

        self._image_analysis_pub = self.create_publisher(
            String, "/perception/image_analysis", 10
        )
        self._health_pub = self.create_publisher(
            PerceptionHealth, "/perception/health", 10
        )

        # 订阅相机图像，供 CaptureImageSkill 保存最近一帧
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            10,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_callback,
            10,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            10,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            PointCloud2,
            self._pointcloud_topic,
            self._pointcloud_callback,
            10,
            callback_group=self._sensor_callback_group,
        )
        self._detections_sub = self.create_subscription(
            DetectionArray,
            self._detections_topic,
            self._detections_callback,
            10,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            String,
            self._detector_status_topic,
            self._detector_status_callback,
            10,
            callback_group=self._sensor_callback_group,
        )

        self._capture_srv = self.create_service(
            CaptureImage,
            "/perception/capture_image",
            self._capture_image_callback,
            callback_group=self._capture_callback_group,
        )
        self._analyze_srv = self.create_service(
            AnalyzeImage,
            "/perception/analyze_image",
            self._analyze_image_callback,
            callback_group=self._vlm_callback_group,
        )

        self._health_timer = self.create_timer(
            0.5,
            self._publish_health,
            callback_group=self._sensor_callback_group,
        )
        self._auto_analyze_timer = self.create_timer(
            1.0,
            self._auto_analyze_callback,
            callback_group=self._vlm_callback_group,
        )

        self.get_logger().info(
            f"感知节点已启动：image_topic={self._image_topic}, capture_dir={self._capture_dir}, "
            f"vlm_enabled={self._vlm_enabled}, "
            f"vlm_model={self._vlm_model or '未配置'}, "
            f"vlm_api_key={'已配置' if self._vlm_api_key else '未配置'}, "
            f"auto_analyze={self._auto_analyze_enabled}"
        )

    def _image_callback(self, msg: Image):
        """缓存最近一帧图像，供拍照服务保存。"""
        with self._data_lock:
            self._latest_image = msg
            self._record_signal("rgb")

    def _depth_callback(self, _msg: Image):
        with self._data_lock:
            self._record_signal("depth")

    def _camera_info_callback(self, _msg: CameraInfo):
        with self._data_lock:
            self._record_signal("camera_info")

    def _pointcloud_callback(self, _msg: PointCloud2):
        with self._data_lock:
            self._record_signal("pointcloud")

    def _detections_callback(self, msg: DetectionArray):
        detections = []
        for item in msg.detections:
            if item.confidence <= 0.0 or not item.class_name:
                continue
            detections.append({
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "x": float(item.x),
                "y": float(item.y),
                "width": float(item.width),
                "height": float(item.height),
            })
        with self._data_lock:
            self._latest_detections = detections
            self._latest_detections_at = time.time()
            self._record_signal("detections", self._latest_detections_at)

    def _detector_status_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"status": "ERROR", "message": "检测器状态消息格式错误"}
        status = str(payload.get("status", "MISSING")).upper()
        with self._data_lock:
            self._detector_message = str(payload.get("message", ""))
            self._detector_error = (
                self._detector_message if status == "ERROR" else ""
            )

    def _capture_image_callback(self, request, response):
        self.get_logger().info("收到拍照保存请求")
        with self._data_lock:
            msg = self._latest_image
        if msg is None or not msg.data:
            response.success = False
            response.message = f"暂无可保存图像，请确认 {self._image_topic} 有数据"
            self.get_logger().warn(response.message)
            return response

        try:
            os.makedirs(self._capture_dir, exist_ok=True)
            stamp = self.get_clock().now().to_msg()
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            label = self._safe_label(request.label or "front_rgb")
            file_path = self._save_image_file(msg, label, timestamp)

            response.success = True
            response.message = f"图像已保存: {file_path}"
            response.file_path = file_path
            response.width = int(msg.width)
            response.height = int(msg.height)
            response.encoding = msg.encoding
            response.timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
        except Exception as exc:
            response.success = False
            response.message = f"保存图像失败: {exc}"

        self.get_logger().info(response.message)
        return response

    def _analyze_image_callback(self, request, response):
        prompt = request.prompt.strip() or "请分析前视相机画面中的场景、目标、风险和下一步建议。"
        with self._data_lock:
            msg = self._latest_image
        if msg is None or not msg.data:
            response.success = False
            response.message = f"暂无可分析图像，请确认 {self._image_topic} 有数据"
            response.risk_level = "unknown"
            return response

        result = self._analyze_latest_image(prompt, bool(request.use_vlm))
        response.success = True
        response.message = result["message"]
        response.scene = result["scene"]
        response.risk_level = result["risk_level"]
        response.suggestion = result["suggestion"]
        response.objects_json = json.dumps(result["objects"], ensure_ascii=False)
        response.raw_response = json.dumps(result, ensure_ascii=False)
        response.width = int(msg.width)
        response.height = int(msg.height)
        response.encoding = msg.encoding
        return response

    def _analyze_latest_image(self, prompt: str, use_vlm_request: bool):
        with self._data_lock:
            msg = self._latest_image
            detections = (
                list(self._latest_detections)
                if time.time() - self._latest_detections_at <= 2.0
                else []
            )
        if msg is None or not msg.data:
            raise RuntimeError(f"暂无可分析图像，请确认 {self._image_topic} 有数据")
        fallback = self._rule_image_analysis(msg, prompt, detections)
        analysis_payload = dict(fallback)
        use_vlm = bool(use_vlm_request and self._vlm_enabled and self._vlm_api_url and self._vlm_model)
        if not use_vlm:
            self._publish_image_analysis(analysis_payload)
            return analysis_payload

        try:
            vlm = self._call_vlm(msg, prompt, fallback)
            with self._data_lock:
                self._vlm_last_at = time.time()
                self._vlm_error = ""
                self._vlm_message = "最近一次 VLM 分析成功"
            message = vlm.get("message") or vlm.get("summary") or fallback["message"]
            scene = vlm.get("scene") or fallback["scene"]
            risk_level = vlm.get("risk_level") or fallback["risk_level"]
            suggestion = vlm.get("suggestion") or fallback["suggestion"]
            objects = vlm.get("objects") if isinstance(vlm.get("objects"), list) else fallback["objects"]
            analysis_payload = {
                "message": message,
                "scene": scene,
                "risk_level": risk_level,
                "suggestion": suggestion,
                "objects": objects,
                "prompt": prompt,
                "image": {
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": msg.encoding,
                },
                "source": "vlm",
                "raw_response": vlm,
            }
        except Exception as exc:
            with self._data_lock:
                self._vlm_last_at = time.time()
                self._vlm_error = str(exc)
                self._vlm_message = f"VLM 调用失败: {exc}"
            analysis_payload["message"] = f"{fallback['message']}（VLM 调用失败，已使用规则摘要：{exc}）"
            analysis_payload["source"] = "rule+detection+vlm_failed"
            analysis_payload["vlm_error"] = str(exc)
            self.get_logger().warn(f"VLM 图像分析失败: {exc}")
        self._publish_image_analysis(analysis_payload)
        return analysis_payload

    def _auto_analyze_callback(self):
        if not self._auto_analyze_enabled:
            return
        now = time.time()
        with self._data_lock:
            if now - self._last_auto_analysis_time < self._auto_analyze_interval_sec:
                return
            if self._latest_image is None or not self._latest_image.data:
                return
            self._last_auto_analysis_time = now
        self._analyze_latest_image(
            "自动低频分析当前无人机前视画面，输出场景、目标、风险和是否建议继续飞行。",
            self._auto_analyze_use_vlm,
        )

    def _publish_image_analysis(self, payload: dict):
        payload = dict(payload)
        payload["stamp"] = time.time()
        with self._data_lock:
            self._latest_analysis = payload
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._image_analysis_pub.publish(msg)

    def _rule_image_analysis(self, msg: Image, prompt: str, detections: list[dict]):
        objects = [
            {
                "name": item["class_name"],
                "confidence": round(item["confidence"], 3),
                "bbox": [item["x"], item["y"], item["width"], item["height"]],
            }
            for item in detections
        ]
        object_names = [item["name"] for item in objects]
        if object_names:
            scene = f"前视 RGB 画面可用，检测到 {', '.join(object_names[:6])}"
        else:
            scene = "前视 RGB 画面可用，当前未收到稳定目标检测结果"

        risk_level = "low"
        suggestion = "可继续低速观察，必要时结合深度/点云确认距离"
        if any(name in ("person", "人") for name in object_names):
            risk_level = "medium"
            suggestion = "画面中有人，建议悬停观察或保持安全距离"
        if any(name in ("car", "truck", "bus", "vehicle", "汽车", "车辆") for name in object_names):
            risk_level = "medium"
            suggestion = "画面中有车辆目标，建议低速接近并保持可视确认"

        return {
            "message": "图像语义分析完成",
            "scene": scene,
            "risk_level": risk_level,
            "suggestion": suggestion,
            "objects": objects,
            "prompt": prompt,
            "image": {
                "width": int(msg.width),
                "height": int(msg.height),
                "encoding": msg.encoding,
            },
            "source": "rule+detection",
        }

    def _call_vlm(self, msg: Image, prompt: str, fallback: dict):
        if not self._vlm_api_key:
            raise RuntimeError(
                "VLM API Key 未配置，请设置 AEROMIND_VLM_API_KEY 或 DASHSCOPE_API_KEY"
            )
        image_b64 = self._image_png_base64(msg)
        if not image_b64:
            raise RuntimeError("当前图像编码无法转换为 PNG")
        user_text = (
            f"{prompt}\n"
            "请返回 JSON，对象字段必须包含：message, scene, risk_level, suggestion, objects。"
            "risk_level 只能是 low/medium/high。"
            "只输出一个合法 JSON 对象，不要使用 Markdown 代码块，不要输出解释或推理过程；"
            "字符串内部的双引号、换行和反斜杠必须按 JSON 规范转义。"
            f"当前 YOLO/规则摘要：{json.dumps(fallback, ensure_ascii=False)}"
        )
        body = json.dumps(
            {
                "model": self._vlm_model,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": user_text},
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/png;base64,{image_b64}"},
                            },
                        ],
                    }
                ],
                "temperature": 0.2,
                "response_format": {"type": "json_object"},
            },
            ensure_ascii=False,
        ).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self._vlm_api_key:
            headers["Authorization"] = f"Bearer {self._vlm_api_key}"
        request = urllib.request.Request(
            self._vlm_api_url.rstrip("/") + "/chat/completions",
            data=body,
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self._vlm_timeout_sec) as http_response:
            payload = json.loads(http_response.read().decode("utf-8"))
        content = payload["choices"][0]["message"]["content"]
        result = normalize_vlm_content(content, fallback)
        if result.get("format_warning"):
            self.get_logger().warn(
                "VLM 返回非标准 JSON，已保留语义文本并使用规则字段兜底"
            )
        return result

    def _image_png_base64(self, msg: Image):
        if PilImage is None:
            return ""
        encoding = msg.encoding.lower()
        if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8", "8uc1"):
            return ""
        if encoding in ("mono8", "8uc1"):
            data = self._mono_bytes(msg)
            image = PilImage.frombytes("L", (int(msg.width), int(msg.height)), data)
        else:
            data = self._rgb_bytes(msg, encoding)
            image = PilImage.frombytes("RGB", (int(msg.width), int(msg.height)), data)
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return base64.b64encode(buffer.getvalue()).decode("ascii")

    def _extract_json_object(self, content: str):
        return extract_vlm_json(content)

    def _save_image_file(self, msg: Image, label: str, timestamp: str):
        encoding = msg.encoding.lower()
        if encoding in ("rgb8", "bgr8", "rgba8", "bgra8"):
            rgb_data = self._rgb_bytes(msg, encoding)
            if PilImage is not None:
                file_path = os.path.join(self._capture_dir, f"{timestamp}_{label}.png")
                image = PilImage.frombytes("RGB", (int(msg.width), int(msg.height)), rgb_data)
                image.save(file_path)
                return file_path
            ext = "ppm"
            data = rgb_data
            header = f"P6\n{int(msg.width)} {int(msg.height)}\n255\n".encode("ascii")
        elif encoding in ("mono8", "8uc1"):
            mono_data = self._mono_bytes(msg)
            if PilImage is not None:
                file_path = os.path.join(self._capture_dir, f"{timestamp}_{label}.png")
                image = PilImage.frombytes("L", (int(msg.width), int(msg.height)), mono_data)
                image.save(file_path)
                return file_path
            ext = "pgm"
            data = mono_data
            header = f"P5\n{int(msg.width)} {int(msg.height)}\n255\n".encode("ascii")
        else:
            ext = "raw"
            data = bytes(msg.data)
            header = b""

        file_path = os.path.join(self._capture_dir, f"{timestamp}_{label}.{ext}")
        with open(file_path, "wb") as f:
            f.write(header)
            f.write(data)
        return file_path

    def _rgb_bytes(self, msg: Image, encoding: str):
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        source = bytes(msg.data)
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        packed_step = width * channels
        if encoding == "rgb8" and step == packed_step:
            return source[: height * packed_step]

        rows = []
        for y in range(height):
            row_start = y * step
            row = source[row_start : row_start + width * channels]
            if encoding == "rgb8":
                rows.append(row)
            elif encoding == "rgba8":
                rows.append(self._strip_alpha(row, "rgb"))
            elif encoding == "bgra8":
                rows.append(self._strip_alpha(row, "bgr"))
            else:
                rows.append(self._bgr_to_rgb(row))
        return b"".join(rows)

    def _bgr_to_rgb(self, row: bytes):
        out = bytearray()
        for i in range(0, len(row), 3):
            if i + 3 > len(row):
                break
            out.extend((row[i + 2], row[i + 1], row[i]))
        return bytes(out)

    def _strip_alpha(self, row: bytes, order: str):
        out = bytearray()
        for i in range(0, len(row), 4):
            if i + 4 > len(row):
                break
            if order == "rgb":
                out.extend((row[i], row[i + 1], row[i + 2]))
            else:
                out.extend((row[i + 2], row[i + 1], row[i]))
        return bytes(out)

    def _mono_bytes(self, msg: Image):
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        source = bytes(msg.data)
        if step == width:
            return source[: height * width]
        rows = []
        for y in range(height):
            row_start = y * step
            rows.append(source[row_start : row_start + width])
        return b"".join(rows)

    def _safe_label(self, value: str):
        label = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip())
        return label[:48] or "capture"

    def _record_signal(self, name, received_at=None):
        self._signal_times[name].append(float(received_at or time.time()))

    def _signal_age(self, name, now):
        values = self._signal_times[name]
        return max(0.0, now - values[-1]) if values else None

    def _signal_rate(self, name):
        values = self._signal_times[name]
        if len(values) < 2:
            return 0.0
        span = values[-1] - values[0]
        return (len(values) - 1) / span if span > 0.0 else 0.0

    @staticmethod
    def _age_value(age):
        return float(age) if age is not None else float("nan")

    def _publish_health(self):
        now = time.time()
        with self._data_lock:
            ages = {
                name: self._signal_age(name, now)
                for name in self._signal_times
            }
            rates = {
                name: self._signal_rate(name)
                for name in self._signal_times
            }
            detector_error = self._detector_error
            vlm_error = self._vlm_error
            vlm_last_at = self._vlm_last_at
            vlm_message = self._vlm_message
        statuses = {
            "rgb": classify_signal(ages["rgb"], self._sensor_timeout_sec),
            "depth": classify_signal(ages["depth"], self._sensor_timeout_sec),
            "camera_info": classify_signal(ages["camera_info"], 5.0),
            "pointcloud": classify_signal(ages["pointcloud"], self._sensor_timeout_sec),
            "detections": classify_signal(
                ages["detections"], self._sensor_timeout_sec,
                enabled=self._yolo_enabled, error=detector_error,
            ),
        }
        if not self._vlm_enabled:
            vlm_status = "DISABLED"
        elif vlm_error:
            vlm_status = "ERROR"
        else:
            vlm_status = "OK"
        required = [statuses["rgb"], statuses["depth"], statuses["camera_info"], statuses["pointcloud"]]
        if "ERROR" in required or statuses["rgb"] in {"MISSING", "STALE"}:
            overall = "ERROR"
        elif any(value != "OK" for value in required + [statuses["detections"]] if value != "DISABLED"):
            overall = "DEGRADED"
        else:
            overall = "OK"

        msg = PerceptionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "perception"
        msg.overall_status = overall
        msg.yolo_enabled = self._yolo_enabled
        msg.vlm_enabled = self._vlm_enabled
        msg.rgb_status = statuses["rgb"]
        msg.rgb_age_sec = self._age_value(ages["rgb"])
        msg.rgb_rate_hz = float(rates["rgb"])
        msg.depth_status = statuses["depth"]
        msg.depth_age_sec = self._age_value(ages["depth"])
        msg.depth_rate_hz = float(rates["depth"])
        msg.camera_info_status = statuses["camera_info"]
        msg.camera_info_age_sec = self._age_value(ages["camera_info"])
        msg.pointcloud_status = statuses["pointcloud"]
        msg.pointcloud_age_sec = self._age_value(ages["pointcloud"])
        msg.pointcloud_rate_hz = float(rates["pointcloud"])
        msg.detections_status = statuses["detections"]
        msg.detections_age_sec = self._age_value(ages["detections"])
        msg.detections_rate_hz = float(rates["detections"])
        msg.vlm_status = vlm_status
        msg.vlm_age_sec = self._age_value(
            max(0.0, now - vlm_last_at) if vlm_last_at else None
        )
        msg.vlm_message = vlm_message
        msg.message = (
            f"RGB={statuses['rgb']} Depth={statuses['depth']} "
            f"CameraInfo={statuses['camera_info']} LiDAR={statuses['pointcloud']} "
            f"YOLO={statuses['detections']} VLM={vlm_status}"
        )
        self._health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
