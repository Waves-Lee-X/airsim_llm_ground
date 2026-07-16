#!/usr/bin/env python3
"""
perception_node.py - 感知节点（占位）

后续将挂载：
  - YOLO 目标检测 → /perception/detection
  - OctoMap 建图 → /perception/obstacle_map
  - 相机图像处理 → 订阅 /sensor/camera/image

当前提供基础图像快照保存服务，目标检测/建图仍为占位输出。
"""

import base64
import io
import json
import os
import re
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from aeromind_interfaces.msg import Detection, DetectionArray, ObstacleMap
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


class PerceptionNode(Node):
    """感知节点"""

    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("image_topic", "/sensor/camera/rgb/front_center")
        self.declare_parameter("detections_topic", "/perception/detections")
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
        self._detections_topic = str(self.get_parameter("detections_topic").value)
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
        self._latest_image = None
        self._latest_detections = []
        self._latest_detections_at = 0.0
        self._latest_analysis = None
        self._last_auto_analysis_time = 0.0

        # 发布者
        self._detection_pub = self.create_publisher(
            Detection, "/perception/detection", 10
        )
        self._obstacle_pub = self.create_publisher(
            ObstacleMap, "/perception/obstacle_map", 10
        )
        self._image_analysis_pub = self.create_publisher(
            String, "/perception/image_analysis", 10
        )

        # 订阅相机图像，供 CaptureImageSkill 保存最近一帧
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._image_callback, 10
        )
        self._detections_sub = self.create_subscription(
            DetectionArray, self._detections_topic, self._detections_callback, 10
        )

        self._capture_srv = self.create_service(
            CaptureImage, "/perception/capture_image", self._capture_image_callback
        )
        self._analyze_srv = self.create_service(
            AnalyzeImage, "/perception/analyze_image", self._analyze_image_callback
        )

        # 定时器：1Hz 发送占位消息
        self._timer = self.create_timer(1.0, self._timer_callback)
        self._auto_analyze_timer = self.create_timer(1.0, self._auto_analyze_callback)

        self.get_logger().info(
            f"感知节点已启动：image_topic={self._image_topic}, capture_dir={self._capture_dir}, "
            f"vlm_enabled={self._vlm_enabled}, "
            f"vlm_model={self._vlm_model or '未配置'}, "
            f"vlm_api_key={'已配置' if self._vlm_api_key else '未配置'}, "
            f"auto_analyze={self._auto_analyze_enabled}"
        )

    def _image_callback(self, msg: Image):
        """缓存最近一帧图像，供拍照服务保存。"""
        self._latest_image = msg

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
        self._latest_detections = detections
        self._latest_detections_at = time.time()

    def _capture_image_callback(self, request, response):
        self.get_logger().info("收到拍照保存请求")
        if self._latest_image is None or not self._latest_image.data:
            response.success = False
            response.message = f"暂无可保存图像，请确认 {self._image_topic} 有数据"
            self.get_logger().warn(response.message)
            return response

        try:
            os.makedirs(self._capture_dir, exist_ok=True)
            msg = self._latest_image
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
        msg = self._latest_image
        detections = (
            list(self._latest_detections)
            if time.time() - self._latest_detections_at <= 2.0
            else []
        )
        fallback = self._rule_image_analysis(msg, prompt, detections)
        analysis_payload = dict(fallback)
        use_vlm = bool(use_vlm_request and self._vlm_enabled and self._vlm_api_url and self._vlm_model)
        if not use_vlm:
            self._publish_image_analysis(analysis_payload)
            return analysis_payload

        try:
            vlm = self._call_vlm(msg, prompt, fallback)
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

    def _timer_callback(self):
        """定时发送空消息"""
        # 空检测结果
        detection = Detection()
        self._detection_pub.publish(detection)

        # 空障碍物地图
        obs_map = ObstacleMap()
        obs_map.timestamp = self.get_clock().now().to_msg()
        obs_map.width = 0
        obs_map.height = 0
        obs_map.resolution = 0.0
        obs_map.data = []
        self._obstacle_pub.publish(obs_map)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
