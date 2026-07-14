#!/usr/bin/env python3
"""YOLO object detection node.

Subscribes to the front RGB camera and publishes:
  - /perception/detections: all detections
  - /perception/detection: best detection for legacy consumers
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from aeromind_interfaces.msg import Detection, DetectionArray

try:
    import numpy as np
except Exception:
    np = None

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        self.declare_parameter("image_topic", "/sensor/camera/rgb/front_center")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("inference_interval_sec", 0.2)
        self.declare_parameter("max_detections", 20)

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._model_name = str(self.get_parameter("model").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._inference_interval_sec = float(self.get_parameter("inference_interval_sec").value)
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._model = None
        self._last_inference_time = 0.0

        self._detections_pub = self.create_publisher(
            DetectionArray, "/perception/detections", 10
        )
        self._best_detection_pub = self.create_publisher(
            Detection, "/perception/detection", 10
        )
        self.create_subscription(Image, self._image_topic, self._image_callback, 10)

        self._load_model()
        self.get_logger().info(
            f"YOLO 检测节点已启动: image_topic={self._image_topic}, model={self._model_name}"
        )

    def _load_model(self):
        if YOLO is None or np is None:
            self.get_logger().error(
                "YOLO 检测不可用：请安装 ultralytics 和 numpy，例如 pip install ultralytics"
            )
            return
        try:
            self._model = YOLO(self._model_name)
        except Exception as exc:
            self.get_logger().error(f"加载 YOLO 模型失败: {exc}")
            self._model = None

    def _image_callback(self, msg: Image):
        now = time.monotonic()
        if now - self._last_inference_time < self._inference_interval_sec:
            return
        self._last_inference_time = now

        if self._model is None:
            return

        frame = self._image_to_numpy(msg)
        if frame is None:
            return

        try:
            results = self._model.predict(
                source=frame,
                conf=self._confidence_threshold,
                verbose=False,
                max_det=self._max_detections,
            )
        except Exception as exc:
            self.get_logger().warn(f"YOLO 推理失败: {exc}", throttle_duration_sec=5.0)
            return

        detections = self._results_to_detections(results)
        array_msg = DetectionArray()
        array_msg.header = msg.header
        array_msg.detections = detections
        self._detections_pub.publish(array_msg)

        if detections:
            best = max(detections, key=lambda item: item.confidence)
            self._best_detection_pub.publish(best)

    def _image_to_numpy(self, msg: Image):
        if np is None:
            return None
        width = int(msg.width)
        height = int(msg.height)
        encoding = msg.encoding.lower()
        if width <= 0 or height <= 0 or not msg.data:
            return None

        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        if encoding in ("mono8", "8uc1"):
            channels = 1
        if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8", "8uc1"):
            self.get_logger().warn(f"暂不支持 YOLO 图像编码: {msg.encoding}", throttle_duration_sec=5.0)
            return None

        step = int(msg.step) or width * channels
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        rows = []
        for y in range(height):
            start = y * step
            rows.append(data[start : start + width * channels])
        packed = np.concatenate(rows)

        if channels == 1:
            image = packed.reshape((height, width))
            return np.stack([image, image, image], axis=-1)

        image = packed.reshape((height, width, channels))
        if encoding == "bgr8":
            image = image[:, :, ::-1]
        elif encoding == "rgba8":
            image = image[:, :, :3]
        elif encoding == "bgra8":
            image = image[:, :, [2, 1, 0]]
        return image

    def _results_to_detections(self, results):
        detections = []
        if not results:
            return detections
        result = results[0]
        names = getattr(result, "names", {}) or {}
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return detections

        for box in boxes:
            try:
                xywh = box.xywh[0].tolist()
                cls_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
            except Exception:
                continue
            detection = Detection()
            detection.class_name = str(names.get(cls_id, cls_id))
            detection.confidence = confidence
            detection.x = float(xywh[0])
            detection.y = float(xywh[1])
            detection.width = float(xywh[2])
            detection.height = float(xywh[3])
            detections.append(detection)
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
