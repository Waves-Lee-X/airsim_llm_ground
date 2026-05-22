from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import tempfile
import time
from typing import Any, Optional

import cv2
import numpy as np


class YOLOv5Wrapper:
    """包装YOLOv5模型以提供与ultralytics YOLOv8兼容的接口"""
    
    def __init__(self, model: Any) -> None:
        self._model = model
        self.names = getattr(model, 'names', {})
    
    def __call__(self, image: np.ndarray, conf: float = 0.25, verbose: bool = False) -> list[Any]:
        """提供与YOLOv8兼容的调用接口"""
        # 设置置信度阈值
        self._model.conf = conf
        
        # 执行推理
        results = self._model(image)
        
        # 包装结果以提供兼容的boxes接口
        return [YOLOv5ResultWrapper(results)]


class YOLOv5ResultWrapper:
    """包装YOLOv5结果以提供与YOLOv8结果兼容的接口"""
    
    def __init__(self, results: Any) -> None:
        self._results = results
        self.names = getattr(results, 'names', {})
        self.boxes = YOLOv5BoxesWrapper(results)


class YOLOv5BoxesWrapper:
    """包装YOLOv5的检测结果以提供与YOLOv8 boxes兼容的接口"""
    
    def __init__(self, results: Any) -> None:
        self._results = results
        self._detections = self._extract_detections()
    
    def _extract_detections(self) -> list[dict[str, Any]]:
        """从YOLOv5结果中提取检测信息"""
        detections = []
        try:
            # YOLOv5结果通常有pandas格式的xyxy属性
            if hasattr(self._results, 'xyxy') and len(self._results.xyxy) > 0:
                for det in self._results.xyxy[0]:
                    # det格式: [x1, y1, x2, y2, conf, cls]
                    detections.append({
                        'xyxy': [[float(det[0]), float(det[1]), float(det[2]), float(det[3])]],
                        'conf': [float(det[4])],
                        'cls': [int(det[5])]
                    })
            # 或者使用pandas数据框
            elif hasattr(self._results, 'pandas') and self._results.pandas() is not None:
                df = self._results.pandas().xyxy[0]
                for _, row in df.iterrows():
                    detections.append({
                        'xyxy': [[row['xmin'], row['ymin'], row['xmax'], row['ymax']]],
                        'conf': [row['confidence']],
                        'cls': [int(row['class'])]
                    })
        except Exception:
            pass
        return detections
    
    def __iter__(self):
        """使boxes可迭代"""
        for det in self._detections:
            yield YOLOv5BoxWrapper(det)
    
    def __len__(self) -> int:
        return len(self._detections)


class YOLOv5BoxWrapper:
    """包装单个YOLOv5检测框以提供与YOLOv8 box兼容的接口"""
    
    def __init__(self, detection: dict[str, Any]) -> None:
        self.xyxy = detection.get('xyxy')
        self.conf = detection.get('conf')
        self.cls = detection.get('cls')


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    bbox: tuple[int, int, int, int]

    def to_api(self) -> dict[str, object]:
        return {
            "label": self.label,
            "confidence": round(float(self.confidence), 4),
            "bbox": list(self.bbox),
        }


@dataclass(frozen=True)
class DetectionResult:
    ok: bool
    detections: list[Detection]
    model_path: str = ""
    backend: str = ""
    message: str = ""
    elapsed_ms: int = 0
    image_shape: tuple[int, int] | None = None

    def to_api(self) -> dict[str, object]:
        return {
            "ok": self.ok,
            "detections": [item.to_api() for item in self.detections],
            "model_path": self.model_path,
            "backend": self.backend,
            "message": self.message,
            "elapsed_ms": self.elapsed_ms,
            "image_shape": list(self.image_shape) if self.image_shape else None,
        }


class VisionDetector:
    def __init__(self, model_path: str | Path | None = None, confidence: float = 0.25) -> None:
        self.model_path = self._resolve_model_path(model_path)
        self.confidence = float(confidence)
        self._model: Any = None
        self._load_error = ""
        self.backend = "ultralytics"
        self._supported_labels: set[str] = set()
        self._last_detection_time = 0.0
        self._detection_interval_ms = 100

    def status(self) -> dict[str, object]:
        return {
            "available": self.model_path is not None,
            "loaded": self._model is not None,
            "model_path": str(self.model_path) if self.model_path else "",
            "backend": self.backend,
            "error": self._load_error,
            "confidence": self.confidence,
            "supported_labels": sorted(list(self._supported_labels)) if self._model else [],
            "last_detection_time": self._last_detection_time,
        }

    def detect(self, image: bytes, target_filter: str | None = None) -> DetectionResult:
        started = time.time()
        
        if not image:
            return DetectionResult(False, [], self._model_path_text(), self.backend, "没有可用图像帧。")
        
        model = self._load_model()
        if model is None:
            message = self._load_error or "未找到本地 YOLO 模型或缺少 ultralytics 依赖。"
            return DetectionResult(False, [], self._model_path_text(), self.backend, message)

        try:
            image_array = self._decode_image(image)
            if image_array is None:
                return DetectionResult(False, [], self._model_path_text(), self.backend, "无法解码图像")
            
            image_shape = (image_array.shape[0], image_array.shape[1])
            
            results = model(image_array, conf=self.confidence, verbose=False)
            detections = self._parse_ultralytics(results)
            
            if target_filter:
                detections = self._filter_by_target(detections, target_filter)
            
            self._last_detection_time = time.time()
            
            return DetectionResult(
                True,
                detections,
                self._model_path_text(),
                self.backend,
                f"检测完成，发现 {len(detections)} 个目标。",
                int((time.time() - started) * 1000),
                image_shape,
            )
        except Exception as exc:
            return DetectionResult(
                False, [], self._model_path_text(), self.backend, f"YOLO 推理失败：{exc}"
            )

    def detect_batch(self, images: list[bytes], target_filter: str | None = None) -> list[DetectionResult]:
        results = []
        for image in images:
            result = self.detect(image, target_filter)
            results.append(result)
        return results

    def detect_with_visualization(self, image: bytes, target_filter: str | None = None) -> tuple[DetectionResult, bytes]:
        result = self.detect(image, target_filter)
        if not result.ok or not result.detections:
            return result, image
        
        image_array = self._decode_image(image)
        if image_array is None:
            return result, image
        
        image_with_boxes = self._draw_detections(image_array, result.detections)
        return result, self._encode_image(image_with_boxes)

    def _decode_image(self, image_bytes: bytes) -> np.ndarray | None:
        try:
            nparr = np.frombuffer(image_bytes, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if image is None:
                image = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
                if image is not None:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            return image
        except Exception:
            return None

    def _encode_image(self, image_array: np.ndarray) -> bytes:
        success, encoded = cv2.imencode(".jpg", image_array)
        return encoded.tobytes() if success else b""

    def _draw_detections(self, image: np.ndarray, detections: list[Detection]) -> np.ndarray:
        output = image.copy()
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            color = (0, 255, 0)
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            label = f"{det.label}: {det.confidence:.2f}"
            cv2.putText(output, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return output

    def _filter_by_target(self, detections: list[Detection], target: str) -> list[Detection]:
        target_lower = target.lower()
        return [
            det for det in detections
            if target_lower in det.label.lower()
        ]

    def _load_model(self) -> Any:
        if self._model is not None:
            return self._model
        if self.model_path is None:
            self._load_error = "未找到本地权重文件。"
            return None
        try:
            from ultralytics import YOLO

            self._model = YOLO(str(self.model_path))
            self._load_error = ""
            self.backend = "ultralytics"
            
            if hasattr(self._model, 'names') and self._model.names:
                self._supported_labels = set(self._model.names.values())
            
            return self._model
        except ImportError:
            self._load_error = "缺少 ultralytics 库，请安装: pip install ultralytics"
            return None
        except Exception as exc:
            error_msg = str(exc)
            # 检查是否是YOLOv5模型兼容性问题
            if "YOLOv5" in error_msg and "forwards compatible" in error_msg:
                return self._try_load_yolov5_model()
            self._load_error = f"无法加载 YOLO：{exc}"
            return None

    def _try_load_yolov5_model(self) -> Any:
        """尝试使用YOLOv5兼容模式加载模型"""
        try:
            # 尝试使用torch.hub加载YOLOv5模型
            import torch
            
            # 使用YOLOv5的torch.hub加载方式
            model = torch.hub.load(
                'ultralytics/yolov5',
                'custom',
                path=str(self.model_path),
                force_reload=False,
                verbose=False
            )
            
            # 包装模型以提供兼容的接口
            self._model = YOLOv5Wrapper(model)
            self.backend = "yolov5-torchhub"
            self._load_error = ""
            
            # 获取支持的标签
            if hasattr(model, 'names') and model.names:
                self._supported_labels = set(model.names.values())
            
            return self._model
        except ImportError:
            self._load_error = "无法加载YOLOv5模型：缺少torch或ultralytics依赖"
            return None
        except Exception as exc:
            self._load_error = f"YOLOv5兼容模式加载失败：{exc}"
            return None

    def _parse_ultralytics(self, results: Any) -> list[Detection]:
        detections: list[Detection] = []
        if not results:
            return detections
        result = results[0]
        names = getattr(result, "names", {}) or {}
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return detections
        for box in boxes:
            xyxy = getattr(box, "xyxy", None)
            conf = getattr(box, "conf", None)
            cls = getattr(box, "cls", None)
            if xyxy is None or conf is None or cls is None:
                continue
            coords = xyxy[0].tolist()
            class_id = int(cls[0].item())
            label = str(names.get(class_id, class_id))
            confidence = float(conf[0].item())
            detections.append(
                Detection(
                    label=label,
                    confidence=confidence,
                    bbox=(int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])),
                )
            )
        return detections

    def _model_path_text(self) -> str:
        return str(self.model_path) if self.model_path else ""

    @staticmethod
    def _resolve_model_path(model_path: str | Path | None) -> Path | None:
        if model_path:
            path = Path(model_path)
            return path if path.exists() else None
        
        root = Path(__file__).resolve().parents[2]
        candidates = (
            root / "Airsim_Yolov5_ODRT-master" / "my_best.pt",
            root / "Airsim_Yolov5_ODRT-master" / "yolov8n.pt",
            root / "Airsim_Yolov5_ODRT-master" / "yolov5s.pt",
            root / "models" / "yolov8n.pt",
            root / "models" / "yolov5s.pt",
        )
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None