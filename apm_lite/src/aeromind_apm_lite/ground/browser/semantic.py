"""Non-ROS visual semantics and mission-preview services."""

from __future__ import annotations

import asyncio
import base64
import json
import os
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from .camera import CameraFrame


class SemanticUnavailable(RuntimeError):
    """Raised when model configuration or input is unavailable."""


class SemanticBusy(RuntimeError):
    """Raised when another model request is already in progress."""


@dataclass(frozen=True)
class SemanticConfig:
    api_url: str = ""
    api_key: str = ""
    vision_model: str = ""
    mission_model: str = ""
    timeout_s: float = 30.0
    max_image_bytes: int = 10 * 1024 * 1024

    @classmethod
    def from_env(cls) -> "SemanticConfig":
        vision_model = os.environ.get("AEROMIND_VLM_MODEL", "").strip()
        return cls(
            api_url=os.environ.get("AEROMIND_VLM_API_URL", "").strip(),
            api_key=(
                os.environ.get("AEROMIND_VLM_API_KEY", "").strip()
                or os.environ.get("DASHSCOPE_API_KEY", "").strip()
            ),
            vision_model=vision_model,
            mission_model=(
                os.environ.get("AEROMIND_LLM_MODEL", "").strip()
                or vision_model
            ),
            timeout_s=max(
                1.0,
                float(os.environ.get("AEROMIND_VLM_TIMEOUT_SEC", "30")),
            ),
        )

    @property
    def vision_available(self) -> bool:
        return bool(self.api_url and self.vision_model)

    @property
    def mission_available(self) -> bool:
        return bool(self.api_url and self.mission_model)


def _strip_json_fence(content: str) -> str:
    text = str(content or "").strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text, flags=re.IGNORECASE)
        text = re.sub(r"\s*```$", "", text)
    return text.strip()


def extract_json_object(content: str) -> dict[str, Any]:
    """Extract a JSON object while tolerating fences and trailing commas."""
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
    raise ValueError("模型返回内容不是合法 JSON 对象")


def _confidence(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if 1.0 < result <= 100.0:
        result /= 100.0
    if not 0.0 <= result <= 1.0:
        return None
    return round(result, 4)


def _as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def normalize_visual_result(content: str) -> dict[str, Any]:
    """Return a stable browser contract even for a non-JSON model answer."""
    try:
        parsed = extract_json_object(content)
        warning = None
    except ValueError as exc:
        parsed = {}
        warning = str(exc)

    target_source = parsed.get("target")
    target = target_source if isinstance(target_source, dict) else {}
    label = target.get("label") or parsed.get("target_label")
    color = target.get("color") or parsed.get("color")
    found_value = target.get("found", parsed.get("target_found"))
    found = bool(found_value) if found_value is not None else bool(label)
    risk_level = str(parsed.get("risk_level", "unknown")).lower()
    if risk_level not in {"low", "medium", "high", "unknown"}:
        risk_level = "unknown"
    scene = str(parsed.get("scene") or "").strip()
    message = str(parsed.get("message") or "").strip()
    if not scene and not message:
        scene = _strip_json_fence(content)
        message = "视觉语义分析完成（模型返回为非结构化文本）"

    return {
        "message": message or "视觉语义分析完成",
        "scene": scene,
        "target": {
            "found": found,
            "label": str(label or "").strip() or None,
            "box_id": str(target.get("box_id") or "").strip() or None,
            "color": str(color or "").strip() or None,
            "face": str(target.get("face") or "").strip() or None,
            "confidence": _confidence(
                target.get("confidence", parsed.get("confidence"))
            ),
            "evidence": str(target.get("evidence") or "").strip() or None,
        },
        "objects": _as_list(parsed.get("objects")),
        "risk_level": risk_level,
        "suggestion": str(parsed.get("suggestion") or "").strip(),
        "mission_relevance": str(
            parsed.get("mission_relevance") or ""
        ).strip(),
        "format_warning": warning,
    }


def normalize_mission_result(content: str, instruction: str) -> dict[str, Any]:
    try:
        parsed = extract_json_object(content)
        warning = None
    except ValueError as exc:
        parsed = {}
        warning = str(exc)
    steps = [item for item in _as_list(parsed.get("steps")) if isinstance(item, dict)]
    for index, step in enumerate(steps, start=1):
        step.setdefault("order", index)
    return {
        "summary": str(parsed.get("summary") or instruction).strip(),
        "intent": str(parsed.get("intent") or "unknown").strip(),
        "vehicle_ids": _as_list(parsed.get("vehicle_ids")),
        "requires_visual": bool(parsed.get("requires_visual", False)),
        "targets": _as_list(parsed.get("targets")),
        "steps": steps,
        "constraints": _as_list(parsed.get("constraints")),
        "safety_checks": _as_list(parsed.get("safety_checks")),
        "ambiguities": _as_list(parsed.get("ambiguities")),
        "executable": False,
        "execution_policy": "preview_only",
        "format_warning": warning,
    }


class SemanticService:
    """Call an OpenAI-compatible model without granting flight authority."""

    _DEFAULT_VISION_PROMPT = (
        "识别当前画面中的箱体、编号、颜色标识及颜色所在表面，"
        "同时说明障碍物和人员等安全风险。"
    )

    def __init__(self, config: SemanticConfig | None = None) -> None:
        self.config = config or SemanticConfig.from_env()
        self._request_lock = asyncio.Lock()
        self._latest_visual: dict[str, Any] | None = None
        self._latest_mission: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._active_kind: str | None = None

    def status_payload(self) -> dict[str, Any]:
        return {
            "configured": (
                self.config.vision_available or self.config.mission_available
            ),
            "vision_available": self.config.vision_available,
            "mission_parser_available": self.config.mission_available,
            "vision_model": self.config.vision_model or None,
            "mission_model": self.config.mission_model or None,
            "busy": self._request_lock.locked(),
            "active_kind": self._active_kind,
            "last_error": self._last_error,
            "has_visual_result": self._latest_visual is not None,
            "has_mission_result": self._latest_mission is not None,
            "execution_policy": "preview_only",
        }

    def latest_payload(self) -> dict[str, Any]:
        return {
            "visual": self._latest_visual,
            "mission": self._latest_mission,
            "status": self.status_payload(),
        }

    async def analyze_frame(
        self,
        frame: CameraFrame,
        prompt: str = "",
    ) -> dict[str, Any]:
        if not self.config.vision_available:
            raise SemanticUnavailable(
                "VLM 未配置，请设置 AEROMIND_VLM_API_URL 和 "
                "AEROMIND_VLM_MODEL"
            )
        if not frame.data:
            raise SemanticUnavailable("当前相机帧为空")
        if len(frame.data) > self.config.max_image_bytes:
            raise SemanticUnavailable("当前相机帧超过 VLM 图像大小限制")
        prompt = prompt.strip() or self._DEFAULT_VISION_PROMPT
        request_started = time.monotonic()
        async with self._exclusive_request("visual"):
            try:
                content = await asyncio.to_thread(
                    self._call_vision,
                    frame,
                    prompt,
                )
                result = normalize_visual_result(content)
                payload = {
                    "kind": "visual_analysis",
                    "analyzed_at_utc": datetime.now(timezone.utc).isoformat(),
                    "latency_ms": round(
                        (time.monotonic() - request_started) * 1000.0,
                        1,
                    ),
                    "model": self.config.vision_model,
                    "prompt": prompt,
                    "frame": {
                        "sequence": frame.sequence,
                        "captured_at_utc": frame.captured_at_utc.isoformat(),
                        "media_type": frame.media_type,
                        "size_bytes": len(frame.data),
                    },
                    "result": result,
                    "raw_response": content,
                    "flight_command_generated": False,
                }
                self._latest_visual = payload
                self._last_error = None
                return payload
            except Exception as exc:
                self._last_error = self._public_error(exc)
                raise

    async def parse_mission(self, instruction: str) -> dict[str, Any]:
        if not self.config.mission_available:
            raise SemanticUnavailable(
                "任务解析模型未配置，请设置 AEROMIND_VLM_API_URL 和模型名"
            )
        instruction = instruction.strip()
        if not instruction:
            raise ValueError("任务指令不能为空")
        request_started = time.monotonic()
        visual_context = (
            self._latest_visual.get("result") if self._latest_visual else None
        )
        async with self._exclusive_request("mission"):
            try:
                content = await asyncio.to_thread(
                    self._call_mission,
                    instruction,
                    visual_context,
                )
                plan = normalize_mission_result(content, instruction)
                payload = {
                    "kind": "mission_preview",
                    "parsed_at_utc": datetime.now(timezone.utc).isoformat(),
                    "latency_ms": round(
                        (time.monotonic() - request_started) * 1000.0,
                        1,
                    ),
                    "model": self.config.mission_model,
                    "instruction": instruction,
                    "visual_context_used": visual_context is not None,
                    "plan": plan,
                    "raw_response": content,
                    "flight_command_generated": False,
                }
                self._latest_mission = payload
                self._last_error = None
                return payload
            except Exception as exc:
                self._last_error = self._public_error(exc)
                raise

    class _RequestContext:
        def __init__(self, service: "SemanticService", kind: str) -> None:
            self.service = service
            self.kind = kind

        async def __aenter__(self) -> None:
            if self.service._request_lock.locked():
                raise SemanticBusy("已有视觉或任务解析请求正在执行")
            await self.service._request_lock.acquire()
            self.service._active_kind = self.kind

        async def __aexit__(self, exc_type, exc, traceback) -> None:
            self.service._active_kind = None
            self.service._request_lock.release()

    def _exclusive_request(self, kind: str) -> "SemanticService._RequestContext":
        return self._RequestContext(self, kind)

    def _call_vision(self, frame: CameraFrame, prompt: str) -> str:
        encoded = base64.b64encode(frame.data).decode("ascii")
        system_text = (
            "你是无人机视觉语义分析器。只描述图像中可见或有证据支持的内容，"
            "不得把推测表述为事实。只返回一个合法 JSON 对象，不输出 Markdown。"
        )
        user_text = (
            f"任务：{prompt}\n"
            "返回字段：message, scene, target, objects, risk_level, suggestion, "
            "mission_relevance。target 必须包含 found, label, box_id, color, "
            "face, confidence, evidence；confidence 为 0 到 1。risk_level 只能为 "
            "low、medium 或 high。没有看清的字段使用 null。"
        )
        messages = [
            {"role": "system", "content": system_text},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": user_text},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:{frame.media_type};base64,{encoded}"
                        },
                    },
                ],
            },
        ]
        return self._call_api(self.config.vision_model, messages)

    def _call_mission(
        self,
        instruction: str,
        visual_context: dict[str, Any] | None,
    ) -> str:
        context_text = (
            json.dumps(visual_context, ensure_ascii=False)
            if visual_context is not None
            else "无"
        )
        system_text = (
            "你是 AeroMind APM Lite 的任务语义解析器。你的输出只是供操作员审核的"
            "任务草案，绝不能声称已经控制或执行无人机。只返回合法 JSON 对象，不输出"
            " Markdown。不要生成 MAVLink 命令或编造坐标。"
        )
        user_text = (
            f"操作员任务：{instruction}\n"
            f"最近一次视觉结果：{context_text}\n"
            "返回字段：summary, intent, vehicle_ids, requires_visual, targets, steps, "
            "constraints, safety_checks, ambiguities。steps 中每项包含 order, action, "
            "vehicle_id, target, completion_condition。缺少坐标、颜色、编号或安全前提时"
            "必须写入 ambiguities。"
        )
        messages = [
            {"role": "system", "content": system_text},
            {"role": "user", "content": user_text},
        ]
        return self._call_api(self.config.mission_model, messages)

    def _call_api(self, model: str, messages: list[dict[str, Any]]) -> str:
        payload = {
            "model": model,
            "messages": messages,
            "temperature": 0.2,
            "response_format": {"type": "json_object"},
        }
        headers = {"Content-Type": "application/json"}
        if self.config.api_key:
            headers["Authorization"] = f"Bearer {self.config.api_key}"
        request = urllib.request.Request(
            self.config.api_url.rstrip("/") + "/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=self.config.timeout_s,
            ) as response:
                result = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"模型服务返回 HTTP {exc.code}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError("无法连接模型服务") from exc
        try:
            content = result["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError("模型服务响应缺少 message.content") from exc
        if isinstance(content, list):
            content = "".join(
                str(item.get("text", ""))
                for item in content
                if isinstance(item, dict)
            )
        text = str(content or "").strip()
        if not text:
            raise RuntimeError("模型未返回可显示内容")
        return text

    @staticmethod
    def _public_error(exc: Exception) -> str:
        if isinstance(exc, (SemanticUnavailable, SemanticBusy, ValueError)):
            return str(exc)
        if isinstance(exc, RuntimeError):
            return str(exc)
        return "视觉语义服务执行失败"
