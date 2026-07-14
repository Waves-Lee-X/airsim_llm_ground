"""Validated OpenAI-compatible vision analysis for uploaded images."""

from __future__ import annotations

import asyncio
import base64
from concurrent.futures import Future
import json
import threading
import urllib.request

from PIL import Image

from .config import GatewayConfig


ALLOWED_FORMATS = {
    "JPEG": ("image/jpeg", ".jpg"),
    "PNG": ("image/png", ".png"),
    "WEBP": ("image/webp", ".webp"),
}


class VisionAnalyzer:
    def __init__(self, config: GatewayConfig):
        self._api_url = config.vlm_api_url
        self._api_key = config.vlm_api_key
        self._model = config.vlm_model
        self._timeout = config.vlm_timeout_sec
        self._max_bytes = config.feishu_image_max_bytes

    @property
    def available(self) -> bool:
        return bool(self._api_url and self._model)

    def validate(self, data: bytes) -> dict:
        if not data:
            raise ValueError("图片内容为空")
        if len(data) > self._max_bytes:
            raise ValueError(
                f"图片超过大小限制: {len(data)} > {self._max_bytes} bytes"
            )
        try:
            from io import BytesIO

            with Image.open(BytesIO(data)) as image:
                image_format = str(image.format or "").upper()
                width, height = image.size
                image.verify()
        except Exception as exc:
            raise ValueError(f"无法识别或图片已损坏: {exc}") from exc
        if image_format not in ALLOWED_FORMATS:
            raise ValueError("仅支持 JPEG、PNG 和 WEBP 图片")
        if width <= 0 or height <= 0 or width > 12000 or height > 12000:
            raise ValueError("图片尺寸无效或超过 12000x12000")
        if width * height > 25_000_000:
            raise ValueError("图片总像素超过 2500 万")
        mime_type, extension = ALLOWED_FORMATS[image_format]
        return {
            "format": image_format,
            "mime_type": mime_type,
            "extension": extension,
            "width": width,
            "height": height,
            "size_bytes": len(data),
        }

    async def analyze(self, data: bytes, prompt: str, metadata: dict) -> str:
        if not self.available:
            raise RuntimeError(
                "VLM 未配置，请设置 AEROMIND_VLM_API_URL、"
                "AEROMIND_VLM_MODEL 和 AEROMIND_VLM_API_KEY"
            )
        future: Future = Future()
        thread = threading.Thread(
            target=self._analyze_worker,
            args=(future, data, prompt, metadata),
            name="agent-gateway-vlm",
            daemon=True,
        )
        thread.start()
        while not future.done():
            await asyncio.sleep(0.02)
        return future.result()

    def _analyze_worker(
        self, future: Future, data: bytes, prompt: str, metadata: dict
    ):
        try:
            future.set_result(self._call_api(data, prompt, metadata))
        except Exception as exc:
            future.set_exception(exc)

    def _call_api(self, data: bytes, prompt: str, metadata: dict) -> str:
        encoded = base64.b64encode(data).decode("ascii")
        user_text = (
            f"{prompt}\n"
            "请使用中文说明：画面场景、主要目标、潜在风险、可执行建议。"
            "明确区分可见事实和推断，不确定时直接说明。\n"
            f"图像元数据：{json.dumps(metadata, ensure_ascii=False)}"
        )
        payload = {
            "model": self._model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": user_text},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": (
                                    f"data:{metadata['mime_type']};base64,{encoded}"
                                )
                            },
                        },
                    ],
                }
            ],
            "temperature": 0.2,
        }
        headers = {"Content-Type": "application/json"}
        if self._api_key:
            headers["Authorization"] = f"Bearer {self._api_key}"
        request = urllib.request.Request(
            self._api_url.rstrip("/") + "/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self._timeout) as response:
            result = json.loads(response.read().decode("utf-8"))
        content = result["choices"][0]["message"]["content"]
        if isinstance(content, list):
            content = "".join(
                str(item.get("text", ""))
                for item in content
                if isinstance(item, dict)
            )
        text = str(content or "").strip()
        if not text:
            raise RuntimeError("VLM 未返回可显示内容")
        return text
