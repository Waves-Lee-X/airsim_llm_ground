"""Tool-capable streaming runtime for OpenAI-compatible model providers."""

from __future__ import annotations

import asyncio
import json
from typing import Any

import httpx

from .claude_runtime import SYSTEM_PROMPT
from .config import GatewayConfig, OpenAIProviderConfig
from .drone_tools import (
    OPENAI_DRONE_TOOLS,
    ControlRequester,
    execute_openai_drone_tool,
)
from .ros_state import RosStateBridge
from .runtime import EmitCallback


class OpenAICompatibleRuntime:
    runtime_session_id = None

    def __init__(
        self,
        config: GatewayConfig,
        provider: OpenAIProviderConfig,
        ros_state: RosStateBridge,
        model: str,
        history: list[dict[str, Any]] | None = None,
        request_control: ControlRequester | None = None,
    ):
        self.provider = provider.name
        self.model = model
        self._config = config
        self._provider = provider
        self._ros_state = ros_state
        self._request_control = request_control
        self._client: httpx.AsyncClient | None = None
        self._lock = asyncio.Lock()
        self._interrupted = False
        self._messages = [
            {
                "role": item["role"],
                "content": (
                    item["content"]
                    if isinstance(item.get("content"), str)
                    else json.dumps(item.get("content"), ensure_ascii=False)
                ),
            }
            for item in (history or [])
            if item.get("role") in {"user", "assistant"}
        ][-40:]

    async def start(self):
        if self._client is not None:
            return
        if not self._provider.api_key:
            raise RuntimeError(f"Provider {self.provider} 未配置 API Key")
        self._validate_model(self.model)
        self._client = httpx.AsyncClient(
            base_url=self._provider.base_url,
            headers={"Authorization": f"Bearer {self._provider.api_key}"},
            timeout=httpx.Timeout(90.0, connect=15.0),
        )

    async def close(self):
        if self._client is not None:
            await self._client.aclose()
            self._client = None

    async def interrupt(self):
        self._interrupted = True

    async def set_model(self, model: str):
        await self.start()
        self._validate_model(model)
        self.model = model

    def _validate_model(self, model: str):
        if model not in self._provider.models:
            raise ValueError(
                f"Provider {self.provider} 不允许模型 {model}，"
                f"可用模型: {', '.join(self._provider.models)}"
            )

    async def run_turn(self, prompt: str, emit: EmitCallback) -> dict[str, Any]:
        await self.start()
        assert self._client is not None
        self._interrupted = False
        turn_messages = [*self._messages, {"role": "user", "content": prompt}]
        final_text = ""
        usage = None
        async with self._lock:
            for _ in range(self._config.max_turns):
                response = await self._stream_completion(turn_messages, emit)
                final_text += response["content"]
                usage = response.get("usage") or usage
                assistant = {
                    "role": "assistant",
                    "content": response["content"] or None,
                }
                if response["tool_calls"]:
                    assistant["tool_calls"] = response["tool_calls"]
                turn_messages.append(assistant)
                if not response["tool_calls"]:
                    self._messages = turn_messages[-40:]
                    return {
                        "text": final_text.strip(),
                        "model": self.model,
                        "provider": self.provider,
                        "runtime_session_id": None,
                        "usage": usage,
                        "cost_usd": None,
                        "is_error": False,
                        "error": "",
                    }
                for call in response["tool_calls"]:
                    result = await self._execute_tool(call, emit)
                    turn_messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": call["id"],
                            "content": json.dumps(result, ensure_ascii=False),
                        }
                    )
            return {
                "text": final_text.strip(),
                "model": self.model,
                "provider": self.provider,
                "runtime_session_id": None,
                "usage": usage,
                "cost_usd": None,
                "is_error": True,
                "error": "模型工具调用轮数超过限制",
            }

    async def _stream_completion(
        self, messages: list[dict[str, Any]], emit: EmitCallback
    ) -> dict[str, Any]:
        assert self._client is not None
        payload = {
            "model": self.model,
            "messages": [{"role": "system", "content": SYSTEM_PROMPT}, *messages],
            "tools": OPENAI_DRONE_TOOLS,
            "tool_choice": "auto",
            "stream": True,
        }
        content_parts: list[str] = []
        tool_calls: dict[int, dict[str, Any]] = {}
        usage = None
        async with self._client.stream(
            "POST", "/chat/completions", json=payload
        ) as response:
            response.raise_for_status()
            async for line in response.aiter_lines():
                if self._interrupted:
                    raise asyncio.CancelledError
                if not line.startswith("data:"):
                    continue
                data = line[5:].strip()
                if not data or data == "[DONE]":
                    continue
                chunk = json.loads(data)
                usage = chunk.get("usage") or usage
                choices = chunk.get("choices") or []
                if not choices:
                    continue
                delta = choices[0].get("delta") or {}
                text = delta.get("content") or ""
                if text:
                    content_parts.append(text)
                    await emit(
                        "assistant.delta",
                        {"delta": text, "model": self.model, "provider": self.provider},
                    )
                for fragment in delta.get("tool_calls") or []:
                    index = int(fragment.get("index", 0))
                    call = tool_calls.setdefault(
                        index,
                        {
                            "id": "",
                            "type": "function",
                            "function": {"name": "", "arguments": ""},
                        },
                    )
                    call["id"] += str(fragment.get("id") or "")
                    function = fragment.get("function") or {}
                    call["function"]["name"] += str(function.get("name") or "")
                    call["function"]["arguments"] += str(
                        function.get("arguments") or ""
                    )
        return {
            "content": "".join(content_parts),
            "tool_calls": [tool_calls[index] for index in sorted(tool_calls)],
            "usage": usage,
        }

    async def _execute_tool(
        self, call: dict[str, Any], emit: EmitCallback
    ) -> dict[str, Any]:
        name = str(call.get("function", {}).get("name", ""))
        try:
            args = json.loads(call.get("function", {}).get("arguments") or "{}")
            if not isinstance(args, dict):
                raise ValueError("工具参数必须是 JSON 对象")
        except (json.JSONDecodeError, ValueError) as exc:
            args = {}
            parse_error = str(exc)
        else:
            parse_error = ""
        await emit(
            "tool.started",
            {"tool_call_id": call.get("id"), "tool": name, "arguments": args},
        )
        try:
            if parse_error:
                raise ValueError(parse_error)
            result = await execute_openai_drone_tool(
                name, args, self._ros_state, self._request_control
            )
            await emit(
                "tool.completed",
                {
                    "tool_call_id": call.get("id"),
                    "success": True,
                    "result": result,
                },
            )
            return result
        except Exception as exc:
            result = {"success": False, "message": str(exc)}
            await emit(
                "tool.completed",
                {
                    "tool_call_id": call.get("id"),
                    "success": False,
                    "result": result,
                },
            )
            return result
