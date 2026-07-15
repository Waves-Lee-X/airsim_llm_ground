"""Persistent Claude Agent SDK conversation runtime."""

from __future__ import annotations

import asyncio
import os
from typing import Any

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ClaudeSDKClient,
    ResultMessage,
    StreamEvent,
    SystemMessage,
    TextBlock,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)
from claude_agent_sdk.types import PermissionResultAllow, PermissionResultDeny

from .config import GatewayConfig
from .drone_tools import (
    CONTROL_REQUEST_TOOL_NAMES,
    READ_ONLY_TOOL_NAMES,
    ControlRequester,
    create_drone_mcp_server,
)
from .ros_state import RosStateBridge
from .runtime import EmitCallback


SYSTEM_PROMPT = """你是无人机地面站的任务分析助手。
你可以通过 drone MCP 工具读取 ROS 2 中的无人机状态、里程计、目标检测和自主避障摘要。
用户要求“分析当前画面/描述画面/看看相机里有什么”时，必须调用 analyze_current_image；不得用遥测、历史 Mission 或过期 detection 代替图像分析。工具结果 source=vlm 才表示 VLM 成功，source=rule+detection+vlm_failed 表示 VLM 失败并已降级，回答中必须如实说明。
控制类 request_* 工具只会创建人工确认请求，不会立即改变无人机状态。
基础动作先查询 list_capabilities，组合任务优先查询 list_skills；已有技能使用 request_skill，其他多步任务可使用 request_workflow。
用户要求“检查状态/确认安全后起飞”时，必须调用 request_safe_takeoff，禁止只给文字建议。
用户明确要求起飞、降落、移动或返航时，应创建对应确认请求；可以报告风险，但不要自行替代确定性安全检查而拒绝创建请求。
工具返回的 fresh、age_sec 和 available 字段决定数据是否仍有效；近期 Mission 只是历史记录，不是当前动作的阻断条件。
本项目中 battery=null 表示电池遥测未接入，不表示电量为 0%；DroneState 的 gps_fix 映射为 0=无定位、1=2D、2=3D、3=差分、4=RTK，gps_usable=true 表示定位满足当前控制阈值。
整套 workflow 只创建一次人工确认，步骤必须使用白名单动作，禁止生成代码或 shell 命令。
workflow 中 move 是不可幂等的相对位移动作，必须设置 retries=0 或省略 retries，禁止自动重发移动步骤。
调用控制请求工具后必须告诉用户正在等待确认，不能声称已经起飞、移动、降落或改变 PX4 状态。
只有控制工具返回 confirmation_id 后才能声称请求已创建；严禁自行编造 confirm-* 确认编号。
禁止尝试 Bash、文件修改或 shell ROS 命令控制无人机。
坐标默认使用 ENU，位置和距离单位为米，速度单位为米每秒。
回答应区分传感器事实、你的推断和建议；数据不可用时直接说明。
默认使用简体中文和清晰的 Markdown 排版。先用一行给出结论，再按需使用“状态”“依据”“建议”“下一步”等短标题。
每个标题、段落和列表项单独换行，段落之间保留空行；列表使用“- ”开头，每项只表达一个信息。
避免把多个状态挤在同一长段落中，避免无必要的表格、重复说明和大段免责声明。
控制请求必须单独写明“等待确认”，并在下一行说明用户可以发送“确认执行”或“取消执行”。
"""


class ClaudeRuntime:
    provider = "claude"

    def __init__(
        self,
        config: GatewayConfig,
        ros_state: RosStateBridge,
        model: str,
        runtime_session_id: str | None = None,
        request_control: ControlRequester | None = None,
    ):
        self.model = model
        self.runtime_session_id = runtime_session_id
        self._config = config
        self._ros_state = ros_state
        self._client: ClaudeSDKClient | None = None
        self._lock = asyncio.Lock()
        self._request_control = request_control

    async def start(self):
        if self._client is not None:
            return

        async def can_use_tool(name, _input, _context):
            if name in READ_ONLY_TOOL_NAMES or name in CONTROL_REQUEST_TOOL_NAMES:
                return PermissionResultAllow()
            return PermissionResultDeny(
                message="当前 Agent Gateway 仅允许无人机只读 MCP 工具。",
                interrupt=False,
            )

        options = ClaudeAgentOptions(
            model=self.model,
            resume=self.runtime_session_id,
            cwd=self._config.workspace,
            tools=[],
            mcp_servers={
                "drone": create_drone_mcp_server(
                    self._ros_state, self._request_control
                )
            },
            allowed_tools=READ_ONLY_TOOL_NAMES + CONTROL_REQUEST_TOOL_NAMES,
            disallowed_tools=["Bash", "Write", "Edit", "NotebookEdit"],
            permission_mode="default",
            can_use_tool=can_use_tool,
            system_prompt=SYSTEM_PROMPT,
            setting_sources=[],
            skills=[],
            include_partial_messages=True,
            max_turns=self._config.max_turns,
            max_budget_usd=self._config.max_budget_usd or None,
            env=self._sdk_env(),
        )
        self._client = ClaudeSDKClient(options=options)
        await self._client.connect()

    def _sdk_env(self) -> dict[str, str]:
        keys = (
            "ANTHROPIC_API_KEY",
            "ANTHROPIC_AUTH_TOKEN",
            "ANTHROPIC_BASE_URL",
            "CLAUDE_CODE_USE_BEDROCK",
            "CLAUDE_CODE_USE_VERTEX",
            "AWS_PROFILE",
            "AWS_REGION",
        )
        return {key: os.environ[key] for key in keys if os.environ.get(key)}

    async def close(self):
        if self._client is None:
            return
        client = self._client
        self._client = None
        await client.disconnect()

    async def interrupt(self):
        if self._client is not None:
            await self._client.interrupt()

    async def set_model(self, model: str):
        await self.start()
        assert self._client is not None
        async with self._lock:
            await self._client.set_model(model)
            self.model = model

    async def run_turn(self, prompt: str, emit: EmitCallback) -> dict[str, Any]:
        await self.start()
        assert self._client is not None
        text_parts: list[str] = []
        usage = None
        cost = None
        is_error = False
        error_message = ""

        async with self._lock:
            await self._client.query(prompt)
            async for message in self._client.receive_response():
                if isinstance(message, StreamEvent):
                    delta = _stream_text_delta(message.event)
                    if delta:
                        text_parts.append(delta)
                        await emit(
                            "assistant.delta",
                            {"delta": delta, "model": self.model},
                        )
                    continue

                if isinstance(message, SystemMessage):
                    session_id = str(message.data.get("session_id", "")).strip()
                    if session_id:
                        self.runtime_session_id = session_id
                        await emit(
                            "runtime.session",
                            {"runtime_session_id": session_id},
                        )
                    continue

                if isinstance(message, AssistantMessage):
                    self.runtime_session_id = message.session_id or self.runtime_session_id
                    for block in message.content:
                        if isinstance(block, ToolUseBlock):
                            await emit(
                                "tool.started",
                                {
                                    "tool_call_id": block.id,
                                    "tool": block.name,
                                    "arguments": block.input,
                                },
                            )
                        elif isinstance(block, ToolResultBlock):
                            await emit(
                                "tool.completed",
                                {
                                    "tool_call_id": block.tool_use_id,
                                    "success": not bool(block.is_error),
                                    "result": block.content,
                                },
                            )
                        elif isinstance(block, TextBlock) and not text_parts:
                            text_parts.append(block.text)
                            await emit(
                                "assistant.delta",
                                {"delta": block.text, "model": self.model},
                            )
                    continue

                if isinstance(message, UserMessage) and isinstance(message.content, list):
                    for block in message.content:
                        if isinstance(block, ToolResultBlock):
                            await emit(
                                "tool.completed",
                                {
                                    "tool_call_id": block.tool_use_id,
                                    "success": not bool(block.is_error),
                                    "result": block.content,
                                },
                            )
                    continue

                if isinstance(message, ResultMessage):
                    self.runtime_session_id = message.session_id
                    usage = message.usage
                    cost = message.total_cost_usd
                    is_error = bool(message.is_error)
                    if message.result and not text_parts:
                        text_parts.append(message.result)
                    if message.errors:
                        error_message = "; ".join(message.errors)

        return {
            "text": "".join(text_parts).strip(),
            "model": self.model,
            "runtime_session_id": self.runtime_session_id,
            "usage": usage,
            "cost_usd": cost,
            "is_error": is_error,
            "error": error_message,
        }


def _stream_text_delta(event: dict[str, Any]) -> str:
    if event.get("type") != "content_block_delta":
        return ""
    delta = event.get("delta") or {}
    if delta.get("type") != "text_delta":
        return ""
    return str(delta.get("text", ""))
