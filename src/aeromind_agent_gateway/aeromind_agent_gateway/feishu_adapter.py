"""Feishu long-connection channel backed by the shared Agent sessions."""

from __future__ import annotations

import asyncio
from concurrent.futures import Future
import json
import logging
import os
from pathlib import Path
import queue
import threading
import time
import uuid
from typing import TYPE_CHECKING, Any

import lark_oapi as lark
from lark_oapi.api.im.v1 import (
    CreateMessageRequest,
    CreateMessageRequestBody,
    GetMessageResourceRequest,
    PatchMessageRequest,
    PatchMessageRequestBody,
    P2ImMessageReceiveV1,
    UpdateMessageRequest,
    UpdateMessageRequestBody,
)
from lark_oapi.event.callback.model.p2_card_action_trigger import (
    P2CardActionTrigger,
    P2CardActionTriggerResponse,
)

from .config import GatewayConfig
from .store import SessionStore
from .vision import VisionAnalyzer

if TYPE_CHECKING:
    from .session_manager import SessionManager


LOGGER = logging.getLogger(__name__)
MODEL_ALIASES = {
    "sonnet": "sonnet",
    "opus": "opus",
    "haiku": "haiku",
    "deepseek": "deepseek:deepseek-chat",
    "qwen": "qwen:qwen-plus",
}


class FeishuAdapter:
    def __init__(
        self,
        config: GatewayConfig,
        sessions: SessionManager,
        store: SessionStore,
        api_client: Any | None = None,
        ws_client: Any | None = None,
        vision_analyzer: VisionAnalyzer | None = None,
    ):
        self._config = config
        self._sessions = sessions
        self._store = store
        self._api_client = api_client
        self._ws_client = ws_client
        self._vision = vision_analyzer or VisionAnalyzer(config)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._send_thread: threading.Thread | None = None
        self._send_queue: queue.Queue = queue.Queue()
        self._relay_tasks: dict[str, asyncio.Task] = {}
        self._targets: dict[str, str] = {}
        self._target_chat_types: dict[str, str] = {}
        self._active_messages: dict[tuple[str, str], str] = {}
        self._stream_buffers: dict[tuple[str, str], str] = {}
        self._last_updates: dict[tuple[str, str], float] = {}
        self._confirmation_messages: dict[str, str] = {}
        self._confirmation_chats: dict[str, str] = {}
        self._stopping = False

    async def start(self):
        if not self._config.feishu_enabled:
            return
        self._validate_config()
        self._loop = asyncio.get_running_loop()
        if self._api_client is None:
            self._api_client = (
                lark.Client.builder()
                .app_id(self._config.feishu_app_id)
                .app_secret(self._config.feishu_app_secret)
                .build()
            )
        if self._ws_client is None:
            handler = (
                lark.EventDispatcherHandler.builder("", "")
                .register_p2_im_message_receive_v1(self._on_message)
                .register_p2_card_action_trigger(self._on_card_action)
                .build()
            )
            self._ws_client = lark.ws.Client(
                self._config.feishu_app_id,
                self._config.feishu_app_secret,
                log_level=lark.LogLevel.INFO,
                event_handler=handler,
                auto_reconnect=True,
            )
        self._start_sender()
        self._thread = threading.Thread(
            target=self._run_ws,
            name="agent-gateway-feishu",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info("Feishu adapter started with %d allowed users", len(self._allowed))

    def _start_sender(self):
        if self._send_thread is not None and self._send_thread.is_alive():
            return
        self._send_queue = queue.Queue()
        self._send_thread = threading.Thread(
            target=self._run_sender,
            name="agent-gateway-feishu-sender",
            daemon=True,
        )
        self._send_thread.start()

    async def stop(self):
        self._stopping = True
        for task in self._relay_tasks.values():
            task.cancel()
        if self._relay_tasks:
            await asyncio.gather(*self._relay_tasks.values(), return_exceptions=True)
        self._relay_tasks.clear()
        self._send_queue.put(None)
        if self._send_thread is not None:
            self._send_thread.join(timeout=0.5)
            self._send_thread = None
        if self._ws_client is not None:
            self._ws_client._auto_reconnect = False
            try:
                from lark_oapi.ws.client import loop as lark_loop

                if lark_loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        self._ws_client._disconnect(), lark_loop
                    )
                    await asyncio.wait_for(asyncio.wrap_future(future), timeout=2.0)
            except Exception:
                LOGGER.debug("Feishu websocket disconnect was not available", exc_info=True)

    @property
    def _allowed(self) -> set[str]:
        return set(self._config.feishu_allowed_open_ids)

    def _validate_config(self):
        if not self._config.feishu_app_id or not self._config.feishu_app_secret:
            raise RuntimeError("FEISHU_APP_ID 和 FEISHU_APP_SECRET 必须同时配置")
        if not self._allowed:
            raise RuntimeError("FEISHU_ALLOWED_OPEN_IDS 不能为空，拒绝无白名单启动")

    def _run_ws(self):
        try:
            self._ws_client.start()
        except Exception:
            if not self._stopping:
                LOGGER.exception("Feishu long connection stopped unexpectedly")

    def _run_sender(self):
        while True:
            job = self._send_queue.get()
            if job is None:
                return
            method, request, future = job
            try:
                response = method(request)
            except Exception as exc:
                future.set_exception(exc)
            else:
                future.set_result(response)

    def _on_message(self, data: P2ImMessageReceiveV1):
        if self._loop is None or self._stopping:
            return
        future = asyncio.run_coroutine_threadsafe(self.handle_event(data), self._loop)
        future.add_done_callback(self._log_callback_error)

    def _on_card_action(
        self, data: P2CardActionTrigger
    ) -> P2CardActionTriggerResponse:
        if self._loop is None or self._stopping:
            return P2CardActionTriggerResponse(
                {"toast": {"type": "error", "content": "控制网关未就绪"}}
            )
        future = asyncio.run_coroutine_threadsafe(
            self._handle_card_action_with_feedback(data), self._loop
        )
        future.add_done_callback(self._log_callback_error)
        return P2CardActionTriggerResponse(
            {"toast": {"type": "info", "content": "确认请求已受理"}}
        )

    async def _handle_card_action_with_feedback(self, data: P2CardActionTrigger):
        try:
            await self.handle_card_action(data)
        except (ValueError, PermissionError, RuntimeError) as exc:
            context = getattr(getattr(data, "event", None), "context", None)
            chat_id = str(getattr(context, "open_chat_id", "") or "")
            if chat_id:
                await self.send_text(chat_id, f"确认处理失败：{exc}")
            else:
                raise

    @staticmethod
    def _log_callback_error(future):
        try:
            future.result()
        except Exception:
            LOGGER.exception("Failed to handle Feishu message")

    async def handle_event(self, data: P2ImMessageReceiveV1):
        event = getattr(data, "event", None)
        sender = getattr(event, "sender", None)
        sender_id = getattr(sender, "sender_id", None)
        message = getattr(event, "message", None)
        open_id = str(getattr(sender_id, "open_id", "") or "")
        message_id = str(getattr(message, "message_id", "") or "")
        chat_id = str(getattr(message, "chat_id", "") or "")
        chat_type = str(getattr(message, "chat_type", "") or "")
        message_type = str(getattr(message, "message_type", "") or "")

        if not open_id or not message_id or not chat_id:
            LOGGER.warning("Ignored malformed Feishu message event")
            return
        if open_id not in self._allowed:
            LOGGER.warning("Rejected Feishu user not in allowlist: %s", open_id)
            return
        if not self._store.claim_inbound_message(message_id, "feishu", open_id):
            return
        if message_type == "image":
            if chat_type != "p2p":
                return
            try:
                content = json.loads(str(getattr(message, "content", "") or "{}"))
                image_key = str(content.get("image_key", ""))
            except (TypeError, ValueError, json.JSONDecodeError):
                image_key = ""
            if not image_key:
                await self.send_text(chat_id, "图片消息缺少 image_key，无法下载。")
                return
            await self.handle_image(
                open_id, chat_id, chat_type, message_id, image_key
            )
            return
        if message_type != "text":
            await self.send_text(chat_id, "当前仅支持文本消息。")
            return
        try:
            content = json.loads(str(getattr(message, "content", "") or "{}"))
            text = str(content.get("text", "")).strip()
        except (TypeError, ValueError, json.JSONDecodeError):
            text = ""
        mentions = getattr(message, "mentions", None) or []
        if chat_type != "p2p":
            if not mentions:
                return
            for mention in mentions:
                key = str(getattr(mention, "key", "") or "")
                if key:
                    text = text.replace(key, "").strip()
        if not text:
            await self.send_text(chat_id, "消息内容为空，请重新输入。")
            return
        await self.handle_text(open_id, chat_id, chat_type, text)

    async def handle_text(
        self, open_id: str, chat_id: str, chat_type: str, text: str
    ):
        session_id = f"feishu-{open_id}"
        user_id = f"feishu:{open_id}"
        self._sessions.ensure_session(session_id, user_id=user_id, channel="feishu")
        self._targets[session_id] = chat_id
        self._target_chat_types[session_id] = chat_type
        await self._ensure_relay(session_id)

        command, _, argument = text.strip().partition(" ")
        command = command.lower()
        argument = argument.strip()
        if command in {"/confirm", "/cancel"}:
            if chat_type != "p2p":
                await self.send_text(chat_id, "飞行控制确认只能在机器人私聊中完成。")
                return
            if not argument:
                await self.send_text(chat_id, f"用法：{command} <confirmation_id>")
                return
            try:
                await self._sessions.respond_confirmation(
                    session_id=session_id,
                    user_id=user_id,
                    confirmation_id=argument,
                    decision="approve" if command == "/confirm" else "cancel",
                )
            except (ValueError, PermissionError) as exc:
                await self.send_text(chat_id, f"确认处理失败：{exc}")
            return
        if command == "/model":
            model = MODEL_ALIASES.get(argument.lower()) or (
                argument if ":" in argument else None
            )
            if model is None:
                await self.send_text(
                    chat_id,
                    "可用模型：sonnet、opus、haiku、deepseek、qwen，"
                    "也可使用 provider:model。",
                )
                return
            try:
                await self._sessions.switch_model(session_id, model)
            except (ValueError, RuntimeError) as exc:
                await self.send_text(chat_id, f"模型切换失败：{exc}")
            return
        if command == "/status":
            text = "读取当前 ROS 无人机状态并给出简洁摘要，不要执行任何控制动作。"
        elif command == "/help":
            await self.send_text(
                chat_id,
                "可直接发送自然语言任务。命令：\n"
                "/status 查看状态\n/model sonnet|opus|haiku 切换模型\n"
                "/model deepseek|qwen 或 provider:model 切换 Provider\n"
                "/confirm <id> 确认执行\n/cancel <id> 取消执行",
            )
            return

        request_id = f"feishu-{uuid.uuid4().hex}"
        message_id = await self.send_text(chat_id, "任务已接收，正在分析。")
        if message_id:
            self._active_messages[(session_id, request_id)] = message_id
        self._sessions.submit_chat(
            session_id,
            text,
            request_id=request_id,
        )

    async def handle_image(
        self,
        open_id: str,
        chat_id: str,
        chat_type: str,
        message_id: str,
        image_key: str,
    ):
        session_id = f"feishu-{open_id}"
        self._sessions.ensure_session(
            session_id, user_id=f"feishu:{open_id}", channel="feishu"
        )
        self._targets[session_id] = chat_id
        self._target_chat_types[session_id] = chat_type
        await self._ensure_relay(session_id)
        reply_id = await self.send_text(chat_id, "图片已接收，正在进行安全校验和视觉分析。")
        mission_id = ""
        try:
            data = await self._download_image(message_id, image_key)
            metadata = self._vision.validate(data)
            mission_id, image_path = self._archive_image(
                data, metadata, message_id, open_id
            )
            metadata = {**metadata, "mission_id": mission_id}
            analysis = await self._vision.analyze(
                data,
                "分析用户从飞书上传的无人机任务相关图片。",
                metadata,
            )
            self._finalize_image_mission(mission_id, "completed", analysis)
            if reply_id:
                await self.update_text(reply_id, analysis)
            else:
                await self.send_text(chat_id, analysis)
            await self._sessions.record_vision_result(
                session_id, image_path, analysis, metadata
            )
        except Exception as exc:
            error = f"图片分析失败：{exc}"
            if mission_id:
                self._finalize_image_mission(mission_id, "failed", error)
            if reply_id:
                await self.update_text(reply_id, error)
            else:
                await self.send_text(chat_id, error)

    async def _download_image(self, message_id: str, image_key: str) -> bytes:
        request = (
            GetMessageResourceRequest.builder()
            .message_id(message_id)
            .file_key(image_key)
            .type("image")
            .build()
        )
        response = await self._enqueue(
            self._api_client.im.v1.message_resource.get, request
        )
        file_object = getattr(response, "file", None)
        if file_object is None:
            raise RuntimeError("飞书图片资源响应为空")
        data = file_object.read()
        if not isinstance(data, bytes):
            data = bytes(data)
        return data

    def _archive_image(
        self,
        data: bytes,
        metadata: dict[str, Any],
        message_id: str,
        open_id: str,
    ) -> tuple[str, str]:
        safe_message = "".join(ch for ch in message_id if ch.isalnum())[-12:]
        mission_id = f"mission-feishu-{int(time.time() * 1000)}-{safe_message}"
        mission_dir = Path(self._config.workspace) / "missions" / mission_id
        captures_dir = mission_dir / "captures"
        captures_dir.mkdir(parents=True, exist_ok=False)
        image_path = captures_dir / f"uploaded{metadata['extension']}"
        image_path.write_bytes(data)
        manifest = {
            "id": mission_id,
            "type": "feishu_image_analysis",
            "status": "analyzing",
            "source": "feishu",
            "user_id": f"feishu:{open_id}",
            "source_message_id": message_id,
            "created_at": time.time(),
            "captures": [
                {
                    "filename": image_path.name,
                    "path": str(image_path),
                    **metadata,
                }
            ],
        }
        (mission_dir / "mission.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return mission_id, os.fspath(image_path)

    def _finalize_image_mission(
        self, mission_id: str, status: str, analysis: str
    ):
        manifest_path = (
            Path(self._config.workspace) / "missions" / mission_id / "mission.json"
        )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["status"] = status
        manifest["finished_at"] = time.time()
        manifest["updated_at"] = manifest["finished_at"]
        manifest["message"] = analysis
        manifest["vision_analysis"] = analysis
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        capture = (manifest.get("captures") or [{}])[0]
        report = (
            "# 飞书图片视觉分析报告\n\n"
            f"- 任务编号：`{mission_id}`\n"
            f"- 状态：`{status}`\n"
            f"- 模型：`{self._config.vlm_model or '未配置'}`\n"
            f"- 图像：`{capture.get('filename', '')}`\n"
            f"- 尺寸：{capture.get('width', 0)} x {capture.get('height', 0)}\n\n"
            "## 分析结果\n\n"
            f"{analysis}\n"
        )
        (manifest_path.parent / "report.md").write_text(report, encoding="utf-8")

    async def handle_card_action(self, data: P2CardActionTrigger):
        event = getattr(data, "event", None)
        operator = getattr(event, "operator", None)
        action = getattr(event, "action", None)
        context = getattr(event, "context", None)
        open_id = str(getattr(operator, "open_id", "") or "")
        value = getattr(action, "value", None) or {}
        confirmation_id = str(value.get("confirmation_id", ""))
        decision = str(value.get("decision", ""))
        chat_id = str(getattr(context, "open_chat_id", "") or "")
        if open_id not in self._allowed:
            raise PermissionError("当前飞书用户不在控制白名单")
        if decision not in {"approve", "cancel"} or not confirmation_id:
            raise ValueError("无效的确认卡片动作")
        expected_chat = self._confirmation_chats.get(confirmation_id)
        if expected_chat is None or expected_chat != chat_id:
            raise PermissionError("确认卡片来源无效或网关已重启，请使用文本命令")
        session_id = f"feishu-{open_id}"
        await self._sessions.respond_confirmation(
            session_id=session_id,
            user_id=f"feishu:{open_id}",
            confirmation_id=confirmation_id,
            decision=decision,
        )

    async def _ensure_relay(self, session_id: str):
        task = self._relay_tasks.get(session_id)
        if task is not None and not task.done():
            return
        queue = await self._sessions.subscribe(session_id)
        task = asyncio.create_task(self._relay(session_id, queue))
        self._relay_tasks[session_id] = task

    async def _relay(self, session_id: str, queue: asyncio.Queue):
        try:
            while True:
                event = await queue.get()
                try:
                    if await self._handle_stream_event(session_id, event):
                        continue
                    if event.get("type") == "confirmation.required":
                        await self._send_confirmation(session_id, event["confirmation"])
                        continue
                    if event.get("type") in {
                        "confirmation.resolved",
                        "confirmation.expired",
                        "control.progress",
                        "control.completed",
                    }:
                        await self._update_confirmation(event)
                    text = format_event(event)
                    target = self._targets.get(session_id)
                    if text and target:
                        await self.send_text(target, text)
                except asyncio.CancelledError:
                    raise
                except Exception:
                    LOGGER.exception(
                        "Failed to relay Feishu event %s for %s",
                        event.get("type"),
                        session_id,
                    )
        finally:
            await self._sessions.unsubscribe(session_id, queue)

    async def _handle_stream_event(
        self, session_id: str, event: dict[str, Any]
    ) -> bool:
        request_id = str(event.get("request_id", ""))
        key = (session_id, request_id)
        message_id = self._active_messages.get(key)
        if not request_id or not message_id:
            return False
        event_type = event.get("type")
        if event_type == "assistant.delta":
            content = self._stream_buffers.get(key, "") + str(event.get("delta", ""))
            self._stream_buffers[key] = content
            now = time.monotonic()
            if now - self._last_updates.get(key, 0.0) >= 0.8:
                self._last_updates[key] = now
                await self.update_text(message_id, f"正在分析...\n\n{content}")
            return True
        if event_type == "tool.started":
            content = self._stream_buffers.get(key, "")
            tool = event.get("tool", "unknown")
            await self.update_text(message_id, f"{content}\n\n正在调用工具：{tool}")
            return True
        if event_type in {"assistant.completed", "error"}:
            text = format_event(event) or "任务处理结束。"
            await self.update_text(message_id, text)
            self._active_messages.pop(key, None)
            self._stream_buffers.pop(key, None)
            self._last_updates.pop(key, None)
            return True
        return event_type in {"chat.accepted", "assistant.started", "runtime.session"}

    async def _send_confirmation(self, session_id: str, item: dict[str, Any]):
        chat_id = self._targets.get(session_id)
        if not chat_id:
            return
        if self._target_chat_types.get(session_id) != "p2p":
            await self.send_text(
                chat_id,
                format_event(
                    {"type": "confirmation.required", "confirmation": item}
                ),
            )
            return
        card = confirmation_card(item, status="pending")
        message_id = await self.send_card(chat_id, card)
        if message_id:
            self._confirmation_messages[item["id"]] = message_id
            self._confirmation_chats[item["id"]] = chat_id

    async def _update_confirmation(self, event: dict[str, Any]):
        item = event.get("confirmation", {})
        confirmation_id = str(item.get("id", ""))
        message_id = self._confirmation_messages.get(confirmation_id)
        if not message_id:
            return
        if event.get("type") == "confirmation.expired":
            status = "expired"
        elif event.get("type") == "confirmation.resolved":
            status = "executing" if event.get("decision") == "approve" else "cancelled"
        elif event.get("type") == "control.progress":
            status = "executing"
        else:
            status = event.get("status") or (
                "completed" if event.get("success") else "failed"
            )
        await self.update_card(message_id, confirmation_card(item, status=status))
        if status in {"cancelled", "completed", "failed", "expired"}:
            self._confirmation_messages.pop(confirmation_id, None)
            self._confirmation_chats.pop(confirmation_id, None)

    async def send_text(self, chat_id: str, text: str) -> str | None:
        return await self._create_message(chat_id, "text", {"text": text})

    async def send_card(self, chat_id: str, card: dict[str, Any]) -> str | None:
        return await self._create_message(chat_id, "interactive", card)

    async def _create_message(
        self, chat_id: str, message_type: str, content: dict[str, Any]
    ) -> str | None:
        request = (
            CreateMessageRequest.builder()
            .receive_id_type("chat_id")
            .request_body(
                CreateMessageRequestBody.builder()
                .receive_id(chat_id)
                .msg_type(message_type)
                .content(json.dumps(content, ensure_ascii=False))
                .uuid(uuid.uuid4().hex)
                .build()
            )
            .build()
        )
        response = await self._enqueue(self._api_client.im.v1.message.create, request)
        data = getattr(response, "data", None)
        return str(getattr(data, "message_id", "") or "") or None

    async def update_text(self, message_id: str, text: str):
        await self._update_message(message_id, "text", {"text": text})

    async def update_card(self, message_id: str, card: dict[str, Any]):
        request = (
            PatchMessageRequest.builder()
            .message_id(message_id)
            .request_body(
                PatchMessageRequestBody.builder()
                .content(json.dumps(card, ensure_ascii=False))
                .build()
            )
            .build()
        )
        await self._enqueue(self._api_client.im.v1.message.patch, request)

    async def _update_message(
        self, message_id: str, message_type: str, content: dict[str, Any]
    ):
        request = (
            UpdateMessageRequest.builder()
            .message_id(message_id)
            .request_body(
                UpdateMessageRequestBody.builder()
                .msg_type(message_type)
                .content(json.dumps(content, ensure_ascii=False))
                .build()
            )
            .build()
        )
        await self._enqueue(self._api_client.im.v1.message.update, request)

    async def _enqueue(self, method, request):
        if self._send_thread is None or not self._send_thread.is_alive():
            raise RuntimeError("飞书发送线程尚未启动")
        future: Future = Future()
        self._send_queue.put((method, request, future))
        while not future.done():
            await asyncio.sleep(0.01)
        response = future.result()
        if not response.success():
            raise RuntimeError(
                f"飞书消息发送失败: code={response.code}, msg={response.msg}, "
                f"log_id={response.get_log_id()}"
            )
        return response


def format_event(event: dict[str, Any]) -> str | None:
    event_type = event.get("type")
    if event_type == "assistant.completed":
        message = event.get("message", {})
        content = message.get("content", "") if isinstance(message, dict) else message
        return str(content).strip() or "任务分析完成。"
    if event_type == "error":
        return f"处理失败：{event.get('message', '未知错误')}"
    if event_type == "confirmation.required":
        item = event.get("confirmation", {})
        return (
            f"需要确认：{item.get('summary', '飞行控制')}\n"
            f"风险等级：{item.get('risk_level', 'high')}\n"
            f"确认编号：{item.get('id', '')}\n"
            f"私聊发送 /confirm {item.get('id', '')} 执行，"
            f"或 /cancel {item.get('id', '')} 取消。"
        )
    if event_type == "confirmation.resolved":
        decision = event.get("decision")
        return "确认已通过，准备执行。" if decision == "approve" else "任务已取消。"
    if event_type == "confirmation.expired":
        return "确认请求已过期，任务未执行。"
    if event_type == "control.started":
        return f"控制任务开始执行：{event.get('action', 'unknown')}"
    if event_type == "control.progress":
        details = event.get("details", {})
        return f"执行阶段 {event.get('phase', 'executing')}：{details.get('message', '')}"
    if event_type == "control.completed":
        result = event.get("result", {})
        prefix = "执行完成" if event.get("success") else "执行失败"
        return f"{prefix}：{result.get('message', result)}"
    if event_type == "model.changed":
        return f"模型已切换为 {event.get('model', '')}。"
    if event_type == "assistant.interrupted":
        return "当前回复已中断。"
    return None


def confirmation_card(item: dict[str, Any], status: str) -> dict[str, Any]:
    labels = {
        "pending": ("飞行控制确认", "orange", "等待操作员确认"),
        "executing": ("飞行任务执行中", "blue", "确认已通过，正在执行"),
        "cancelled": ("飞行任务已取消", "grey", "操作员已取消任务"),
        "completed": ("飞行任务已完成", "green", "控制任务执行完成"),
        "failed": ("飞行任务执行失败", "red", "请检查飞控状态和任务日志"),
        "expired": ("飞行确认已过期", "grey", "任务未执行，请重新发起"),
    }
    title, template, state_text = labels.get(status, labels["pending"])
    confirmation_id = str(item.get("id", ""))
    elements: list[dict[str, Any]] = [
        {
            "tag": "markdown",
            "content": (
                f"**任务**：{item.get('summary', '飞行控制')}\n"
                f"**风险等级**：{item.get('risk_level', 'high')}\n"
                f"**状态**：{state_text}\n"
                f"**确认编号**：`{confirmation_id}`"
            ),
        }
    ]
    if status == "pending":
        elements.extend(
            [
                {
                    "tag": "button",
                    "element_id": f"approve_{confirmation_id}",
                    "text": {"tag": "plain_text", "content": "确认执行"},
                    "type": "primary",
                    "width": "fill",
                    "value": {
                        "confirmation_id": confirmation_id,
                        "decision": "approve",
                    },
                },
                {
                    "tag": "button",
                    "element_id": f"cancel_{confirmation_id}",
                    "text": {"tag": "plain_text", "content": "取消"},
                    "type": "default",
                    "width": "fill",
                    "value": {
                        "confirmation_id": confirmation_id,
                        "decision": "cancel",
                    },
                },
            ]
        )
    return {
        "schema": "2.0",
        "config": {"update_multi": True},
        "header": {
            "title": {"tag": "plain_text", "content": title},
            "template": template,
        },
        "body": {"elements": elements},
    }
