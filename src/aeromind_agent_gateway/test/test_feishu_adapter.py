import asyncio
import base64
import json
import os
import tempfile
import unittest

from lark_oapi.api.im.v1 import P2ImMessageReceiveV1
from lark_oapi.event.callback.model.p2_card_action_trigger import P2CardActionTrigger

from aeromind_agent_gateway.config import GatewayConfig
from aeromind_agent_gateway.feishu_adapter import (
    FeishuAdapter,
    confirmation_card,
    format_event,
)
from aeromind_agent_gateway.store import SessionStore


class FakeResponse:
    code = 0
    msg = "ok"

    def __init__(self, message_id="om_test"):
        self.data = type("Data", (), {"message_id": message_id})()

    def success(self):
        return True

    def get_log_id(self):
        return "log"


class FakeMessageApi:
    def __init__(self):
        self.requests = []

    def create(self, request):
        self.requests.append(("create", request))
        return FakeResponse()

    def update(self, request):
        self.requests.append(("update", request))
        return FakeResponse()

    def patch(self, request):
        self.requests.append(("patch", request))
        return FakeResponse()


class FakeApiClient:
    def __init__(self):
        self.im = type("Im", (), {})()
        self.im.v1 = type("V1", (), {})()
        self.im.v1.message = FakeMessageApi()


class RecordingFeishuAdapter(FeishuAdapter):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sent = []
        self.updated = []
        self.image_data = b""

    async def send_text(self, chat_id, text):
        self.sent.append(("text", chat_id, text))
        return f"om-{len(self.sent)}"

    async def send_card(self, chat_id, card):
        self.sent.append(("card", chat_id, card))
        return f"om-{len(self.sent)}"

    async def update_text(self, message_id, text):
        self.updated.append(("text", message_id, text))

    async def update_card(self, message_id, card):
        self.updated.append(("card", message_id, card))

    async def _download_image(self, _message_id, _image_key):
        return self.image_data


class FakeVisionAnalyzer:
    available = True

    def validate(self, data):
        if not data.startswith(b"\x89PNG"):
            raise ValueError("invalid image")
        return {
            "format": "PNG",
            "mime_type": "image/png",
            "extension": ".png",
            "width": 1,
            "height": 1,
            "size_bytes": len(data),
        }

    async def analyze(self, _data, _prompt, _metadata):
        return "画面中可见一个测试目标，风险较低。"


class FakeSessions:
    def __init__(self):
        self.submitted = []
        self.confirmed = []
        self.models = []
        self.queues = {}
        self.vision_results = []

    def ensure_session(self, session_id, user_id, channel):
        return {"id": session_id, "user_id": user_id, "channel": channel}

    async def subscribe(self, session_id):
        queue = asyncio.Queue()
        self.queues[session_id] = queue
        return queue

    async def unsubscribe(self, _session_id, _queue):
        return None

    def submit_chat(self, session_id, content, request_id=None):
        self.submitted.append((session_id, content, request_id))

    async def respond_confirmation(
        self, session_id, user_id, confirmation_id, decision
    ):
        self.confirmed.append((session_id, user_id, confirmation_id, decision))

    async def switch_model(self, session_id, model):
        self.models.append((session_id, model))

    async def record_vision_result(
        self, session_id, image_path, analysis, metadata
    ):
        self.vision_results.append((session_id, image_path, analysis, metadata))


class FeishuAdapterTest(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def tearDownClass(cls):
        from lark_oapi.ws.client import loop as lark_loop

        if not lark_loop.is_running() and not lark_loop.is_closed():
            lark_loop.close()

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        database_path = os.path.join(self.temp_dir.name, "agent.db")
        self.store = SessionStore(database_path)
        self.config = GatewayConfig(
            host="127.0.0.1",
            port=8090,
            database_path=database_path,
            workspace=self.temp_dir.name,
            access_token="",
            default_model="sonnet",
            max_turns=4,
            max_budget_usd=0.1,
            feishu_enabled=True,
            feishu_app_id="cli_test",
            feishu_app_secret="secret",
            feishu_allowed_open_ids=("ou_allowed",),
        )
        self.sessions = FakeSessions()
        self.api = FakeApiClient()
        self.adapter = RecordingFeishuAdapter(
            self.config,
            self.sessions,
            self.store,
            api_client=self.api,
            vision_analyzer=FakeVisionAnalyzer(),
        )
        self.adapter.image_data = base64.b64decode(
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAA"
            "C0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
        )

    async def asyncTearDown(self):
        await self.adapter.stop()
        self.store.close()
        self.temp_dir.cleanup()

    async def test_text_event_is_deduplicated(self):
        event = make_event("m1", "ou_allowed", "p2p", "检查状态")
        await self.adapter.handle_event(event)
        await self.adapter.handle_event(event)

        self.assertEqual(len(self.sessions.submitted), 1)
        self.assertEqual(self.sessions.submitted[0][0], "feishu-ou_allowed")
        self.assertEqual(len(self.adapter.sent), 1)

    async def test_unknown_user_is_rejected(self):
        await self.adapter.handle_event(make_event("m2", "ou_other", "p2p", "起飞"))
        self.assertEqual(self.sessions.submitted, [])

    async def test_status_exposes_operator_channel_metadata(self):
        status = self.adapter.status()
        self.assertTrue(status["enabled"])
        self.assertFalse(status["running"])
        self.assertEqual(status["allowed_user_count"], 1)
        self.assertEqual(status["allowed_open_ids"], ["ou_allowed"])
        self.assertFalse(status["vision_enabled"])

    async def test_image_is_archived_analyzed_and_recorded(self):
        await self.adapter.handle_event(
            make_image_event("m-image", "ou_allowed", "p2p", "img_test")
        )
        self.assertEqual(len(self.sessions.vision_results), 1)
        session_id, image_path, analysis, metadata = self.sessions.vision_results[0]
        self.assertEqual(session_id, "feishu-ou_allowed")
        self.assertTrue(os.path.isfile(image_path))
        self.assertIn("测试目标", analysis)
        mission_path = os.path.join(
            self.temp_dir.name,
            "missions",
            metadata["mission_id"],
            "mission.json",
        )
        with open(mission_path, encoding="utf-8") as file:
            manifest = json.load(file)
        self.assertEqual(manifest["status"], "completed")
        self.assertIn("vision_analysis", manifest)
        self.assertTrue(os.path.isfile(os.path.join(os.path.dirname(mission_path), "report.md")))
        self.assertEqual(self.adapter.updated[-1][1], "om-1")

    async def test_duplicate_image_message_is_not_analyzed_twice(self):
        event = make_image_event("m-image-dup", "ou_allowed", "p2p", "img_test")
        await self.adapter.handle_event(event)
        await self.adapter.handle_event(event)
        self.assertEqual(len(self.sessions.vision_results), 1)

    async def test_group_message_without_mention_is_ignored(self):
        await self.adapter.handle_event(
            make_event("m-group", "ou_allowed", "group", "检查状态")
        )
        self.assertEqual(self.sessions.submitted, [])

    async def test_confirmation_requires_private_chat(self):
        await self.adapter.handle_text(
            "ou_allowed", "oc_group", "group", "/confirm confirm-1"
        )
        self.assertEqual(self.sessions.confirmed, [])

        await self.adapter.handle_text(
            "ou_allowed", "oc_private", "p2p", "/confirm confirm-1"
        )
        self.assertEqual(self.sessions.confirmed[-1][-1], "approve")

    async def test_model_command(self):
        await self.adapter.handle_text(
            "ou_allowed", "oc_private", "p2p", "/model opus"
        )
        self.assertEqual(self.sessions.models, [("feishu-ou_allowed", "opus")])

    async def test_sender_queue_calls_feishu_api(self):
        adapter = FeishuAdapter(
            self.config, self.sessions, self.store, api_client=self.api
        )
        adapter._start_sender()
        await adapter.send_text("oc_private", "测试回复")
        await adapter.stop()
        self.assertEqual(len(self.api.im.v1.message.requests), 1)
        self.assertEqual(self.api.im.v1.message.requests[0][0], "create")

    async def test_sender_queue_supports_text_and_card_updates(self):
        adapter = FeishuAdapter(
            self.config, self.sessions, self.store, api_client=self.api
        )
        adapter._start_sender()
        await adapter.send_card(
            "oc_private",
            confirmation_card(
                {"id": "confirm-1", "summary": "降落", "risk_level": "high"},
                "pending",
            ),
        )
        await adapter.update_text("om_text", "分析完成")
        await adapter.update_card(
            "om_card",
            confirmation_card(
                {"id": "confirm-1", "summary": "降落", "risk_level": "high"},
                "completed",
            ),
        )
        await adapter.stop()
        self.assertEqual(
            [operation for operation, _request in self.api.im.v1.message.requests],
            ["create", "update", "patch"],
        )

    async def test_stream_updates_original_message(self):
        key = ("feishu-ou_allowed", "request-1")
        self.adapter._active_messages[key] = "om-stream"
        handled = await self.adapter._handle_stream_event(
            key[0],
            {
                "type": "assistant.delta",
                "request_id": key[1],
                "delta": "正在读取状态",
            },
        )
        self.assertTrue(handled)
        self.assertEqual(self.adapter.updated[-1][1], "om-stream")

        await self.adapter._handle_stream_event(
            key[0],
            {
                "type": "assistant.completed",
                "request_id": key[1],
                "message": {"content": "状态正常"},
            },
        )
        self.assertEqual(self.adapter.updated[-1][2], "状态正常")
        self.assertNotIn(key, self.adapter._active_messages)

    async def test_private_confirmation_uses_card_and_updates_status(self):
        session_id = "feishu-ou_allowed"
        item = {
            "id": "confirm-1",
            "summary": "起飞到 5 米",
            "risk_level": "high",
        }
        self.adapter._targets[session_id] = "oc_private"
        self.adapter._target_chat_types[session_id] = "p2p"
        await self.adapter._send_confirmation(session_id, item)
        self.assertEqual(self.adapter.sent[-1][0], "card")
        self.assertEqual(self.adapter._confirmation_messages["confirm-1"], "om-1")

        await self.adapter._update_confirmation(
            {
                "type": "confirmation.resolved",
                "decision": "approve",
                "confirmation": item,
            }
        )
        self.assertEqual(self.adapter.updated[-1][0], "card")
        self.assertIn("执行中", self.adapter.updated[-1][2]["header"]["title"]["content"])

    async def test_card_action_uses_operator_identity(self):
        self.adapter._confirmation_chats["confirm-1"] = "oc_private"
        await self.adapter.handle_card_action(
            make_card_action("ou_allowed", "oc_private", "confirm-1", "approve")
        )
        self.assertEqual(
            self.sessions.confirmed[-1],
            (
                "feishu-ou_allowed",
                "feishu:ou_allowed",
                "confirm-1",
                "approve",
            ),
        )

    async def test_card_action_rejects_forwarded_card(self):
        self.adapter._confirmation_chats["confirm-1"] = "oc_private"
        with self.assertRaises(PermissionError):
            await self.adapter.handle_card_action(
                make_card_action("ou_allowed", "oc_other", "confirm-1", "approve")
            )

    def test_confirmation_event_text(self):
        text = format_event(
            {
                "type": "confirmation.required",
                "confirmation": {
                    "id": "confirm-1",
                    "summary": "起飞到 5 米",
                    "risk_level": "high",
                },
            }
        )
        self.assertIn("/confirm confirm-1", text)
        self.assertIn("起飞到 5 米", text)

        progress = format_event(
            {
                "type": "control.progress",
                "phase": "verifying",
                "details": {"message": "等待飞控遥测确认"},
            }
        )
        self.assertIn("verifying", progress)
        self.assertIn("等待飞控遥测确认", progress)

    def test_confirmation_card_uses_schema_two_buttons(self):
        card = confirmation_card(
            {
                "id": "confirm-1",
                "summary": "起飞到 5 米",
                "risk_level": "high",
            },
            "pending",
        )
        self.assertEqual(card["schema"], "2.0")
        tags = [element["tag"] for element in card["body"]["elements"]]
        self.assertEqual(tags, ["markdown", "button", "button"])


def make_event(message_id, open_id, chat_type, text):
    return P2ImMessageReceiveV1(
        {
            "event": {
                "sender": {"sender_id": {"open_id": open_id}},
                "message": {
                    "message_id": message_id,
                    "chat_id": "oc_test",
                    "chat_type": chat_type,
                    "message_type": "text",
                    "content": json.dumps({"text": text}, ensure_ascii=False),
                },
            }
        }
    )


def make_image_event(message_id, open_id, chat_type, image_key):
    return P2ImMessageReceiveV1(
        {
            "event": {
                "sender": {"sender_id": {"open_id": open_id}},
                "message": {
                    "message_id": message_id,
                    "chat_id": "oc_test",
                    "chat_type": chat_type,
                    "message_type": "image",
                    "content": json.dumps({"image_key": image_key}),
                },
            }
        }
    )


def make_card_action(open_id, chat_id, confirmation_id, decision):
    return P2CardActionTrigger(
        {
            "event": {
                "operator": {"open_id": open_id},
                "context": {"open_chat_id": chat_id},
                "action": {
                    "tag": "button",
                    "value": {
                        "confirmation_id": confirmation_id,
                        "decision": decision,
                    },
                },
            }
        }
    )


if __name__ == "__main__":
    unittest.main()
