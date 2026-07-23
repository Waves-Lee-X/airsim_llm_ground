import os
import tempfile
import unittest

import numpy as np

from aeromind_agent_gateway.store import SessionStore


class SessionStoreTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.store = SessionStore(os.path.join(self.temp_dir.name, "agent.db"))

    def tearDown(self):
        self.store.close()
        self.temp_dir.cleanup()

    def test_session_messages_and_replayable_events(self):
        session = self.store.ensure_session("s1", "u1", "web", "sonnet")
        self.assertEqual(session["model"], "sonnet")

        self.store.add_message("s1", "user", "检查状态")
        first = self.store.append_event("s1", "chat.accepted", {"request_id": "r1"})
        second = self.store.append_event("s1", "assistant.started", {"request_id": "r1"})

        self.assertEqual(first["sequence"], 1)
        self.assertEqual(second["sequence"], 2)
        self.assertEqual(self.store.messages("s1")[0]["content"], "检查状态")
        self.assertEqual(
            [event["type"] for event in self.store.events_after("s1", sequence=1)],
            ["assistant.started"],
        )

    def test_model_and_runtime_session_are_persisted(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        self.store.update_model("s1", "opus")
        self.store.update_runtime_session("s1", "claude-session")

        session = self.store.get_session("s1")
        self.assertEqual(session["model"], "opus")
        self.assertEqual(session["runtime_session_id"], "claude-session")

        self.store.update_provider_model("s1", "deepseek", "deepseek-chat")
        session = self.store.get_session("s1")
        self.assertEqual(session["provider"], "deepseek")
        self.assertEqual(session["model"], "deepseek-chat")
        self.assertIsNone(session["runtime_session_id"])

    def test_clear_session_history_keeps_control_records(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        self.store.update_runtime_session("s1", "claude-session")
        self.store.add_message("s1", "user", "起飞")
        confirmation = self.store.create_confirmation(
            "s1", "u1", "takeoff", {"altitude": 10.0}, "起飞", "high", 60
        )
        self.store.create_gateway_mission(confirmation)

        result = self.store.clear_session_history("s1")

        self.assertEqual(result["deleted_messages"], 1)
        self.assertEqual(self.store.messages("s1"), [])
        self.assertIsNone(self.store.get_session("s1")["runtime_session_id"])
        self.assertIsNotNone(self.store.get_confirmation(confirmation["id"]))
        self.assertIsNotNone(
            self.store.gateway_mission_for_confirmation(confirmation["id"])
        )

    def test_confirmation_round_trip(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        item = self.store.create_confirmation(
            "s1", "u1", "takeoff", {"altitude": 5.0}, "起飞到 5 米", "high", 60
        )
        claimed = self.store.resolve_confirmation(item["id"], "u1", "approve")
        self.assertEqual(claimed["status"], "executing")
        completed = self.store.complete_confirmation(
            item["id"], "completed", {"success": True}
        )
        self.assertEqual(completed["status"], "completed")
        self.assertTrue(completed["result"]["success"])

    def test_numpy_float32_workflow_evidence_is_persisted(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        item = self.store.create_confirmation(
            "s1", "u1", "takeoff", {"altitude": 5.0}, "起飞", "high", 60
        )
        self.store.create_gateway_mission(item)
        self.store.update_gateway_mission(
            item["id"],
            "executing",
            {
                "success": True,
                "evidence": {
                    "position_covariance": [
                        np.float32(0.1),
                        np.float32(0.0),
                    ]
                },
            },
            phase="workflow_step",
        )

        mission = self.store.gateway_mission_for_confirmation(item["id"])
        self.assertAlmostEqual(
            mission["result"]["evidence"]["position_covariance"][0], 0.1
        )

    def test_gateway_mission_terminal_state_is_monotonic_and_revisioned(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        item = self.store.create_confirmation(
            "s1", "u1", "takeoff", {"altitude": 5.0}, "起飞", "high", 60
        )
        created = self.store.create_gateway_mission(item)
        executing = self.store.update_gateway_mission(
            item["id"], "executing", phase="verifying"
        )
        completed = self.store.update_gateway_mission(
            item["id"], "completed", {"success": True},
            phase="completed", physical_complete=True,
        )
        late = self.store.update_gateway_mission(
            item["id"], "executing", {"message": "late progress"},
            phase="verifying", physical_complete=False,
        )

        self.assertGreater(executing["revision"], created["revision"])
        self.assertGreater(completed["revision"], executing["revision"])
        self.assertEqual(late["revision"], completed["revision"])
        self.assertEqual(late["status"], "completed")
        self.assertEqual(late["phase"], "completed")
        self.assertTrue(late["physical_complete"])
        self.assertTrue(late["result"]["success"])

    def test_duplicate_confirmation_resolution_is_idempotent(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        item = self.store.create_confirmation(
            "s1", "u1", "land", {}, "降落", "high", 60
        )
        first = self.store.resolve_confirmation(item["id"], "u1", "approve")
        second = self.store.resolve_confirmation(item["id"], "u1", "approve")
        self.assertEqual(first["status"], "executing")
        self.assertTrue(second["idempotent"])
        with self.assertRaises(ValueError):
            self.store.resolve_confirmation(item["id"], "u1", "cancel")

    def test_inbound_message_is_claimed_once(self):
        self.assertTrue(self.store.claim_inbound_message("m1", "feishu", "ou_1"))
        self.assertFalse(self.store.claim_inbound_message("m1", "feishu", "ou_1"))

    def test_latest_event_by_type(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        self.store.append_event("s1", "vision.completed", {"result": "first"})
        self.store.append_event("s1", "assistant.started", {})
        self.store.append_event("s1", "vision.completed", {"result": "latest"})
        event = self.store.latest_event("s1", "vision.completed")
        self.assertEqual(event["result"], "latest")

    def test_explicit_identity_rebind_updates_owned_records(self):
        self.store.ensure_session("s1", "web-local", "web", "sonnet")
        self.store.upsert_operator_memory("web-local", "历史摘要", "s1", 2)
        item = self.store.create_confirmation(
            "s1", "web-local", "land", {}, "降落", "high", 60
        )
        self.store.create_gateway_mission(item)
        self.store.update_gateway_mission(
            item["id"],
            "executing",
            phase="verifying",
            ros_mission_id="mission-1",
            physical_complete=False,
        )
        self.store.rebind_user("web-local", "operator:waves")
        self.assertEqual(self.store.get_session("s1")["user_id"], "operator:waves")
        self.assertEqual(
            self.store.get_confirmation(item["id"])["user_id"], "operator:waves"
        )
        self.assertEqual(
            self.store.gateway_mission_for_confirmation(item["id"])["user_id"],
            "operator:waves",
        )
        mission = self.store.gateway_mission_for_confirmation(item["id"])
        self.assertEqual(mission["phase"], "verifying")
        self.assertEqual(mission["ros_mission_id"], "mission-1")
        self.assertFalse(mission["physical_complete"])
        self.assertEqual(
            self.store.operator_memory("operator:waves")["summary"], "历史摘要"
        )

    def test_operator_messages_and_memory_are_persisted(self):
        self.store.ensure_session("web", "operator:1", "web", "sonnet")
        self.store.ensure_session("feishu", "operator:1", "feishu", "sonnet")
        self.store.add_message("web", "user", "检查状态")
        self.store.add_message("feishu", "assistant", "状态正常")

        messages = self.store.operator_messages("operator:1", limit=10)
        self.assertEqual([item["channel"] for item in messages], ["web", "feishu"])
        memory = self.store.upsert_operator_memory(
            "operator:1", "状态检查已完成", "feishu", len(messages)
        )
        self.assertEqual(memory["message_count"], 2)
        self.assertEqual(
            self.store.operator_memory("operator:1")["summary"],
            "状态检查已完成",
        )
        timeline = self.store.operator_timeline("operator:1")
        self.assertEqual([item["channel"] for item in timeline], ["web", "feishu"])

    def test_semantic_memory_is_deduplicated_searchable_and_deletable(self):
        self.store.ensure_session("web", "operator:1", "web", "sonnet")
        self.store.upsert_operator_memory("operator:1", "滚动摘要", "web", 2)
        items = [
            {
                "kind": "preference",
                "content": "用户偏好低速巡检一号区域",
                "importance": 0.9,
            },
            {
                "kind": "decision",
                "content": "移动任务必须经过人工确认",
                "importance": 1.0,
            },
        ]
        self.store.update_semantic_memory(
            "operator:1", "web", "haiku", 8, "巡检配置已记录", items
        )
        self.store.update_semantic_memory(
            "operator:1", "web", "haiku", 10, "巡检配置已更新", items[:1]
        )

        memories = self.store.semantic_memories(
            "operator:1", query="一号区域巡检"
        )
        self.assertEqual(len(memories), 2)
        self.assertIn("一号区域", memories[0]["content"])
        self.assertEqual(
            self.store.operator_memory("operator:1")["semantic_message_count"],
            10,
        )
        self.assertTrue(
            self.store.delete_semantic_memory("operator:1", memories[0]["id"])
        )
        self.assertEqual(len(self.store.semantic_memories("operator:1")), 1)

    def test_executing_workflow_is_marked_interrupted_after_restart(self):
        self.store.ensure_session("s1", "u1", "web", "sonnet")
        workflow = {
            "workflow_id": "workflow-1",
            "name": "巡检任务",
            "steps": [{"id": "hover", "action": "hover"}],
        }
        item = self.store.create_confirmation(
            "s1", "u1", "workflow", workflow, "执行巡检任务", "high", 60
        )
        self.store.create_gateway_mission(item)
        self.store.resolve_confirmation(item["id"], "u1", "approve")
        self.store.update_gateway_mission(
            item["id"], "executing", phase="workflow_step"
        )

        interrupted = self.store.interrupt_executing_workflows()
        self.assertEqual(len(interrupted), 1)
        self.assertEqual(interrupted[0]["phase"], "interrupted")
        self.assertEqual(
            self.store.get_confirmation(item["id"])["status"], "failed"
        )


if __name__ == "__main__":
    unittest.main()
