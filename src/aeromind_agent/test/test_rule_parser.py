import unittest
from types import SimpleNamespace

from aeromind_agent.agent_node import AgentNode


class RuleParserTest(unittest.TestCase):
    def setUp(self):
        self.node = AgentNode.__new__(AgentNode)

    def test_takeoff_altitude_is_not_parsed_as_forward_motion(self):
        parsed = self.node._parse_task_with_rules("起飞到 10.0 米")

        self.assertEqual(parsed["intent"], "takeoff")
        self.assertEqual(parsed["skill"], "TakeoffSkill")
        self.assertEqual(parsed["args"]["altitude"], 10.0)

    def test_real_fly_to_command_remains_move(self):
        parsed = self.node._parse_task_with_rules("飞到前方 10 米")

        self.assertEqual(parsed["intent"], "move_to")
        self.assertEqual(parsed["args"]["forward_m"], 10.0)

    def test_takeoff_then_move_remains_sequence(self):
        parsed = self.node._parse_task_with_rules("起飞到 10 米，然后向前飞 20 米")

        self.assertEqual(parsed["intent"], "mission_sequence")
        self.assertTrue(parsed["args"]["takeoff"])
        self.assertEqual(parsed["args"]["move"]["forward_m"], 20.0)

    def test_deterministic_takeoff_does_not_call_llm_again(self):
        self.node._llm_enabled = True
        self.node._parse_task_with_llm = lambda _task: self.fail(
            "明确起飞命令不应再次交给 LLM 解析"
        )

        parsed = self.node._parse_task("起飞到 10 米")

        self.assertEqual(parsed["intent"], "takeoff")
        self.assertEqual(parsed["parser"], "rules")

    def test_structured_takeoff_bypasses_language_parser(self):
        parsed = self.node._structured_action("takeoff", {"altitude": 8})

        self.assertEqual(parsed["intent"], "takeoff")
        self.assertEqual(parsed["args"]["altitude"], 8.0)
        self.assertFalse(parsed["need_confirm"])

    def test_structured_move_accepts_three_axis_vector(self):
        parsed = self.node._structured_action(
            "move", {"forward_m": 8, "right_m": 5, "up_m": 1}
        )

        self.assertEqual(parsed["intent"], "move_to")
        self.assertEqual(parsed["args"]["forward_m"], 8.0)
        self.assertEqual(parsed["args"]["right_m"], 5.0)
        self.assertEqual(parsed["args"]["up_m"], 1.0)

    def test_structured_action_rejects_unknown_capability(self):
        with self.assertRaises(ValueError):
            self.node._structured_action("run_shell", {})

    def test_sync_service_wait_returns_completed_future_without_nested_spin(self):
        class Future:
            def add_done_callback(self, callback):
                callback(self)

            def result(self):
                return SimpleNamespace(
                    success=True,
                    message="图像已保存",
                    file_path="/tmp/capture.png",
                )

        class Client:
            def service_is_ready(self):
                return True

            def call_async(self, _request):
                return Future()

        result = self.node._call_service(
            Client(), object(), "/perception/capture_image", timeout_sec=0.1
        )
        self.assertTrue(result["success"])
        self.assertEqual(result["file_path"], "/tmp/capture.png")


if __name__ == "__main__":
    unittest.main()
