import unittest

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


if __name__ == "__main__":
    unittest.main()
