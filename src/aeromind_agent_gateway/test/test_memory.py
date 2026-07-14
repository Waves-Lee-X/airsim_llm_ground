import json
import unittest

from aeromind_agent_gateway.memory import _redact_sensitive_text, parse_memory_result


class MemoryParsingTest(unittest.TestCase):
    def test_memory_json_is_normalized_and_sensitive_items_are_removed(self):
        value = json.dumps(
            {
                "summary": " 用户偏好低速巡检。 ",
                "memories": [
                    {
                        "kind": "preference",
                        "content": "用户偏好以 2 m/s 低速巡检",
                        "importance": 0.9,
                    },
                    {
                        "kind": "configuration",
                        "content": "API_KEY=secret",
                        "importance": 1.0,
                    },
                    {
                        "kind": "unknown",
                        "content": "一号区域是当前巡检重点",
                        "importance": "invalid",
                    },
                ],
            },
            ensure_ascii=False,
        )
        result = parse_memory_result(f"```json\n{value}\n```")
        self.assertEqual(result["summary"], "用户偏好低速巡检。")
        self.assertEqual(len(result["memories"]), 2)
        self.assertEqual(result["memories"][1]["kind"], "fact")
        self.assertEqual(result["memories"][1]["importance"], 0.5)

    def test_invalid_model_output_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_memory_result("not json")

    def test_history_credentials_are_redacted_before_summarization(self):
        value = _redact_sensitive_text(
            "API_KEY=abc123 password: hello Bearer token-value"
        )
        self.assertNotIn("abc123", value)
        self.assertNotIn("hello", value)
        self.assertNotIn("token-value", value)


if __name__ == "__main__":
    unittest.main()
