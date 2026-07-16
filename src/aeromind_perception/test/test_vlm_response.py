import unittest

from aeromind_perception.perception_node import (
    extract_vlm_json,
    normalize_vlm_content,
)


FALLBACK = {
    "message": "规则分析完成",
    "scene": "前方画面可用",
    "risk_level": "medium",
    "suggestion": "保持安全距离",
    "objects": [{"name": "person"}],
}


class VlmResponseTest(unittest.TestCase):
    def test_extracts_fenced_json(self):
        result = extract_vlm_json(
            '```json\n{"message":"完成","scene":"道路","risk_level":"low",'
            '"suggestion":"继续观察","objects":[]}\n```'
        )
        self.assertEqual(result["scene"], "道路")

    def test_extracts_embedded_json_and_removes_trailing_comma(self):
        result = extract_vlm_json(
            '分析如下：\n{"message":"完成","scene":"空旷",'
            '"risk_level":"low","suggestion":"观察","objects":[],}'
        )
        self.assertEqual(result["scene"], "空旷")

    def test_invalid_json_preserves_semantic_text(self):
        content = '画面中有一名行人，位于道路右侧，距离较近。'
        result = normalize_vlm_content(content, FALLBACK)
        self.assertIn("行人", result["scene"])
        self.assertEqual(result["risk_level"], "medium")
        self.assertEqual(result["objects"], [{"name": "person"}])
        self.assertIn("format_warning", result)

    def test_invalid_risk_level_uses_fallback(self):
        result = normalize_vlm_content(
            '{"message":"完成","scene":"道路","risk_level":"critical",'
            '"suggestion":"停止","objects":[]}',
            FALLBACK,
        )
        self.assertEqual(result["risk_level"], "medium")


if __name__ == "__main__":
    unittest.main()
