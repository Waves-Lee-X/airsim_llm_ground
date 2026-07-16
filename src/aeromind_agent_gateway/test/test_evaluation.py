import json
import tempfile
import unittest
from pathlib import Path

from aeromind_agent_gateway.evaluation import (
    evaluate_case,
    load_cases,
    normalize_plan,
    rules_plan,
    summarize,
)


class EvaluationTest(unittest.TestCase):
    def test_rules_plan_extracts_takeoff_altitude(self):
        plan = rules_plan("请起飞到12.5米")
        self.assertEqual(plan["intent"], "takeoff")
        self.assertEqual(plan["args"], {"altitude": 12.5})
        self.assertTrue(plan["need_confirm"])

    def test_rules_plan_reports_unhandled_without_executing(self):
        plan = rules_plan("检查前方环境")
        self.assertEqual(plan["intent"], "unknown")
        self.assertEqual(plan["actions"], [])

    def test_workflow_actions_are_extracted(self):
        plan = rules_plan("飞一个边长10米的正方形")
        self.assertEqual(plan["intent"], "workflow")
        self.assertIn("follow_waypoints", plan["workflow_actions"])

    def test_case_scoring_accepts_expected_argument_subset(self):
        case = {
            "id": "takeoff-1",
            "input": "起飞到10米",
            "expected_intent": "takeoff",
            "expected_args": {"altitude": 10},
            "expected_confirm": True,
            "expected_actions": ["takeoff"],
        }
        result = evaluate_case(case, rules_plan(case["input"]))
        self.assertTrue(result["passed"])

    def test_summary_contains_accuracy_and_category(self):
        case = {
            "id": "land-1",
            "category": "flight",
            "input": "降落",
            "expected_intent": "land",
            "expected_confirm": True,
            "expected_actions": ["land"],
        }
        result = evaluate_case(case, rules_plan(case["input"]))
        summary = summarize([result], "rules")
        self.assertEqual(summary["overall_accuracy"], 1.0)
        self.assertEqual(summary["categories"]["flight"]["passed"], 1)
        self.assertEqual(summary["args_accuracy_samples"], 0)
        self.assertEqual(summary["high_risk_confirmation_recall"], 1.0)

    def test_dataset_rejects_duplicate_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cases.jsonl"
            value = {"id": "same", "input": "状态"}
            path.write_text(
                json.dumps(value, ensure_ascii=False) + "\n"
                + json.dumps(value, ensure_ascii=False),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "重复"):
                load_cases(path)

    def test_normalize_plan_sanitizes_invalid_collections(self):
        plan = normalize_plan({"intent": "TAKEOFF", "actions": "takeoff"})
        self.assertEqual(plan["intent"], "takeoff")
        self.assertEqual(plan["actions"], [])


if __name__ == "__main__":
    unittest.main()
