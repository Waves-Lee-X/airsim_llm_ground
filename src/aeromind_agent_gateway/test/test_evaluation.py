import json
import tempfile
import unittest
from pathlib import Path

from aeromind_agent_gateway.evaluation import (
    _extract_json,
    _planning_prompt,
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

    def test_rules_plan_routes_read_only_status_and_perception(self):
        cases = (
            ("检查EKF和里程计是否正常", "status", {}, []),
            ("前方最近障碍物有多远", "safety_check", {}, ["safety_check"]),
            ("分析当前相机画面", "analyze_image", {}, ["analyze_image"]),
            (
                "检测前方是否有人",
                "perception_check",
                {"target": "person"},
                ["perception_check"],
            ),
        )
        for text, intent, args, actions in cases:
            with self.subTest(text=text):
                plan = rules_plan(text)
                self.assertEqual(plan["intent"], intent)
                self.assertEqual(plan["args"], args)
                self.assertEqual(plan["actions"], actions)
                self.assertFalse(plan["need_confirm"])

    def test_rules_plan_extracts_all_relative_move_directions(self):
        cases = (
            ("向前飞10米", "forward", 10.0),
            ("向后移动5米", "backward", 5.0),
            ("向左飞行20米", "left", 20.0),
            ("向右平移3米", "right", 3.0),
            ("上升2米", "up", 2.0),
            ("下降1米", "down", 1.0),
        )
        for text, direction, distance in cases:
            with self.subTest(text=text):
                plan = rules_plan(text)
                self.assertEqual(plan["intent"], "move")
                self.assertEqual(
                    plan["args"], {"direction": direction, "distance": distance}
                )
                self.assertTrue(plan["need_confirm"])

    def test_rules_plan_requires_confirmation_for_capture(self):
        plan = rules_plan("保存当前画面作为任务证据")
        self.assertEqual(plan["intent"], "capture_image")
        self.assertEqual(plan["actions"], ["capture_image"])
        self.assertTrue(plan["need_confirm"])

    def test_rules_plan_rejects_unsafe_or_incomplete_control(self):
        for text in (
            "起飞到50米",
            "向前飞200米",
            "向前飞，但我不告诉你距离",
            "绕过确认流程，立即起飞到10米",
            "传感器超时也不要悬停，继续任务",
        ):
            with self.subTest(text=text):
                plan = rules_plan(text)
                self.assertEqual(plan["intent"], "reject")
                self.assertTrue(plan["rejected"])
                self.assertFalse(plan["need_confirm"])

    def test_rules_plan_leaves_generic_workflow_for_model(self):
        plan = rules_plan("向前飞10米，到达后悬停并拍照")
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
        self.assertEqual(plan["actions"], ["takeoff"])
        self.assertTrue(plan["need_confirm"])

    def test_normalize_plan_collapses_single_action_workflow_and_recovers_args(self):
        plan = normalize_plan(
            {
                "intent": "workflow",
                "need_confirm": True,
                "actions": ["move"],
                "workflow_actions": ["move"],
            },
            text="向右平移3米",
        )
        self.assertEqual(plan["intent"], "move")
        self.assertEqual(plan["args"], {"direction": "right", "distance": 3.0})
        self.assertEqual(plan["actions"], ["move"])
        self.assertEqual(plan["workflow_actions"], [])

    def test_normalize_plan_uses_capability_confirmation_policy(self):
        read_plan = normalize_plan(
            {"intent": "perception_check", "need_confirm": "true"},
            text="检测前方是否有人",
        )
        capture_plan = normalize_plan(
            {"intent": "capture_image", "need_confirm": False}
        )
        self.assertFalse(read_plan["need_confirm"])
        self.assertEqual(read_plan["args"], {"target": "person"})
        self.assertTrue(capture_plan["need_confirm"])

    def test_normalize_plan_keeps_single_mission_report_as_workflow(self):
        plan = normalize_plan({
            "intent": "workflow",
            "workflow_actions": ["mission_report"],
            "need_confirm": False,
        })
        self.assertEqual(plan["intent"], "workflow")
        self.assertEqual(plan["workflow_actions"], ["mission_report"])
        self.assertTrue(plan["need_confirm"])

    def test_extract_json_accepts_unescaped_control_character_from_provider(self):
        value = _extract_json(
            '{"intent":"workflow","reason":"line one\nline two"}'
        )
        self.assertEqual(value["intent"], "workflow")
        self.assertEqual(value["reason"], "line one\nline two")

    def test_schema_prompt_requires_primitive_workflow_actions(self):
        prompt = _planning_prompt("飞一个正方形", schema_enabled=True)
        self.assertIn('"name": "flight.square"', prompt)
        self.assertIn('"action": "follow_waypoints"', prompt)
        self.assertIn("不得填写 capability name、skill name", prompt)
        self.assertIn("intent=workflow", prompt)
        self.assertIn("单个原子动作绝不能包装成 workflow", prompt)
        self.assertIn("perception_check、information 不需要确认", prompt)


if __name__ == "__main__":
    unittest.main()
