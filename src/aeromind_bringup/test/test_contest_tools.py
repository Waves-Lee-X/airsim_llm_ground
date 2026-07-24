import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from aeromind_bringup_tools.mission_metrics import (
    _workflow_metrics,
    load_file_missions,
    load_gateway_missions,
    select_campaign,
    summarize,
    write_outputs,
)
from aeromind_bringup_tools.preflight import REQUIRED_TOPICS, build_report, evaluate_status


class PreflightTest(unittest.TestCase):
    @patch.dict(
        "os.environ",
        {
            "AEROMIND_LLM_API_KEY": "secret-llm",
            "AEROMIND_VLM_API_KEY": "secret-vlm",
            "AEROMIND_VLM_API_URL": "https://example.invalid/v1",
            "AEROMIND_VLM_MODEL": "vision-model",
        },
        clear=True,
    )
    def test_ready_system_has_no_failed_checks_and_hides_secrets(self):
        web = {
            "telemetry_meta": {"state_fresh": True, "odom_fresh": True},
            "depth": {"valid_samples": 100},
            "pointcloud": {"sampled_points": 500},
            "services": {"arm": True, "takeoff": True, "land": True, "agent": True},
        }
        gateway = {"ros_state_available": True, "feishu_vision_enabled": False}
        topics = set(REQUIRED_TOPICS) | {"/perception/detections"}
        checks = evaluate_status(
            web, "", {"success": True, "width": 1280, "height": 720}, "",
            gateway, "", topics, "", require_yolo=True, require_vlm=True,
        )
        report = build_report(checks)
        self.assertTrue(report["ready"])
        self.assertEqual(report["summary"]["FAIL"], 0)
        self.assertNotIn("secret-llm", json.dumps(report))
        self.assertNotIn("secret-vlm", json.dumps(report))

    @patch.dict("os.environ", {}, clear=True)
    def test_stale_telemetry_and_missing_topics_fail(self):
        checks = evaluate_status(
            {"telemetry_meta": {}, "services": {}}, "", None, "no image",
            {"ros_state_available": False}, "", set(), "",
            require_yolo=False, require_vlm=False,
        )
        report = build_report(checks)
        self.assertFalse(report["ready"])
        self.assertGreater(report["summary"]["FAIL"], 0)


class MissionMetricsTest(unittest.TestCase):
    def test_gateway_and_file_missions_are_aggregated(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            db = root / "gateway.db"
            connection = sqlite3.connect(db)
            connection.execute(
                """
                CREATE TABLE gateway_missions (
                    id TEXT, action TEXT, title TEXT, status TEXT, phase TEXT,
                    physical_complete INTEGER, result_json TEXT,
                    created_at REAL, updated_at REAL
                )
                """
            )
            connection.executemany(
                "INSERT INTO gateway_missions VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                [
                    ("m1", "takeoff", "起飞", "completed", "completed", 1, "{}", 10.0, 15.0),
                    ("m2", "move", "前进", "failed", "failed", 0, '{"message":"里程计超时"}', 20.0, 28.0),
                ],
            )
            connection.commit()
            connection.close()

            mission_dir = root / "missions" / "m3"
            mission_dir.mkdir(parents=True)
            (mission_dir / "mission.json").write_text(
                json.dumps({
                    "id": "m3", "status": "done", "message": "完成",
                    "created_at": 30.0, "updated_at": 36.0,
                    "parsed": {"intent": "capture_image"}, "steps": [],
                }),
                encoding="utf-8",
            )

            records = load_gateway_missions(db) + load_file_missions(root / "missions")
            summary = summarize(records)
            self.assertEqual(summary["total"], 3)
            self.assertEqual(summary["completed"], 2)
            self.assertEqual(summary["success_rate"], 0.6667)
            self.assertEqual(summary["failure_reasons"], {"里程计超时": 1})
            report = {"generated_at": 40.0, "summary": summary, "records": records}
            paths = write_outputs(report, root / "output", "metrics")
            self.assertTrue(all(path.is_file() for path in paths.values()))
            self.assertIn("66.7%", paths["markdown"].read_text(encoding="utf-8"))

    def test_campaign_uses_gateway_events_and_workflow_steps(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            db = root / "gateway.db"
            connection = sqlite3.connect(db)
            connection.executescript(
                """
                CREATE TABLE confirmations (
                    id TEXT, resolved_at REAL
                );
                CREATE TABLE gateway_missions (
                    id TEXT, confirmation_id TEXT, action TEXT, title TEXT,
                    status TEXT, phase TEXT, revision INTEGER,
                    physical_complete INTEGER, result_json TEXT,
                    created_at REAL, updated_at REAL
                );
                CREATE TABLE events (
                    type TEXT, payload_json TEXT, created_at REAL
                );
                """
            )
            workflow = {
                "message": "组合任务失败：分析当前画面",
                "workflow": {
                    "steps": [
                        {"id": "move", "label": "向前飞行", "status": "completed"},
                        {"id": "analyze", "label": "分析当前画面", "status": "failed"},
                        {"id": "land", "label": "降落", "status": "skipped"},
                    ]
                },
            }
            connection.executemany(
                "INSERT INTO confirmations VALUES (?, ?)",
                [("c1", 102.0), ("c2", 202.0)],
            )
            connection.executemany(
                "INSERT INTO gateway_missions VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                [
                    ("m1", "c1", "workflow", "校赛演示主链-01", "completed", "completed", 8, 1, "{}", 100.0, 112.0),
                    ("m2", "c2", "workflow", "校赛演示主链-02", "failed", "failed", 9, 0, json.dumps(workflow), 200.0, 215.0),
                    ("m3", "c3", "takeoff", "其他任务", "completed", "completed", 4, 1, "{}", 300.0, 305.0),
                ],
            )
            events = [
                ("confirmation.required", {"confirmation": {"id": "c1"}}, 100.0),
                ("confirmation.resolved", {"confirmation": {"id": "c1"}}, 102.0),
                ("control.started", {"confirmation_id": "c1"}, 103.0),
                ("control.progress", {"confirmation_id": "c1", "phase": "verifying"}, 108.0),
                ("control.completed", {"confirmation": {"id": "c1"}}, 112.0),
                ("confirmation.required", {"confirmation": {"id": "c2"}}, 200.0),
                ("confirmation.resolved", {"confirmation": {"id": "c2"}}, 202.0),
                ("control.started", {"confirmation_id": "c2"}, 203.0),
                ("control.completed", {"confirmation": {"id": "c2"}}, 215.0),
            ]
            connection.executemany(
                "INSERT INTO events VALUES (?, ?, ?)",
                [(kind, json.dumps(payload), timestamp) for kind, payload, timestamp in events],
            )
            connection.commit()
            connection.close()

            records = load_gateway_missions(db)
            selected = select_campaign(records, "workflow", "校赛演示主链", 10)
            self.assertEqual([item["id"] for item in selected], ["m1", "m2"])
            self.assertEqual(selected[0]["confirmation_latency_sec"], 2.0)
            self.assertEqual(selected[0]["execution_duration_sec"], 9.0)
            self.assertEqual(selected[0]["verification_duration_sec"], 4.0)
            self.assertEqual(selected[1]["failed_step"], "分析当前画面")
            self.assertEqual(selected[1]["workflow_step_skipped"], 1)

            summary = summarize(selected, expected_runs=2, target_success_rate=0.5)
            self.assertTrue(summary["campaign_ready"])
            self.assertEqual(summary["terminal_consistency_rate"], 1.0)
            self.assertEqual(summary["failed_steps"], {"分析当前画面": 1})
            self.assertEqual(summary["confirmation_mean_sec"], 2.0)

    def test_duplicate_terminal_event_fails_campaign_gate(self):
        record = {
            "status": "completed", "physical_complete": True,
            "duration_sec": 1.0, "action": "workflow", "failure_reason": "",
            "terminal_event_count": 2, "terminal_consistent": False,
        }
        summary = summarize([record], expected_runs=1, target_success_rate=1.0)
        self.assertFalse(summary["campaign_ready"])
        self.assertEqual(summary["duplicate_terminal_events"], 1)

    def test_campaign_can_require_takeoff_and_return_to_start(self):
        result = {
            "workflow": {
                "steps": [
                    {"id": "takeoff", "action": "takeoff", "status": "completed"},
                    {
                        "id": "return",
                        "action": "return_to_start",
                        "args": {"horizontal_tolerance_m": 1.0},
                        "status": "completed",
                    },
                    {"id": "land", "action": "land", "status": "completed"},
                ]
            },
            "step_results": {
                "return": {
                    "success": True,
                    "horizontal_error_m": 0.42,
                    "tolerance_m": 1.0,
                }
            },
        }
        record = {
            "status": "completed",
            "physical_complete": True,
            "duration_sec": 10.0,
            "action": "workflow",
            "failure_reason": "",
            "terminal_event_count": 1,
            "terminal_consistent": True,
            "event_trace_available": True,
            **_workflow_metrics(result),
        }

        summary = summarize(
            [record],
            expected_runs=1,
            target_success_rate=1.0,
            require_takeoff_completed=True,
            require_return_to_start=True,
        )

        self.assertTrue(summary["campaign_ready"])
        self.assertEqual(summary["takeoff_completion_rate"], 1.0)
        self.assertEqual(summary["return_to_start_rate"], 1.0)
        self.assertEqual(record["return_to_start_error_m"], 0.42)

        record["takeoff_status"] = "skipped"
        rejected = summarize(
            [record],
            expected_runs=1,
            target_success_rate=1.0,
            require_takeoff_completed=True,
            require_return_to_start=True,
        )
        self.assertFalse(rejected["campaign_ready"])


if __name__ == "__main__":
    unittest.main()
