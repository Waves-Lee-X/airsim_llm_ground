import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from aeromind_bringup_tools.mission_metrics import (
    load_file_missions,
    load_gateway_missions,
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


if __name__ == "__main__":
    unittest.main()
