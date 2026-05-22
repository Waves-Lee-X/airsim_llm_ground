from __future__ import annotations

import json
import sys
import threading
import unittest
from pathlib import Path
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import server  # noqa: E402


SEARCH_TEXT = "\u641c\u7d22\u524d\u65b960\u7c73\u533a\u57df\uff0c\u53d1\u73b0\u8f66\u8f86\u540e\u60ac\u505c\u5e76\u62a5\u544a\u5750\u6807"
PATH_TEXT = "\u89c4\u5212\u4e00\u6761\u5411\u524d60\u7c73\u7684\u822a\u70b9\u8def\u7ebf\uff0c\u9ad8\u5ea68\u7c73"
FORMATION_TEXT = "\u4e09\u67b6\u65e0\u4eba\u673a\u7ec4\u6210 V \u5b57\u7f16\u961f\uff0c\u4fdd\u630110\u7c73\u9ad8\u5ea6\u5411\u524d\u98de\u884c40\u7c73"


class TaskPreviewApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.httpd = server.MissionConsoleServer(("127.0.0.1", 0), server.Handler)
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()
        cls.base_url = f"http://127.0.0.1:{cls.httpd.server_address[1]}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.httpd.shutdown()
        cls.httpd.server_close()
        cls.thread.join(timeout=2)

    def post_preview(self, payload: dict[str, object]) -> dict[str, object]:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = Request(
            f"{self.base_url}/api/task/preview",
            data=body,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        with urlopen(request, timeout=6) as response:
            self.assertEqual(response.status, 200)
            return json.loads(response.read().decode("utf-8"))

    def test_preview_returns_executable_search_area_and_preflight(self) -> None:
        payload = {
            "text": SEARCH_TEXT,
            "params": {
                "target": "vehicle",
                "altitude_m": 8,
                "speed_mps": 2,
                "spacing_m": 10,
                "area": {"x_min": 0, "x_max": 60, "y_min": -20, "y_max": 20},
                "pre_scan": True,
                "planned_avoidance": True,
            },
        }

        preview = self.post_preview(payload)

        self.assertEqual(preview["intent"], "area_search")
        self.assertEqual(preview["target"], "vehicle")
        self.assertEqual(preview["area"]["x_max"], 60)
        self.assertEqual(preview["executable"]["tool_call"]["tool"], "search_area")
        self.assertTrue(preview["executable"]["accepted"])
        self.assertGreater(preview["preview"]["waypoint_count"], 0)
        self.assertIn(preview["preflight"]["overall"], {"ready", "caution"})
        self.assertTrue(preview["preflight"]["checks"])

    def test_preview_reports_safety_error_for_bad_altitude(self) -> None:
        payload = {
            "text": SEARCH_TEXT,
            "params": {
                "target": "vehicle",
                "altitude_m": 60,
                "speed_mps": 2,
                "spacing_m": 10,
                "area": {"x_min": 0, "x_max": 60, "y_min": -20, "y_max": 20},
            },
        }

        preview = self.post_preview(payload)

        self.assertFalse(preview["executable"]["accepted"])
        self.assertEqual(preview["preflight"]["overall"], "blocked")
        self.assertIn("Altitude", preview["executable"]["safety"]["reason"])

    def test_preview_reports_missing_tool_mapping(self) -> None:
        preview = self.post_preview({"text": "\u4fdd\u6301\u5f53\u524d\u72b6\u6001"})

        self.assertEqual(preview["intent"], "general_uav_task")
        self.assertIsNone(preview["executable"])
        self.assertEqual(preview["preflight"]["overall"], "blocked")

    def test_preview_maps_path_planning_to_waypoint_route(self) -> None:
        preview = self.post_preview(
            {
                "text": PATH_TEXT,
                "params": {
                    "altitude_m": 8,
                    "speed_mps": 2,
                    "area": {"x_min": 0, "x_max": 60, "y_min": -10, "y_max": 10},
                    "avoidance": True,
                },
            }
        )

        self.assertEqual(preview["intent"], "path_planning")
        self.assertEqual(preview["executable"]["tool_call"]["tool"], "waypoint_route")
        self.assertTrue(preview["executable"]["accepted"])
        self.assertEqual(preview["preview"]["waypoint_count"], 2)

    def test_preview_maps_formation_to_formation_tool(self) -> None:
        preview = self.post_preview(
            {
                "text": FORMATION_TEXT,
                "params": {
                    "altitude_m": 10,
                    "speed_mps": 2,
                    "count": 3,
                    "shape": "v",
                    "area": {"x_min": 0, "x_max": 40, "y_min": -15, "y_max": 15},
                },
            }
        )

        self.assertEqual(preview["intent"], "formation_flight")
        self.assertEqual(preview["executable"]["tool_call"]["tool"], "formation_flight")
        self.assertTrue(preview["executable"]["accepted"])
        self.assertEqual(preview["preview"]["waypoint_count"], 2)


if __name__ == "__main__":
    unittest.main()
