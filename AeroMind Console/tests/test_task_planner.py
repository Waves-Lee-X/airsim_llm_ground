from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.task_planner import TaskPlanner  # noqa: E402


class TaskPlannerChineseTests(unittest.TestCase):
    def setUp(self) -> None:
        self.planner = TaskPlanner()

    def test_chinese_area_search_vehicle(self) -> None:
        plan = self.planner.plan("搜索前方 60 米区域，发现车辆后悬停并报告坐标")

        self.assertEqual(plan.intent, "area_search")
        self.assertEqual(plan.target, "vehicle")
        self.assertEqual(plan.altitude_m, 8.0)
        self.assertIsNotNone(plan.area)
        self.assertEqual(plan.area.x_max, 60.0)

    def test_chinese_search_person_area_and_altitude(self) -> None:
        plan = self.planner.plan("查找前方80米左右15米范围内的行人，高度6米")

        self.assertEqual(plan.intent, "area_search")
        self.assertEqual(plan.target, "person")
        self.assertEqual(plan.altitude_m, 6.0)
        self.assertIsNotNone(plan.area)
        self.assertEqual(plan.area.x_max, 80.0)
        self.assertEqual(plan.area.y_min, -15.0)
        self.assertEqual(plan.area.y_max, 15.0)

    def test_chinese_scan_building_altitude(self) -> None:
        plan = self.planner.plan("在 12 米高度扫描目标建筑并保存关键帧")

        self.assertEqual(plan.intent, "object_or_area_scan")
        self.assertEqual(plan.target, "building")
        self.assertEqual(plan.altitude_m, 12.0)

    def test_chinese_formation(self) -> None:
        plan = self.planner.plan("三架无人机组成 V 字编队，保持 10 米高度")

        self.assertEqual(plan.intent, "formation_flight")
        self.assertEqual(plan.altitude_m, 10.0)


if __name__ == "__main__":
    unittest.main()
