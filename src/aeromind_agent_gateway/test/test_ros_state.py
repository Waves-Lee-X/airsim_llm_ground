import json
import unittest
from unittest.mock import patch

from aeromind_agent_gateway.ros_state import (
    _action_task,
    _evaluate_action_completion,
    _extract_ros_mission_id,
    _parse_agent_payload,
    _record_with_freshness,
)


class RosTaskMappingTest(unittest.TestCase):
    def test_control_actions_map_to_expected_agent_intents(self):
        self.assertEqual(_action_task("takeoff", {"altitude": 5.0}), ("起飞到 5.0 米", "takeoff"))
        self.assertEqual(_action_task("land", {}), ("降落", "land"))
        self.assertEqual(_action_task("return_home", {}), ("返航", "return_home"))
        self.assertEqual(
            _action_task("move", {"direction": "左侧", "distance": 20.0}),
            ("向左侧飞行 20.0 米", "move_to"),
        )

    def test_agent_payload_is_parsed_for_intent_verification(self):
        payload = {"parsed_task": {"intent": "takeoff"}}
        self.assertEqual(
            _parse_agent_payload(json.dumps(payload, ensure_ascii=False)), payload
        )
        self.assertEqual(_parse_agent_payload("not-json"), {})

    def test_ros_mission_id_is_extracted_from_tool_result(self):
        payload = {
            "tool_calls": [
                {
                    "name": "create_mission",
                    "result": {"id": "mission-123", "status": "running"},
                }
            ]
        }
        self.assertEqual(_extract_ros_mission_id(payload), "mission-123")

    def test_takeoff_requires_fresh_height_and_armed_state(self):
        snapshot = self._snapshot(armed=True, altitude=9.2, stamp=101.0)
        result = _evaluate_action_completion(
            "takeoff", {"altitude": 10.0}, snapshot, started_at=100.0
        )
        self.assertTrue(result["terminal"])
        self.assertTrue(result["success"])

        stale = self._snapshot(armed=True, altitude=10.0, stamp=90.0)
        result = _evaluate_action_completion(
            "takeoff", {"altitude": 10.0}, stale, started_at=100.0
        )
        self.assertFalse(result["terminal"])

    def test_move_goal_and_blocked_are_terminal(self):
        arrived = self._snapshot(stamp=101.0, strategy="goal_reached")
        result = _evaluate_action_completion(
            "move", {"distance": 10.0}, arrived, started_at=100.0
        )
        self.assertTrue(result["success"])

        blocked = self._snapshot(
            stamp=101.0,
            autonomy_state="BLOCKED_HOLD",
            strategy="blocked_hold",
            message="前方阻塞",
        )
        result = _evaluate_action_completion(
            "move", {"distance": 10.0}, blocked, started_at=100.0
        )
        self.assertTrue(result["terminal"])
        self.assertFalse(result["success"])

    def test_move_recovery_message_is_not_terminal_failure(self):
        recovering = self._snapshot(
            stamp=101.0,
            autonomy_state="RECOVERY",
            strategy="recovery_right",
            message="自主任务阻塞：尝试右绕 5.0 m",
        )
        result = _evaluate_action_completion(
            "move", {"distance": 10.0}, recovering, started_at=100.0
        )
        self.assertFalse(result["terminal"])
        self.assertFalse(result["success"])

    def test_move_uses_latched_terminal_after_status_returns_to_hover(self):
        snapshot = self._snapshot(
            stamp=102.0,
            autonomy_state="HOLD",
            strategy="hover_no_goal",
        )
        snapshot["autonomy_terminal"] = {
            "stamp": 101.0,
            "state": "ARRIVED",
            "active_strategy": "goal_reached",
            "message": "自主任务完成：已到达目标附近，目标已清空",
        }
        result = _evaluate_action_completion(
            "move", {"forward_m": 10.0, "right_m": 5.0}, snapshot, started_at=100.0
        )
        self.assertTrue(result["terminal"])
        self.assertTrue(result["success"])

    def test_move_ignores_terminal_from_previous_step(self):
        snapshot = self._snapshot(
            stamp=202.0,
            autonomy_state="TRACKING",
            strategy="direct_goal",
        )
        snapshot["autonomy_terminal"] = {
            "stamp": 199.8,
            "state": "ARRIVED",
            "active_strategy": "goal_reached",
        }
        result = _evaluate_action_completion(
            "move", {"forward_m": 10.0}, snapshot, started_at=200.0
        )
        self.assertFalse(result["terminal"])

    def test_matching_ros_mission_has_priority(self):
        snapshot = self._snapshot(stamp=101.0)
        snapshot["mission"] = {
            "id": "mission-1",
            "status": "done",
            "message": "复合任务完成",
        }
        result = _evaluate_action_completion(
            "move", {}, snapshot, started_at=100.0, ros_mission_id="mission-1"
        )
        self.assertTrue(result["success"])
        self.assertEqual(result["message"], "复合任务完成")

    @patch("aeromind_agent_gateway.ros_state.time.time", return_value=105.0)
    def test_record_freshness_is_explicit(self, _time):
        fresh = _record_with_freshness({"stamp": 104.0}, 2.0)
        stale = _record_with_freshness({"stamp": 100.0}, 2.0)
        self.assertEqual(fresh["age_sec"], 1.0)
        self.assertTrue(fresh["fresh"])
        self.assertEqual(stale["age_sec"], 5.0)
        self.assertFalse(stale["fresh"])

    @staticmethod
    def _snapshot(
        *,
        armed=False,
        altitude=0.0,
        stamp=0.0,
        autonomy_state="ACTIVE",
        strategy="direct_goal",
        message="",
    ):
        return {
            "state": {"stamp": stamp, "armed": armed, "mode": "OFFBOARD"},
            "odometry": {
                "stamp": stamp,
                "position_m": {"x": 0.0, "y": 0.0, "z": altitude},
                "velocity_mps": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "autonomy": {
                "stamp": stamp,
                "state": autonomy_state,
                "active_strategy": strategy,
                "message": message,
            },
            "mission": None,
        }


if __name__ == "__main__":
    unittest.main()
