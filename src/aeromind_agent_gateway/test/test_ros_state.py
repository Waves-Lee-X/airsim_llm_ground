import json
import unittest

from aeromind_agent_gateway.ros_state import (
    _action_task,
    _evaluate_action_completion,
    _extract_ros_mission_id,
    _parse_agent_payload,
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
            stamp=101.0, autonomy_state="BLOCKED_HOLD", message="前方阻塞"
        )
        result = _evaluate_action_completion(
            "move", {"distance": 10.0}, blocked, started_at=100.0
        )
        self.assertTrue(result["terminal"])
        self.assertFalse(result["success"])

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
