import unittest

from aeromind_control.control_node import autonomy_trajectory_action


class TrajectoryPriorityTest(unittest.TestCase):
    def test_sensor_timeout_cannot_override_takeoff(self):
        action = autonomy_trajectory_action("TAKEOFF", "sensor_timeout_hold", False, True)
        self.assertEqual(action, "ignore_flight_mode")

    def test_valid_trajectory_cannot_override_native_flight_modes(self):
        for mode in ("TAKEOFF", "LAND", "RETURN_HOME"):
            with self.subTest(mode=mode):
                action = autonomy_trajectory_action(mode, "direct_goal", True, True)
                self.assertEqual(action, "ignore_flight_mode")

    def test_no_goal_does_not_create_a_zero_velocity_override(self):
        action = autonomy_trajectory_action("IDLE", "hover_no_goal", False, True)
        self.assertEqual(action, "ignore_no_goal")

    def test_active_autonomy_holds_on_sensor_failure(self):
        action = autonomy_trajectory_action("AUTONOMY", "sensor_timeout_hold", False, True)
        self.assertEqual(action, "hold_unsafe")

    def test_ready_mode_accepts_valid_autonomy_trajectory(self):
        action = autonomy_trajectory_action("READY", "direct_goal", True, True)
        self.assertEqual(action, "execute")


if __name__ == "__main__":
    unittest.main()
