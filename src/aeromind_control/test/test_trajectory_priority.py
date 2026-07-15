import unittest
from types import SimpleNamespace

from aeromind_control.control_node import (
    autonomy_trajectory_action,
    estimator_status_flags_healthy,
    interpolate_trajectory,
    px4_gps_fix_to_drone_fix,
    should_accept_replan,
    should_accept_trajectory_update,
    trajectory_handoff_elapsed,
    trajectory_update_mode,
)
from aeromind_control.px4_control import _state_to_trajectory_setpoint


class TrajectoryPriorityTest(unittest.TestCase):
    def test_px4_gps_fix_is_mapped_to_drone_state_levels(self):
        self.assertEqual(px4_gps_fix_to_drone_fix(0), 0)
        self.assertEqual(px4_gps_fix_to_drone_fix(2), 1)
        self.assertEqual(px4_gps_fix_to_drone_fix(3), 2)
        self.assertEqual(px4_gps_fix_to_drone_fix(4), 3)
        self.assertEqual(px4_gps_fix_to_drone_fix(6), 4)

    def test_estimator_health_requires_alignment_and_no_faults(self):
        flags = type(
            "Flags",
            (),
            {
                "cs_tilt_align": True,
                "cs_yaw_align": True,
                "cs_inertial_dead_reckoning": False,
                "fs_bad_hdg": False,
            },
        )()
        self.assertTrue(estimator_status_flags_healthy(flags))
        flags.fs_bad_hdg = True
        self.assertFalse(estimator_status_flags_healthy(flags))
        flags.fs_bad_hdg = False
        flags.cs_inertial_dead_reckoning = True
        self.assertFalse(estimator_status_flags_healthy(flags))

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

    def test_replan_replacement_waits_for_active_segment_progress(self):
        self.assertTrue(should_accept_replan(False, 0.0, 0.4))
        self.assertFalse(should_accept_replan(True, 0.1, 0.4))
        self.assertTrue(should_accept_replan(True, 0.4, 0.4))

    def test_same_long_trajectory_id_refreshes_without_resetting_time(self):
        self.assertEqual(
            trajectory_update_mode("mission-1", "mission-1", False),
            "heartbeat",
        )
        self.assertEqual(
            trajectory_update_mode("mission-1", "mission-1", True),
            "replace",
        )
        self.assertEqual(
            trajectory_update_mode("mission-1", "mission-2", False),
            "replace",
        )

    def test_replan_reset_is_not_classified_as_heartbeat(self):
        self.assertEqual(
            trajectory_update_mode("mission-1", "mission-1", True),
            "replace",
        )
        self.assertTrue(
            should_accept_trajectory_update(True, True, 0.05, 0.6)
        )
        self.assertFalse(
            should_accept_trajectory_update(False, True, 0.05, 0.6)
        )

    def test_trajectory_is_interpolated_continuously(self):
        def point(t, position, velocity, acceleration, yaw=0.0):
            return SimpleNamespace(
                time_from_start=SimpleNamespace(
                    sec=int(t), nanosec=int((t - int(t)) * 1_000_000_000)
                ),
                position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
                velocity=SimpleNamespace(x=velocity[0], y=velocity[1], z=velocity[2]),
                acceleration=SimpleNamespace(
                    x=acceleration[0], y=acceleration[1], z=acceleration[2]
                ),
                yaw=yaw,
            )

        state = interpolate_trajectory(
            [
                point(0.0, (0.0, 0.0, 10.0), (0.0, 0.0, 0.0), (0.2, 0.0, 0.0)),
                point(1.0, (2.0, 0.0, 10.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ],
            0.5,
        )
        self.assertEqual(state["position"], (1.0, 0.0, 10.0))
        self.assertEqual(state["velocity"], (0.5, 0.0, 0.0))
        self.assertEqual(state["acceleration"], (0.1, 0.0, 0.0))

    def test_replan_handoff_continues_near_previous_setpoint(self):
        def point(t, x, vx):
            return SimpleNamespace(
                time_from_start=SimpleNamespace(
                    sec=int(t), nanosec=int((t - int(t)) * 1_000_000_000)
                ),
                position=SimpleNamespace(x=x, y=0.0, z=10.0),
                velocity=SimpleNamespace(x=vx, y=0.0, z=0.0),
                acceleration=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                yaw=0.0,
            )

        elapsed = trajectory_handoff_elapsed(
            [point(0.0, 0.0, 0.5), point(0.3, 0.8, 1.0), point(0.6, 1.5, 1.0)],
            {"position": (0.9, 0.0, 10.0), "velocity": (1.0, 0.0, 0.0)},
            0.6,
        )

        self.assertAlmostEqual(elapsed, 0.3, places=6)

    def test_first_trajectory_starts_at_zero_elapsed(self):
        self.assertEqual(trajectory_handoff_elapsed([], None, 0.6), 0.0)

    def test_px4_setpoint_contains_position_velocity_and_acceleration(self):
        setpoint = _state_to_trajectory_setpoint(
            (1.0, 2.0, 3.0),
            (0.4, 0.5, 0.6),
            (0.1, 0.2, 0.3),
            yaw=0.7,
        )
        self.assertEqual(list(setpoint.position), [2.0, 1.0, -3.0])
        for actual, expected in zip(setpoint.velocity, (0.5, 0.4, -0.6)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(setpoint.acceleration[0], 0.2, places=6)
        self.assertAlmostEqual(setpoint.acceleration[1], 0.1, places=6)
        self.assertAlmostEqual(setpoint.acceleration[2], -0.3, places=6)


if __name__ == "__main__":
    unittest.main()
