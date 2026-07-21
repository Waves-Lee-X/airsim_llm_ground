import unittest

from aeromind_autonomy.semantic_guard import SemanticTrajectoryGuard


class SemanticTrajectoryGuardTest(unittest.TestCase):
    def test_intrusion_holds_until_clear_dwell_finishes(self):
        guard = SemanticTrajectoryGuard(clear_dwell_sec=1.0, hold_timeout_sec=10.0)
        guard.update_event("person_entered_path", True, ["person_1"], 1.0)
        self.assertEqual(guard.decision(1.2), "hold")
        guard.update_event("person_cleared_path", False, ["person_1"], 2.0)
        self.assertEqual(guard.decision(2.5), "clearing")
        self.assertEqual(guard.decision(3.0), "resume")
        self.assertAlmostEqual(guard.consume_resume(3.0), 2.0)
        self.assertEqual(guard.decision(3.1), "tracking")

    def test_multiple_people_must_all_clear(self):
        guard = SemanticTrajectoryGuard(clear_dwell_sec=0.0)
        guard.update_event("person_entered_path", True, ["person_1", "person_2"], 1.0)
        guard.update_event("person_cleared_path", False, ["person_1"], 2.0)
        self.assertEqual(guard.decision(2.0), "hold")
        guard.update_event("person_cleared_path", False, ["person_2"], 3.0)
        self.assertEqual(guard.decision(3.0), "resume")

    def test_hold_times_out_without_a_clear_event(self):
        guard = SemanticTrajectoryGuard(clear_dwell_sec=1.0, hold_timeout_sec=5.0)
        guard.update_event("person_entered_path", True, ["person_1"], 10.0)
        self.assertEqual(guard.decision(14.9), "hold")
        self.assertEqual(guard.decision(15.0), "timeout")

    def test_active_intrusion_is_preserved_between_missions(self):
        guard = SemanticTrajectoryGuard(clear_dwell_sec=0.0, hold_timeout_sec=5.0)
        guard.update_event("person_entered_path", True, ["person_1"], 1.0)
        guard.end_mission()
        guard.begin_mission(100.0)
        self.assertEqual(guard.decision(100.1), "hold")
        self.assertEqual(guard.decision(105.0), "timeout")


if __name__ == "__main__":
    unittest.main()
