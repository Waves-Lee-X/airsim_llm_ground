import unittest

from aeromind_autonomy.minimum_snap import (
    sample_minimum_snap,
    sample_minimum_snap_boundary,
    sample_minimum_snap_waypoints,
    splice_replanned_trajectory,
    smootherstep7,
)


class MinimumSnapTest(unittest.TestCase):
    def test_seventh_order_boundary_derivatives_are_zero(self):
        for normalized in (0.0, 1.0):
            position, velocity, acceleration, jerk = smootherstep7(normalized)
            self.assertAlmostEqual(position, normalized, places=8)
            self.assertAlmostEqual(velocity, 0.0, places=8)
            self.assertAlmostEqual(acceleration, 0.0, places=8)
            self.assertAlmostEqual(jerk, 0.0, places=8)

    def test_sampled_trajectory_reaches_endpoint(self):
        samples = sample_minimum_snap((1.0, 2.0, 3.0), (5.0, 4.0, 3.0), 2.5, 20)
        self.assertEqual(samples[0]["position"], (1.0, 2.0, 3.0))
        self.assertEqual(samples[-1]["position"], (5.0, 4.0, 3.0))
        self.assertEqual(samples[0]["velocity"], (0.0, 0.0, 0.0))
        self.assertEqual(samples[-1]["jerk"], (0.0, 0.0, 0.0))

    def test_nonzero_velocity_boundaries_are_preserved(self):
        samples = sample_minimum_snap_boundary(
            (0.0, 0.0, 2.0),
            (4.0, 1.0, 2.0),
            2.5,
            20,
            start_velocity=(0.8, 0.1, 0.0),
            end_velocity=(1.2, 0.3, 0.0),
        )
        for actual, expected in zip(samples[0]["velocity"], (0.8, 0.1, 0.0)):
            self.assertAlmostEqual(actual, expected, places=7)
        for actual, expected in zip(samples[-1]["velocity"], (1.2, 0.3, 0.0)):
            self.assertAlmostEqual(actual, expected, places=7)
        for value in samples[0]["acceleration"] + samples[-1]["acceleration"]:
            self.assertAlmostEqual(value, 0.0, places=7)

    def test_multi_waypoint_trajectory_passes_through_without_stopping(self):
        samples, waypoint_times = sample_minimum_snap_waypoints(
            (0.0, 0.0, 2.0),
            [(4.0, 0.0, 2.0), (8.0, 3.0, 2.0)],
            cruise_speed=2.0,
            sample_dt=0.05,
        )
        self.assertEqual(len(waypoint_times), 2)
        boundary = min(samples, key=lambda item: abs(item["t"] - waypoint_times[0]))
        for actual, expected in zip(boundary["position"], (4.0, 0.0, 2.0)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertGreater(sum(value * value for value in boundary["velocity"]), 0.1)
        for actual, expected in zip(samples[-1]["position"], (8.0, 3.0, 2.0)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertTrue(all(a["t"] < b["t"] for a, b in zip(samples, samples[1:])))

    def test_multi_waypoint_rejects_duplicate_adjacent_points(self):
        with self.assertRaisesRegex(ValueError, "零长度"):
            sample_minimum_snap_waypoints(
                (0.0, 0.0, 1.0), [(0.0, 0.0, 1.0)], cruise_speed=1.0
            )

    def test_local_replan_splices_remaining_waypoints_continuously(self):
        local = sample_minimum_snap_boundary(
            (0.0, 0.0, 2.0),
            (2.0, 1.5, 2.0),
            2.0,
            21,
            end_velocity=(1.0, 0.2, 0.0),
        )
        combined, waypoint_times = splice_replanned_trajectory(
            local,
            [(6.0, 0.0, 2.0), (10.0, 0.0, 2.0)],
            cruise_speed=1.5,
            sample_dt=0.05,
        )
        boundary_index = len(local) - 1
        self.assertEqual(combined[boundary_index]["position"], local[-1]["position"])
        self.assertEqual(combined[boundary_index]["velocity"], local[-1]["velocity"])
        self.assertGreater(combined[boundary_index + 1]["t"], local[-1]["t"])
        self.assertEqual(len(waypoint_times), 2)
        for actual, expected in zip(combined[-1]["position"], (10.0, 0.0, 2.0)):
            self.assertAlmostEqual(actual, expected, places=6)


if __name__ == "__main__":
    unittest.main()
