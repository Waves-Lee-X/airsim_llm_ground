import unittest

from aeromind_autonomy.minimum_snap import sample_minimum_snap, smootherstep7


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


if __name__ == "__main__":
    unittest.main()
