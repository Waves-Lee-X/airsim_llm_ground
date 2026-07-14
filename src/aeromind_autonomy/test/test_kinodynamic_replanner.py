import unittest

from aeromind_autonomy.kinodynamic_replanner import KinodynamicReplanner
from aeromind_autonomy.local_esdf import LocalEsdfMap


class KinodynamicReplannerTest(unittest.TestCase):
    def test_clear_route_tracks_goal(self):
        planner = KinodynamicReplanner(
            safety_radius=0.8,
            max_speed=2.0,
            min_altitude=0.5,
        )
        result = planner.replan(
            LocalEsdfMap(resolution=0.2),
            (0.0, 0.0, 2.0),
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 2.0),
        )
        self.assertEqual(result.state, "TRACKING")
        self.assertEqual(result.strategy, "direct_goal")
        self.assertTrue(result.collision_free)

    def test_obstacle_on_direct_path_selects_safe_side_primitive(self):
        esdf = LocalEsdfMap(resolution=0.2, decay_sec=20.0)
        obstacle = [
            (x, y, z)
            for x in (1.8, 2.0, 2.2)
            for y in (-0.4, -0.2, 0.0, 0.2, 0.4)
            for z in tuple(value / 5 for value in range(2, 26))
        ]
        esdf.insert_points(obstacle, stamp=1.0)
        planner = KinodynamicReplanner(
            safety_radius=0.8,
            max_speed=2.0,
            horizon_sec=2.5,
            start_ignore_radius=0.2,
            min_altitude=0.5,
        )

        result = planner.replan(
            esdf,
            (0.0, 0.0, 2.0),
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 2.0),
        )

        self.assertEqual(result.state, "TRACKING")
        self.assertTrue(result.collision_free)
        self.assertNotIn(result.strategy, {"direct_goal", "slow_goal"})
        self.assertGreater(abs(result.command_velocity[1]), 0.2)

    def test_arrival_requires_low_speed(self):
        planner = KinodynamicReplanner(min_altitude=0.5)
        esdf = LocalEsdfMap()
        moving = planner.replan(
            esdf,
            (0.0, 0.0, 2.0),
            (1.0, 0.0, 0.0),
            (0.1, 0.0, 2.0),
        )
        stopped = planner.replan(
            esdf,
            (0.0, 0.0, 2.0),
            (0.0, 0.0, 0.0),
            (0.1, 0.0, 2.0),
        )
        self.assertNotEqual(moving.state, "ARRIVED")
        self.assertEqual(stopped.state, "ARRIVED")


if __name__ == "__main__":
    unittest.main()
