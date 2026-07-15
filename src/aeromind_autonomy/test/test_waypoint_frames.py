import math
import unittest
from types import SimpleNamespace

from aeromind_autonomy.autonomy_node import AutonomyNode


class WaypointFrameTest(unittest.TestCase):
    def test_user_forward_right_up_segments_are_accumulated_in_world_frame(self):
        points = [
            SimpleNamespace(x=4.0, y=0.0, z=1.0),
            SimpleNamespace(x=0.0, y=3.0, z=0.0),
        ]
        world = AutonomyNode._relative_waypoints_to_world(
            (10.0, 20.0, 5.0), 0.0, points
        )
        self.assertEqual(world[0], (10.0, 24.0, 6.0))
        self.assertEqual(world[1], (13.0, 24.0, 6.0))

    def test_user_frame_rotates_with_initial_heading(self):
        point = [SimpleNamespace(x=2.0, y=0.0, z=0.0)]
        world = AutonomyNode._relative_waypoints_to_world(
            (0.0, 0.0, 3.0), math.pi / 2.0, point
        )
        self.assertAlmostEqual(world[0][0], -2.0, places=6)
        self.assertAlmostEqual(world[0][1], 0.0, places=6)
        self.assertEqual(world[0][2], 3.0)


if __name__ == "__main__":
    unittest.main()
