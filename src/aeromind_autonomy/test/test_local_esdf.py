import math
import unittest

from aeromind_autonomy.frame_transform import body_to_world, rotate_vector
from aeromind_autonomy.local_esdf import LocalEsdfMap


class LocalEsdfTest(unittest.TestCase):
    def test_body_point_rotates_into_world_frame(self):
        yaw_90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        rotated = rotate_vector((2.0, 0.0, 0.0), yaw_90)
        world = body_to_world((2.0, 0.0, 0.0), (10.0, 4.0, 3.0), yaw_90)

        self.assertAlmostEqual(rotated[0], 0.0, places=6)
        self.assertAlmostEqual(rotated[1], 2.0, places=6)
        self.assertEqual(tuple(round(value, 6) for value in world), (10.0, 6.0, 3.0))

    def test_standard_optical_frame_points_forward_in_body_x(self):
        optical_to_body = (-0.5, 0.5, -0.5, 0.5)
        forward = rotate_vector((0.0, 0.0, 3.0), optical_to_body)
        image_right = rotate_vector((2.0, 0.0, 0.0), optical_to_body)

        self.assertEqual(tuple(round(value, 6) for value in forward), (3.0, 0.0, 0.0))
        self.assertEqual(tuple(round(value, 6) for value in image_right), (0.0, -2.0, 0.0))

    def test_new_scan_clears_old_obstacle_along_visible_ray(self):
        esdf = LocalEsdfMap(resolution=0.5, decay_sec=10.0)
        esdf.insert_points([(2.0, 0.0, 1.0)], stamp=1.0, sensor_origin=(0.0, 0.0, 1.0))
        self.assertLess(esdf.clearance((2.0, 0.0, 1.0)), 0.5)

        esdf.insert_points([(4.0, 0.0, 1.0)], stamp=2.0, sensor_origin=(0.0, 0.0, 1.0))

        self.assertGreater(esdf.clearance((2.0, 0.0, 1.0), 1.0), 0.9)
        self.assertLess(esdf.clearance((4.0, 0.0, 1.0)), 0.5)

    def test_stale_and_outside_voxels_are_pruned(self):
        esdf = LocalEsdfMap(
            resolution=0.5,
            rolling_radius=3.0,
            decay_sec=2.0,
        )
        esdf.insert_points([(1.0, 0.0, 1.0), (8.0, 0.0, 1.0)], stamp=1.0)
        esdf.prune((0.0, 0.0, 1.0), stamp=1.5)
        self.assertEqual(len(esdf.occupied), 1)
        esdf.prune((0.0, 0.0, 1.0), stamp=4.0)
        self.assertFalse(esdf.occupied)

    def test_map_points_are_voxel_centers(self):
        esdf = LocalEsdfMap(resolution=0.5)
        esdf.insert_points([(1.1, -0.1, 2.2)], stamp=1.0)
        self.assertEqual(esdf.points(), [(1.25, -0.25, 2.25)])


if __name__ == "__main__":
    unittest.main()
