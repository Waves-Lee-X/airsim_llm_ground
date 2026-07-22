import unittest

from aeromind_autonomy.dynamic_obstacles import DynamicObstacleField


class DynamicObstacleFieldTest(unittest.TestCase):
    def test_crossing_person_reduces_future_trajectory_clearance(self):
        field = DynamicObstacleField(
            timeout_sec=2.0,
            default_radius=0.4,
            uncertainty_sigma=0.0,
            minimum_confidence=0.5,
        )
        field.update(
            [{
                "id": "person_1",
                "class_name": "person",
                "confidence": 0.9,
                "position": (2.0, -2.0, 2.0),
                "velocity": (0.0, 1.0, 0.0),
                "size": (0.4, 0.4, 1.7),
                "position_covariance": [0.0] * 9,
            }],
            now=10.0,
        )
        result = field.trajectory_clearance(
            [
                {"t": 0.0, "position": (0.0, 0.0, 2.0)},
                {"t": 2.0, "position": (2.0, 0.0, 2.0)},
            ],
            now=10.0,
        )
        self.assertEqual(result.object_id, "person_1")
        self.assertAlmostEqual(result.time_to_closest_sec, 2.0)
        self.assertEqual(result.clearance, 0.0)

    def test_uncertainty_inflates_dynamic_exclusion_radius(self):
        field = DynamicObstacleField(
            default_radius=0.3,
            uncertainty_sigma=2.0,
            minimum_confidence=0.0,
        )
        covariance = [0.0] * 9
        covariance[0] = covariance[4] = covariance[8] = 0.25
        field.update([{
            "id": "person_1",
            "class_name": "person",
            "confidence": 0.8,
            "position": (2.0, 0.0, 2.0),
            "velocity": (0.0, 0.0, 0.0),
            "size": (0.0, 0.0, 0.0),
            "position_covariance": covariance,
        }], now=1.0)
        result = field.current_clearance((0.0, 0.0, 2.0), now=1.0)
        self.assertAlmostEqual(result.clearance, 0.7)

    def test_stale_or_low_confidence_objects_do_not_constrain_path(self):
        field = DynamicObstacleField(timeout_sec=1.0, minimum_confidence=0.5)
        field.update([{
            "id": "person_1",
            "class_name": "person",
            "confidence": 0.2,
            "position": (0.0, 0.0, 0.0),
            "velocity": (0.0, 0.0, 0.0),
        }], now=1.0)
        self.assertEqual(field.current_clearance((0.0, 0.0, 0.0), now=1.0).clearance, 12.0)
        field.update([{
            "id": "person_2",
            "class_name": "person",
            "confidence": 0.9,
            "position": (0.0, 0.0, 0.0),
            "velocity": (0.0, 0.0, 0.0),
        }], now=1.0)
        self.assertEqual(field.current_clearance((0.0, 0.0, 0.0), now=2.1).clearance, 12.0)


if __name__ == "__main__":
    unittest.main()
