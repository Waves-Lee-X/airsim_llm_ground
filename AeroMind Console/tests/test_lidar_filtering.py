from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.airsim_adapter import AirSimAdapter  # noqa: E402


class LidarFilteringTests(unittest.TestCase):
    def test_denoise_removes_isolated_points(self) -> None:
        points = [(0.1, 0.1), (10.0, 10.0), (20.0, 20.0)]

        filtered = AirSimAdapter._denoise_world_points(points, cell_m=2.0, min_points=2)

        self.assertEqual(filtered, [])

    def test_denoise_keeps_clustered_points(self) -> None:
        points = [(10.0, 10.0), (10.4, 10.2), (10.7, 10.6), (30.0, 30.0)]

        filtered = AirSimAdapter._denoise_world_points(points, cell_m=2.0, min_points=2)

        self.assertEqual(len(filtered), 3)
        self.assertNotIn((30.0, 30.0), filtered)


if __name__ == "__main__":
    unittest.main()
