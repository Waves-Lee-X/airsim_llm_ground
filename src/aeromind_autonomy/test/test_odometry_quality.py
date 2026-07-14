import math
import unittest

from nav_msgs.msg import Odometry

from aeromind_autonomy.odometry_quality import odometry_quality_issue


class OdometryQualityTest(unittest.TestCase):
    def test_valid_zero_covariance_px4_odometry_is_accepted(self):
        msg = self._message()
        self.assertIsNone(self._check(msg))

    def test_non_finite_pose_is_rejected(self):
        msg = self._message()
        msg.pose.pose.position.x = math.nan
        self.assertIn("NaN", self._check(msg))

    def test_vio_covariance_threshold_is_enforced(self):
        msg = self._message()
        msg.pose.covariance[0] = 3.0
        self.assertIn("位置协方差", self._check(msg))

    def test_invalid_quaternion_is_rejected(self):
        msg = self._message()
        msg.pose.pose.orientation.w = 0.0
        self.assertIn("四元数", self._check(msg))

    @staticmethod
    def _message():
        msg = Odometry()
        msg.pose.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def _check(msg):
        return odometry_quality_issue(
            msg,
            max_position_variance=2.0,
            max_orientation_variance=0.5,
        )


if __name__ == "__main__":
    unittest.main()
