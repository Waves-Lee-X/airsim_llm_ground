import unittest

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from aeromind_bridge.airsim_bridge_node import AirSimBridgeNode
from aeromind_bridge.px4_bridge import (
    ned_to_enu_position,
    ned_to_enu_quaternion,
    ned_to_enu_velocity,
)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def sendTransform(self, message):
        self.messages.append(message)


class OdometryTransformTest(unittest.TestCase):
    def test_odometry_publish_also_broadcasts_matching_transform(self):
        bridge = type("FakeBridge", (), {})()
        bridge._odom_pub = Recorder()
        bridge._tf_broadcaster = Recorder()
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 3.0
        odom.pose.pose.position.y = -2.0
        odom.pose.pose.position.z = 5.0
        odom.pose.pose.orientation.w = 1.0

        AirSimBridgeNode._publish_odometry(bridge, odom)

        self.assertIs(bridge._odom_pub.messages[0], odom)
        transform = bridge._tf_broadcaster.messages[0]
        self.assertEqual(transform.header.frame_id, "odom")
        self.assertEqual(transform.child_frame_id, "base_link")
        self.assertEqual(transform.transform.translation.x, 3.0)
        self.assertEqual(transform.transform.rotation.w, 1.0)

    def test_ned_values_are_converted_to_ros_enu(self):
        self.assertEqual(ned_to_enu_position(4.0, 2.0, -3.0), (2.0, 4.0, 3.0))
        self.assertEqual(ned_to_enu_velocity(1.0, -2.0, 0.5), (-2.0, 1.0, -0.5))
        converted = ned_to_enu_quaternion(
            Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
        )
        self.assertAlmostEqual(converted.x, 0.2)
        self.assertAlmostEqual(converted.y, 0.1)
        self.assertAlmostEqual(converted.z, -0.3)
        self.assertAlmostEqual(converted.w, 0.9)


if __name__ == "__main__":
    unittest.main()
