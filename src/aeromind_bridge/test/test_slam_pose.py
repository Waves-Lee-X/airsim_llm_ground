import math

from aeromind_bridge.slam_pose import compose_pose, multiply_quaternions


def test_compose_pose_applies_map_to_odom_correction():
    half = math.sqrt(0.5)
    position, orientation = compose_pose(
        (10.0, 2.0, 1.0),
        (0.0, 0.0, half, half),
        (2.0, 0.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert math.isclose(position[0], 10.0, abs_tol=1e-6)
    assert math.isclose(position[1], 4.0, abs_tol=1e-6)
    assert math.isclose(position[2], 4.0, abs_tol=1e-6)
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(orientation, (0.0, 0.0, half, half)))


def test_quaternion_multiplication_preserves_unit_norm():
    result = multiply_quaternions((0.1, 0.2, 0.3, 0.9), (0.2, 0.0, 0.1, 0.95))
    assert math.isclose(sum(value * value for value in result), 1.0, abs_tol=1e-6)
