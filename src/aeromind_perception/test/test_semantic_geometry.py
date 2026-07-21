import math

from aeromind_perception.semantic_geometry import (
    associate_track,
    project_pixel,
    transform_point,
)


def test_project_pixel_uses_camera_intrinsics():
    point = project_pixel(420.0, 290.0, 5.0, [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0])
    assert point == (1.0, 0.5, 5.0)


def test_transform_point_rotates_and_translates():
    half = math.sqrt(0.5)
    point = transform_point((1.0, 0.0, 0.0), (10.0, 2.0, 3.0), (0.0, 0.0, half, half))
    assert math.isclose(point[0], 10.0, abs_tol=1e-6)
    assert math.isclose(point[1], 3.0, abs_tol=1e-6)
    assert math.isclose(point[2], 3.0, abs_tol=1e-6)


def test_associate_track_requires_class_and_distance_gate():
    tracks = {
        "person_0001": {
            "class_name": "person",
            "position_valid": True,
            "position": (1.0, 2.0, 0.0),
        },
        "car_0002": {
            "class_name": "car",
            "position_valid": True,
            "position": (1.1, 2.0, 0.0),
        },
    }
    assert associate_track("person", (1.2, 2.0, 0.0), tracks, 1.0) == "person_0001"
    assert associate_track("person", (4.0, 2.0, 0.0), tracks, 1.0) is None
