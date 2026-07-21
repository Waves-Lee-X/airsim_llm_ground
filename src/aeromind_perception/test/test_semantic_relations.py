import math

from aeromind_perception.semantic_relations import (
    PathIntrusionMonitor,
    point_to_path_distance,
    point_to_segment_distance,
    predicted_path_distance,
)


def test_point_to_segment_distance_clamps_to_segment_ends():
    assert point_to_segment_distance((5, 2, 0), (0, 0, 0), (10, 0, 0)) == 2.0
    assert math.isclose(
        point_to_segment_distance((12, 0, 0), (0, 0, 0), (10, 0, 0)), 2.0
    )


def test_point_to_path_uses_nearest_segment():
    path = [(0, 0, 0), (10, 0, 0), (10, 10, 0)]
    assert point_to_path_distance((8, 3, 0), path) == 2.0


def test_prediction_detects_object_moving_towards_path():
    distance, time_offset = predicted_path_distance(
        (5, 5, 0), (0, -2, 0), [(0, 0, 0), (10, 0, 0)], horizon_sec=3.0
    )
    assert distance == 0.0
    assert time_offset == 2.5


def test_intrusion_monitor_requires_confirm_and_clear_hysteresis():
    monitor = PathIntrusionMonitor(confirm_frames=2, clear_frames=2)
    assert monitor.update(["person_1"])["entered"] == []
    assert monitor.update(["person_1"])["entered"] == ["person_1"]
    assert monitor.update([])["cleared"] == []
    result = monitor.update([])
    assert result["cleared"] == ["person_1"]
    assert result["active"] == []
