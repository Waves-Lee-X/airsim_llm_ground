import math

from aeromind_perception.object_tracking import (
    ConstantVelocityTrack,
    greedy_association,
)


def test_constant_velocity_filter_estimates_motion_and_reduces_uncertainty():
    track = ConstantVelocityTrack(
        "person_0001", "person", (0.0, 0.0, 0.0), 0.8, 1.0
    )
    initial_variance = track.position_covariance[0]
    track.update((1.0, 0.0, 0.0), 0.9, 2.0)
    track.update((2.0, 0.0, 0.0), 0.9, 3.0)

    assert 1.7 < track.position[0] < 2.2
    assert track.velocity[0] > 0.4
    assert track.position_covariance[0] < initial_variance
    assert track.lifecycle(minimum_hits=3, lost_after_misses=3) == "confirmed"


def test_prediction_increases_uncertainty_and_lifecycle_becomes_lost():
    track = ConstantVelocityTrack(
        "car_0001", "car", (0.0, 0.0, 0.0), 0.7, 1.0
    )
    before = track.position_covariance[0]
    track.mark_missed(2.0)
    track.mark_missed(3.0)
    track.mark_missed(4.0)

    assert track.position_covariance[0] > before
    assert track.lifecycle(minimum_hits=3, lost_after_misses=3) == "lost"


def test_greedy_association_preserves_unique_same_class_matches():
    tracks = {
        "person_0001": ConstantVelocityTrack(
            "person_0001", "person", (0.0, 0.0, 0.0), 0.9, 1.0
        ),
        "person_0002": ConstantVelocityTrack(
            "person_0002", "person", (5.0, 0.0, 0.0), 0.9, 1.0
        ),
    }
    observations = [
        {"class_name": "person", "position_valid": True, "position": (0.2, 0.0, 0.0)},
        {"class_name": "person", "position_valid": True, "position": (4.8, 0.0, 0.0)},
    ]

    matches = greedy_association(observations, tracks, maximum_distance=1.0)
    assert matches == {0: "person_0001", 1: "person_0002"}
    assert math.dist(tracks[matches[0]].position, observations[0]["position"]) < 1.0
