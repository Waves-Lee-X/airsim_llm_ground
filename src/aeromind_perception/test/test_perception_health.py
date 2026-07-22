import math

from aeromind_perception.perception_node import classify_signal


def test_signal_status_distinguishes_disabled_missing_stale_and_ok():
    assert classify_signal(None, 2.0, enabled=False) == "DISABLED"
    assert classify_signal(None, 2.0) == "MISSING"
    assert classify_signal(2.1, 2.0) == "STALE"
    assert classify_signal(0.2, 2.0) == "OK"


def test_signal_error_has_priority_over_age():
    assert classify_signal(0.1, 2.0, error="model failed") == "ERROR"


def test_missing_age_is_representable_as_nan_in_ros_float_field():
    assert math.isnan(float("nan"))
