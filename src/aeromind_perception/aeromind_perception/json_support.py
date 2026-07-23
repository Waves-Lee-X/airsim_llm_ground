"""Helpers for producing strict JSON from ROS and NumPy values."""

from __future__ import annotations

import math


def json_compatible(value):
    """Recursively convert ROS/NumPy containers into strict JSON values."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [json_compatible(item) for item in value]

    item = getattr(value, "item", None)
    if callable(item):
        try:
            converted = item()
            if converted is not value:
                return json_compatible(converted)
        except (TypeError, ValueError):
            pass

    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        try:
            converted = tolist()
            if converted is not value:
                return json_compatible(converted)
        except (TypeError, ValueError):
            pass

    return str(value)
