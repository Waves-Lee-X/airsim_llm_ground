"""Dependency-free seventh-order minimum-snap trajectory primitives.

The polynomial satisfies position plus zero velocity, acceleration and jerk at
both ends. It is the exact single-segment minimum-snap boundary solution. A
future global optimizer can replace it for coupled waypoint constraints while
keeping the same sampled trajectory contract.
"""

from __future__ import annotations


def smootherstep7(s: float) -> tuple[float, float, float, float]:
    s = max(0.0, min(1.0, s))
    position = 35.0 * s**4 - 84.0 * s**5 + 70.0 * s**6 - 20.0 * s**7
    velocity = 140.0 * s**3 - 420.0 * s**4 + 420.0 * s**5 - 140.0 * s**6
    acceleration = 420.0 * s**2 - 1680.0 * s**3 + 2100.0 * s**4 - 840.0 * s**5
    jerk = 840.0 * s - 5040.0 * s**2 + 8400.0 * s**3 - 4200.0 * s**4
    return position, velocity, acceleration, jerk


def sample_minimum_snap(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    duration: float,
    count: int,
) -> list[dict]:
    duration = max(0.1, duration)
    count = max(2, count)
    delta = tuple(end[i] - start[i] for i in range(3))
    samples = []
    for index in range(count):
        elapsed = duration * index / (count - 1)
        normalized = elapsed / duration
        pos_gain, vel_gain, acc_gain, jerk_gain = smootherstep7(normalized)
        samples.append(
            {
                "t": elapsed,
                "position": tuple(start[i] + delta[i] * pos_gain for i in range(3)),
                "velocity": tuple(delta[i] * vel_gain / duration for i in range(3)),
                "acceleration": tuple(
                    delta[i] * acc_gain / (duration * duration) for i in range(3)
                ),
                "jerk": tuple(
                    delta[i] * jerk_gain / (duration**3) for i in range(3)
                ),
            }
        )
    return samples


def sample_quintic(start, end, duration, count):
    """Backward-compatible alias retained for older local callers."""
    return sample_minimum_snap(start, end, duration, count)
