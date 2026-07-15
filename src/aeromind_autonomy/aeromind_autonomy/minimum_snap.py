"""Dependency-free seventh-order minimum-snap trajectory primitives.

The polynomial satisfies position plus zero velocity, acceleration and jerk at
both ends. It is the exact single-segment minimum-snap boundary solution. A
future global optimizer can replace it for coupled waypoint constraints while
keeping the same sampled trajectory contract.
"""

from __future__ import annotations

import math


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
    return sample_minimum_snap_boundary(
        start,
        end,
        duration,
        count,
        start_velocity=(0.0, 0.0, 0.0),
        end_velocity=(0.0, 0.0, 0.0),
    )


def sample_minimum_snap_boundary(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    duration: float,
    count: int,
    start_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    end_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    start_acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0),
    end_acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> list[dict]:
    """Sample a seventh-order segment with continuous boundary derivatives."""
    duration = max(0.1, duration)
    count = max(2, count)
    coefficients = [
        _boundary_coefficients(
            start[axis],
            end[axis],
            start_velocity[axis],
            end_velocity[axis],
            start_acceleration[axis],
            end_acceleration[axis],
            duration,
        )
        for axis in range(3)
    ]
    samples = []
    for index in range(count):
        elapsed = duration * index / (count - 1)
        s = elapsed / duration
        states = [_evaluate(coefficients[axis], s, duration) for axis in range(3)]
        samples.append(
            {
                "t": elapsed,
                "position": tuple(state[0] for state in states),
                "velocity": tuple(state[1] for state in states),
                "acceleration": tuple(state[2] for state in states),
                "jerk": tuple(state[3] for state in states),
            }
        )
    return samples


def sample_minimum_snap_waypoints(
    start: tuple[float, float, float],
    waypoints: list[tuple[float, float, float]],
    cruise_speed: float,
    sample_dt: float = 0.1,
    start_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[list[dict], list[float]]:
    """Create one time-continuous trajectory through all world waypoints."""
    if not waypoints:
        return [], []
    speed = max(0.2, float(cruise_speed))
    sample_dt = max(0.02, float(sample_dt))
    points = [tuple(float(value) for value in start)] + [
        tuple(float(value) for value in point) for point in waypoints
    ]
    distances = [
        math.dist(points[index], points[index + 1])
        for index in range(len(points) - 1)
    ]
    if any(distance < 1e-3 for distance in distances):
        raise ValueError("连续多航点中不能包含零长度航段")
    durations = [max(0.8, distance / speed) for distance in distances]
    velocities = [tuple(float(value) for value in start_velocity)]
    for index in range(1, len(points) - 1):
        total_duration = durations[index - 1] + durations[index]
        tangent = tuple(
            (points[index + 1][axis] - points[index - 1][axis]) / total_duration
            for axis in range(3)
        )
        magnitude = math.sqrt(sum(value * value for value in tangent))
        scale = min(1.0, speed / magnitude) if magnitude > 1e-9 else 0.0
        velocities.append(tuple(value * scale for value in tangent))
    velocities.append((0.0, 0.0, 0.0))

    samples: list[dict] = []
    waypoint_times: list[float] = []
    elapsed_offset = 0.0
    for index, duration in enumerate(durations):
        count = max(3, int(math.ceil(duration / sample_dt)) + 1)
        segment = sample_minimum_snap_boundary(
            points[index],
            points[index + 1],
            duration,
            count,
            start_velocity=velocities[index],
            end_velocity=velocities[index + 1],
        )
        if samples:
            segment = segment[1:]
        for sample in segment:
            sample["t"] += elapsed_offset
            samples.append(sample)
        elapsed_offset += duration
        waypoint_times.append(elapsed_offset)
    return samples, waypoint_times


def splice_replanned_trajectory(
    local_trajectory: list[dict],
    remaining_waypoints: list[tuple[float, float, float]],
    cruise_speed: float,
    sample_dt: float = 0.1,
    reached_tolerance: float = 0.15,
) -> tuple[list[dict], list[float]]:
    """Append remaining waypoints to an already safe local replan segment.

    The local segment is preserved exactly. Its final velocity becomes the
    boundary velocity of the regenerated tail, so the controller can switch
    time axes without introducing a stop or derivative discontinuity.
    Returned waypoint times correspond only to ``remaining_waypoints``.
    """
    if not local_trajectory:
        raise ValueError("局部重规划轨迹不能为空")
    local = [dict(sample) for sample in local_trajectory]
    local_duration = float(local[-1]["t"])
    local_end = tuple(float(value) for value in local[-1]["position"])
    end_velocity = tuple(float(value) for value in local[-1]["velocity"])
    remaining = [tuple(float(value) for value in point) for point in remaining_waypoints]
    if not remaining:
        return local, []

    reached_first = math.dist(local_end, remaining[0]) <= reached_tolerance
    tail_targets = remaining[1:] if reached_first else remaining
    waypoint_times = [local_duration] if reached_first else []
    if not tail_targets:
        return local, waypoint_times
    tail, tail_times = sample_minimum_snap_waypoints(
        local_end,
        tail_targets,
        cruise_speed,
        sample_dt=sample_dt,
        start_velocity=end_velocity,
    )
    for sample in tail[1:]:
        value = dict(sample)
        value["t"] = float(value["t"]) + local_duration
        local.append(value)
    waypoint_times.extend(local_duration + value for value in tail_times)
    return local, waypoint_times


def _boundary_coefficients(p0, p1, v0, v1, a0, a1, duration):
    c0 = p0
    c1 = v0 * duration
    c2 = 0.5 * a0 * duration**2
    c3 = 0.0
    position_residual = p1 - (c0 + c1 + c2 + c3)
    velocity_residual = v1 * duration - (c1 + 2.0 * c2 + 3.0 * c3)
    acceleration_residual = a1 * duration**2 - (2.0 * c2 + 6.0 * c3)
    jerk_residual = -6.0 * c3
    c4 = (
        35.0 * position_residual
        - 15.0 * velocity_residual
        + 2.5 * acceleration_residual
        - jerk_residual / 6.0
    )
    c5 = (
        -84.0 * position_residual
        + 39.0 * velocity_residual
        - 7.0 * acceleration_residual
        + 0.5 * jerk_residual
    )
    c6 = (
        70.0 * position_residual
        - 34.0 * velocity_residual
        + 6.5 * acceleration_residual
        - 0.5 * jerk_residual
    )
    c7 = (
        -20.0 * position_residual
        + 10.0 * velocity_residual
        - 2.0 * acceleration_residual
        + jerk_residual / 6.0
    )
    return (c0, c1, c2, c3, c4, c5, c6, c7)


def _evaluate(coefficients, s, duration):
    position = sum(value * s**power for power, value in enumerate(coefficients))
    velocity_s = sum(
        power * coefficients[power] * s ** (power - 1)
        for power in range(1, 8)
    )
    acceleration_s = sum(
        power * (power - 1) * coefficients[power] * s ** (power - 2)
        for power in range(2, 8)
    )
    jerk_s = sum(
        power * (power - 1) * (power - 2) * coefficients[power] * s ** (power - 3)
        for power in range(3, 8)
    )
    return (
        position,
        velocity_s / duration,
        acceleration_s / duration**2,
        jerk_s / duration**3,
    )


def sample_quintic(start, end, duration, count):
    """Backward-compatible alias retained for older local callers."""
    return sample_minimum_snap(start, end, duration, count)
