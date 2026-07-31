"""Fleet planning: slot assignment, separation checks and loss handling."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .formations import (
    Offset,
    validate_spacing,
)

Position = tuple[float, float, float]
Trajectory = Sequence[tuple[float, float, float, float]]  # (t, x, y, z)


class FleetPlanError(ValueError):
    """Raised when a fleet plan violates a safety constraint."""


@dataclass(frozen=True)
class FleetDirective:
    """One per-vehicle decision produced by the coordinator."""

    vehicle_id: int
    action: str  # "goto" | "hold" | "land" | "rtl"
    target_map_m: Position | None = None
    reason: str = ""
    slot: int | None = None


@dataclass(frozen=True)
class FleetVehicleState:
    """Minimal vehicle context needed for formation decisions."""

    vehicle_id: int
    ready: bool = False
    link_ok: bool = False
    last_seen_age_s: float | None = None
    position_map_m: Position | None = None


@dataclass(frozen=True)
class FleetDecision:
    directives: tuple[FleetDirective, ...]
    phase: str  # syncing | transit | formation_hold | degraded | aborted
    reason: str = ""


def assign_slots(
    vehicle_ids: Sequence[int],
    positions: Mapping[int, Position],
    offsets: tuple[Offset, ...],
) -> dict[int, int]:
    """Deterministically assign vehicles to slots by nearest position.

    Uses a greedy nearest-pair loop with stable tie-breaking so the same
    inputs always produce the same assignment.
    """
    if len(vehicle_ids) != len(offsets):
        raise FleetPlanError(
            "vehicle count must match the number of formation slots"
        )
    remaining_vehicles = sorted(int(vehicle_id) for vehicle_id in vehicle_ids)
    remaining_slots = list(range(len(offsets)))
    assignment: dict[int, int] = {}
    while remaining_vehicles:
        best: tuple[Any, int, int] | None = None
        for vehicle_id in remaining_vehicles:
            position = positions.get(vehicle_id)
            origin = position if position is not None else (0.0, 0.0, 0.0)
            for slot in remaining_slots:
                offset_x, offset_y = offsets[slot]
                distance_sq = (
                    (origin[0] - offset_x) ** 2 + (origin[1] - offset_y) ** 2
                )
                key = (distance_sq, vehicle_id, slot)
                if best is None or key < best[0]:
                    best = (key, vehicle_id, slot)
        if best is None:
            raise FleetPlanError("slot assignment failed")
        _, vehicle_id, slot = best
        assignment[vehicle_id] = slot
        remaining_vehicles.remove(vehicle_id)
        remaining_slots.remove(slot)
    return assignment


def _interpolate(trajectory: Trajectory, time_s: float) -> Position:
    if not trajectory:
        raise FleetPlanError("trajectory must contain at least one sample")
    first = trajectory[0]
    last = trajectory[-1]
    if time_s <= first[0]:
        return (first[1], first[2], first[3])
    if time_s >= last[0]:
        return (last[1], last[2], last[3])
    for index in range(len(trajectory) - 1):
        t0, x0, y0, z0 = trajectory[index]
        t1, x1, y1, z1 = trajectory[index + 1]
        if t0 <= time_s <= t1:
            span = t1 - t0
            fraction = (time_s - t0) / span if span > 0.0 else 0.0
            return (
                round(x0 + (x1 - x0) * fraction, 6),
                round(y0 + (y1 - y0) * fraction, 6),
                round(z0 + (z1 - z0) * fraction, 6),
            )
    raise FleetPlanError("trajectory interpolation failed")


def _segment_min_distance(
    start_a: tuple[float, float, float, float],
    end_a: tuple[float, float, float, float],
    start_b: tuple[float, float, float, float],
    end_b: tuple[float, float, float, float],
) -> float:
    """Minimum 3D distance between two linear segments over their time overlap."""
    t0 = max(start_a[0], start_b[0])
    t1 = min(end_a[0], end_b[0])
    if t1 < t0:
        return math.inf

    def velocity(start, end) -> tuple[float, float, float]:
        span = end[0] - start[0]
        if span <= 0.0:
            return (0.0, 0.0, 0.0)
        return tuple((end[k] - start[k]) / span for k in (1, 2, 3))

    velocity_a = velocity(start_a, end_a)
    velocity_b = velocity(start_b, end_b)
    offset = tuple(
        (start_a[k] - start_b[k]) for k in (1, 2, 3)
    )
    relative_velocity = tuple(
        velocity_a[k] - velocity_b[k] for k in range(3)
    )
    quadratic = sum(value * value for value in relative_velocity)
    linear = 2.0 * sum(
        offset[k] * relative_velocity[k] for k in range(3)
    )
    constant = sum(value * value for value in offset)
    candidates = [t0, t1]
    if quadratic > 0.0:
        vertex = -linear / (2.0 * quadratic)
        if t0 <= vertex <= t1:
            candidates.append(vertex)
    squared = min(
        quadratic * t * t + linear * t + constant for t in candidates
    )
    return math.sqrt(max(0.0, squared))


def trajectory_separation_report(
    trajectories: Mapping[int, Trajectory],
) -> dict[str, Any]:
    """Return the minimum pairwise distance over the whole time horizon."""
    vehicle_ids = sorted(int(vehicle_id) for vehicle_id in trajectories)
    if len(vehicle_ids) < 2:
        return {"min_distance_m": math.inf, "closest_pair": None}
    min_distance = math.inf
    closest_pair: tuple[int, int, float] | None = None
    for index in range(len(vehicle_ids)):
        for other in range(index + 1, len(vehicle_ids)):
            first_id = vehicle_ids[index]
            second_id = vehicle_ids[other]
            first_trajectory = trajectories[first_id]
            second_trajectory = trajectories[second_id]
            for first_index in range(len(first_trajectory) - 1):
                start_a = first_trajectory[first_index]
                end_a = first_trajectory[first_index + 1]
                for second_index in range(len(second_trajectory) - 1):
                    start_b = second_trajectory[second_index]
                    end_b = second_trajectory[second_index + 1]
                    distance = _segment_min_distance(
                        start_a,
                        end_a,
                        start_b,
                        end_b,
                    )
                    if distance < min_distance:
                        min_distance = distance
                        at_time = max(start_a[0], start_b[0])
                        closest_pair = (first_id, second_id, at_time)
    return {
        "min_distance_m": round(min_distance, 6),
        "closest_pair": closest_pair,
    }


def validate_trajectory_separation(
    trajectories: Mapping[int, Trajectory],
    min_distance_m: float,
    tolerance_m: float = 1e-6,
) -> dict[str, Any]:
    """Raise FleetPlanError when trajectories come closer than allowed."""
    if min_distance_m <= 0.0:
        raise ValueError("min_distance_m must be positive")
    report = trajectory_separation_report(trajectories)
    if report["min_distance_m"] < min_distance_m - tolerance_m:
        pair = report["closest_pair"]
        raise FleetPlanError(
            f"trajectory separation {report['min_distance_m']:.3f} m "
            f"< required {min_distance_m} m near pair {pair}"
        )
    return report


class FleetCoordinator:
    """Pure fleet decision logic; emits data directives, never MAVLink."""

    def __init__(
        self,
        *,
        min_spacing_m: float = 3.0,
        sync_timeout_s: float = 15.0,
        lost_timeout_s: float = 5.0,
        degrade_on_lost: bool = True,
    ) -> None:
        if min_spacing_m <= 0.0:
            raise ValueError("min_spacing_m must be positive")
        if sync_timeout_s <= 0.0 or lost_timeout_s <= 0.0:
            raise ValueError("timeouts must be positive")
        self.min_spacing_m = min_spacing_m
        self.sync_timeout_s = sync_timeout_s
        self.lost_timeout_s = lost_timeout_s
        self.degrade_on_lost = degrade_on_lost

    def plan(
        self,
        states: Mapping[int, FleetVehicleState],
        formation: Any,
        leader_target_map_m: Position,
        spacing_m: float | None = None,
        transition_progress: float = 1.0,
    ) -> FleetDecision:
        """Produce per-vehicle directives for the requested formation."""
        if not states:
            raise FleetPlanError("fleet must contain at least one vehicle")
        spacing = spacing_m if spacing_m is not None else self.min_spacing_m
        vehicle_ids = sorted(int(vehicle_id) for vehicle_id in states)
        state_by_id = {int(vehicle_id): states[vehicle_id] for vehicle_id in vehicle_ids}

        # "Lost" means the vehicle was seen before and then exceeded the lost
        # timeout. Vehicles that never connected are simply not ready yet and
        # keep the fleet in the syncing phase instead of aborting it.
        lost = [
            vehicle_id
            for vehicle_id in vehicle_ids
            if not state_by_id[vehicle_id].link_ok
            and state_by_id[vehicle_id].last_seen_age_s is not None
            and state_by_id[vehicle_id].last_seen_age_s > self.lost_timeout_s
        ]
        if lost:
            if self.degrade_on_lost and len(vehicle_ids) - len(lost) >= 2:
                survivors = [
                    vehicle_id
                    for vehicle_id in vehicle_ids
                    if vehicle_id not in lost
                ]
                return self._plan_formation(
                    survivors,
                    state_by_id,
                    formation,
                    leader_target_map_m,
                    spacing,
                    transition_progress,
                    phase="degraded",
                    reason=f"lost vehicles {sorted(lost)}; formation degraded",
                )
            return FleetDecision(
                directives=tuple(
                    FleetDirective(
                        vehicle_id=vehicle_id,
                        action="hold",
                        reason=f"fleet aborted after lost vehicles {sorted(lost)}",
                    )
                    for vehicle_id in vehicle_ids
                ),
                phase="aborted",
                reason=f"lost vehicles {sorted(lost)}",
            )

        stale = [
            vehicle_id
            for vehicle_id in vehicle_ids
            if state_by_id[vehicle_id].last_seen_age_s is not None
            and state_by_id[vehicle_id].last_seen_age_s > self.sync_timeout_s
        ]
        if stale:
            return FleetDecision(
                directives=tuple(
                    FleetDirective(
                        vehicle_id=vehicle_id,
                        action="hold",
                        reason="stale vehicle data; waiting for sync",
                    )
                    for vehicle_id in vehicle_ids
                ),
                phase="syncing",
                reason=f"stale vehicles {stale}",
            )

        not_ready = [
            vehicle_id
            for vehicle_id in vehicle_ids
            if not state_by_id[vehicle_id].ready
            or not state_by_id[vehicle_id].link_ok
            or state_by_id[vehicle_id].position_map_m is None
        ]
        if not_ready:
            return FleetDecision(
                directives=tuple(
                    FleetDirective(
                        vehicle_id=vehicle_id,
                        action="hold",
                        reason="awaiting fleet readiness",
                    )
                    for vehicle_id in vehicle_ids
                ),
                phase="syncing",
                reason=f"not ready {not_ready}",
            )

        return self._plan_formation(
            vehicle_ids,
            state_by_id,
            formation,
            leader_target_map_m,
            spacing,
            transition_progress,
            phase="transit" if transition_progress < 1.0 else "formation_hold",
            reason="formation slots assigned",
        )

    def _plan_formation(
        self,
        vehicle_ids: list[int],
        state_by_id: Mapping[int, FleetVehicleState],
        formation: Any,
        leader_target_map_m: Position,
        spacing_m: float,
        transition_progress: float,
        *,
        phase: str,
        reason: str,
    ) -> FleetDecision:
        from .formations import transition_offsets

        offsets = transition_offsets(
            formation,
            formation,
            len(vehicle_ids),
            spacing_m,
            transition_progress,
        )
        validate_spacing(offsets, self.min_spacing_m)
        positions = {
            vehicle_id: state_by_id[vehicle_id].position_map_m
            or (0.0, 0.0, 0.0)
            for vehicle_id in vehicle_ids
        }
        assignment = assign_slots(vehicle_ids, positions, offsets)
        directives = []
        for vehicle_id in vehicle_ids:
            slot = assignment[vehicle_id]
            offset_x, offset_y = offsets[slot]
            target = (
                round(leader_target_map_m[0] + offset_x, 6),
                round(leader_target_map_m[1] + offset_y, 6),
                round(leader_target_map_m[2], 6),
            )
            directives.append(
                FleetDirective(
                    vehicle_id=vehicle_id,
                    action="goto",
                    target_map_m=target,
                    reason=(
                        f"{phase}: slot {slot} "
                        f"{formation.value if hasattr(formation, 'value') else formation}"
                    ),
                    slot=slot,
                )
            )
        return FleetDecision(
            directives=tuple(directives),
            phase=phase,
            reason=reason,
        )
