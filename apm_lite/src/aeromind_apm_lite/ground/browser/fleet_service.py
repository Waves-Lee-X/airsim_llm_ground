"""Fleet orchestration service for the browser ground station.

The service is a planning layer: it turns vehicle telemetry snapshots into
per-vehicle formation directives, but never sends MAVLink itself. Commands
still require the existing command gate (simulation allowlist or the onboard
real-machine allowlist) before they can be executed.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

from aeromind_apm_lite.common.formation import (
    FORMATION_LABELS,
    FleetCoordinator,
    FleetDecision,
    FleetPlanError,
    FleetVehicleState,
    FormationType,
)

DEFAULT_FLEET_VEHICLES: tuple[int, ...] = (1, 2, 3, 4)


class FleetConfigError(ValueError):
    """Raised when a fleet configuration request is invalid."""


def _finite_tuple(value: Sequence[float], label: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise FleetConfigError(f"{label} must contain exactly three numbers")
    result = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in result):
        raise FleetConfigError(f"{label} values must be finite")
    if any(abs(item) > 1_000_000.0 for item in result):
        raise FleetConfigError(f"{label} values are out of range")
    return result


class FleetService:
    """Hold the current formation configuration and produce decisions."""

    def __init__(
        self,
        *,
        min_spacing_m: float = 3.0,
        coordinator: FleetCoordinator | None = None,
    ) -> None:
        self.coordinator = coordinator or FleetCoordinator(
            min_spacing_m=min_spacing_m
        )
        self.formation = FormationType.LINE
        self.spacing_m = min_spacing_m
        self.leader_target_map_m: tuple[float, float, float] = (0.0, 0.0, -3.0)
        self.transition_progress = 1.0
        self.vehicle_ids = DEFAULT_FLEET_VEHICLES
        self._last_decision: FleetDecision | None = None
        self._last_error: str | None = None

    def set_formation(
        self,
        formation: FormationType,
        *,
        leader_target_map_m: Sequence[float] | None = None,
        spacing_m: float | None = None,
        transition_progress: float | None = None,
        vehicle_ids: Sequence[int] | None = None,
    ) -> dict[str, Any]:
        """Apply a validated formation configuration and return its status."""
        self.formation = formation
        if leader_target_map_m is not None:
            self.leader_target_map_m = _finite_tuple(
                leader_target_map_m,
                "leader_target_map_m",
            )
        if spacing_m is not None:
            if not 0.5 <= float(spacing_m) <= 50.0:
                raise FleetConfigError("spacing_m must be in [0.5, 50] metres")
            self.spacing_m = float(spacing_m)
        if transition_progress is not None:
            progress = float(transition_progress)
            if not 0.0 <= progress <= 1.0:
                raise FleetConfigError("transition_progress must be in [0, 1]")
            self.transition_progress = progress
        if vehicle_ids is not None:
            ids = tuple(int(vehicle_id) for vehicle_id in vehicle_ids)
            if len(ids) < 2 or len(ids) > 8:
                raise FleetConfigError("fleet requires between 2 and 8 vehicles")
            if len(ids) != len(set(ids)):
                raise FleetConfigError("vehicle_ids must be unique")
            if any(not 1 <= vehicle_id <= 255 for vehicle_id in ids):
                raise FleetConfigError("vehicle_ids must be in [1, 255]")
            self.vehicle_ids = ids
        self._last_decision = None
        return self.status_payload()

    def status_payload(
        self,
        states: Mapping[int, FleetVehicleState] | None = None,
    ) -> dict[str, Any]:
        """Return the current configuration and the latest decision."""
        decision = None
        if states is not None:
            try:
                decision = self.coordinator.plan(
                    states,
                    self.formation,
                    self.leader_target_map_m,
                    spacing_m=self.spacing_m,
                    transition_progress=self.transition_progress,
                )
                self._last_decision = decision
            except FleetPlanError as exc:
                decision = None
                self._last_error = str(exc)
        return {
            "formation": self.formation.value,
            "formation_label": FORMATION_LABELS[self.formation],
            "spacing_m": self.spacing_m,
            "leader_target_map_m": list(self.leader_target_map_m),
            "transition_progress": self.transition_progress,
            "vehicle_ids": list(self.vehicle_ids),
            "phase": decision.phase if decision is not None else "idle",
            "reason": decision.reason if decision is not None else (
                getattr(self, "_last_error", "")
            ),
            "directives": [
                {
                    "vehicle_id": directive.vehicle_id,
                    "action": directive.action,
                    "slot": directive.slot,
                    "target_map_m": (
                        list(directive.target_map_m)
                        if directive.target_map_m is not None
                        else None
                    ),
                    "reason": directive.reason,
                }
                for directive in (decision.directives if decision is not None else ())
            ],
            "preview_only": True,
        }
