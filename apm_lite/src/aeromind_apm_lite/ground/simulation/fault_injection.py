"""Deterministic navigation health faults for SITL safety regression."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum

from aeromind_apm_lite.onboard.navigation_mission import NavigationObservation


class NavigationFaultType(str, Enum):
    GPS_LOSS = "gps_loss"
    HDOP_EXCEEDED = "hdop_exceeded"
    EKF_FAILURE = "ekf_failure"
    MISSION_EXPIRED = "mission_expired"
    GROUND_LINK_LOSS = "ground_link_loss"


@dataclass(frozen=True)
class NavigationFaultInjection:
    fault: NavigationFaultType
    trigger_phase: str = "goto"
    trigger_observation: int = 2
    injected_hdop: float = 99.0

    def __post_init__(self) -> None:
        if not self.trigger_phase.strip():
            raise ValueError("trigger_phase must not be empty")
        if self.trigger_observation < 1:
            raise ValueError("trigger_observation must be positive")
        if self.injected_hdop <= 0.0:
            raise ValueError("injected_hdop must be positive")


class NavigationFaultInjector:
    """Observation filter that activates one persistent synthetic fault."""

    def __init__(self, injection: NavigationFaultInjection) -> None:
        self.injection = injection
        self._phase_observations = 0
        self._active = False

    @property
    def active(self) -> bool:
        return self._active

    def __call__(
        self,
        phase: str,
        observation: NavigationObservation,
    ) -> NavigationObservation:
        if not self._active and phase == self.injection.trigger_phase:
            self._phase_observations += 1
            self._active = (
                self._phase_observations >= self.injection.trigger_observation
            )
        if not self._active:
            return observation

        fault = self.injection.fault
        if fault == NavigationFaultType.GROUND_LINK_LOSS:
            return replace(observation, ground_link_ok=False)
        if fault == NavigationFaultType.MISSION_EXPIRED:
            return replace(
                observation,
                mission_deadline_monotonic_s=observation.observed_monotonic_s,
            )

        snapshot = observation.snapshot
        if fault == NavigationFaultType.GPS_LOSS:
            snapshot = replace(
                snapshot,
                gps_fix_type=1,
                gps_hdop=None,
                gps_healthy=False,
            )
        elif fault == NavigationFaultType.HDOP_EXCEEDED:
            snapshot = replace(snapshot, gps_hdop=self.injection.injected_hdop)
        elif fault == NavigationFaultType.EKF_FAILURE:
            snapshot = replace(snapshot, ekf_flags=0)
        return replace(observation, snapshot=snapshot)
