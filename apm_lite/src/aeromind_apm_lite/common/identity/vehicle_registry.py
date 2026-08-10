"""Thread-safe in-memory registry with deterministic JSON persistence."""

from __future__ import annotations

import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Any, Iterable

from pydantic import Field

from aeromind_apm_lite.common.contracts.models import StrictModel, Vector3
from aeromind_apm_lite.common.contracts.twin import (
    TwinState,
    VehicleProfile,
    distance_between,
)


class RegistryError(ValueError):
    pass


class RegistryMatch(StrictModel):
    profile: VehicleProfile
    state: TwinState | None = None
    distance_m: float | None = Field(default=None, ge=0.0)
    matched_capabilities: tuple[str, ...] = ()


class VehicleRegistry:
    """Own profiles and the latest provenance-preserving state per vehicle."""

    def __init__(self) -> None:
        self._profiles: dict[int, VehicleProfile] = {}
        self._states: dict[int, TwinState] = {}
        self._lock = threading.RLock()

    def register(
        self,
        profile: VehicleProfile,
        state: TwinState | None = None,
        *,
        replace: bool = False,
    ) -> None:
        with self._lock:
            if state is not None and state.vehicle_id != profile.vehicle_id:
                raise RegistryError("profile and state vehicle_id values do not match")
            existing = self._profiles.get(profile.vehicle_id)
            if existing is not None and existing != profile and not replace:
                raise RegistryError(f"vehicle {profile.vehicle_id} is already registered")
            self._profiles[profile.vehicle_id] = profile
            if state is not None:
                self._states[profile.vehicle_id] = state

    def update_state(self, state: TwinState) -> None:
        with self._lock:
            if state.vehicle_id not in self._profiles:
                raise RegistryError(f"vehicle {state.vehicle_id} is not registered")
            self._states[state.vehicle_id] = state

    def get_profile(self, vehicle_id: int) -> VehicleProfile:
        with self._lock:
            try:
                return self._profiles[int(vehicle_id)]
            except KeyError as exc:
                raise RegistryError(f"vehicle {vehicle_id} is not registered") from exc

    def get_state(self, vehicle_id: int) -> TwinState:
        with self._lock:
            try:
                return self._states[int(vehicle_id)]
            except KeyError as exc:
                raise RegistryError(f"vehicle {vehicle_id} has no state") from exc

    def query(
        self,
        *,
        required_capabilities: Iterable[str] = (),
        minimum_energy: float = 0.0,
        minimum_communication_quality: float = 0.0,
        origin_m: Vector3 | None = None,
        maximum_distance_m: float | None = None,
        maximum_whitelist_level: int | None = None,
    ) -> tuple[RegistryMatch, ...]:
        if not 0.0 <= minimum_energy <= 1.0:
            raise ValueError("minimum_energy must be in [0, 1]")
        if not 0.0 <= minimum_communication_quality <= 1.0:
            raise ValueError("minimum_communication_quality must be in [0, 1]")
        if maximum_distance_m is not None and maximum_distance_m < 0.0:
            raise ValueError("maximum_distance_m must be non-negative")
        if maximum_distance_m is not None and origin_m is None:
            raise ValueError("maximum_distance_m requires origin_m")
        if maximum_whitelist_level is not None and not 0 <= maximum_whitelist_level <= 10:
            raise ValueError("maximum_whitelist_level must be in [0, 10]")
        required = {str(item).strip().lower() for item in required_capabilities}

        with self._lock:
            matches: list[RegistryMatch] = []
            for vehicle_id in sorted(self._profiles):
                profile = self._profiles[vehicle_id]
                state = self._states.get(vehicle_id)
                available = set(profile.capabilities)
                if not required.issubset(available):
                    continue
                if maximum_whitelist_level is not None:
                    if profile.whitelist_level > maximum_whitelist_level:
                        continue
                if minimum_energy > 0.0:
                    if state is None or state.energy_remaining < minimum_energy:
                        continue
                if minimum_communication_quality > 0.0:
                    if (
                        state is None
                        or state.communication_quality < minimum_communication_quality
                    ):
                        continue
                distance = None
                if origin_m is not None:
                    if state is None:
                        continue
                    distance = distance_between(origin_m, state.position_m)
                    if maximum_distance_m is not None and distance > maximum_distance_m:
                        continue
                matches.append(
                    RegistryMatch(
                        profile=profile,
                        state=state,
                        distance_m=distance,
                        matched_capabilities=tuple(sorted(required)),
                    )
                )
        return tuple(
            sorted(
                matches,
                key=lambda item: (
                    item.distance_m if item.distance_m is not None else 0.0,
                    -(item.state.energy_remaining if item.state is not None else -1.0),
                    item.profile.vehicle_id,
                ),
            )
        )

    def public_payload(self) -> dict[str, Any]:
        with self._lock:
            return {
                "schema_version": "1.0",
                "profiles": [
                    self._profiles[vehicle_id].model_dump(mode="json")
                    for vehicle_id in sorted(self._profiles)
                ],
                "states": [
                    self._states[vehicle_id].model_dump(mode="json")
                    for vehicle_id in sorted(self._states)
                ],
            }

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=destination.parent,
                prefix=f".{destination.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                json.dump(
                    self.public_payload(),
                    temporary,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                temporary.write("\n")
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_path, destination)
        except OSError as exc:
            raise RegistryError(f"failed to save registry {destination}: {exc}") from exc
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()

    @classmethod
    def load(cls, path: str | Path) -> "VehicleRegistry":
        source = Path(path)
        try:
            payload = json.loads(source.read_text(encoding="utf-8"))
            if payload.get("schema_version") != "1.0":
                raise ValueError("unsupported registry schema_version")
            profiles = [VehicleProfile.model_validate(item) for item in payload["profiles"]]
            states = [TwinState.model_validate(item) for item in payload["states"]]
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise RegistryError(f"failed to load registry {source}: {exc}") from exc

        registry = cls()
        for profile in profiles:
            registry.register(profile)
        for state in states:
            registry.update_state(state)
        return registry


__all__ = ["RegistryError", "RegistryMatch", "VehicleRegistry"]
