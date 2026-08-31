"""Fast deterministic event-level fleet model for broad strategy screening."""

from __future__ import annotations

import hashlib
import random
from datetime import timedelta
from typing import Any

from aeromind_apm_lite.common.contracts import Vector3
from aeromind_apm_lite.common.contracts.twin import HealthStatus

from .branch import BranchResult, BranchSpec, canonical_hash
from .metrics import metric_records, minimum_separation


class L0Model:
    version = "l0-event-v1"

    def run(self, branch: BranchSpec) -> BranchResult:
        settings = self._settings(branch)
        seed_material = f"{branch.strategy.random_seed}:{branch.branch_id}".encode("utf-8")
        seed = int.from_bytes(hashlib.sha256(seed_material).digest()[:8], "big")
        random_source = random.Random(seed)

        failed: set[int] = set()
        link_overrides: dict[int, float] = {}
        energy_penalties: dict[int, float] = {}
        target_count = branch.baseline.target_count
        failure_reasons: list[str] = []
        for event in branch.events:
            if event.event_type == "vehicle_failure":
                failed.update(event.affected_vehicle_ids)
                failure_reasons.append(
                    "vehicle_failure:" + ",".join(map(str, event.affected_vehicle_ids))
                )
            elif event.event_type == "link_degradation":
                quality = float(event.parameters.get("quality", 0.25))
                for vehicle_id in event.affected_vehicle_ids:
                    link_overrides[vehicle_id] = min(1.0, max(0.0, quality))
                failure_reasons.append(
                    "link_degradation:" + ",".join(map(str, event.affected_vehicle_ids))
                )
            elif event.event_type == "low_battery":
                penalty = float(event.parameters.get("energy_penalty", 0.25))
                for vehicle_id in event.affected_vehicle_ids:
                    energy_penalties[vehicle_id] = max(0.0, penalty)
                failure_reasons.append(
                    "low_battery:" + ",".join(map(str, event.affected_vehicle_ids))
                )
            elif event.event_type == "target_change":
                target_count = max(1, target_count + int(event.parameters.get("delta", 0)))
                failure_reasons.append("target_change")
            else:
                failure_reasons.append(f"unmodelled_event:{event.event_type}")

        duration = branch.baseline.duration_s
        timestamp = branch.baseline.captured_at_utc + timedelta(seconds=duration)
        final_states = []
        consumptions = []
        link_qualities = []
        progress = settings["cruise_speed_m_s"] * duration * 0.2
        for index, state in enumerate(branch.baseline.twins):
            jitter = random_source.uniform(-0.2, 0.2)
            consumption = (
                duration / 3_600.0
                * settings["energy_per_hour"]
                * random_source.uniform(0.95, 1.05)
            ) + energy_penalties.get(state.vehicle_id, 0.0)
            energy = max(0.0, state.energy_remaining - consumption)
            communication = link_overrides.get(
                state.vehicle_id,
                max(0.0, state.communication_quality - settings["communication_load"] * 0.05),
            )
            is_failed = state.vehicle_id in failed
            state_payload = state.model_dump(mode="python")
            state_payload.update(
                {
                    "position": Vector3(
                        x=state.position_m.x + (0.0 if is_failed else progress + jitter),
                        y=state.position_m.y + (index % 2) * jitter,
                        z=state.position_m.z,
                    ),
                    "position_m": Vector3(
                        x=state.position_m.x + (0.0 if is_failed else progress + jitter),
                        y=state.position_m.y + (index % 2) * jitter,
                        z=state.position_m.z,
                    ),
                    "energy_remaining": energy,
                    "communication_quality": communication,
                    "task_phase": "failed" if is_failed else "evaluated",
                    "mission_phase": "failed" if is_failed else "evaluated",
                    "health_status": HealthStatus.FAILED if is_failed else state.health_status,
                    "confidence": min(state.confidence, 0.7) if is_failed else state.confidence,
                    "source": "predicted",
                    "sampled_at_utc": timestamp,
                    "wall_time": timestamp,
                    "received_wall_time": timestamp,
                    "mirrored_wall_time": timestamp,
                    "sim_time": float(duration),
                    "simulated_at_utc": timestamp,
                    "mapped_at_utc": None,
                }
            )
            final_states.append(type(state).model_validate(state_payload))
            consumptions.append(max(0.0, state.energy_remaining - energy))
            link_qualities.append(communication)

        final_tuple = tuple(final_states)
        active_count = max(0, len(final_tuple) - len(failed))
        capacity = (
            active_count
            * duration
            * settings["productivity"]
            * (sum(link_qualities) / max(1, len(link_qualities)))
            / 75.0
        )
        completion = min(1.0, capacity / target_count)
        separation = minimum_separation(final_tuple)
        elapsed = duration * (1.08 - settings["productivity"] * 0.12)
        recovery = (
            1.0
            if not failed and not link_overrides
            else settings["recovery_factor"]
            * max(0.0, active_count / max(1, len(final_tuple)))
        )
        values = {
            "task_completion_rate": completion,
            "minimum_separation_m": separation,
            "elapsed_time_s": elapsed,
            "energy_consumption_ratio": sum(consumptions) / max(1, len(consumptions)),
            "communication_load_ratio": min(
                1.0,
                settings["communication_load"]
                * (1.0 + len(branch.events) / max(1, len(final_tuple))),
            ),
            "recovery_rate": recovery,
        }
        metrics = metric_records(branch.branch_id, values, timestamp)
        violations = []
        if separation < branch.baseline.minimum_separation_m:
            violations.append(
                f"minimum separation {separation:.3f} m below "
                f"{branch.baseline.minimum_separation_m:.3f} m"
            )
        low_energy = [
            state.vehicle_id
            for state in final_tuple
            if state.energy_remaining < branch.baseline.reserve_energy
        ]
        if low_energy:
            violations.append("reserve energy violated by " + ",".join(map(str, low_energy)))

        hash_payload: dict[str, Any] = {
            "branch_id": branch.branch_id,
            "baseline_hash": branch.baseline.baseline_hash,
            "final_states": [item.model_dump(mode="json") for item in final_tuple],
            "metrics": [item.model_dump(mode="json") for item in metrics],
            "violations": violations,
            "failure_reasons": failure_reasons,
        }
        return BranchResult(
            branch_id=branch.branch_id,
            strategy_id=branch.strategy.strategy_id,
            strategy_kind=branch.strategy.kind,
            baseline_hash=branch.baseline.baseline_hash,
            event_count=len(branch.events),
            final_states=final_tuple,
            metrics=metrics,
            hard_constraint_violations=tuple(violations),
            failure_reasons=tuple(failure_reasons),
            reproducibility_hash=canonical_hash(hash_payload),
        )

    @staticmethod
    def _settings(branch: BranchSpec) -> dict[str, float]:
        defaults = {
            "productivity": 0.8,
            "cruise_speed_m_s": 6.0,
            "energy_per_hour": 0.2,
            "communication_load": 0.5,
            "recovery_factor": 0.5,
        }
        values = {
            name: float(branch.strategy.parameters.get(name, default))
            for name, default in defaults.items()
        }
        for name in ("productivity", "energy_per_hour", "communication_load", "recovery_factor"):
            if not 0.0 <= values[name] <= 1.0:
                raise ValueError(f"strategy parameter {name} must be in [0, 1]")
        if not 0.0 < values["cruise_speed_m_s"] <= 100.0:
            raise ValueError("strategy parameter cruise_speed_m_s must be in (0, 100]")
        return values


__all__ = ["L0Model"]
