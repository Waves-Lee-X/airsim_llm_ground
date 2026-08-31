"""Crash-safe state and append-only audit persistence."""

from __future__ import annotations

import json
import os
import tempfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import UUID

from pydantic import BaseModel

from aeromind_apm_lite.common.contracts import TwinStateSet

from .models import (
    ApprovalReceipt,
    ApprovalRequest,
    DispatchAuthorizationUse,
    LiveTrajectoryComparison,
    LiveMirrorStatus,
    MissionPlan,
    ObservedReplayRecord,
    canonical_hash,
    utc_now,
)


class BridgeStore:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._lock = threading.RLock()

    def save_plan(self, plan: MissionPlan) -> Path:
        path = self.root / "plans" / f"{plan.mission_id}.json"
        _atomic_json(path, plan.model_dump(mode="json"))
        return path

    def load_plan(self, mission_id: UUID | str) -> MissionPlan:
        path = self.root / "plans" / f"{mission_id}.json"
        return MissionPlan.model_validate(_read_json(path))

    def save_approval_request(self, request: ApprovalRequest) -> Path:
        path = self.root / "approval_requests" / f"{request.mission_id}.json"
        _atomic_json(path, request.model_dump(mode="json"))
        return path

    def load_approval_request(self, mission_id: UUID | str) -> ApprovalRequest:
        path = self.root / "approval_requests" / f"{mission_id}.json"
        return ApprovalRequest.model_validate(_read_json(path))

    def save_approval_receipt(self, receipt: ApprovalReceipt) -> Path:
        path = self.root / "approvals" / f"{receipt.mission_id}.json"
        _exclusive_json(path, receipt.model_dump(mode="json"))
        return path

    def load_approval_receipt(self, mission_id: UUID | str) -> ApprovalReceipt:
        path = self.root / "approvals" / f"{mission_id}.json"
        return ApprovalReceipt.model_validate(_read_json(path))

    def claim_dispatch(self, receipt: ApprovalReceipt) -> DispatchAuthorizationUse:
        claim = DispatchAuthorizationUse(
            request_id=receipt.request_id,
            mission_id=receipt.mission_id,
            mission_hash=receipt.mission_hash,
            predicted_evidence_hash=receipt.predicted_evidence_hash,
            operator_id=receipt.operator_id,
            receipt_hash=canonical_hash(receipt),
        )
        path = self.root / "dispatch_authorizations" / f"{receipt.mission_id}.json"
        _exclusive_json(path, claim.model_dump(mode="json"))
        return claim

    def load_dispatch_authorization(
        self,
        mission_id: UUID | str,
    ) -> DispatchAuthorizationUse | None:
        path = self.root / "dispatch_authorizations" / f"{mission_id}.json"
        if not path.is_file():
            return None
        return DispatchAuthorizationUse.model_validate(_read_json(path))

    def complete_dispatch_authorization(
        self,
        mission_id: UUID | str,
        *,
        status: str,
        detail: str,
    ) -> DispatchAuthorizationUse:
        if status not in {"dispatched", "failed"}:
            raise ValueError("dispatch authorization completion status is invalid")
        current = self.load_dispatch_authorization(mission_id)
        if current is None:
            raise ValueError("dispatch authorization has not been claimed")
        if current.status != "claimed":
            raise ValueError("dispatch authorization is already completed")
        completed = DispatchAuthorizationUse.model_validate(
            {
                **current.model_dump(mode="python"),
                "status": status,
                "completed_at_utc": utc_now(),
                "detail": detail,
            }
        )
        path = self.root / "dispatch_authorizations" / f"{mission_id}.json"
        _atomic_json(path, completed.model_dump(mode="json"))
        return completed

    def save_comparison(self, comparison: LiveTrajectoryComparison) -> Path:
        path = self.root / "comparisons" / f"{comparison.mission_id}.json"
        _atomic_json(path, comparison.model_dump(mode="json"))
        return path

    def save_latest(self, line_id: str, role: str, payload: BaseModel) -> Path:
        path = self.root / "twin" / line_id / f"{role}.json"
        _atomic_json(path, payload.model_dump(mode="json"))
        return path

    def save_state_set(self, line_id: str, value: TwinStateSet) -> Path:
        path = self.root / "twin" / line_id / "state-set.json"
        _atomic_json(path, value.model_dump(mode="json"))
        return path

    def load_state_set(self, line_id: str) -> TwinStateSet | None:
        path = self.root / "twin" / line_id / "state-set.json"
        if not path.is_file():
            return None
        return TwinStateSet.model_validate(_read_json(path))

    def save_mirror_status(self, status: LiveMirrorStatus) -> Path:
        path = self.root / "twin" / status.line.value / "real-status.json"
        _atomic_json(path, status.model_dump(mode="json"))
        return path

    def append_observed(self, record: ObservedReplayRecord) -> Path:
        path = self.root / "replay" / record.line.value / "observed.jsonl"
        encoded = json.dumps(
            record.model_dump(mode="json"),
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        with self._lock:
            previous = self._last_observed_unlocked(path)
            if previous is not None:
                if record.sequence != previous.sequence + 1:
                    raise ValueError("observed sequence must be contiguous")
                if record.monotonic_time_s <= previous.monotonic_time_s:
                    raise ValueError("observed monotonic arrival must be strictly increasing")
                if record.wall_time < previous.wall_time:
                    raise ValueError("observed wall_time must not move backwards")
            elif record.sequence != 1:
                raise ValueError("first observed sequence must be one")
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8", newline="\n") as stream:
                stream.write(encoded + "\n")
                stream.flush()
                os.fsync(stream.fileno())
        return path

    def read_observed(self, line_id: str) -> tuple[ObservedReplayRecord, ...]:
        path = self.root / "replay" / line_id / "observed.jsonl"
        if not path.is_file():
            return ()
        records: list[ObservedReplayRecord] = []
        try:
            for raw_line in path.read_text(encoding="utf-8").splitlines():
                if raw_line.strip():
                    records.append(ObservedReplayRecord.model_validate_json(raw_line))
        except (OSError, ValueError) as exc:
            raise ValueError(f"failed to read observed replay {path}: {exc}") from exc
        for previous, current in zip(records, records[1:]):
            if current.sequence != previous.sequence + 1:
                raise ValueError(f"observed replay has a sequence gap in {path}")
            if current.monotonic_time_s <= previous.monotonic_time_s:
                raise ValueError(f"observed replay monotonic time moved backwards in {path}")
            if current.wall_time < previous.wall_time:
                raise ValueError(f"observed replay wall time moved backwards in {path}")
        return tuple(records)

    def merged_observed(
        self,
        line_ids: tuple[str, ...] | list[str],
    ) -> tuple[ObservedReplayRecord, ...]:
        records = [
            record
            for line_id in line_ids
            for record in self.read_observed(line_id)
        ]
        return tuple(
            sorted(
                records,
                key=lambda item: (item.wall_time, item.line.value, item.sequence),
            )
        )

    def last_observed(self, line_id: str) -> ObservedReplayRecord | None:
        path = self.root / "replay" / line_id / "observed.jsonl"
        with self._lock:
            return self._last_observed_unlocked(path)

    @staticmethod
    def _last_observed_unlocked(path: Path) -> ObservedReplayRecord | None:
        if not path.is_file():
            return None
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
            last = next(line for line in reversed(lines) if line.strip())
            return ObservedReplayRecord.model_validate_json(last)
        except (OSError, ValueError, StopIteration) as exc:
            raise ValueError(f"failed to read last observed record {path}: {exc}") from exc

    def append_event(self, event_type: str, payload: Any) -> None:
        event = {
            "event_type": event_type,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "payload": _jsonable(payload),
        }
        path = self.root / "audit" / "events.jsonl"
        encoded = json.dumps(
            event,
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        with self._lock:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8", newline="\n") as stream:
                stream.write(encoded + "\n")
                stream.flush()
                os.fsync(stream.fileno())


def _jsonable(value: Any) -> Any:
    if isinstance(value, BaseModel):
        return value.model_dump(mode="json")
    if isinstance(value, UUID):
        return str(value)
    if isinstance(value, datetime):
        return value.astimezone(timezone.utc).isoformat()
    return value


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"failed to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"expected an object in {path}")
    return value


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(payload, temporary, ensure_ascii=True, indent=2, sort_keys=True)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def _exclusive_json(path: Path, payload: dict[str, Any]) -> None:
    """Create one immutable ledger entry without a check-then-write race."""

    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        indent=2,
        sort_keys=True,
    ) + "\n"
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as exc:
        raise ValueError(f"immutable ledger entry already exists: {path}") from exc
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())


__all__ = ["BridgeStore"]
