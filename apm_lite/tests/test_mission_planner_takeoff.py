"""Unit tests for the mission planner takeoff retry loop."""

import asyncio
import time
from types import SimpleNamespace
from uuid import uuid4

import pytest

from aeromind_apm_lite.ground.browser import mission_planner as planner_module
from aeromind_apm_lite.ground.browser.mission_planner import MissionPlanner
from aeromind_apm_lite.onboard.apm_link import CommandHandle
from aeromind_apm_lite.onboard.mavlink.models import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    MavlinkAckEvidence,
    PhysicalCompletionEvidence,
)


class _SettledLink:
    """Fake link that reports ON_GROUND, zero altitude and armed state."""

    def __init__(self, armed: bool = True):
        self._armed = armed

    def telemetry_snapshot(self):
        return SimpleNamespace(
            landed_state=1,
            relative_altitude_m=0.0,
            armed=self._armed,
        )


class _FakeRuntime:
    def __init__(self, link):
        self._link = link

    def link_for(self, vehicle_id):
        return self._link


class _FakeTakeoffService:
    """Service whose takeoff attempts time out N times, then succeed."""

    def __init__(self, fail_attempts: int = 1):
        self.fail_attempts = fail_attempts
        self.calls = 0
        self.arm_calls = 0

    def arm(self, *, expires_monotonic_s=None):
        self.arm_calls += 1
        return _handle(FcuAction.ARM, complete=True)

    def takeoff(self, altitude_m, *, expires_monotonic_s=None):
        self.calls += 1
        if self.calls <= self.fail_attempts:
            return _handle(FcuAction.TAKEOFF, complete=False)
        return _handle(FcuAction.TAKEOFF, complete=True)


def _handle(action, *, complete: bool) -> CommandHandle:
    request_id = uuid4()
    application = ApplicationAcceptance(
        accepted=True, observed_monotonic_s=0.0, detail="ok"
    )
    ack = MavlinkAckEvidence(
        applicable=False,
        received=False,
        command_id=None,
        result=None,
        observed_monotonic_s=0.0,
        detail="",
    )
    physical = PhysicalCompletionEvidence(
        confirmed=True, observed_monotonic_s=0.0, detail="ok"
    )
    result: asyncio.Future[CommandResult] = (
        asyncio.get_event_loop().create_future()
    )
    if complete:
        result.set_result(
            CommandResult(
                request_id=request_id,
                action=action,
                status=CommandStatus.COMPLETED,
                application=application,
                sent_monotonic_s=0.0,
                mavlink_ack=ack,
                physical_completion=physical,
                detail="ok",
            )
        )
    return CommandHandle(
        request_id=request_id,
        action=action,
        application=application,
        _mavlink_ack=asyncio.get_event_loop().create_future(),
        _physical_completion=asyncio.get_event_loop().create_future(),
        _result=result,
    )


def test_takeoff_retries_after_timeout(monkeypatch):
    monkeypatch.setattr(
        planner_module, "_TAKEOFF_SETTLE_WAIT_S", 0.0
    )
    monkeypatch.setattr(
        planner_module, "_TAKEOFF_ATTEMPT_WINDOW_S", 0.1
    )
    monkeypatch.setattr(
        planner_module, "_TAKEOFF_RETRY_GAP_S", 0.0
    )

    service = _FakeTakeoffService(fail_attempts=1)
    planner = MissionPlanner(
        _FakeRuntime(_SettledLink()), clock=time.monotonic
    )

    async def scenario():
        await planner._takeoff_with_retry(1, service, 3.0)

    asyncio.run(scenario())

    assert service.calls == 2
    steps = [item["step"] for item in planner._results]
    assert "takeoff_retry" in steps
    completed = [
        item for item in planner._results if item["step"] == "takeoff"
    ]
    assert len(completed) == 1
    assert completed[0]["status"] == "completed"


def test_takeoff_fails_after_all_attempts(monkeypatch):
    monkeypatch.setattr(planner_module, "_TAKEOFF_SETTLE_WAIT_S", 0.0)
    monkeypatch.setattr(planner_module, "_TAKEOFF_ATTEMPT_WINDOW_S", 0.05)
    monkeypatch.setattr(planner_module, "_TAKEOFF_RETRY_GAP_S", 0.0)

    service = _FakeTakeoffService(fail_attempts=3)
    planner = MissionPlanner(
        _FakeRuntime(_SettledLink()), clock=time.monotonic
    )

    async def scenario():
        with pytest.raises(Exception) as excinfo:
            await planner._takeoff_with_retry(1, service, 3.0)
        assert "次尝试后失败" in str(excinfo.value)

    asyncio.run(scenario())

    assert service.calls == 3
    retries = [
        item
        for item in planner._results
        if item["step"] == "takeoff_retry"
    ]
    assert len(retries) == 2


def test_takeoff_rearms_when_disarmed(monkeypatch):
    monkeypatch.setattr(planner_module, "_TAKEOFF_SETTLE_WAIT_S", 0.0)
    monkeypatch.setattr(planner_module, "_TAKEOFF_ATTEMPT_WINDOW_S", 0.1)
    monkeypatch.setattr(planner_module, "_TAKEOFF_RETRY_GAP_S", 0.0)

    service = _FakeTakeoffService(fail_attempts=0)
    planner = MissionPlanner(
        _FakeRuntime(_SettledLink(armed=False)), clock=time.monotonic
    )

    async def scenario():
        await planner._takeoff_with_retry(1, service, 3.0)

    asyncio.run(scenario())

    assert service.arm_calls == 1
    assert service.calls == 1
    steps = [item["step"] for item in planner._results]
    assert "arm_retry" in steps
