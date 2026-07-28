from uuid import uuid4

import pytest

from aeromind_apm_lite.common.mission import (
    MissionState,
    MissionStateMachine,
    MissionTransitionError,
)


def test_happy_path_has_one_monotonic_terminal_state():
    machine = MissionStateMachine(uuid4())

    assert machine.transition(MissionState.VALIDATED, reason="schema valid")
    assert machine.transition(MissionState.ACCEPTED, reason="vehicle accepted")
    assert machine.transition(MissionState.RUNNING, reason="telemetry confirms movement")
    assert machine.transition(MissionState.COMPLETED, reason="physical goal confirmed")

    assert machine.state == MissionState.COMPLETED
    assert machine.terminal
    assert len(machine.history) == 4


def test_duplicate_state_is_idempotent_and_terminal_cannot_change():
    machine = MissionStateMachine(uuid4())
    assert machine.transition(MissionState.VALIDATED, reason="valid")
    assert machine.transition(MissionState.VALIDATED, reason="duplicate") is None
    assert len(machine.history) == 1

    machine.transition(MissionState.FAILED, reason="preflight rejected")
    with pytest.raises(MissionTransitionError):
        machine.transition(MissionState.RUNNING, reason="late packet")


def test_state_machine_rejects_skipped_phases():
    machine = MissionStateMachine(uuid4())
    with pytest.raises(MissionTransitionError):
        machine.transition(MissionState.RUNNING, reason="invalid jump")
