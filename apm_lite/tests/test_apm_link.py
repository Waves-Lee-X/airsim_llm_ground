import asyncio
import time
from dataclasses import dataclass

import pytest

from aeromind_apm_lite.onboard import ApmLink, FcuStartupError, FlightCommandService
from aeromind_apm_lite.onboard.mavlink import (
    CommandStatus,
    FakeMavlinkTransport,
    FcuAction,
    FcuCommand,
    MavCommandId,
    MavLandedState,
    MavMessageId,
    MavResult,
    MavlinkEnvelope,
    SentCommandLong,
    SentHeartbeat,
    SentLocalPositionTarget,
)


def envelope(name, fields, *, system=1, component=1):
    return MavlinkEnvelope(
        name=name,
        fields=fields,
        source_system=system,
        source_component=component,
    )


def heartbeat(*, armed=False, mode="STABILIZE", system=1):
    return envelope(
        "HEARTBEAT",
        {
            "autopilot": 3,
            "type": 2,
            "base_mode": 128 if armed else 0,
            "custom_mode": 0,
            "mode_name": mode,
        },
        system=system,
    )


def autopilot_version():
    return envelope(
        "AUTOPILOT_VERSION",
        {
            "capabilities": 123,
            "flight_sw_version": (4 << 24) | (5 << 16) | (7 << 8) | 255,
            "board_version": 42,
            "flight_custom_version": [0xAA, 0xBB, 0xCC],
            "vendor_id": 17,
            "product_id": 23,
            "uid": 999,
        },
    )


async def started_link(fake, **overrides):
    fake.feed(heartbeat())
    fake.feed(autopilot_version())
    options = {
        "target_system": 1,
        "poll_interval_s": 0.005,
        "startup_timeout_s": 0.2,
        "ack_timeout_s": 0.05,
        "physical_timeout_s": 0.2,
        "telemetry_freshness_s": 0.15,
        "completion_hold_s": 0.0,
        "rejection_diagnostic_window_s": 0.01,
        "companion_heartbeat_hz": None,
        "telemetry_intervals_hz": (),
    }
    options.update(overrides)
    link = ApmLink(fake, **options)
    identity = await link.start()
    return link, identity


async def wait_until(predicate, *, timeout_s=0.5):
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("condition did not become true before timeout")
        await asyncio.sleep(0.005)


def test_discovery_ignores_other_system_and_records_firmware_identity():
    async def scenario():
        fake = FakeMavlinkTransport()
        fake.feed(heartbeat(system=9))
        fake.feed(heartbeat())
        fake.feed(autopilot_version())
        link = ApmLink(
            fake,
            target_system=1,
            poll_interval_s=0.005,
            startup_timeout_s=0.2,
        )

        identity = await link.start()

        assert identity.system_id == 1
        assert identity.flight_sw_version == "4.5.7-255"
        assert identity.custom_version_hex == "aabbcc"
        assert identity.capabilities == 123
        assert isinstance(fake.sent[0], SentCommandLong)
        assert fake.sent[0].command_id == MavCommandId.REQUEST_MESSAGE
        assert fake.sent[0].params[0] == 148
        assert link.telemetry_snapshot().fcu_link_ok
        await link.stop()
        assert fake.closed
        assert fake.owner_violations == 0

    asyncio.run(scenario())


def test_link_sends_companion_heartbeats_and_requests_required_streams():
    async def scenario():
        fake = FakeMavlinkTransport()
        fake.feed(heartbeat())
        fake.feed(autopilot_version())
        intervals = (
            (MavMessageId.LOCAL_POSITION_NED, 10.0),
            (MavMessageId.HOME_POSITION, 0.5),
        )
        link = ApmLink(
            fake,
            target_system=1,
            startup_timeout_s=0.2,
            poll_interval_s=0.005,
            companion_heartbeat_hz=5.0,
            telemetry_intervals_hz=intervals,
        )

        await link.start()

        assert isinstance(fake.sent[0], SentCommandLong)
        assert fake.sent[0].command_id == MavCommandId.REQUEST_MESSAGE
        requests = [
            item
            for item in fake.sent
            if isinstance(item, SentCommandLong)
            and item.command_id == MavCommandId.SET_MESSAGE_INTERVAL
        ]
        assert [int(item.params[0]) for item in requests] == [
            MavMessageId.LOCAL_POSITION_NED,
            MavMessageId.HOME_POSITION,
        ]
        assert any(isinstance(item, SentHeartbeat) for item in fake.sent)
        assert fake.owner_violations == 0
        await link.stop()

    asyncio.run(scenario())


def test_discovery_requires_autopilot_version():
    async def scenario():
        fake = FakeMavlinkTransport()
        fake.feed(heartbeat())
        link = ApmLink(
            fake,
            target_system=1,
            poll_interval_s=0.005,
            startup_timeout_s=0.03,
        )
        with pytest.raises(FcuStartupError, match="AUTOPILOT_VERSION"):
            await link.start()
        await link.stop()

    asyncio.run(scenario())


def test_arm_exposes_application_ack_and_physical_evidence_separately():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        service = FlightCommandService(link)

        handle = service.arm()
        assert handle.application.accepted
        assert not handle.mavlink_ack.done()
        sent = await fake.wait_for_sent(2)
        assert sent == SentCommandLong(
            command_id=MavCommandId.COMPONENT_ARM_DISARM,
            params=(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.COMPONENT_ARM_DISARM, "result": MavResult.ACCEPTED},
            )
        )
        ack = await handle.mavlink_ack
        assert ack.received and ack.result == MavResult.ACCEPTED
        assert not handle.physical_completion.done()

        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        physical = await handle.physical_completion
        result = await handle.result
        assert physical.confirmed
        assert result.status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_mavlink_rejection_never_becomes_physical_completion():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        handle = FlightCommandService(link).takeoff(2.0)
        await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.NAV_TAKEOFF, "result": MavResult.DENIED},
            )
        )

        result = await handle.result

        assert result.status == CommandStatus.MAVLINK_REJECTED
        assert result.mavlink_ack.received
        assert not result.physical_completion.confirmed
        await link.stop()

    asyncio.run(scenario())


def test_rejection_collects_new_statustext_without_becoming_completion():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            rejection_diagnostic_window_s=0.04,
        )
        handle = FlightCommandService(link).arm()
        await fake.wait_for_sent(2)

        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.FAILED,
                },
            )
        )
        ack = await handle.mavlink_ack
        assert ack.result == MavResult.FAILED
        assert not handle.result.done()

        fake.feed(envelope("STATUSTEXT", {"text": "Arm: Safety Switch"}))
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        result = await handle.result

        assert result.status == CommandStatus.MAVLINK_REJECTED
        assert not result.physical_completion.confirmed
        assert "Arm: Safety Switch" in result.detail
        await link.stop()

    asyncio.run(scenario())


def test_in_progress_ack_waits_for_a_terminal_ack():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        handle = FlightCommandService(link).arm()
        await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.COMPONENT_ARM_DISARM, "result": MavResult.IN_PROGRESS},
            )
        )
        await asyncio.sleep(0.01)
        assert not handle.mavlink_ack.done()
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.COMPONENT_ARM_DISARM, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        assert (await handle.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_missing_ack_can_be_reconciled_by_fresh_physical_telemetry():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, ack_timeout_s=0.03)
        handle = FlightCommandService(link).arm()
        await fake.wait_for_sent(2)
        fake.feed(heartbeat(armed=True, mode="GUIDED"))

        result = await handle.result

        assert result.status == CommandStatus.COMPLETED
        assert result.mavlink_ack.applicable
        assert not result.mavlink_ack.received
        assert result.physical_completion.confirmed
        await link.stop()

    asyncio.run(scenario())


def test_ack_success_without_state_change_times_out_physically():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, physical_timeout_s=0.06)
        handle = FlightCommandService(link).arm()
        await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.COMPONENT_ARM_DISARM, "result": MavResult.ACCEPTED},
            )
        )

        result = await handle.result

        assert result.status == CommandStatus.PHYSICAL_TIMEOUT
        assert result.mavlink_ack.received
        assert not result.physical_completion.confirmed
        await link.stop()

    asyncio.run(scenario())


def test_heartbeat_loss_fails_pending_action_and_requires_new_link():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            heartbeat_timeout_s=0.04,
            physical_timeout_s=0.2,
        )
        handle = FlightCommandService(link).arm()
        await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.COMPONENT_ARM_DISARM, "result": MavResult.ACCEPTED},
            )
        )

        result = await handle.result

        assert result.status == CommandStatus.FCU_LINK_LOST
        assert not link.ready
        rejected = FlightCommandService(link).hold()
        assert not rejected.application.accepted
        await link.stop()

    asyncio.run(scenario())


def test_guided_local_target_repeats_setpoints_and_has_no_command_ack():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, setpoint_rate_hz=50.0)
        service = FlightCommandService(link)
        handle = service.goto_local_ned(3.0, 1.0, -2.0)

        first = await fake.wait_for_sent(2)
        second = await fake.wait_for_sent(3)
        assert isinstance(first, SentLocalPositionTarget)
        assert isinstance(second, SentLocalPositionTarget)
        assert first == second
        ack = await handle.mavlink_ack
        assert not ack.applicable and not ack.received

        fake.feed(
            envelope(
                "LOCAL_POSITION_NED",
                {"x": 3.1, "y": 1.0, "z": -2.0, "vx": 0.1, "vy": 0.0, "vz": 0.0},
                )
            )
        await asyncio.sleep(0.02)
        assert not handle.result.done()
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        result = await handle.result
        assert result.status == CommandStatus.COMPLETED
        assert result.physical_completion.confirmed
        await link.stop()

    asyncio.run(scenario())


def test_hold_preempts_active_setpoint_and_waits_for_stable_loiter():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, setpoint_rate_hz=50.0)
        service = FlightCommandService(link)
        goto = service.goto_local_ned(20.0, 0.0, -2.0)
        await fake.wait_for_sent(2)

        hold = service.hold()
        goto_result = await goto.result
        assert goto_result.status == CommandStatus.PREEMPTED
        sent_hold = await fake.wait_for_sent(3)
        assert isinstance(sent_hold, SentCommandLong)
        assert sent_hold.command_id == MavCommandId.DO_SET_MODE

        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.DO_SET_MODE, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=True, mode="LOITER"))
        fake.feed(
            envelope(
                "LOCAL_POSITION_NED",
                {"x": 1.0, "y": 0.0, "z": -2.0, "vx": 0.0, "vy": 0.0, "vz": 0.0},
            )
        )
        hold_result = await hold.result
        assert hold_result.status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_land_requires_on_ground_and_disarmed_telemetry():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        handle = FlightCommandService(link).land()
        await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.NAV_LAND, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=False, mode="LAND"))
        assert not handle.physical_completion.done()
        fake.feed(
            envelope(
                "EXTENDED_SYS_STATE",
                {"landed_state": MavLandedState.ON_GROUND},
            )
        )

        result = await handle.result

        assert result.status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_rtl_requires_home_proximity_on_ground_and_disarmed():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        handle = FlightCommandService(link).rtl()
        await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.NAV_RETURN_TO_LAUNCH, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(
            envelope(
                "HOME_POSITION",
                {
                    "latitude": 34_0000000,
                    "longitude": 113_0000000,
                    "altitude": 100000,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                },
            )
        )
        fake.feed(
            envelope(
                "GLOBAL_POSITION_INT",
                {
                    "lat": 34_0000001,
                    "lon": 113_0000001,
                    "alt": 100000,
                    "relative_alt": 0,
                    "vx": 0,
                    "vy": 0,
                    "vz": 0,
                },
            )
        )
        fake.feed(
            envelope(
                "GPS_RAW_INT",
                {"fix_type": 3, "satellites_visible": 12},
            )
        )
        fake.feed(heartbeat(armed=False, mode="RTL"))
        fake.feed(
            envelope(
                "EXTENDED_SYS_STATE",
                {"landed_state": MavLandedState.ON_GROUND},
            )
        )
        assert (await handle.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_arm_and_takeoff_sequence_stops_only_after_each_physical_step():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        service = FlightCommandService(link)

        async def emulate_autopilot():
            set_mode = await fake.wait_for_sent(2)
            assert isinstance(set_mode, SentCommandLong)
            assert set_mode.command_id == MavCommandId.DO_SET_MODE
            fake.feed(
                envelope(
                    "COMMAND_ACK",
                    {"command": MavCommandId.DO_SET_MODE, "result": MavResult.ACCEPTED},
                )
            )
            fake.feed(heartbeat(armed=False, mode="GUIDED"))

            arm = await fake.wait_for_sent(3)
            assert isinstance(arm, SentCommandLong)
            assert arm.command_id == MavCommandId.COMPONENT_ARM_DISARM
            fake.feed(
                envelope(
                    "COMMAND_ACK",
                    {
                        "command": MavCommandId.COMPONENT_ARM_DISARM,
                        "result": MavResult.ACCEPTED,
                    },
                )
            )
            fake.feed(heartbeat(armed=True, mode="GUIDED"))

            takeoff = await fake.wait_for_sent(4)
            assert isinstance(takeoff, SentCommandLong)
            assert takeoff.command_id == MavCommandId.NAV_TAKEOFF
            fake.feed(
                envelope(
                    "COMMAND_ACK",
                    {"command": MavCommandId.NAV_TAKEOFF, "result": MavResult.ACCEPTED},
                )
            )
            fake.feed(heartbeat(armed=True, mode="GUIDED"))
            fake.feed(
                envelope(
                    "GLOBAL_POSITION_INT",
                    {
                        "lat": 34_0000000,
                        "lon": 113_0000000,
                        "alt": 102000,
                        "relative_alt": 2000,
                        "vx": 0,
                        "vy": 0,
                        "vz": 0,
                    },
                )
            )

        sequence, _ = await asyncio.gather(service.arm_and_takeoff(2.0), emulate_autopilot())
        assert sequence.completed
        assert [step.action for step in sequence.steps] == [
            FcuAction.SET_MODE,
            FcuAction.ARM,
            FcuAction.TAKEOFF,
        ]
        await link.stop()

    asyncio.run(scenario())


def test_bounded_queue_rejects_excess_normal_commands():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, queue_size=1, setpoint_rate_hz=50.0)
        service = FlightCommandService(link)
        active = service.goto_local_ned(20.0, 0.0, -2.0)
        await fake.wait_for_sent(2)
        queued = service.arm()
        rejected = service.takeoff(2.0)

        rejected_result = await rejected.result
        assert not rejected.application.accepted
        assert rejected_result.status == CommandStatus.APPLICATION_REJECTED

        safety = service.hold()
        assert (await active.result).status == CommandStatus.PREEMPTED
        assert (await queued.result).status == CommandStatus.PREEMPTED
        await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.DO_SET_MODE, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=True, mode="LOITER"))
        fake.feed(
            envelope(
                "LOCAL_POSITION_NED",
                {"x": 1.0, "y": 0.0, "z": -2.0, "vx": 0.0, "vy": 0.0, "vz": 0.0},
            )
        )
        assert (await safety.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_expired_command_is_never_sent():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        handle = link.submit(
            FcuCommand(FcuAction.ARM),
            expires_monotonic_s=0.0,
        )

        result = await handle.result

        assert not handle.application.accepted
        assert result.status == CommandStatus.EXPIRED_BEFORE_SEND
        assert len(fake.sent) == 1
        await link.stop()

    asyncio.run(scenario())


def test_expired_guided_target_stops_stream_and_queues_internal_hold():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            setpoint_rate_hz=50.0,
            trajectory_timeout_s=0.04,
        )
        handle = FlightCommandService(link).goto_local_ned(
            10.0,
            0.0,
            -2.0,
        )
        await fake.wait_for_sent(2)

        result = await handle.result

        assert result.status == CommandStatus.EXPIRED_DURING_EXECUTION
        internal_hold = link.last_internal_safety_handle
        assert internal_hold is not None
        sent_hold = await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)
        assert sent_hold.command_id == MavCommandId.DO_SET_MODE
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.DO_SET_MODE, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=True, mode="LOITER"))
        fake.feed(
            envelope(
                "LOCAL_POSITION_NED",
                {"x": 1.0, "y": 0.0, "z": -2.0, "vx": 0.0, "vy": 0.0, "vz": 0.0},
            )
        )
        assert (await internal_hold.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_cancelling_a_waiter_does_not_cancel_the_fcu_action():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        handle = FlightCommandService(link).arm()
        await fake.wait_for_sent(2)

        async def wait_for_result():
            return await handle.result

        waiter = asyncio.create_task(wait_for_result())
        await asyncio.sleep(0)
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter

        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.COMPONENT_ARM_DISARM, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        result = await handle.result
        assert result.status == CommandStatus.COMPLETED
        assert link.ready
        await link.stop()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "method_name,command_id",
    [
        ("disarm", MavCommandId.COMPONENT_ARM_DISARM),
        ("land", MavCommandId.NAV_LAND),
        ("rtl", MavCommandId.NAV_RETURN_TO_LAUNCH),
    ],
)
def test_terminal_safety_commands_preempt_and_duplicate_hold_is_merged(
    method_name,
    command_id,
):
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        service = FlightCommandService(link)

        hold = service.hold()
        duplicate = service.hold()
        assert duplicate is hold
        await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)

        terminal = getattr(service, method_name)()

        assert (await hold.result).status == CommandStatus.PREEMPTED
        sent = await fake.wait_for_command_long(command_id)
        assert sent.command_id == command_id
        assert terminal.application.accepted
        await link.stop()

    asyncio.run(scenario())


@pytest.mark.parametrize("method_name", ["land", "rtl"])
def test_terminal_safety_timeout_latches_until_explicit_reset(method_name):
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, physical_timeout_s=0.06)
        service = FlightCommandService(link)
        handle = getattr(service, method_name)()
        sent = await fake.wait_for_sent(2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": sent.command_id, "result": MavResult.ACCEPTED},
            )
        )

        result = await handle.result

        assert result.status == CommandStatus.PHYSICAL_TIMEOUT
        assert link.safety_latch is not None
        assert link.safety_latch.action == handle.action
        rejected = service.arm()
        assert not rejected.application.accepted
        assert (await rejected.result).status == CommandStatus.APPLICATION_REJECTED
        with pytest.raises(ValueError, match="operator_confirmed"):
            link.reset_safety_latch()
        link.reset_safety_latch(operator_confirmed=True)
        assert link.safety_latch is None
        assert service.arm().application.accepted
        await link.stop()

    asyncio.run(scenario())


def test_land_timeout_latch_clears_on_late_fresh_physical_evidence():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, physical_timeout_s=0.06)
        handle = FlightCommandService(link).land()
        await fake.wait_for_command_long(MavCommandId.NAV_LAND)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.NAV_LAND, "result": MavResult.ACCEPTED},
            )
        )
        assert (await handle.result).status == CommandStatus.PHYSICAL_TIMEOUT
        assert link.safety_latch is not None

        fake.feed(heartbeat(armed=False, mode="LAND"))
        fake.feed(
            envelope(
                "EXTENDED_SYS_STATE",
                {"landed_state": MavLandedState.ON_GROUND},
            )
        )
        await wait_until(lambda: link.safety_latch is None)
        assert FlightCommandService(link).arm().application.accepted
        await link.stop()

    asyncio.run(scenario())


def test_unknown_mode_fails_only_that_command_and_link_remains_usable():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        service = FlightCommandService(link)

        invalid = await service.set_mode("NOT_A_MODE").result

        assert invalid.status == CommandStatus.DISPATCH_FAILED
        assert link.ready
        arm = service.arm()
        await fake.wait_for_command_long(MavCommandId.COMPONENT_ARM_DISARM)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.ACCEPTED,
                },
            )
        )
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        assert (await arm.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


class FailOneArmSendTransport(FakeMavlinkTransport):
    def __init__(self):
        super().__init__()
        self._fail_arm = True

    async def send_command_long(self, command_id, params):
        if command_id == MavCommandId.COMPONENT_ARM_DISARM and self._fail_arm:
            self._fail_arm = False
            raise OSError("injected single-command send failure")
        await super().send_command_long(command_id, params)


def test_single_command_send_failure_does_not_close_link():
    async def scenario():
        fake = FailOneArmSendTransport()
        link, _ = await started_link(fake)
        service = FlightCommandService(link)

        failed = await service.arm().result

        assert failed.status == CommandStatus.DISPATCH_FAILED
        assert link.ready
        mode = service.set_mode("GUIDED")
        await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {"command": MavCommandId.DO_SET_MODE, "result": MavResult.ACCEPTED},
            )
        )
        fake.feed(heartbeat(armed=False, mode="GUIDED"))
        assert (await mode.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


class FailOneSetpointSendTransport(FakeMavlinkTransport):
    def __init__(self):
        super().__init__()
        self._fail_setpoint = True

    async def send_local_position_target(
        self,
        north_m,
        east_m,
        down_m,
        yaw_rad,
    ):
        if self._fail_setpoint:
            self._fail_setpoint = False
            raise OSError("injected setpoint send failure")
        await super().send_local_position_target(north_m, east_m, down_m, yaw_rad)


def test_guided_setpoint_send_failure_queues_hold_without_closing_link():
    async def scenario():
        fake = FailOneSetpointSendTransport()
        link, _ = await started_link(fake)
        goto = FlightCommandService(link).goto_local_ned(2.0, 0.0, -1.0)

        result = await goto.result

        assert result.status == CommandStatus.DISPATCH_FAILED
        assert link.ready
        hold = link.last_internal_safety_handle
        assert hold is not None
        await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)
        await link.stop()

    asyncio.run(scenario())


def test_cancelling_arm_takeoff_sequence_queues_land_compensation():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake)
        service = FlightCommandService(link)
        sequence = asyncio.create_task(service.arm_and_takeoff(2.0))
        await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)

        sequence.cancel()
        with pytest.raises(asyncio.CancelledError):
            await sequence

        compensation = service.last_compensation_handle
        assert compensation is not None
        assert compensation.action == FcuAction.LAND
        await fake.wait_for_command_long(MavCommandId.NAV_LAND)
        assert link.ready
        await link.stop()

    asyncio.run(scenario())


def test_cancelling_start_closes_transport_and_runner():
    async def scenario():
        fake = FakeMavlinkTransport()
        fake.feed(heartbeat())
        link = ApmLink(
            fake,
            target_system=1,
            startup_timeout_s=1.0,
            poll_interval_s=0.005,
        )
        starting = asyncio.create_task(link.start())
        await fake.wait_for_command_long(MavCommandId.REQUEST_MESSAGE)

        starting.cancel()
        with pytest.raises(asyncio.CancelledError):
            await starting

        assert fake.closed
        assert not link.ready
        assert link._runner is not None and link._runner.done()

    asyncio.run(scenario())


def test_ack_target_fields_must_match_companion_identity_when_present():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            source_system=201,
            source_component=191,
        )
        arm = FlightCommandService(link).arm()
        await fake.wait_for_command_long(MavCommandId.COMPONENT_ARM_DISARM)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.ACCEPTED,
                    "target_system": 202,
                    "target_component": 191,
                },
            )
        )
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        await asyncio.sleep(0.01)
        assert not arm.mavlink_ack.done()

        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.ACCEPTED,
                    "target_system": 201,
                    "target_component": 191,
                },
            )
        )
        assert (await arm.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_reused_command_id_isolated_before_send_and_real_rejection_honored():
    async def scenario():
        clock = ManualClock()
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            clock=clock,
            ack_timeout_s=0.04,
            rejection_diagnostic_window_s=0.0,
        )
        service = FlightCommandService(link)
        arm = service.arm()
        await fake.wait_for_command_long(MavCommandId.COMPONENT_ARM_DISARM)
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        clock.value = 100.041
        arm_result = await arm.result
        assert arm_result.status == CommandStatus.COMPLETED
        assert not arm_result.mavlink_ack.received

        retry = service.arm()
        await wait_until(
            lambda: link._pending is not None
            and link._pending.queued.handle is retry
        )
        assert link._pending is not None
        assert link._pending.sent_monotonic_s is None

        def arm_sends():
            return sum(
                isinstance(item, SentCommandLong)
                and item.command_id == MavCommandId.COMPONENT_ARM_DISARM
                for item in fake.sent
            )

        assert arm_sends() == 1

        clock.value = 100.05
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.DENIED,
                },
            )
        )
        await wait_until(
            lambda: link._pending is not None
            and link._pending.dispatch_not_before_s == pytest.approx(100.09)
        )
        assert not retry.mavlink_ack.done()

        clock.value = 100.089
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        await wait_until(
            lambda: link._telemetry.received_at.get("heartbeat")
            == pytest.approx(100.089)
        )
        assert arm_sends() == 1

        clock.value = 100.091
        await wait_until(lambda: arm_sends() == 2)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.DENIED,
                },
            )
        )

        result = await asyncio.wait_for(retry.result, timeout=0.2)

        assert result.status == CommandStatus.MAVLINK_REJECTED
        assert result.sent_monotonic_s == pytest.approx(100.091)
        assert result.mavlink_ack.received
        assert result.mavlink_ack.result == MavResult.DENIED
        assert fake.owner_violations == 0
        await link.stop()

    asyncio.run(scenario())


def test_acknowledged_same_id_retry_honors_immediate_real_rejection():
    async def scenario():
        clock = ManualClock()
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            clock=clock,
            ack_timeout_s=0.04,
            rejection_diagnostic_window_s=0.0,
        )
        service = FlightCommandService(link)

        first = service.arm()
        await fake.wait_for_command_long(MavCommandId.COMPONENT_ARM_DISARM)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.ACCEPTED,
                },
            )
        )
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        assert (await first.result).status == CommandStatus.COMPLETED

        retry = service.arm()
        await wait_until(
            lambda: sum(
                isinstance(item, SentCommandLong)
                and item.command_id == MavCommandId.COMPONENT_ARM_DISARM
                for item in fake.sent
            )
            == 2
        )
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.DENIED,
                },
            )
        )

        result = await asyncio.wait_for(retry.result, timeout=0.2)

        assert result.status == CommandStatus.MAVLINK_REJECTED
        assert result.sent_monotonic_s == pytest.approx(100.0)
        assert result.mavlink_ack.result == MavResult.DENIED
        assert fake.owner_violations == 0
        await link.stop()

    asyncio.run(scenario())


def test_safety_preempts_command_waiting_for_same_id_isolation():
    async def scenario():
        clock = ManualClock()
        fake = FakeMavlinkTransport()
        link, _ = await started_link(fake, clock=clock, ack_timeout_s=0.04)
        service = FlightCommandService(link)

        first = service.set_mode("GUIDED")
        await fake.wait_for_command_long(MavCommandId.DO_SET_MODE)
        fake.feed(heartbeat(mode="GUIDED"))
        clock.value = 100.041
        first_result = await first.result
        assert first_result.status == CommandStatus.COMPLETED
        assert not first_result.mavlink_ack.received

        retry = service.set_mode("GUIDED")
        await wait_until(
            lambda: link._pending is not None
            and link._pending.queued.handle is retry
        )
        assert link._pending is not None
        assert link._pending.sent_monotonic_s is None

        hold = service.hold()
        retry_result = await retry.result

        assert retry_result.status == CommandStatus.PREEMPTED
        assert retry_result.sent_monotonic_s is None
        assert not retry_result.mavlink_ack.applicable

        await wait_until(
            lambda: sum(
                isinstance(item, SentCommandLong)
                and item.command_id == MavCommandId.DO_SET_MODE
                for item in fake.sent
            )
            == 2
        )
        assert link._pending is not None
        assert link._pending.queued.handle is hold
        assert link._pending.sent_monotonic_s == pytest.approx(100.041)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.DO_SET_MODE,
                    "result": MavResult.ACCEPTED,
                },
            )
        )
        fake.feed(heartbeat(mode="LOITER"))
        fake.feed(
            envelope(
                "LOCAL_POSITION_NED",
                {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0},
            )
        )
        assert (await hold.result).status == CommandStatus.COMPLETED
        assert fake.owner_violations == 0
        await link.stop()

    asyncio.run(scenario())


def test_rtl_rejects_stale_home_position_and_gps_evidence():
    async def scenario():
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            telemetry_freshness_s=0.03,
            physical_timeout_s=0.25,
        )
        rtl = FlightCommandService(link).rtl()
        await fake.wait_for_command_long(MavCommandId.NAV_RETURN_TO_LAUNCH)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.NAV_RETURN_TO_LAUNCH,
                    "result": MavResult.ACCEPTED,
                },
            )
        )

        def feed_navigation_evidence():
            fake.feed(
                envelope(
                    "HOME_POSITION",
                    {
                        "latitude": 34_0000000,
                        "longitude": 113_0000000,
                        "altitude": 100000,
                    },
                )
            )
            fake.feed(
                envelope(
                    "GLOBAL_POSITION_INT",
                    {
                        "lat": 34_0000001,
                        "lon": 113_0000001,
                        "alt": 100000,
                        "relative_alt": 0,
                        "vx": 0,
                        "vy": 0,
                        "vz": 0,
                    },
                )
            )
            fake.feed(
                envelope(
                    "GPS_RAW_INT",
                    {"fix_type": 3, "satellites_visible": 12},
                )
            )

        feed_navigation_evidence()
        await asyncio.sleep(0.05)
        fake.feed(heartbeat(armed=False, mode="RTL"))
        fake.feed(
            envelope(
                "EXTENDED_SYS_STATE",
                {"landed_state": MavLandedState.ON_GROUND},
            )
        )
        await asyncio.sleep(0.01)
        assert not rtl.result.done()

        feed_navigation_evidence()
        assert (await rtl.result).status == CommandStatus.COMPLETED
        await link.stop()

    asyncio.run(scenario())


def test_fcu_command_rejects_ambiguous_or_non_finite_targets():
    with pytest.raises(ValueError):
        FcuCommand(FcuAction.TAKEOFF)
    with pytest.raises(ValueError):
        FcuCommand(FcuAction.GOTO_LOCAL_NED, target_position_ned_m=(1.0, 2.0))
    with pytest.raises(ValueError):
        FcuCommand(
            FcuAction.GOTO_LOCAL_NED,
            target_position_ned_m=(1.0, float("nan"), -2.0),
        )


@dataclass
class ManualClock:
    value: float = 100.0

    def __call__(self):
        return self.value


def test_completion_hold_restarts_after_physical_condition_is_interrupted():
    async def scenario():
        clock = ManualClock()
        fake = FakeMavlinkTransport()
        link, _ = await started_link(
            fake,
            clock=clock,
            completion_hold_s=0.5,
            ack_timeout_s=1.0,
            physical_timeout_s=5.0,
            heartbeat_timeout_s=5.0,
            telemetry_freshness_s=5.0,
        )
        handle = FlightCommandService(link).arm()
        await fake.wait_for_command_long(MavCommandId.COMPONENT_ARM_DISARM)
        fake.feed(
            envelope(
                "COMMAND_ACK",
                {
                    "command": MavCommandId.COMPONENT_ARM_DISARM,
                    "result": MavResult.ACCEPTED,
                },
            )
        )
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        await wait_until(
            lambda: link._pending is not None
            and link._pending.physical_candidate_since_s == pytest.approx(100.0)
        )

        clock.value = 100.3
        fake.feed(heartbeat(armed=False, mode="GUIDED"))
        await wait_until(
            lambda: link._pending is not None
            and link._pending.physical_candidate_since_s is None
        )

        clock.value = 100.6
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        await wait_until(
            lambda: link._pending is not None
            and link._pending.physical_candidate_since_s == pytest.approx(100.6)
        )
        assert not handle.result.done()

        clock.value = 101.09
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        await wait_until(
            lambda: link._telemetry.received_at.get("heartbeat")
            == pytest.approx(101.09)
        )
        assert not handle.result.done()

        clock.value = 101.11
        fake.feed(heartbeat(armed=True, mode="GUIDED"))
        result = await asyncio.wait_for(handle.result, timeout=0.2)

        assert result.status == CommandStatus.COMPLETED
        assert result.physical_completion.confirmed
        await link.stop()

    asyncio.run(scenario())


def test_telemetry_fields_keep_independent_ages():
    async def scenario():
        clock = ManualClock()
        fake = FakeMavlinkTransport()
        fake.feed(heartbeat())
        fake.feed(autopilot_version())
        link = ApmLink(fake, target_system=1, clock=clock, startup_timeout_s=0.2)
        await link.start()
        clock.value += 0.5
        sensor_mask = (1 << 5) | (1 << 28)
        fake.feed(
            envelope(
                "SYS_STATUS",
                {
                    "battery_remaining": 80,
                    "voltage_battery": 16000,
                    "onboard_control_sensors_present": sensor_mask,
                    "onboard_control_sensors_enabled": sensor_mask,
                    "onboard_control_sensors_health": 1 << 28,
                },
            )
        )
        await asyncio.sleep(0.03)
        snapshot = link.telemetry_snapshot()
        assert snapshot.field_ages_s["heartbeat"] == pytest.approx(0.5)
        assert snapshot.field_ages_s["battery"] == pytest.approx(0.0)
        assert snapshot.prearm_ok is True
        assert snapshot.gps_healthy is False
        assert snapshot.field_ages_s["gps_health"] == pytest.approx(0.0)
        await link.stop()

    asyncio.run(scenario())
