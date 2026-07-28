import asyncio
import time
from dataclasses import dataclass
from uuid import UUID, uuid4

import pytest
import websockets
from websockets.exceptions import ConnectionClosed

from aeromind_apm_lite.common.communication import (
    BoundedLatestQueue,
    ClientHello,
    ClientProof,
    ServerChallenge,
    SessionAccepted,
    authentication_proof,
    decode_handshake_frame,
    encode_frame,
    encode_signed_message,
    verify_authentication_proof,
)
from aeromind_apm_lite.common.contracts import (
    AckStatus,
    Heartbeat,
    VehicleCommandType,
)
from aeromind_apm_lite.ground.server import GroundServer
from aeromind_apm_lite.onboard.agent import OnboardAgent
from aeromind_apm_lite.onboard.apm_link import CommandHandle
from aeromind_apm_lite.onboard.mavlink.models import (
    ApplicationAcceptance,
    CommandResult,
    CommandStatus,
    FcuAction,
    FcuCommand,
    MavCommandId,
    MavlinkAckEvidence,
    MavResult,
    PhysicalCompletionEvidence,
    TelemetrySnapshot,
)

SECRET_1 = b"vehicle-one-independent-secret-32-bytes"
SECRET_2 = b"vehicle-two-independent-secret-32-bytes"


def run(coroutine):
    return asyncio.run(coroutine)


async def wait_until(predicate, timeout_s=2.0):
    deadline = asyncio.get_running_loop().time() + timeout_s
    while not predicate():
        if asyncio.get_running_loop().time() >= deadline:
            raise TimeoutError("condition did not become true")
        await asyncio.sleep(0.01)


@dataclass
class StubApmLink:
    physical_confirmed: bool = True
    block_first_result: bool = False
    initial_mode: str = "GUIDED"
    initial_armed: bool = True
    initial_altitude_m: float = 2.0
    fail_action: FcuAction | None = None

    guided_mode = "GUIDED"

    def __post_init__(self):
        self.submitted = []
        self._blocked_result = None
        self._mode = self.initial_mode
        self._armed = self.initial_armed
        self._altitude_m = self.initial_altitude_m

    def telemetry_snapshot(self):
        now = time.monotonic()
        return TelemetrySnapshot(
            observed_monotonic_s=now,
            fcu_link_ok=True,
            armed=self._armed,
            mode=self._mode,
            relative_altitude_m=self._altitude_m,
            local_position_ned_m=(1.0, 2.0, -self._altitude_m),
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            global_position_deg_m=None,
            attitude_rpy_rad=None,
            battery_remaining=0.75,
            battery_voltage_v=15.2,
            gps_fix_type=3,
            satellites_visible=14,
            gps_healthy=True,
            prearm_ok=True,
            ekf_flags=None,
            landed_state=None,
            home_position_deg_m=None,
            home_position_ned_m=None,
            last_status_text=None,
            last_heartbeat_monotonic_s=now,
            field_ages_s={"heartbeat": 0.0, "local_position": 0.0},
        )

    def submit(self, command, *, expires_monotonic_s=None):
        self.submitted.append((command, expires_monotonic_s))
        loop = asyncio.get_running_loop()
        request_id = uuid4()
        application = ApplicationAcceptance(True, time.monotonic(), "FCU queue accepted")
        failed = command.action == self.fail_action
        command_id = {
            FcuAction.ARM: MavCommandId.COMPONENT_ARM_DISARM,
            FcuAction.DISARM: MavCommandId.COMPONENT_ARM_DISARM,
            FcuAction.SET_MODE: MavCommandId.DO_SET_MODE,
            FcuAction.TAKEOFF: MavCommandId.NAV_TAKEOFF,
            FcuAction.HOLD: MavCommandId.DO_SET_MODE,
            FcuAction.LAND: MavCommandId.NAV_LAND,
            FcuAction.RTL: MavCommandId.NAV_RETURN_TO_LAUNCH,
        }[command.action]
        mavlink_ack = MavlinkAckEvidence(
            applicable=True,
            received=True,
            command_id=int(command_id),
            result=int(MavResult.DENIED if failed else MavResult.ACCEPTED),
            observed_monotonic_s=time.monotonic(),
            detail=f"{command.action.value} MAVLink {'denied' if failed else 'accepted'}",
        )
        physical_confirmed = self.physical_confirmed and not failed
        physical = PhysicalCompletionEvidence(
            confirmed=physical_confirmed,
            observed_monotonic_s=(time.monotonic() if physical_confirmed else None),
            detail=f"{command.action.value} physical state observed",
        )
        result = CommandResult(
            request_id=request_id,
            action=command.action,
            status=(
                CommandStatus.MAVLINK_REJECTED
                if failed
                else CommandStatus.COMPLETED
            ),
            application=application,
            sent_monotonic_s=time.monotonic(),
            mavlink_ack=mavlink_ack,
            physical_completion=physical,
            detail=f"{command.action.value} command result",
        )
        if not failed and physical_confirmed:
            if command.action == FcuAction.SET_MODE:
                self._mode = command.mode or self._mode
            elif command.action == FcuAction.ARM:
                self._armed = True
            elif command.action == FcuAction.DISARM:
                self._armed = False
            elif command.action == FcuAction.TAKEOFF:
                self._altitude_m = command.target_altitude_m or self._altitude_m
        ack_future = loop.create_future()
        ack_future.set_result(mavlink_ack)
        physical_future = loop.create_future()
        physical_future.set_result(physical)
        result_future = loop.create_future()
        if self.block_first_result and len(self.submitted) == 1:
            self._blocked_result = (result_future, result)
        else:
            result_future.set_result(result)
        return CommandHandle(
            request_id=request_id,
            action=command.action,
            application=application,
            _mavlink_ack=ack_future,
            _physical_completion=physical_future,
            _result=result_future,
        )

    def finish_blocked_result(self):
        assert self._blocked_result is not None
        future, result = self._blocked_result
        future.set_result(result)
        self._blocked_result = None


async def authenticate_raw_client(uri: str, session_id: UUID):
    websocket = await websockets.connect(uri)
    client_challenge = "c" * 40
    await websocket.send(
        encode_frame(
            ClientHello(
                vehicle_id=1,
                session_id=session_id,
                client_challenge=client_challenge,
            )
        )
    )
    challenge = decode_handshake_frame(await websocket.recv())
    assert isinstance(challenge, ServerChallenge)
    assert verify_authentication_proof(
        challenge.server_proof,
        role="server",
        challenge=client_challenge,
        vehicle_id=1,
        session_id=session_id,
        secret=SECRET_1,
    )
    await websocket.send(
        encode_frame(
            ClientProof(
                vehicle_id=1,
                session_id=session_id,
                client_proof=authentication_proof(
                    role="client",
                    challenge=challenge.server_challenge,
                    vehicle_id=1,
                    session_id=session_id,
                    secret=SECRET_1,
                ),
            )
        )
    )
    accepted = decode_handshake_frame(await websocket.recv())
    assert isinstance(accepted, SessionAccepted)
    return websocket


def test_vehicle_credentials_must_be_long_and_independent():
    with pytest.raises(ValueError, match="at least 32"):
        GroundServer(vehicle_secrets={1: b"short"})
    with pytest.raises(ValueError, match="independent"):
        GroundServer(vehicle_secrets={1: SECRET_1, 2: SECRET_1})
    GroundServer(vehicle_secrets={1: SECRET_1, 2: SECRET_2})


def test_bounded_queue_evicts_old_telemetry_but_preserves_control_messages():
    async def scenario():
        queue = BoundedLatestQueue(2)
        assert await queue.put("ack", replaceable=False)
        assert await queue.put("telemetry-old", replaceable=True)
        assert await queue.put("telemetry-new", replaceable=True)
        assert queue.dropped_replaceable == 1
        assert await queue.get() == "ack"
        assert await queue.get() == "telemetry-new"

        assert await queue.put("control-1", replaceable=False)
        assert await queue.put("control-2", replaceable=False)
        assert not await queue.put("telemetry-dropped", replaceable=True)
        assert queue.qsize == 2
        await queue.close()

    run(scenario())


def test_disconnected_session_cannot_be_authenticated_again():
    async def scenario():
        server = GroundServer(vehicle_secrets={1: SECRET_1})
        await server.start()
        session_id = uuid4()
        try:
            websocket = await authenticate_raw_client(server.uri, session_id)
            await websocket.close()
            await wait_until(lambda: server.session_info(1) is None)

            replay = await websockets.connect(server.uri)
            await replay.send(
                encode_frame(
                    ClientHello(
                        vehicle_id=1,
                        session_id=session_id,
                        client_challenge="r" * 40,
                    )
                )
            )
            with pytest.raises(ConnectionClosed):
                await replay.recv()
            assert replay.close_code == 1008
        finally:
            await server.stop()

    run(scenario())


def test_real_loopback_websocket_maps_commands_and_reports_three_stage_evidence():
    async def scenario():
        link = StubApmLink()
        server = GroundServer(
            vehicle_secrets={1: SECRET_1},
            calibration_ids={1: "venue-v1"},
            queue_size=8,
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
            queue_size=8,
        )
        try:
            session_id = await agent.start()
            assert await server.wait_for_vehicle(1) == session_id
            command = await server.send_command(
                1,
                VehicleCommandType.TAKEOFF,
                target_altitude_m=2.5,
                ttl_ms=1_000,
            )
            accepted = await server.wait_for_ack(
                1, command.message_id, AckStatus.ACCEPTED
            )
            running = await server.wait_for_ack(
                1, command.message_id, AckStatus.RUNNING
            )
            completed = await server.wait_for_ack(
                1, command.message_id, AckStatus.COMPLETED
            )

            assert not accepted.physical_completion_confirmed
            assert not running.physical_completion_confirmed
            assert completed.physical_completion_confirmed
            assert completed.apm_command_result == 0
            submitted, expires_at = link.submitted[0]
            assert submitted == FcuCommand(FcuAction.TAKEOFF, target_altitude_m=2.5)
            assert expires_at is not None and expires_at > time.monotonic()

            remaining_commands = (
                (VehicleCommandType.ARM, FcuAction.ARM, AckStatus.COMPLETED),
                (VehicleCommandType.DISARM, FcuAction.DISARM, AckStatus.COMPLETED),
                (VehicleCommandType.HOLD, FcuAction.HOLD, AckStatus.COMPLETED),
                (VehicleCommandType.LAND, FcuAction.LAND, AckStatus.COMPLETED),
                (VehicleCommandType.RTL, FcuAction.RTL, AckStatus.COMPLETED),
                (VehicleCommandType.CANCEL, FcuAction.HOLD, AckStatus.CANCELLED),
            )
            for command_type, expected_action, terminal_status in remaining_commands:
                mapped = await server.send_command(1, command_type)
                await server.wait_for_ack(1, mapped.message_id, AckStatus.RUNNING)
                await server.wait_for_ack(1, mapped.message_id, terminal_status)
                assert link.submitted[-1][0].action == expected_action

            concurrent = await asyncio.gather(
                *(server.send_command(1, VehicleCommandType.ARM) for _ in range(12))
            )
            await asyncio.gather(
                *(
                    server.wait_for_ack(
                        1, message.message_id, AckStatus.COMPLETED
                    )
                    for message in concurrent
                )
            )
            assert [message.sequence for message in concurrent] == sorted(
                message.sequence for message in concurrent
            )

            await wait_until(lambda: server.last_heartbeat(1) is not None)
            await wait_until(lambda: server.latest_telemetry(1) is not None)
            telemetry = server.latest_telemetry(1)
            assert telemetry is not None
            assert telemetry.position_m is not None
            assert telemetry.position_m.z == -2.0
            assert telemetry.health.fcu_link_ok
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_takeoff_sequence_enters_guided_arms_and_takes_off_over_websocket():
    async def scenario():
        link = StubApmLink(
            initial_mode="STABILIZE",
            initial_armed=False,
            initial_altitude_m=0.0,
        )
        server = GroundServer(
            vehicle_secrets={1: SECRET_1}, calibration_ids={1: "venue-v1"}
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
        )
        try:
            await agent.start()
            await server.wait_for_vehicle(1)
            command = await server.send_command(
                1,
                VehicleCommandType.TAKEOFF,
                target_altitude_m=3.0,
                ttl_ms=5_000,
            )
            running = await server.wait_for_ack(
                1, command.message_id, AckStatus.RUNNING
            )
            completed = await server.wait_for_ack(
                1, command.message_id, AckStatus.COMPLETED
            )

            assert [item[0] for item in link.submitted] == [
                FcuCommand(FcuAction.SET_MODE, mode="GUIDED"),
                FcuCommand(FcuAction.ARM),
                FcuCommand(FcuAction.TAKEOFF, target_altitude_m=3.0),
            ]
            assert "takeoff MAVLink accepted" in running.detail
            assert completed.apm_command_result == int(MavResult.ACCEPTED)
            assert completed.physical_completion_confirmed
            assert "takeoff command result" in completed.detail
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_takeoff_sequence_skips_guided_and_arm_when_already_satisfied():
    async def scenario():
        link = StubApmLink(initial_mode="GUIDED", initial_armed=True)
        server = GroundServer(
            vehicle_secrets={1: SECRET_1}, calibration_ids={1: "venue-v1"}
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
        )
        try:
            await agent.start()
            await server.wait_for_vehicle(1)
            command = await server.send_command(
                1,
                VehicleCommandType.TAKEOFF,
                target_altitude_m=4.0,
                ttl_ms=5_000,
            )
            completed = await server.wait_for_ack(
                1, command.message_id, AckStatus.COMPLETED
            )

            assert [item[0] for item in link.submitted] == [
                FcuCommand(FcuAction.TAKEOFF, target_altitude_m=4.0)
            ]
            assert completed.apm_command_result == int(MavResult.ACCEPTED)
            assert completed.physical_completion_confirmed
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_takeoff_sequence_stops_when_guided_mode_switch_fails():
    async def scenario():
        link = StubApmLink(
            initial_mode="STABILIZE",
            initial_armed=False,
            initial_altitude_m=0.0,
            fail_action=FcuAction.SET_MODE,
        )
        server = GroundServer(
            vehicle_secrets={1: SECRET_1}, calibration_ids={1: "venue-v1"}
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
        )
        try:
            await agent.start()
            await server.wait_for_vehicle(1)
            command = await server.send_command(
                1,
                VehicleCommandType.TAKEOFF,
                target_altitude_m=3.0,
                ttl_ms=5_000,
            )
            failed = await server.wait_for_ack(
                1, command.message_id, AckStatus.FAILED
            )

            assert [item[0] for item in link.submitted] == [
                FcuCommand(FcuAction.SET_MODE, mode="GUIDED")
            ]
            assert failed.apm_command_result == int(MavResult.DENIED)
            assert not failed.physical_completion_confirmed
            assert "set_mode_guided" in failed.detail
            with pytest.raises(asyncio.TimeoutError):
                await server.wait_for_ack(
                    1,
                    command.message_id,
                    AckStatus.RUNNING,
                    timeout_s=0.05,
                )
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_agent_stop_cancels_an_in_progress_takeoff_sequence_task():
    async def scenario():
        link = StubApmLink(
            block_first_result=True,
            initial_mode="STABILIZE",
            initial_armed=False,
            initial_altitude_m=0.0,
        )
        server = GroundServer(
            vehicle_secrets={1: SECRET_1}, calibration_ids={1: "venue-v1"}
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
        )
        try:
            await agent.start()
            await server.wait_for_vehicle(1)
            command = await server.send_command(
                1,
                VehicleCommandType.TAKEOFF,
                target_altitude_m=3.0,
                ttl_ms=5_000,
            )
            await server.wait_for_ack(1, command.message_id, AckStatus.ACCEPTED)
            await wait_until(lambda: bool(agent._takeoff_sequence_tasks))

            await agent.stop()

            assert not agent._takeoff_sequence_tasks
            assert not any(
                not task.done() and task.get_name().startswith("takeoff-sequence-")
                for task in asyncio.all_tasks()
            )
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_agent_never_emits_completed_without_physical_confirmation():
    async def scenario():
        link = StubApmLink(physical_confirmed=False)
        server = GroundServer(
            vehicle_secrets={1: SECRET_1}, calibration_ids={1: "venue-v1"}
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.1,
            telemetry_interval_s=0.1,
        )
        try:
            await agent.start()
            await server.wait_for_vehicle(1)
            command = await server.send_command(1, VehicleCommandType.HOLD)
            failed = await server.wait_for_ack(
                1, command.message_id, AckStatus.FAILED
            )
            assert "without physical evidence" in failed.detail
            assert not failed.physical_completion_confirmed
            with pytest.raises(asyncio.TimeoutError):
                await server.wait_for_ack(
                    1,
                    command.message_id,
                    AckStatus.COMPLETED,
                    timeout_s=0.05,
                )
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_safety_command_is_submitted_while_normal_command_is_still_running():
    async def scenario():
        link = StubApmLink(block_first_result=True)
        server = GroundServer(
            vehicle_secrets={1: SECRET_1}, calibration_ids={1: "venue-v1"}
        )
        await server.start()
        agent = OnboardAgent(
            ground_uri=server.uri,
            vehicle_id=1,
            shared_secret=SECRET_1,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="config-hash-v1",
            heartbeat_interval_s=0.1,
            telemetry_interval_s=0.1,
        )
        try:
            await agent.start()
            await server.wait_for_vehicle(1)
            blocking = await server.send_command(1, VehicleCommandType.ARM)
            await server.wait_for_ack(1, blocking.message_id, AckStatus.RUNNING)

            safety = await server.send_command(1, VehicleCommandType.LAND)
            await server.wait_for_ack(1, safety.message_id, AckStatus.COMPLETED)
            assert [submitted[0].action for submitted in link.submitted] == [
                FcuAction.ARM,
                FcuAction.LAND,
            ]

            link.finish_blocked_result()
            await server.wait_for_ack(1, blocking.message_id, AckStatus.COMPLETED)
        finally:
            await agent.stop()
            await server.stop()

    run(scenario())


def test_tampered_signed_message_closes_authenticated_loopback_session():
    async def scenario():
        server = GroundServer(vehicle_secrets={1: SECRET_1})
        await server.start()
        session_id = uuid4()
        try:
            websocket = await authenticate_raw_client(server.uri, session_id)
            heartbeat = Heartbeat(
                vehicle_id=1,
                sequence=1,
                session_id=session_id,
                software_version="test",
                configuration_hash="config-hash-v1",
            )
            wrong_secret = b"wrong-but-still-long-enough-secret-value"
            await websocket.send(encode_signed_message(heartbeat, wrong_secret))
            with pytest.raises(ConnectionClosed):
                await websocket.recv()
            assert websocket.close_code == 1008
        finally:
            await server.stop()

    run(scenario())
