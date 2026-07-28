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
    MavlinkAckEvidence,
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

    def __post_init__(self):
        self.submitted = []
        self._blocked_result = None

    def telemetry_snapshot(self):
        now = time.monotonic()
        return TelemetrySnapshot(
            observed_monotonic_s=now,
            fcu_link_ok=True,
            armed=True,
            mode="GUIDED",
            relative_altitude_m=2.0,
            local_position_ned_m=(1.0, 2.0, -2.0),
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
        mavlink_ack = MavlinkAckEvidence(
            applicable=True,
            received=True,
            command_id=400,
            result=0,
            observed_monotonic_s=time.monotonic(),
            detail="MAVLink accepted",
        )
        physical = PhysicalCompletionEvidence(
            confirmed=self.physical_confirmed,
            observed_monotonic_s=(time.monotonic() if self.physical_confirmed else None),
            detail="physical state observed",
        )
        result = CommandResult(
            request_id=request_id,
            action=command.action,
            status=CommandStatus.COMPLETED,
            application=application,
            sent_monotonic_s=time.monotonic(),
            mavlink_ack=mavlink_ack,
            physical_completion=physical,
            detail="physical command result",
        )
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
