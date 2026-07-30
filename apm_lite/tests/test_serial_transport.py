import asyncio
import json
from uuid import uuid4

from aeromind_apm_lite.common.communication import (
    ReliableSerialLink,
    SerialChannelClosed,
    SerialDeliveryError,
    SerialDirection,
    SerialFrame,
    SerialFrameDecoder,
    SerialFrameKind,
    SerialOnboardChannelFactory,
    encode_serial_frame,
    payload_requires_delivery,
)
from aeromind_apm_lite.common.contracts import AckStatus, VehicleCommandType
from aeromind_apm_lite.ground.browser.app import (
    BrowserAction,
    BrowserGateway,
    CommandRequest,
)
from aeromind_apm_lite.ground.browser.runtime import (
    DemoApmLink,
    ManualRuntime,
    ManualRuntimeConfig,
    ManualRuntimeMode,
)
from aeromind_apm_lite.ground.serial_server import SerialGroundServer
from aeromind_apm_lite.onboard.agent import OnboardAgent


SECRET = b"vehicle-three-independent-secret-32-bytes"


def run(coroutine):
    return asyncio.run(coroutine)


async def wait_for_heartbeat(server, vehicle_id, timeout_s=1.0):
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        heartbeat = server.last_heartbeat(vehicle_id)
        if heartbeat is not None:
            return heartbeat
        await asyncio.sleep(0.01)
    raise TimeoutError("heartbeat did not arrive")


class MemoryByteStream:
    def __init__(self):
        self.incoming = asyncio.Queue()
        self.peer = None
        self.buffer = bytearray()
        self.closed = False
        self.drop_first_ack = False
        self.dropped_ack = False

    async def read(self, max_bytes):
        while not self.buffer:
            item = await self.incoming.get()
            if item is None:
                raise SerialChannelClosed("memory stream closed")
            self.buffer.extend(item)
        data = bytes(self.buffer[:max_bytes])
        del self.buffer[:max_bytes]
        return data

    async def write(self, data):
        if self.closed:
            raise SerialChannelClosed("memory stream closed")
        if self.drop_first_ack and not self.dropped_ack and data[3] == 2:
            self.dropped_ack = True
            return
        await self.peer.incoming.put(bytes(data))

    async def close(self):
        if self.closed:
            return
        self.closed = True
        await self.incoming.put(None)


def memory_stream_pair():
    first = MemoryByteStream()
    second = MemoryByteStream()
    first.peer = second
    second.peer = first
    return first, second


def test_only_replaceable_telemetry_skips_delivery_ack():
    heartbeat = (
        '{"frame_type":"signed_message","message":{"message_type":"heartbeat"}}'
    )
    telemetry = (
        '{"frame_type":"signed_message",'
        '"message":{"message_type":"vehicle_telemetry"}}'
    )
    mission_ack = (
        '{"frame_type":"signed_message","message":{"message_type":"mission_ack"}}'
    )

    assert not payload_requires_delivery(heartbeat)
    assert not payload_requires_delivery(telemetry)
    assert payload_requires_delivery(mission_ack)
    assert payload_requires_delivery('{"frame_type":"client_hello"}')


def test_serial_decoder_recovers_after_noise_fragmentation_and_bad_crc():
    frame = SerialFrame(
        kind=SerialFrameKind.DATA,
        direction=SerialDirection.ONBOARD_TO_GROUND,
        vehicle_id=3,
        sequence=42,
        payload=b'{"frame_type":"client_hello"}',
        reliable=True,
    )
    encoded = encode_serial_frame(frame)
    corrupted = bytearray(encoded)
    corrupted[-1] ^= 0xFF
    decoder = SerialFrameDecoder()

    assert decoder.feed(b"noise" + bytes(corrupted) + encoded[:5]) == []
    assert decoder.feed(encoded[5:]) == [frame]
    assert decoder.crc_errors == 1
    assert decoder.discarded_bytes >= 6


def test_reliable_serial_retries_and_suppresses_duplicate_delivery():
    async def scenario():
        onboard_stream, ground_stream = memory_stream_pair()
        ground_stream.drop_first_ack = True
        onboard = ReliableSerialLink(
            onboard_stream,
            send_direction=SerialDirection.ONBOARD_TO_GROUND,
            ack_timeout_s=0.02,
            max_retries=2,
            initial_sequence=10,
        )
        ground = ReliableSerialLink(
            ground_stream,
            send_direction=SerialDirection.GROUND_TO_ONBOARD,
            ack_timeout_s=0.02,
            max_retries=2,
            initial_sequence=20,
        )
        await onboard.start()
        await ground.start()
        try:
            await onboard.send(3, "reliable-command", reliable=True)
            received = await asyncio.wait_for(ground.receive(), 0.2)
            assert received.payload == b"reliable-command"
            assert onboard.stats.retries == 1
            assert ground.stats.duplicate_frames == 1
            try:
                await asyncio.wait_for(ground.receive(), 0.03)
            except asyncio.TimeoutError:
                pass
            else:  # pragma: no cover - duplicate delivery regression
                raise AssertionError("duplicate reliable frame was delivered")
        finally:
            await onboard.close()
            await ground.close()

    run(scenario())


def test_non_target_vehicle_does_not_ack_broadcast_radio_frame():
    async def scenario():
        ground_stream, onboard_stream = memory_stream_pair()
        ground = ReliableSerialLink(
            ground_stream,
            send_direction=SerialDirection.GROUND_TO_ONBOARD,
            ack_timeout_s=0.01,
            max_retries=1,
            accepted_vehicle_ids={1, 2, 3, 4},
        )
        onboard_three = ReliableSerialLink(
            onboard_stream,
            send_direction=SerialDirection.ONBOARD_TO_GROUND,
            ack_timeout_s=0.01,
            max_retries=1,
            accepted_vehicle_ids={3},
        )
        await ground.start()
        await onboard_three.start()
        try:
            try:
                await ground.send(2, "command-for-uav2", reliable=True)
            except SerialDeliveryError:
                pass
            else:  # pragma: no cover - cross-vehicle ACK regression
                raise AssertionError("UAV3 acknowledged a UAV2 frame")
            assert onboard_three.stats.received_frames == 0
        finally:
            await ground.close()
            await onboard_three.close()

    run(scenario())


def test_ground_restart_requests_stale_onboard_session_to_reauthenticate():
    async def scenario():
        ground_stream, onboard_stream = memory_stream_pair()

        async def open_ground():
            return ground_stream

        server = SerialGroundServer(
            serial_port="COM_TEST",
            stream_factory=open_ground,
            vehicle_secrets={3: SECRET},
            calibration_ids={3: "venue-v1"},
            serial_ack_timeout_s=0.05,
        )
        onboard = ReliableSerialLink(
            onboard_stream,
            send_direction=SerialDirection.ONBOARD_TO_GROUND,
            ack_timeout_s=0.05,
            max_retries=1,
            accepted_vehicle_ids={3},
        )
        await server.start()
        await onboard.start()
        try:
            stale_session = uuid4()
            stale_telemetry = json.dumps(
                {
                    "frame_type": "signed_message",
                    "vehicle_id": 3,
                    "session_id": str(stale_session),
                    "message": {"message_type": "vehicle_telemetry"},
                }
            )
            await onboard.send(3, stale_telemetry, reliable=False)
            reset = await asyncio.wait_for(onboard.receive(), timeout=0.3)
            payload = json.loads(reset.payload.decode("utf-8"))
            assert payload["frame_type"] == "session_reset"
            assert payload["session_id"] == str(stale_session)
            assert payload["reason"] == "ground_session_missing"
            assert server.serial_stats["session_reset_requests"] == 1
        finally:
            await onboard.close()
            await server.stop()

    run(scenario())


def test_complete_lite_protocol_runs_over_serial_radio_transport():
    async def scenario():
        ground_stream, onboard_stream = memory_stream_pair()

        async def open_ground():
            return ground_stream

        async def open_onboard():
            return onboard_stream

        server = SerialGroundServer(
            serial_port="COM_TEST",
            stream_factory=open_ground,
            vehicle_secrets={3: SECRET},
            calibration_ids={3: "venue-v1"},
            serial_ack_timeout_s=0.05,
            session_idle_timeout_s=1.0,
        )
        link = DemoApmLink()
        await link.start()
        agent = OnboardAgent(
            channel_factory=SerialOnboardChannelFactory(
                port="/dev/ttyTEST",
                baudrate=57_600,
                vehicle_id=3,
                stream_factory=open_onboard,
                ack_timeout_s=0.05,
            ),
            vehicle_id=3,
            shared_secret=SECRET,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="serial-config-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
        )
        await server.start()
        try:
            session_id = await agent.start(timeout_s=1.0)
            assert await server.wait_for_vehicle(3, timeout_s=1.0) == session_id
            command = await server.send_command(3, VehicleCommandType.ARM)
            accepted = await server.wait_for_ack(
                3, command.message_id, AckStatus.ACCEPTED, timeout_s=1.0
            )
            completed = await server.wait_for_ack(
                3, command.message_id, AckStatus.COMPLETED, timeout_s=1.0
            )
            assert accepted.status == AckStatus.ACCEPTED
            assert completed.physical_completion_confirmed

            await asyncio.sleep(0.1)
            telemetry = server.latest_telemetry(3)
            assert telemetry is not None
            assert telemetry.armed
            assert telemetry.health.fcu_link_ok is False
            assert server.serial_stats["crc_errors"] == 0
        finally:
            await agent.stop()
            await server.stop()
            await link.stop()

    run(scenario())


def test_read_only_onboard_authenticates_but_rejects_flight_output():
    async def scenario():
        ground_stream, onboard_stream = memory_stream_pair()

        async def open_ground():
            return ground_stream

        async def open_onboard():
            return onboard_stream

        server = SerialGroundServer(
            serial_port="COM_TEST",
            stream_factory=open_ground,
            vehicle_secrets={3: SECRET},
            calibration_ids={3: "venue-v1"},
            serial_ack_timeout_s=0.05,
        )
        link = DemoApmLink()
        await link.start()
        agent = OnboardAgent(
            channel_factory=SerialOnboardChannelFactory(
                port="/dev/ttyTEST",
                baudrate=57_600,
                vehicle_id=3,
                stream_factory=open_onboard,
                ack_timeout_s=0.05,
            ),
            vehicle_id=3,
            shared_secret=SECRET,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="serial-config-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
            command_output_enabled=False,
        )
        await server.start()
        try:
            await agent.start(timeout_s=1.0)
            command = await server.send_command(3, VehicleCommandType.ARM)
            await server.wait_for_ack(
                3, command.message_id, AckStatus.ACCEPTED, timeout_s=1.0
            )
            failed = await server.wait_for_ack(
                3, command.message_id, AckStatus.FAILED, timeout_s=1.0
            )
            assert "output is disabled" in failed.detail
            heartbeat = await wait_for_heartbeat(server, 3)
            assert heartbeat.command_output_enabled is False
            assert not link.telemetry_snapshot().armed
        finally:
            await agent.stop()
            await server.stop()
            await link.stop()

    run(scenario())


def test_onboard_allowlist_passes_only_arm_and_disarm_to_apm():
    async def scenario():
        ground_stream, onboard_stream = memory_stream_pair()

        async def open_ground():
            return ground_stream

        async def open_onboard():
            return onboard_stream

        server = SerialGroundServer(
            serial_port="COM_TEST",
            stream_factory=open_ground,
            vehicle_secrets={3: SECRET},
            calibration_ids={3: "venue-v1"},
            serial_ack_timeout_s=0.05,
        )
        link = DemoApmLink()
        await link.start()
        agent = OnboardAgent(
            channel_factory=SerialOnboardChannelFactory(
                port="/dev/ttyTEST",
                baudrate=57_600,
                vehicle_id=3,
                stream_factory=open_onboard,
                ack_timeout_s=0.05,
            ),
            vehicle_id=3,
            shared_secret=SECRET,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="serial-config-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
            command_output_enabled=True,
            allowed_commands=(
                VehicleCommandType.ARM,
                VehicleCommandType.DISARM,
            ),
        )
        await server.start()
        try:
            await agent.start(timeout_s=1.0)
            heartbeat = await wait_for_heartbeat(server, 3)
            assert heartbeat.command_output_enabled is True
            assert set(heartbeat.allowed_commands) == {
                VehicleCommandType.ARM,
                VehicleCommandType.DISARM,
            }

            arm = await server.send_command(3, VehicleCommandType.ARM)
            await server.wait_for_ack(
                3, arm.message_id, AckStatus.COMPLETED, timeout_s=1.0
            )
            assert link.telemetry_snapshot().armed

            disarm = await server.send_command(3, VehicleCommandType.DISARM)
            await server.wait_for_ack(
                3, disarm.message_id, AckStatus.COMPLETED, timeout_s=1.0
            )
            assert not link.telemetry_snapshot().armed

            for command_type in (
                VehicleCommandType.TAKEOFF,
                VehicleCommandType.HOLD,
                VehicleCommandType.LAND,
                VehicleCommandType.RTL,
            ):
                command = await server.send_command(
                    3,
                    command_type,
                    target_altitude_m=(
                        2.0
                        if command_type == VehicleCommandType.TAKEOFF
                        else None
                    ),
                )
                failed = await server.wait_for_ack(
                    3, command.message_id, AckStatus.FAILED, timeout_s=1.0
                )
                assert "not in the onboard allowlist" in failed.detail
        finally:
            await agent.stop()
            await server.stop()
            await link.stop()

    run(scenario())


def test_web_gateway_controls_remote_onboard_agent_over_serial():
    async def scenario():
        ground_stream, onboard_stream = memory_stream_pair()

        async def open_ground():
            return ground_stream

        async def open_onboard():
            return onboard_stream

        runtime = ManualRuntime(
            ManualRuntimeConfig(
                mode=ManualRuntimeMode.REAL_SERIAL,
                vehicle_id=3,
                vehicle_name="real-uav3",
                frame_calibration_id="venue-v1",
                ground_serial_port="COM3",
                ground_serial_baudrate=57_600,
                startup_timeout_s=1.0,
            ),
            shared_secret=SECRET,
            serial_stream_factory=open_ground,
        )
        gateway = BrowserGateway(runtime, telemetry_interval_s=0.05)
        link = DemoApmLink()
        await link.start()
        agent = OnboardAgent(
            channel_factory=SerialOnboardChannelFactory(
                port="/dev/ttyAMA1",
                baudrate=57_600,
                vehicle_id=3,
                stream_factory=open_onboard,
                ack_timeout_s=0.05,
            ),
            vehicle_id=3,
            shared_secret=SECRET,
            frame_calibration_id="venue-v1",
            apm_link=link,
            configuration_hash="serial-config-v1",
            heartbeat_interval_s=0.05,
            telemetry_interval_s=0.05,
            allowed_commands=(
                VehicleCommandType.ARM,
                VehicleCommandType.DISARM,
            ),
        )
        await gateway.start()
        try:
            await agent.start(timeout_s=1.0)
            await runtime.server.wait_for_vehicle(3, timeout_s=1.0)
            await wait_for_heartbeat(runtime.server, 3)
            outcome = await gateway.issue_command(
                3,
                BrowserAction.ARM,
                CommandRequest(completion_timeout_s=1.0),
            )
            assert outcome["successful"]
            assert outcome["simulated"] is False
            assert outcome["application"]["accepted"]
            assert outcome["physical_completion"]["confirmed"]

            await asyncio.sleep(0.1)
            status = gateway.status_payload()
            assert status["runtime_mode"] == "real_serial"
            assert status["vehicle_connected"]
            assert status["onboard_agent_connected"]
            assert status["command_output_enabled"] is True
            assert status["allowed_commands"] == ["arm", "disarm"]
            assert status["armed"] is True
            assert status["ground_link"]["transport"] == "serial"
            assert status["ground_link"]["stats"]["crc_errors"] == 0
            config = gateway.public_config()
            assert config["ground_serial_port"] == "COM3"
            assert "secret" not in str(config).lower()
        finally:
            await agent.stop()
            await gateway.stop()
            await link.stop()

    run(scenario())
