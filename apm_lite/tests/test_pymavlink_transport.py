import asyncio

import pytest
from pymavlink import mavutil

from aeromind_apm_lite.common.config import FcuConnection
from aeromind_apm_lite.onboard.mavlink.fake import (
    FakeMavlinkTransport,
    SentCommandLong,
    SentHeartbeat,
)
from aeromind_apm_lite.onboard.mavlink.pymavlink_transport import (
    PymavlinkTransport,
)


class FakeMessage:
    def get_type(self):
        return "ATTITUDE"

    def to_dict(self):
        return {"roll": 0.1, "pitch": 0.2, "yaw": 0.3}

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class FakeMavSender:
    def __init__(self):
        self.commands = []
        self.heartbeats = []
        self.targets = []

    def command_long_send(self, *args):
        self.commands.append(args)

    def heartbeat_send(self, *args):
        self.heartbeats.append(args)

    def set_position_target_local_ned_send(self, *args):
        self.targets.append(args)


class FakeConnection:
    def __init__(self):
        self.mav = FakeMavSender()
        self.messages = [FakeMessage()]
        self.closed = False

    def recv_match(self, *, blocking):
        assert blocking is False
        return self.messages.pop(0) if self.messages else None

    def mode_mapping(self):
        return {"GUIDED": 4, "LOITER": 5}

    def close(self):
        self.closed = True


def test_pymavlink_transport_uses_one_owner_and_clean_position_masks():
    async def scenario():
        connection = FakeConnection()
        factory_calls = []

        def factory(endpoint, **kwargs):
            factory_calls.append((endpoint, kwargs))
            return connection

        config = FcuConnection(
            transport="udp",
            endpoint="udpin:0.0.0.0:14550",
            source_system=201,
            source_component=191,
            target_system=1,
            target_component=1,
        )
        transport = PymavlinkTransport(config, connection_factory=factory)
        await transport.open()
        assert factory_calls == [
            (
                "udpin:0.0.0.0:14550",
                {
                    "source_system": 201,
                    "source_component": 191,
                    "autoreconnect": False,
                },
            )
        ]

        message = await transport.receive(0.0)
        assert message is not None and message.name == "ATTITUDE"
        await transport.send_heartbeat()
        await transport.send_command_long(400, (1, 0, 0, 0, 0, 0, 0))
        await transport.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            10.0,
        )
        await transport.send_local_position_target(1.0, 2.0, -3.0, None)
        await transport.send_local_position_target(1.0, 2.0, -3.0, 0.5)
        assert connection.mav.heartbeats == [
            (
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
                3,
            )
        ]
        assert connection.mav.commands == [
            (1, 1, 400, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (
                1,
                1,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED),
                100_000.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
        ]
        assert connection.mav.targets[0][4] == 0x0DF8
        assert connection.mav.targets[1][4] == 0x09F8
        assert await transport.mode_id("guided") == 4

        foreign_task = asyncio.create_task(transport.mode_id("LOITER"))
        with pytest.raises(RuntimeError, match="owner task"):
            await foreign_task
        foreign_heartbeat = asyncio.create_task(transport.send_heartbeat())
        with pytest.raises(RuntimeError, match="owner task"):
            await foreign_heartbeat
        foreign_interval = asyncio.create_task(
            transport.request_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                10.0,
            )
        )
        with pytest.raises(RuntimeError, match="owner task"):
            await foreign_interval
        await transport.close()
        assert connection.closed

    asyncio.run(scenario())


def test_pymavlink_transport_requests_all_critical_message_intervals():
    async def scenario():
        connection = FakeConnection()
        config = FcuConnection(
            transport="udp",
            endpoint="udpin:0.0.0.0:14550",
            source_system=201,
            source_component=191,
            target_system=1,
            target_component=1,
        )
        transport = PymavlinkTransport(
            config,
            connection_factory=lambda *_args, **_kwargs: connection,
        )
        mavlink = mavutil.mavlink
        requests = [
            (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0, 100_000.0),
            (mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0, 200_000.0),
            (mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0, 500_000.0),
            (mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1.0, 1_000_000.0),
            (mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0, 100_000.0),
            (mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5.0, 200_000.0),
            (mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0, 500_000.0),
            (mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2.0, 500_000.0),
            (mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2.0, 500_000.0),
        ]

        await transport.open()
        for message_id, frequency_hz, _interval_us in requests:
            await transport.request_message_interval(message_id, frequency_hz)

        assert connection.mav.commands == [
            (
                1,
                1,
                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                interval_us,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            for message_id, _frequency_hz, interval_us in requests
        ]
        await transport.close()

    asyncio.run(scenario())


def test_fake_transport_records_heartbeat_and_message_interval_command():
    async def scenario():
        transport = FakeMavlinkTransport()
        await transport.open()
        await transport.send_heartbeat()
        await transport.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
            2.5,
        )

        assert transport.sent == [
            SentHeartbeat(),
            SentCommandLong(
                command_id=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                params=(193.0, 400_000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ),
        ]
        await transport.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    ("message_id", "frequency_hz", "error"),
    [
        (-1, 1.0, "message_id"),
        (0x1000000, 1.0, "message_id"),
        (True, 1.0, "message_id"),
        (1, True, "frequency_hz"),
        (1, "10.0", "frequency_hz"),
        (1, 0.0, "frequency_hz"),
        (1, 0.09, "frequency_hz"),
        (1, 100.01, "frequency_hz"),
        (1, float("nan"), "frequency_hz"),
        (1, float("inf"), "frequency_hz"),
    ],
)
def test_message_interval_request_rejects_invalid_values(
    message_id,
    frequency_hz,
    error,
):
    async def scenario():
        transport = FakeMavlinkTransport()
        await transport.open()
        with pytest.raises(ValueError, match=error):
            await transport.request_message_interval(message_id, frequency_hz)
        assert transport.sent == []
        await transport.close()

    asyncio.run(scenario())
