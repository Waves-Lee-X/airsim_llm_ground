import asyncio

from pymavlink import mavutil

from aeromind_apm_lite.common.config import FcuConnection, FcuTransport
from aeromind_apm_lite.ground.parallel_domain.adapters.io import PymavlinkIO


class _Sender:
    def __init__(self):
        self.commands = []
        self.mode_changes = []

    def command_long_send(self, *args):
        self.commands.append(args)

    def set_mode_send(self, *args):
        self.mode_changes.append(args)


class _Connection:
    def __init__(self, mapping):
        self.mav = _Sender()
        self._mapping = mapping

    def mode_mapping(self):
        return self._mapping

    def close(self):
        pass


def test_parallel_domain_io_encodes_px4_and_apm_mode_shapes():
    async def scenario():
        px4 = _Connection({"OFFBOARD": (29, 6, 0)})
        px4_io = PymavlinkIO(
            FcuConnection(
                transport=FcuTransport.UDP,
                endpoint="udpin:0.0.0.0:14541",
                source_system=250,
                source_component=190,
                target_system=1,
                target_component=1,
            ),
            connection_factory=lambda *_args, **_kwargs: px4,
        )
        await px4_io.open()
        await px4_io.set_mode("OFFBOARD")
        await px4_io.close()
        assert px4.mav.commands == [
            (
                1,
                1,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                29.0,
                6.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        ]
        assert px4.mav.mode_changes == []

        apm = _Connection({"GUIDED": 15})
        apm_io = PymavlinkIO(
            FcuConnection(
                transport=FcuTransport.UDP,
                endpoint="udpin:0.0.0.0:14550",
                source_system=250,
                source_component=190,
                target_system=2,
                target_component=1,
            ),
            connection_factory=lambda *_args, **_kwargs: apm,
        )
        await apm_io.open()
        await apm_io.set_mode("GUIDED")
        await apm_io.close()
        assert apm.mav.commands == []
        assert apm.mav.mode_changes == [(2, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)]

    asyncio.run(scenario())
