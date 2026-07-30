import pytest

from aeromind_apm_lite.onboard.mavlink import MavlinkEnvelope
from aeromind_apm_lite.onboard.telemetry import TelemetryAccumulator


def gps_message(*, eph):
    return MavlinkEnvelope(
        "GPS_RAW_INT",
        {
            "fix_type": 3,
            "satellites_visible": 15,
            "eph": eph,
        },
        source_system=1,
        source_component=1,
    )


def test_gps_raw_int_eph_is_converted_to_hdop():
    accumulator = TelemetryAccumulator()
    accumulator.reduce(gps_message(eph=87), now=10.0)

    snapshot = accumulator.snapshot(now=10.1, heartbeat_timeout_s=2.0)
    assert snapshot.gps_hdop == 0.87
    assert snapshot.gps_fix_type == 3
    assert snapshot.field_ages_s["gps"] == pytest.approx(0.1)


def test_unknown_gps_dilution_does_not_become_a_false_good_value():
    accumulator = TelemetryAccumulator()
    accumulator.reduce(gps_message(eph=0xFFFF), now=10.0)

    assert accumulator.snapshot(10.0, 2.0).gps_hdop is None
