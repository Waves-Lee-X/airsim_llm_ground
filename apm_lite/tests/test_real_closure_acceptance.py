import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "deploy" / "windows" / "accept-real-closure.py"
SPEC = importlib.util.spec_from_file_location("accept_real_closure", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def sample(*, agent=True, fcu=True, video=True, output=False, received=10):
    return {
        "runtime_error": None,
        "onboard_agent_connected": agent,
        "fcu_link_ok": fcu,
        "command_output_enabled": output,
        "camera": {
            "stream_available": video,
            "source_fps": 15.0,
            "frame_age_s": 0.08,
        },
        "ground_link": {
            "stats": {
                "received_frames": received,
                "duplicate_frames": 1,
                "crc_errors": 0,
                "retries": 0,
                "invalid_payloads": 0,
            }
        },
    }


def test_accepts_a_stable_read_only_closure():
    result = MODULE.evaluate_samples([sample(received=10), sample(received=20)])

    assert result["accepted"]
    assert result["availability"]["read_only_gate"] == 1.0
    assert result["serial_deltas"]["received_frames"] == 10


def test_rejects_command_output_or_missing_links():
    result = MODULE.evaluate_samples(
        [sample(agent=False, fcu=False, video=False, output=True)]
    )

    assert not result["accepted"]
    assert any("command output" in failure for failure in result["failures"])
    assert any("FCU" in failure for failure in result["failures"])
