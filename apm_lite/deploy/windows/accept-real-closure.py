"""Evaluate the real UAV link closure through the read-only Web API."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from typing import Any
from urllib.error import URLError
from urllib.request import urlopen


def _fetch_json(url: str, timeout_s: float) -> dict[str, Any]:
    with urlopen(url, timeout=timeout_s) as response:  # nosec B310 - operator URL
        return json.loads(response.read().decode("utf-8"))


def evaluate_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {"accepted": False, "failures": ["no status samples"]}

    valid = [sample for sample in samples if "fetch_error" not in sample]
    failures: list[str] = []
    if not valid:
        return {
            "accepted": False,
            "sample_count": len(samples),
            "failures": ["ground Web API was unavailable for every sample"],
        }

    def ratio(predicate) -> float:
        return sum(bool(predicate(sample)) for sample in valid) / len(valid)

    agent_ratio = ratio(
        lambda sample: sample.get("onboard_agent_connected") is True
    )
    fcu_ratio = ratio(lambda sample: sample.get("fcu_link_ok") is True)
    video_ratio = ratio(
        lambda sample: sample.get("camera", {}).get("stream_available") is True
    )
    read_only_ratio = ratio(
        lambda sample: sample.get("command_output_enabled") is False
    )
    runtime_error_count = sum(bool(sample.get("runtime_error")) for sample in valid)

    frame_ages = [
        float(sample["camera"]["frame_age_s"])
        for sample in valid
        if sample.get("camera", {}).get("frame_age_s") is not None
    ]
    source_rates = [
        float(sample["camera"]["source_fps"])
        for sample in valid
        if float(sample.get("camera", {}).get("source_fps") or 0.0) > 0.0
    ]
    first_stats = valid[0].get("ground_link", {}).get("stats", {})
    last_stats = valid[-1].get("ground_link", {}).get("stats", {})

    def delta(name: str) -> int:
        return max(0, int(last_stats.get(name, 0)) - int(first_stats.get(name, 0)))

    if len(valid) / len(samples) < 0.95:
        failures.append("ground Web API availability was below 95%")
    if agent_ratio < 0.95:
        failures.append("authenticated onboard Agent availability was below 95%")
    if fcu_ratio < 0.95:
        failures.append("FCU telemetry availability was below 95%")
    if video_ratio < 0.90:
        failures.append("RGB video availability was below 90%")
    if read_only_ratio != 1.0:
        failures.append("command output was not disabled for every valid sample")
    if runtime_error_count:
        failures.append("ground runtime reported an error")
    if frame_ages and max(frame_ages) > 3.0:
        failures.append("latest decoded video frame exceeded 3 seconds old")
    if source_rates and statistics.median(source_rates) < 10.0:
        failures.append("median decoded source rate was below 10 FPS")
    if delta("invalid_payloads") != 0:
        failures.append("invalid Lite protocol payloads increased during acceptance")

    return {
        "accepted": not failures,
        "sample_count": len(samples),
        "valid_sample_count": len(valid),
        "availability": {
            "ground_api": round(len(valid) / len(samples), 3),
            "onboard_agent": round(agent_ratio, 3),
            "fcu": round(fcu_ratio, 3),
            "video": round(video_ratio, 3),
            "read_only_gate": round(read_only_ratio, 3),
        },
        "video": {
            "median_source_fps": (
                round(statistics.median(source_rates), 1) if source_rates else None
            ),
            "max_bridge_frame_age_s": (
                round(max(frame_ages), 3) if frame_ages else None
            ),
        },
        "serial_deltas": {
            name: delta(name)
            for name in (
                "received_frames",
                "duplicate_frames",
                "crc_errors",
                "retries",
                "invalid_payloads",
            )
        },
        "failures": failures,
    }


def collect_samples(
    base_url: str,
    *,
    duration_s: float,
    interval_s: float,
    timeout_s: float,
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    deadline = time.monotonic() + duration_s
    status_url = f"{base_url.rstrip('/')}/api/status"
    while True:
        started = time.monotonic()
        try:
            samples.append(_fetch_json(status_url, timeout_s))
        except (OSError, TimeoutError, ValueError, URLError) as exc:
            samples.append({"fetch_error": f"{type(exc).__name__}: {exc}"})
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(remaining, max(0.0, interval_s - (time.monotonic() - started))))
    return samples


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a bounded read-only real-link closure acceptance."
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.interval <= 0.0 or args.timeout <= 0.0:
        parser.error("duration, interval and timeout must be positive")

    samples = collect_samples(
        args.base_url,
        duration_s=args.duration,
        interval_s=args.interval,
        timeout_s=args.timeout,
    )
    result = evaluate_samples(samples)
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
