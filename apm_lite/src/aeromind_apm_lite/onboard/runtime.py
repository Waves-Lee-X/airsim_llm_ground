"""Standalone Raspberry Pi runtime for V5+ and P9 serial links."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import signal
from pathlib import Path
from typing import Any

from aeromind_apm_lite.common.communication import SerialOnboardChannelFactory
from aeromind_apm_lite.common.config import VehicleRuntimeConfig
from aeromind_apm_lite.common.contracts import VehicleCommandType
from aeromind_apm_lite.common.credentials import load_shared_secret

from .agent import OnboardAgent
from .apm_link import ApmLink
from .mavlink.pymavlink_transport import PymavlinkTransport
from .probe import select_vehicle


LOGGER = logging.getLogger("aeromind_apm_lite.onboard")


def vehicle_configuration_hash(vehicle: VehicleRuntimeConfig) -> str:
    payload = vehicle.model_dump(mode="json")
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


async def run_onboard(
    vehicle: VehicleRuntimeConfig,
    *,
    shared_secret: bytes,
    ground_serial_port: str,
    ground_serial_baudrate: int,
    telemetry_hz: float = 2.0,
    command_output_enabled: bool = False,
    allowed_commands: tuple[VehicleCommandType, ...] = (),
    stop_event: asyncio.Event | None = None,
    link_factory: Any | None = None,
    channel_factory: Any | None = None,
) -> None:
    if not 0.2 <= telemetry_hz <= 10.0:
        raise ValueError("telemetry_hz must be in [0.2, 10]")
    if link_factory is None:
        transport = PymavlinkTransport(vehicle.fcu)
        link = ApmLink.from_vehicle_config(transport, vehicle)
    else:
        link = link_factory(vehicle)
    if channel_factory is None:
        channel_factory = SerialOnboardChannelFactory(
            port=ground_serial_port,
            baudrate=ground_serial_baudrate,
            vehicle_id=vehicle.vehicle_id,
        )
    agent = OnboardAgent(
        channel_factory=channel_factory,
        vehicle_id=vehicle.vehicle_id,
        shared_secret=shared_secret,
        frame_calibration_id=vehicle.frame_calibration_id,
        apm_link=link,
        configuration_hash=vehicle_configuration_hash(vehicle),
        heartbeat_interval_s=1.0,
        telemetry_interval_s=1.0 / telemetry_hz,
        reconnect_delay_s=1.0,
        handshake_timeout_s=5.0,
        command_output_enabled=command_output_enabled,
        allowed_commands=allowed_commands,
    )
    stop_event = stop_event or asyncio.Event()
    try:
        identity = await link.start()
        LOGGER.info(
            "FCU ready: vehicle=%s autopilot=%s firmware=%s",
            vehicle.vehicle_id,
            getattr(identity, "autopilot", "unknown"),
            getattr(identity, "flight_sw_version", "unknown"),
        )
        await agent.start_background()
        LOGGER.info(
            "P9 agent started: port=%s baud=%s command_output=%s allowlist=%s",
            ground_serial_port,
            ground_serial_baudrate,
            "enabled" if command_output_enabled else "disabled",
            ",".join(command.value for command in allowed_commands) or "none",
        )
        await stop_event.wait()
    finally:
        await agent.stop()
        await link.stop()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the ROS-free AeroMind APM Lite agent on Raspberry Pi."
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--vehicle-id", type=int, required=True)
    parser.add_argument("--ground-serial")
    parser.add_argument("--ground-baud", type=int)
    parser.add_argument("--telemetry-hz", type=float, default=2.0)
    parser.add_argument(
        "--secret-env",
        help="Override the vehicle credential_env field from the fleet config.",
    )
    parser.add_argument(
        "--enable-command-output",
        action="store_true",
        help="Allow authenticated ground commands to reach the FCU.",
    )
    parser.add_argument(
        "--allow-command",
        action="append",
        choices=tuple(command.value for command in VehicleCommandType),
        default=[],
        help=(
            "Allow one command to reach the FCU; repeat as needed. "
            "Requires --enable-command-output."
        ),
    )
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    return parser


async def _run_from_args(args: argparse.Namespace) -> None:
    vehicle = select_vehicle(args.config, args.vehicle_id)
    secret = load_shared_secret(args.secret_env or vehicle.credential_env)
    configured_ground = vehicle.ground_link
    ground_serial_port = args.ground_serial or (
        configured_ground.endpoint if configured_ground is not None else None
    )
    ground_serial_baudrate = args.ground_baud or (
        configured_ground.baudrate if configured_ground is not None else None
    )
    if ground_serial_port is None or ground_serial_baudrate is None:
        raise ValueError(
            "ground serial port and baud are required by config or command line"
        )
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(signum, stop_event.set)
        except NotImplementedError:  # pragma: no cover - Windows development only
            pass
    await run_onboard(
        vehicle,
        shared_secret=secret,
        ground_serial_port=ground_serial_port,
        ground_serial_baudrate=ground_serial_baudrate,
        telemetry_hz=args.telemetry_hz,
        command_output_enabled=args.enable_command_output,
        allowed_commands=tuple(
            VehicleCommandType(command) for command in args.allow_command
        ),
        stop_event=stop_event,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    asyncio.run(_run_from_args(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
