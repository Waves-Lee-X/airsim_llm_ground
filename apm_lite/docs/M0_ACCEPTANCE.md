# M0 Acceptance Record

Date: 2026-07-28

## Result

**Software gate: passed. Hardware identity: established. Runtime/deployment
freeze: waiting for firmware, parameters and bench evidence.**

M0 created an isolated Python package under `apm_lite/`. It does not import
ROS 2, MAVROS, `px4_msgs` or PX4 Offboard code, and the existing prototype
under `src/` remains unchanged.

## Delivered

- Strict protocol v1 models for commands, trajectories, formations,
  telemetry, semantic observations, mission ACKs and heartbeats.
- Stable decoding errors for invalid JSON, unsupported versions, unknown
  message types and invalid payloads.
- Receiver-local TTL checks, vehicle/session/calibration gates, sequence
  replay protection and per-vehicle HMAC authentication.
- A monotonic mission state machine with immutable terminal states.
- Explicit venue-map, ENU and NED transforms with a calibration identity.
- Shared RGB/depth source contracts for AirSim, RealSense and replay.
- Strict four-vehicle simulation and real deployment examples.
- A generated JSON Schema bundle for non-Python clients.
- CI checks that forbid ROS/PX4/MAVROS dependencies and direct AirSim flight
  control calls.

## Verification evidence

```text
23 tests passed
protocol schema export --check passed
Python compileall passed
aeromind_apm_lite-0.1.0-py3-none-any.whl built successfully
```

The tests cover version rejection, strict payload validation, local TTL,
duplicate and replay rejection, wrong vehicle/session/calibration, HMAC
tampering, physical-completion ACK rules, mission transitions, coordinate
round trips, sensor buffers and four-vehicle configuration uniqueness.

## Reviewed hardware baseline

The supplied `郑州550无人机.docx` establishes the following facts:

- The aircraft is a 550-class quadrotor.
- The flight controller is a modern Pixhawk-compatible 雷迅/CUAV V5+ with an
  STM32F765 main processor, five UARTs and two CAN buses.
- The onboard computer is a Raspberry Pi 4B with 4 GB RAM and a stated
  5 V / 3 A input requirement.
- The intended FCU link is V5+ TELEM2 with TX/RX crossed and a common ground.
- The positioning module is a C-RTK 9Ps based on u-blox ZED-F9P.

The legacy real-agent code independently records `/dev/ttyAMA0` at 921600 baud
for the Raspberry Pi-to-FCU link. This is a migration baseline, not current
bench proof. See `docs/HARDWARE_BASELINE.md` for confirmed facts, safety rules
and the remaining checks.

## Open deployment inputs

M1 implementation can start with the fake FCU and runtime discovery, but its
SITL acceptance target must not be frozen until these are known:

1. Installed ArduCopter version, firmware board target and build/commit.
2. Frame type, motor order and a full parameter export from one aircraft.
3. AirSim version and the intended ArduPilot connector.
4. Raspberry Pi OS/UART configuration and a current TELEM2 heartbeat test.
5. TELEM2 logic level/current rating and independent Pi power validation.
6. Actual D435/camera inventory, mounting and USB connection.

Unknown values remain `null` in `configs/compatibility.yaml`. They must be
replaced by reviewed values and hashes before real-flight work.

## Next milestone: M1

M1 will implement one vehicle end to end:

1. A fake FCU transport for deterministic command and timeout tests.
2. A single-owner asynchronous `pymavlink` connection.
3. Heartbeat and `AUTOPILOT_VERSION` discovery.
4. Normalized telemetry snapshots.
5. Arm, takeoff, GUIDED target, HOLD, LAND and RTL command services.
6. Separate application acceptance, MAVLink `COMMAND_ACK` and physical
   completion tracking.
7. A single ArduPilot SITL mission repeated 10 times with one terminal result
   per run.
