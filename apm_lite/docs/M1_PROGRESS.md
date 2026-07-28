# M1 Progress Record

Date: 2026-07-28

## Result

**M1 single-vehicle AirSim/ArduPilot SITL gate: passed.**

The accepted batch completed all 10 requested fixed missions and the separate
RTL safety path. Every flight action retained application acceptance,
MAVLink `COMMAND_ACK` evidence where applicable, and fresh physical telemetry
evidence. No real-aircraft flight command was sent during M1.

The accepted runtime is frozen as:

- AirSim `1.8.1`, Blocks environment;
- ArduPilot Copter `4.7.0` at commit
  `1511f27194f1dcc3728270883047bdf022b3fd53`;
- `pymavlink 2.4.49`;
- one AirSim vehicle, one ArduCopter SITL process and one Lite agent;
- GCS system ID `201`, vehicle system ID `1` and Lite receive endpoint
  `udpin:0.0.0.0:14550`.

This freeze applies only to the M1 AirSim/SITL baseline. It is not evidence of
the firmware, parameters, serial mapping or GPS delay on the real CUAV V5+.

## Accepted mission

Each repeat is an outbound-and-return loop so all runs stay in the same tested
part of the AirSim scene:

```text
GUIDED
  -> ARM
  -> TAKEOFF 2 m
  -> LOCAL_NED north 3 m
  -> HOLD
  -> GUIDED
  -> return to this run's start position
  -> LAND
```

All eight actions must reach a terminal result. An accepted ACK never marks an
action complete by itself.

| Action | MAVLink evidence | Physical completion evidence |
|---|---|---|
| ARM/DISARM | ACK for command 400 | Fresh HEARTBEAT armed bit |
| TAKEOFF | ACK for command 22 | Armed, target relative altitude, fresh settled velocity |
| SET_MODE | ACK for command 176 | Fresh HEARTBEAT mode |
| GUIDED target | ACK not applicable | Fresh LOCAL_POSITION_NED error and settled velocity |
| HOLD | ACK for command 176 | Configured hold mode plus settled velocity |
| LAND | ACK for command 21 | Fresh ON_GROUND state and disarmed heartbeat |
| RTL | ACK for command 20 | At Home tolerance, ON_GROUND and disarmed |

## Acceptance evidence

Accepted evidence directory:

```text
D:\AirSim\aeromind-apm-lite-m1\acceptance4\run-20260728T114147Z-548787-d58b2f77
```

Result:

```text
gate: M1_SINGLE_VEHICLE_AIRSIM_SITL
fixed missions: 10/10 completed (threshold: 9/10)
fixed mission actions: 80/80 physically completed
independent RTL path: completed
overall passed: true
```

Frozen hashes:

```text
ArduPilot binary SHA-256:
55f5f19fe4078ecdcf183b07a855831a1156002abfc054c7f0ccec82646dec4b

M1 parameter file SHA-256:
609657d295acdef35471de27800db222c4c5c307e1c559f02de8cbde5125f0c9

generated AirSim settings SHA-256:
c97a961d19a42bcdb0fe26cf156fa903cc5ed3a219cd9d876153b8a98b4ce5d3
```

The evidence directory is intentionally outside Git because it includes a
31 MB DataFlash log and runtime state. `summary.json`, `fixed-missions.jsonl`,
`rtl.json`, FCU/AirSim logs and the DataFlash BIN remain available for audit.

This final batch was run after the same-MAV_CMD ACK isolation regression was
fixed. UE4 and ArduCopter were both automatically stopped after the gate, and
no owned simulator process remained.

## Software verification

```text
123 tests passed
JSON Schema export --check passed
Python compileall passed
flake8 passed
git diff --check passed
```

## Failed batches retained

The two failed batches are retained as negative evidence:

| Evidence directory | Result | Root cause | Correction |
|---|---:|---|---|
| `D:\AirSim\aeromind-apm-lite-m1\acceptance\run-20260728T104727Z-532493-3da396e7` | 7/10 | AirSim GPS samples arrived about 128-230 ms late while `GPS1_DELAY_MS` declared 20 ms, producing `Arm: GPS 1: not healthy` | Freeze the measured AirSim-only delay at 200 ms and re-check GPS/pre-arm health after entering GUIDED |
| `D:\AirSim\aeromind-apm-lite-m1\acceptance2\run-20260728T110021Z-533669-9b8fc5e9` | 8/10 | The old mission accumulated 3 m north on every repeat and entered the Blocks obstacle area, causing an EKF failure | Make every repeat return to its own starting position before LAND |

`GPS1_DELAY_MS=200` is an AirSim connector calibration, not a real F9P value.
It must never be copied to the V5+ parameter set without real log evidence.

One preflight CLI invocation is also retained at
`D:\AirSim\aeromind-apm-lite-m1\acceptance4\run-20260728T114133Z-548761-cfea746b`.
It was rejected before AirSim/SITL startup because the parameter-file argument
was relative; it is not counted as a flight attempt.

## Delivered

- A transport-neutral MAVLink boundary and deterministic fake FCU transport.
- A real serial/UDP `PymavlinkTransport` with verified LOCAL_NED masks.
- One `ApmLink` asyncio owner for every transport receive and send operation.
- Target-system/component filtering, ArduPilot/quadrotor validation and
  explicit `AUTOPILOT_VERSION` discovery.
- Normalized position, velocity, attitude, battery, GPS, GPS-health, EKF,
  landed-state, Home and status-text telemetry with independent freshness.
- Bounded command queues with HOLD/LAND/RTL/DISARM safety priority.
- Continuous GUIDED target transmission with expiry-triggered HOLD.
- ARM, DISARM, TAKEOFF, SET_MODE, LOCAL_NED, HOLD, LAND and RTL services.
- Three-layer command evidence and a short `STATUSTEXT` diagnostic window
  after a negative ACK.
- HMAC-authenticated WebSocket sessions with TTL, sequence and replay checks.
- AirSim/ArduPilot lifecycle management with exact owned-process cleanup.
- Read-only `aeromind-apm-probe` and repeatable
  `aeromind-apm-m1-acceptance` command-line tools.

## M2 boundary

M2 starts with a propeller-removed V5+ bench, not a low-altitude flight. Before
control is enabled, the bench must identify the installed firmware and board
target, export and hash all parameters, confirm which `SERIALx` maps to
TELEM2, verify MAVLink 2/baud/flow control and validate the TELEM2 logic level.
The Raspberry Pi must use an independent regulated supply; TELEM2 carries only
crossed TX/RX and common GND unless CUAV supplies contrary rated evidence.
