# M1.5 Manual Simulation Acceptance Record

Date: 2026-07-28

Status: **passed**

## Scope

M1.5 validates the operator-owned, non-ROS single-vehicle workflow in an
AirSim-enabled UE scene other than Blocks. UE/AirSim, ArduCopter SITL and the
Lite Web ground station remain separate processes started by the operator.

The accepted path is:

```text
Web browser
  -> FastAPI/WebSocket ground gateway
  -> authenticated OnboardAgent command
  -> ApmLink/pymavlink UDP 14550
  -> ArduCopter SITL
  -> AirSim vehicle physics
```

AirSim RPC is used only for Scene image capture. It is not used to bypass
ArduCopter for flight control.

## Operator Acceptance

The operator completed the formal manual integration in the
`PopulationSystemFullPack` UE project and reported the M1.5 workflow complete.
The session exercised the real SITL gateway on port `8000`, not DEMO command
simulation.

## Machine-Observed Evidence

At evidence collection time:

- `GET /api/health` reported `runtime_mode=sitl`, `ready=true` and
  `fcu_link_ok=true`.
- The onboard agent and FCU link were connected, with healthy GPS, EKF and
  pre-arm state.
- Final telemetry reported `mode=RTL`, `armed=false`, `landed_state=1` and a
  near-zero relative altitude.
- Final local position was approximately NED `(0.051, -0.077, 0.005)` metres
  from the local origin.
- `front_center` was online through AirSim RPC `192.168.16.1:41451`.
- `GET /api/camera/frame` returned HTTP 200, `image/png`, PNG signature
  `89 50 4E 47 0D 0A 1A 0A`, frame sequence `2935` and 1,463,559 bytes.

ArduPilot DataFlash
`/home/waves/aeromind_sitl/manual-uav1/logs/00000002.BIN` recorded:

| Local time | Evidence |
|---|---|
| 22:03:07 | `MAV_CMD_DO_SET_MODE` to GUIDED, result 0 |
| 22:03:08 | `MAV_CMD_NAV_TAKEOFF`, target 3 m, result 0 |
| 22:03:21 | `MAV_CMD_NAV_LAND`, result 0 |
| 22:15:11 | arm command, result 0 |
| 22:15:12 | `MAV_CMD_NAV_TAKEOFF`, target 3 m, result 0 |
| 22:15:20 | `MAV_CMD_NAV_RETURN_TO_LAUNCH`, result 0 |

The final API state confirmed RTL completion by landed and disarmed telemetry,
not by `COMMAND_ACK` alone. The DataFlash file was still open while SITL
continued running, so no mutable-file hash is recorded here.

## Software Gates

- Full Lite test suite: `155 passed`.
- Camera and browser focused tests: `13 passed`.
- Python `compileall`: passed.
- flake8: passed.
- JavaScript syntax check: passed.
- `git diff --check`: passed.

## Accepted Fixes

- Browser TAKEOFF now performs GUIDED, conditional ARM and TAKEOFF as one
  evidence-tracked onboard transaction.
- AirSim Scene image capture runs in a timeout-isolated worker, caches fresh
  frames and automatically reconnects.
- The Web camera panel polls dynamic camera status and displays cached frames
  at about 5 FPS.
- Camera failure remains isolated from MAVLink control and telemetry.

## Boundary

M1.5 does not authorize real-aircraft flight. It does not validate a physical
RGB camera, D435/depth sensing, VLM inference, path planning, formation flight
or multi-vehicle operation. M2 begins with a propeller-removed CUAV V5+ bench.
