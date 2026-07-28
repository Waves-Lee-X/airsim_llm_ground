# AeroMind-APM Lite

AeroMind-APM Lite is the non-ROS implementation described in
[`doc/14-AeroMind-APM-Lite开发计划.md`](../doc/14-AeroMind-APM-Lite开发计划.md).
The existing ROS 2/PX4 packages under `src/` remain a separate prototype.

## Milestone status

M0 established:

- versioned wire-message contracts;
- TTL, session, sequence and replay validation;
- a monotonic mission state machine;
- coordinate and sensor-source primitives;
- simulation and real fleet configuration validation;
- generated JSON Schema and dependency-boundary tests.

M1 is accepted. The implementation includes the transport-neutral fake FCU, a
single-owner `ApmLink`, the real `pymavlink` serial/UDP adapter, FCU identity
discovery, normalized telemetry, command evidence tracking, safety priority,
authenticated WebSocket communication and deterministic AirSim/ArduPilot
SITL lifecycle management. The accepted AirSim 1.8.1 / ArduCopter 4.7.0 batch
completed 10 of 10 fixed missions plus the independent RTL path; see
[`docs/M1_PROGRESS.md`](docs/M1_PROGRESS.md).

M2 is the next gate. It begins with a propeller-removed CUAV V5+ bench and
read-only discovery. No real flight is authorized by the M1 result.

The reviewed V5+/Raspberry Pi wiring facts and unresolved bench checks are in
[`docs/HARDWARE_BASELINE.md`](docs/HARDWARE_BASELINE.md). Do not power the
Raspberry Pi from a TELEM connector without a verified current rating. The
reviewed source does not prove that an RGB camera, D435, depth camera or lidar
is installed; sensor-dependent features remain gated on a physical inventory.

Install the FCU adapter with the onboard extra:

```bash
python3 -m pip install -e ".[onboard]"
```

Read a SITL or propeller-removed bench FCU identity without sending any flight
command:

```bash
aeromind-apm-probe \
  --config configs/sim/fleet.yaml \
  --vehicle-id 1 \
  --observe-seconds 2
```

The probe waits for the configured FCU heartbeat and requests
`AUTOPILOT_VERSION`. It never arms, changes mode or sends a position target.

## Local verification

```bash
cd ~/aeromind_ws/apm_lite
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest
python3 tools/export_schemas.py --check
python3 -m flake8 src tests tools --max-line-length=101 --extend-ignore=E203,W503
```

The package must remain independent from `rclpy`, `px4_msgs` and MAVROS.
