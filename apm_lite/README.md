# AeroMind-APM Lite

AeroMind-APM Lite is the non-ROS implementation described in
[`doc/14-AeroMind-APM-Lite开发计划.md`](../doc/14-AeroMind-APM-Lite开发计划.md).
The existing ROS 2/PX4 packages under `src/` remain a separate prototype.

## M0 scope

The current milestone establishes:

- versioned wire-message contracts;
- TTL, session, sequence and replay validation;
- a monotonic mission state machine;
- coordinate and sensor-source primitives;
- simulation and real fleet configuration validation;
- generated JSON Schema and dependency-boundary tests.

It does not connect to ArduPilot yet. The first `pymavlink`/SITL connection is
milestone M1.

The reviewed V5+/Raspberry Pi wiring facts and unresolved bench checks are in
[`docs/HARDWARE_BASELINE.md`](docs/HARDWARE_BASELINE.md). Do not power the
Raspberry Pi from a TELEM connector without a verified current rating.

## Local verification

```bash
cd ~/aeromind_ws/apm_lite
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest
python3 tools/export_schemas.py --check
```

The package must remain independent from `rclpy`, `px4_msgs` and MAVROS.
