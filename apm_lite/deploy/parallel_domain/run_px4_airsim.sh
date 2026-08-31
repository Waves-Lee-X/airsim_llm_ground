#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${1:?runtime directory is required}"
AIRSIM_HOST="${2:-${AIRSIM_HOST:-}}"
AIRSIM_HOST="${AIRSIM_HOST:?set AIRSIM_HOST to the Windows host IPv4 address}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_BUILD="${PX4_BUILD:-$PX4_ROOT/build/px4_sitl_default}"
mkdir -p "$RUNTIME_DIR/init"
test -x "$PX4_BUILD/bin/px4" || { echo "PX4 binary not found: $PX4_BUILD/bin/px4" >&2; exit 2; }

# Keep PX4 SYSID=1 (instance 0) while moving all local sockets, the AirSim
# simulator TCP listener, and the remote onboard stream off the Gazebo pair.
cp "$PX4_BUILD/etc/init.d-posix/px4-rc.mavlink" "$RUNTIME_DIR/init/px4-rc.mavlink"
cp "$PX4_BUILD/etc/init.d-posix/px4-rc.mavlinksim" "$RUNTIME_DIR/init/px4-rc.mavlinksim"
cp "$PX4_BUILD/etc/init.d-posix/rcS" "$RUNTIME_DIR/rcS"
# PX4's internal instance index is deliberately 1 so it can coexist with the
# Gazebo PX4 process.  The external MAV_SYS_ID remains the line's vehicle_id 1.
sed -i 's#param set MAV_SYS_ID .*#param set MAV_SYS_ID 1#' "$RUNTIME_DIR/rcS"
# The arithmetic expressions below are literal text from PX4's init script.
# shellcheck disable=SC2016
sed -i \
  -e 's/udp_offboard_port_local=$((14580+px4_instance))/udp_offboard_port_local=14581/' \
  -e 's/udp_offboard_port_remote=$((14540+px4_instance))/udp_offboard_port_remote=14541/' \
  -e 's/udp_gcs_port_local=$((18570+px4_instance))/udp_gcs_port_local=18571/' \
  -e 's/udp_onboard_payload_port_local=$((14280+px4_instance))/udp_onboard_payload_port_local=14281/' \
  -e 's/udp_onboard_gimbal_port_local=$((13030+px4_instance))/udp_onboard_gimbal_port_local=13031/' \
  -e 's#mavlink start -x -u $udp_gcs_port_local -r 4000000 -f #mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -o 14541 #' \
  "$RUNTIME_DIR/init/px4-rc.mavlink"
# shellcheck disable=SC2016
sed -i 's/simulator_tcp_port=$((4560+px4_instance))/simulator_tcp_port=4561/' \
  "$RUNTIME_DIR/init/px4-rc.mavlinksim"

cd "$RUNTIME_DIR"
PATH="$RUNTIME_DIR/init:$PATH" PX4_SIM_MODEL=none_iris PX4_SYS_AUTOSTART=10016 \
  PX4_SIM_HOST_ADDR="$AIRSIM_HOST" "$PX4_BUILD/bin/px4" -i 1 -s "$RUNTIME_DIR/rcS" -d "$PX4_BUILD/etc" \
  >px4.log 2>&1
