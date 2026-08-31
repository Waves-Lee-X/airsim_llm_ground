#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${1:?runtime directory is required}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_BUILD="${PX4_BUILD:-$PX4_ROOT/build/px4_sitl_default}"
MODEL="${PX4_MODEL:-iris}"
WORLD="${PX4_WORLD:-empty}"
# Gazebo classic otherwise falls back to PX4's Zurich origin, which would turn
# a valid local trajectory into an ECEF-scale displacement in the shared map.
export PX4_HOME_LAT="${PX4_HOME_LAT:-47.641468}"
export PX4_HOME_LON="${PX4_HOME_LON:--122.140165}"
export PX4_HOME_ALT="${PX4_HOME_ALT:-122.0}"

mkdir -p "$RUNTIME_DIR"
test -x "$PX4_BUILD/bin/px4" || { echo "PX4 binary not found: $PX4_BUILD/bin/px4" >&2; exit 2; }
test -x "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" || {
  echo "PX4 Gazebo classic tree is incomplete: $PX4_ROOT" >&2; exit 2;
}

# This SDF is generated in the runtime directory.  The QGC fan-out is kept
# away from APM's required 14550 endpoint; PX4's own onboard stream is 14540.
# PX4's helper appends to these variables and assumes an interactive shell set
# them already; initialize them for tmux/headless launches.
GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH LD_LIBRARY_PATH
# shellcheck disable=SC1091 # path is provided by PX4_ROOT at runtime
source "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_ROOT" "$PX4_BUILD"
SDF_TMP="$(mktemp --suffix=.sdf "$RUNTIME_DIR/.${MODEL}.XXXXXX")"
cleanup() {
  set +e
  [[ -n "${SDF_TMP:-}" ]] && rm -f "$SDF_TMP"
  [[ -n "${PX4_PID:-}" ]] && kill "$PX4_PID" 2>/dev/null
  [[ -n "${GZ_PID:-}" ]] && kill "$GZ_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM
python3 "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" \
  "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/$MODEL/$MODEL.sdf.jinja" \
  "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
  --mavlink_tcp_port 4560 --mavlink_udp_port 14560 --mavlink_id 1 \
  --output-file "$SDF_TMP"
sed -i 's#<qgc_udp_port>14550</qgc_udp_port>#<qgc_udp_port>14640</qgc_udp_port>#' "$SDF_TMP"
mv -f "$SDF_TMP" "$RUNTIME_DIR/$MODEL.sdf"
SDF_TMP=""
mkdir -p "$RUNTIME_DIR/init"
cp "$PX4_BUILD/etc/init.d-posix/px4-rc.mavlink" "$RUNTIME_DIR/init/px4-rc.mavlink"
# PX4's normal GCS channel defaults to remote UDP 14550. Keep it on the
# isolated PX4 auxiliary port so APM's required 14550 never receives PX4 data.
# shellcheck disable=SC2016
sed -i 's#mavlink start -x -u $udp_gcs_port_local -r 4000000 -f #mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -o 14640 #' \
  "$RUNTIME_DIR/init/px4-rc.mavlink"

gzserver "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/$WORLD.world" \
  >"$RUNTIME_DIR/gazebo.log" 2>&1 &
GZ_PID=$!
for _ in $(seq 1 30); do
  gz stats -d 1 >/dev/null 2>&1 && break
  sleep 1
done
gz stats -d 1 >/dev/null 2>&1 || { echo "Gazebo did not become ready" >&2; exit 3; }

cd "$RUNTIME_DIR"
PATH="$RUNTIME_DIR/init:$PATH" PX4_SIM_MODEL="gazebo-classic_$MODEL" PX4_SYS_AUTOSTART=10015 \
  "$PX4_BUILD/bin/px4" -i 0 -d "$PX4_BUILD/etc" >px4.log 2>&1 &
PX4_PID=$!
sleep 2
for _ in $(seq 1 20); do
  gz model --spawn-file="$RUNTIME_DIR/$MODEL.sdf" --model-name=px4_real_1 -x 0 -y 0 -z 0.83 \
    >>"$RUNTIME_DIR/gazebo.log" 2>&1 && break
  sleep 1
done
gz model -m px4_real_1 -i >/dev/null 2>&1 || {
  echo "PX4 Gazebo model was not spawned" >&2; exit 4;
}
wait "$PX4_PID"
