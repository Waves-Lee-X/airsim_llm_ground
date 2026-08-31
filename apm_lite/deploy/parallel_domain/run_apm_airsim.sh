#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${1:?runtime directory is required}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/ardupilot}"
ARDUCOPTER="${ARDUCOPTER:-$ARDUPILOT_ROOT/build/sitl/bin/arducopter}"
AIRSIM_HOST="${2:-${AIRSIM_HOST:-}}"
AIRSIM_HOST="${AIRSIM_HOST:?set AIRSIM_HOST to the Windows host IPv4 address}"
mkdir -p "$RUNTIME_DIR"
test -x "$ARDUCOPTER" || { echo "ArduCopter binary not found: $ARDUCOPTER" >&2; exit 2; }

cd "$RUNTIME_DIR"
exec "$ARDUCOPTER" --wipe --model airsim-copter --speedup "${SITL_SPEEDUP:-10}" \
  --defaults "$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm,$ARDUPILOT_ROOT/Tools/autotest/default_params/airsim-quadX.parm,$SCRIPT_DIR/../../configs/parallel_domain/apm-native-sitl.parm" \
  --sim-address "$AIRSIM_HOST" --home 47.641468,-122.140165,122,0 --instance 0 \
  --serial0 udpclient:127.0.0.1:14551 --sysid 2 >sitl.log 2>&1
