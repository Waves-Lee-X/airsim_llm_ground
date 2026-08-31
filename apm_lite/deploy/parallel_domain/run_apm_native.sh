#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${1:?runtime directory is required}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/ardupilot}"
ARDUCOPTER="${ARDUCOPTER:-$ARDUPILOT_ROOT/build/sitl/bin/arducopter}"
SITL_PARAMS="$SCRIPT_DIR/../../configs/parallel_domain/apm-native-sitl.parm"
mkdir -p "$RUNTIME_DIR"
test -x "$ARDUCOPTER" || { echo "ArduCopter binary not found: $ARDUCOPTER" >&2; exit 2; }

cd "$RUNTIME_DIR"
exec "$ARDUCOPTER" --wipe --model + --speedup "${SITL_SPEEDUP:-10}" \
  --defaults "$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm,$SITL_PARAMS" \
  --sim-address 127.0.0.1 \
  --home 47.641468,-122.140165,122,0 --instance 20 \
  --serial0 udpclient:127.0.0.1:14550 --sysid 2 >sitl.log 2>&1
