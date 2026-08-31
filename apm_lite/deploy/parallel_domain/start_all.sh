#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUNTIME_ROOT="${PD_RUNTIME_DIR:-$HOME/.aeromind/parallel-domain}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_BUILD="${PX4_BUILD:-$PX4_ROOT/build/px4_sitl_default}"
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/ardupilot}"
ARDUCOPTER="${ARDUCOPTER:-$ARDUPILOT_ROOT/build/sitl/bin/arducopter}"
LINE=all
WORLD=all
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: start_all.sh [--line all|px4|apm] [--world all|real|virtual] [--dry-run]

Starts isolated tmux sessions. AirSim/UE is intentionally not guessed or merged:
start the two UE worlds with the generated settings files shown by this script.
Environment overrides: PX4_ROOT PX4_BUILD ARDUPILOT_ROOT ARDUCOPTER AIRSIM_HOST
PD_RUNTIME_DIR SITL_SPEEDUP.
EOF
}

while (($#)); do
  case "$1" in
    --line) LINE="${2:?--line needs a value}"; shift 2 ;;
    --world) WORLD="${2:?--world needs a value}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
case "$LINE" in all|px4|apm) ;; *) echo "--line must be all, px4, or apm" >&2; exit 2 ;; esac
case "$WORLD" in all|real|virtual) ;; *) echo "--world must be all, real, or virtual" >&2; exit 2 ;; esac

pick_ipv4() {
  hostname -I 2>/dev/null | tr ' ' '\n' | awk '/^[0-9]+\.[0-9]+\./ {print; exit}'
}
WSL_IPV4="${WSL_IPV4:-$(pick_ipv4 || true)}"
WSL_IPV4="${WSL_IPV4:-127.0.0.1}"
AIRSIM_HOST="${AIRSIM_HOST:-$(ip route show default 2>/dev/null | awk '{print $3; exit}' || true)}"
AIRSIM_HOST="${AIRSIM_HOST:-$(awk '/^nameserver[[:space:]]/{print $2; exit}' /etc/resolv.conf 2>/dev/null || true)}"
AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"

need_command() {
  command -v "$1" >/dev/null 2>&1 || { echo "required command not found: $1" >&2; exit 2; }
}
need_file() {
  test -e "$1" || { echo "required path not found: $1" >&2; exit 2; }
}
selected() {
  local wanted_line=$1 wanted_world=$2
  [[ "$LINE" == all || "$LINE" == "$wanted_line" ]] || return 1
  [[ "$WORLD" == all || "$WORLD" == "$wanted_world" ]]
}

PX4_REAL_DIR="$RUNTIME_ROOT/px4-real"
PX4_VIRTUAL_DIR="$RUNTIME_ROOT/px4-virtual"
APM_REAL_DIR="$RUNTIME_ROOT/apm-real"
APM_VIRTUAL_DIR="$RUNTIME_ROOT/apm-virtual"
mkdir -p "$RUNTIME_ROOT"

if ((DRY_RUN == 0)); then
  need_command tmux
  if selected px4 real || selected px4 virtual; then
    need_file "$PX4_BUILD/bin/px4"
    need_file "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash"
  fi
  if selected apm real || selected apm virtual; then need_file "$ARDUCOPTER"; fi
fi

mkdir -p "$RUNTIME_ROOT/airsim"
sed -e "s/__WSL_IPV4__/$WSL_IPV4/g" -e "s/__AIRSIM_HOST__/$AIRSIM_HOST/g" \
  "$PROJECT_DIR/configs/parallel_domain/airsim/px4.settings.json" \
  >"$RUNTIME_ROOT/airsim/px4.settings.json"
sed -e "s/__WSL_IPV4__/$WSL_IPV4/g" -e "s/__AIRSIM_HOST__/$AIRSIM_HOST/g" \
  "$PROJECT_DIR/configs/parallel_domain/airsim/apm.settings.json" \
  >"$RUNTIME_ROOT/airsim/apm.settings.json"

echo "parallel-domain runtime: $RUNTIME_ROOT"
echo "WSL IPv4: $WSL_IPV4; AirSim/Windows host: $AIRSIM_HOST"
echo "AirSim settings: $RUNTIME_ROOT/airsim/px4.settings.json and $RUNTIME_ROOT/airsim/apm.settings.json"

launch() {
  local session=$1 helper=$2 dir=$3
  mkdir -p "$dir"
  local command
  printf -v command '%q ' "$helper" "$dir"
  if [[ -n "${4:-}" ]]; then
    local extra_arg
    printf -v extra_arg '%q ' "$4"
    command+="$extra_arg"
  fi
  if ((DRY_RUN)); then
    echo "DRY-RUN tmux $session: $command"
    return
  fi
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "tmux session already exists: $session (stop it first)" >&2
    exit 3
  fi
  tmux new-session -d -s "$session" -c "$dir" "$command"
  echo "started $session"
}

if selected px4 real; then
  launch pd-px4-real "$SCRIPT_DIR/run_px4_gazebo.sh" "$PX4_REAL_DIR"
fi
if selected apm real; then
  launch pd-apm-real "$SCRIPT_DIR/run_apm_native.sh" "$APM_REAL_DIR"
fi
if selected px4 virtual; then
  launch pd-px4-virtual "$SCRIPT_DIR/run_px4_airsim.sh" "$PX4_VIRTUAL_DIR" "$AIRSIM_HOST"
fi
if selected apm virtual; then
  launch pd-apm-virtual "$SCRIPT_DIR/run_apm_airsim.sh" "$APM_VIRTUAL_DIR" "$AIRSIM_HOST"
fi

if ((DRY_RUN == 0)); then
  sleep 2
  echo "SITL processes are in tmux; attach with: tmux attach -t pd-px4-real"
fi
cat <<EOF
Next: start each UE/AirSim world using its own project/profile and settings file.
PX4 AirSim TCP: $WSL_IPV4:4561; APM AirSim UDP sensor/control: $WSL_IPV4:9003/$WSL_IPV4:9002.
The ground bridge must bind only after the SITLs are up: deploy/sim/start-parallel-domain.sh.
EOF
