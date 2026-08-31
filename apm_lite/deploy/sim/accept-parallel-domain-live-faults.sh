#!/usr/bin/env bash
set -euo pipefail

# Re-run the live negative paths without stopping any of the four SITL/UE
# sessions.  Only the resident ground bridge is swapped for the strict,
# SITL-only acceptance profile.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LIVE_CONFIG="${1:-${ROOT_DIR}/configs/parallel_domain/bridge-live-faults.yaml}"
NORMAL_CONFIG="${NORMAL_CONFIG:-${ROOT_DIR}/configs/parallel_domain/bridge-control.yaml}"
LIVE_SESSION="${PD_LIVE_FAULTS_SESSION:-pd-live-faults}"
NORMAL_SESSION="${PD_CONTROL_SESSION:-pd-control}"
LIVE_STATE="${ROOT_DIR}/.runtime/parallel-domain-live-faults"
HISTORY_ROOT="${ROOT_DIR}/.runtime/parallel-domain-live-faults-history"
export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

command -v tmux >/dev/null 2>&1 || {
  echo "tmux is required; start the four SITL/UE sessions first" >&2
  exit 2
}
[[ -f "${LIVE_CONFIG}" ]] || { echo "live config not found: ${LIVE_CONFIG}" >&2; exit 2; }
[[ -f "${NORMAL_CONFIG}" ]] || { echo "normal config not found: ${NORMAL_CONFIG}" >&2; exit 2; }

kill_session() {
  local session=$1
  if tmux has-session -t "${session}" 2>/dev/null; then
    tmux kill-session -t "${session}"
  fi
}

start_bridge() {
  local session=$1 config=$2
  tmux new-session -d -s "${session}" -c "${ROOT_DIR}" \
    "exec ${ROOT_DIR}/deploy/sim/start-parallel-control.sh ${config}"
}

restore_normal=1
cleanup() {
  local exit_code=$?
  set +e
  kill_session "${LIVE_SESSION}"
  if ((restore_normal)); then
    # Do not touch the four SITL/UE sessions.  A pre-existing normal bridge
    # was stopped above and is recreated with its production TTL (600 s).
    if ! tmux has-session -t "${NORMAL_SESSION}" 2>/dev/null; then
      start_bridge "${NORMAL_SESSION}" "${NORMAL_CONFIG}"
    fi
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

kill_session "${LIVE_SESSION}"
kill_session "${NORMAL_SESSION}"
if [[ -d "${LIVE_STATE}" ]]; then
  mkdir -p "${HISTORY_ROOT}"
  archive_path="${HISTORY_ROOT}/$(date -u +%Y%m%dT%H%M%SZ)-$$"
  mv -- "${LIVE_STATE}" "${archive_path}"
  echo "archived previous live-fault run: ${archive_path}"
fi
start_bridge "${LIVE_SESSION}" "${LIVE_CONFIG}"

audit_path="${LIVE_STATE}/audit/events.jsonl"
deadline=$((SECONDS + 20))
while ((SECONDS < deadline)); do
  if [[ -f "${audit_path}" ]] && grep -q '"event_type":"bridge_started"' "${audit_path}"; then
    break
  fi
  sleep 0.25
done
if [[ ! -f "${audit_path}" ]] || ! grep -q '"event_type":"bridge_started"' "${audit_path}"; then
  echo "live-fault bridge did not start; inspect tmux session ${LIVE_SESSION}" >&2
  exit 1
fi

python3 "${ROOT_DIR}/tools/accept_parallel_domain_live_faults.py" \
  --config "${LIVE_CONFIG}" \
  --output "${LIVE_STATE}/live-fault-acceptance.json" \
  --events-output "${LIVE_STATE}/live-fault-acceptance-events.jsonl"
