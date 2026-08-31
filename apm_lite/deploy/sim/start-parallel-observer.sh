#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG_PATH="${1:-${ROOT_DIR}/configs/parallel_domain/bridge.yaml}"
export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 -m aeromind_apm_lite.ground.parallel_domain.cli observe \
  --config "$CONFIG_PATH" \
  --host "${PD_OBSERVER_HOST:-127.0.0.1}" \
  --port "${PD_OBSERVER_PORT:-8091}" \
  --max-points "${PD_OBSERVER_MAX_POINTS:-1000}"
