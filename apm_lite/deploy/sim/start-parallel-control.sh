#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG_PATH="${1:-${ROOT_DIR}/configs/parallel_domain/bridge-control.yaml}"
export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

python3 - "$CONFIG_PATH" <<'PY'
import sys
from aeromind_apm_lite.ground.parallel_domain.config import load_bridge_config

config = load_bridge_config(sys.argv[1])
if config.observation_only:
    raise SystemExit("control launcher refuses an observation_only config")
print(f"control state directory: {config.state_directory}")
PY

exec python3 -m aeromind_apm_lite.ground.parallel_domain.cli serve \
  --config "$CONFIG_PATH"
