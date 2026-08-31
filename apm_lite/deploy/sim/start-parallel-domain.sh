#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG_PATH="${1:-${ROOT_DIR}/configs/parallel_domain/bridge.yaml}"
export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

command -v python3 >/dev/null || {
  echo "python3 is required" >&2
  exit 2
}
python3 -c 'import pymavlink, pydantic, yaml' || {
  echo "install the ground extras: python3 -m pip install -e .[ground]" >&2
  exit 2
}

exec python3 -m aeromind_apm_lite.ground.parallel_domain.cli serve \
  --config "${CONFIG_PATH}"
