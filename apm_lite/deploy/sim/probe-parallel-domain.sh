#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG_PATH="${1:-${ROOT_DIR}/configs/parallel_domain/bridge.yaml}"
export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 "${ROOT_DIR}/tools/probe_parallel_domain.py" --config "${CONFIG_PATH}"
