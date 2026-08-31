#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
OUTPUT_PATH="${1:-${ROOT_DIR}/.runtime/parallel-domain/acceptance.json}"

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  "${ROOT_DIR}/tests/test_parallel_domain.py" \
  "${ROOT_DIR}/tests/test_parallel_domain_live.py"
python3 "${ROOT_DIR}/tools/accept_parallel_domain.py" --output "${OUTPUT_PATH}"
