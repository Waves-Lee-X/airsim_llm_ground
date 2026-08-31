#!/usr/bin/env bash
set -euo pipefail

# Build one immutable live package.  Verification is deliberately a separate
# command (`package_parallel_domain_live.py verify`) so a copied package can be
# checked without trusting this launcher.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MISSION_ID="${1:?usage: $0 MISSION_ID LINE OUTPUT_ZIP [STATE_DIRECTORY]}"
LINE="${2:?usage: $0 MISSION_ID LINE OUTPUT_ZIP [STATE_DIRECTORY]}"
OUTPUT_ZIP="${3:?usage: $0 MISSION_ID LINE OUTPUT_ZIP [STATE_DIRECTORY]}"
STATE_DIRECTORY="${4:-${ROOT_DIR}/.runtime/parallel-domain-control}"

case "${LINE}" in
  px4)
    FLIGHT_CONTROLLER="PX4-SITL"
    SIMULATOR="Gazebo-Classic"
    CALIBRATION_MEASUREMENTS="${PD_CALIBRATION_MEASUREMENTS:-${ROOT_DIR}/configs/parallel_domain/calibration/sitl-ground-truth-roundtrip.json}"
    ;;
  apm)
    FLIGHT_CONTROLLER="ArduCopter-SITL"
    SIMULATOR="Gazebo-or-native-SITL"
    CALIBRATION_MEASUREMENTS="${PD_CALIBRATION_MEASUREMENTS:-${ROOT_DIR}/configs/parallel_domain/calibration/apm-sitl-ground-truth-roundtrip.json}"
    ;;
  *)
    echo "LINE must be px4 or apm" >&2
    exit 2
    ;;
esac

export PYTHONPATH="${ROOT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
CALIBRATION_DIR="${PD_CALIBRATION_DIR:-${ROOT_DIR}/.runtime/parallel-domain-live-evidence-calibration/$(date -u +%Y%m%dT%H%M%SZ)-$$}"
mkdir -p "${CALIBRATION_DIR}"

python3 "${ROOT_DIR}/tools/calibrate_parallel_domain.py" freeze \
  --draft "${ROOT_DIR}/configs/parallel_domain/sitl-georeference-draft.yaml" \
  --measurements "${CALIBRATION_MEASUREMENTS}" \
  --output-georeference "${CALIBRATION_DIR}/georeference.yaml" \
  --output-report "${CALIBRATION_DIR}/report.json" \
  --output-receipt "${CALIBRATION_DIR}/receipt.json"

python3 "${ROOT_DIR}/tools/package_parallel_domain_live.py" build \
  --state-directory "${STATE_DIRECTORY}" \
  --mission-id "${MISSION_ID}" \
  --georeference "${CALIBRATION_DIR}/georeference.yaml" \
  --calibration-measurements "${CALIBRATION_MEASUREMENTS}" \
  --calibration-report "${CALIBRATION_DIR}/report.json" \
  --calibration-receipt "${CALIBRATION_DIR}/receipt.json" \
  --output "${OUTPUT_ZIP}" \
  --software-root "${ROOT_DIR}" \
  --random-seed "${PD_RANDOM_SEED:-20260817}" \
  --version "aeromind_apm_lite=${PD_AEROMIND_VERSION:-working-tree}" \
  --version "bridge=${PD_BRIDGE_VERSION:-parallel-domain-v1}" \
  --version "flight_controller=${FLIGHT_CONTROLLER}" \
  --version "simulator=${SIMULATOR}" \
  --version "world=${PD_WORLD_VERSION:-parallel-sitl}"

echo "built ${OUTPUT_ZIP}"
echo "verify with: PYTHONPATH=${ROOT_DIR}/src python3 ${ROOT_DIR}/tools/package_parallel_domain_live.py verify ${OUTPUT_ZIP}"
