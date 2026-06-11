#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="${ROOT_DIR}/src/descripe/models"
SOURCE_DIR="${ROOT_DIR}/src"
WORLD_FILE="${ROOT_DIR}/src/descripe/worlds/descripe_test.world"

export GZ_SIM_RESOURCE_PATH="${MODEL_DIR}:${SOURCE_DIR}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

exec gz sim -v 4 "${WORLD_FILE}"
