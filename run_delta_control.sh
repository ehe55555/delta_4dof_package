#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[ERROR] ROS 2 Jazzy is not installed." >&2
  exit 1
fi
if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] Workspace is not built. Run ./RUN_DELTA_4DOF.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

export DELTA_4DOF_ROOT="${ROOT_DIR}"
exec ros2 launch delta_control delta_control.launch.py
