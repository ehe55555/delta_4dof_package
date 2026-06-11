#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
LOG_FILE="${ROOT_DIR}/last_run_delta_4dof.log"

DO_RUN=1
FORCE_BUILD=0
INSTALL_DEPS=0
INSTALL_DESKTOP=1
CHECK_ONLY=0
PACKAGE_DIR=""

usage() {
  cat <<'EOF'
Usage: ./RUN_DELTA_4DOF.sh [options]

Default: check dependencies, build when needed, create Desktop shortcut, run.

Options:
  --check                 Only check the environment.
  --build                 Force a colcon build, then run.
  --rebuild               Remove build/install/log, rebuild, then run.
  --install-deps          Install missing apt packages.
  --install-desktop       Create/update the Desktop shortcut.
  --no-desktop            Do not create the Desktop shortcut.
  --no-run                Check/build/setup only.
  --package [directory]   Create a clean portable source folder and do not run.
  -h, --help              Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --check)
      CHECK_ONLY=1
      DO_RUN=0
      ;;
    --build)
      FORCE_BUILD=1
      ;;
    --rebuild)
      FORCE_BUILD=2
      ;;
    --install-deps)
      INSTALL_DEPS=1
      ;;
    --install-desktop)
      INSTALL_DESKTOP=1
      ;;
    --no-desktop)
      INSTALL_DESKTOP=0
      ;;
    --no-run)
      DO_RUN=0
      ;;
    --package)
      DO_RUN=0
      INSTALL_DESKTOP=0
      if (($# > 1)) && [[ "${2}" != --* ]]; then
        PACKAGE_DIR="$2"
        shift
      else
        PACKAGE_DIR="${ROOT_DIR}_package"
      fi
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
  shift
done

exec > >(tee "${LOG_FILE}") 2>&1

echo "========================================"
echo "          DELTA 4DOF LAUNCHER"
echo "========================================"
echo "Workspace: ${ROOT_DIR}"

if [[ -n "${PACKAGE_DIR}" ]]; then
  PACKAGE_DIR="$(realpath -m "${PACKAGE_DIR}")"
  if [[ "${PACKAGE_DIR}" == "${ROOT_DIR}" || "${PACKAGE_DIR}" == "${ROOT_DIR}/"* ]]; then
    echo "[ERROR] Package output must be outside the workspace folder."
    exit 1
  fi
  if [[ -d "${PACKAGE_DIR}" ]] &&
      find "${PACKAGE_DIR}" -mindepth 1 -print -quit | grep -q .; then
    echo "[ERROR] Package output is not empty: ${PACKAGE_DIR}"
    exit 1
  fi

  echo "[INFO] Creating clean package folder: ${PACKAGE_DIR}"
  mkdir -p "${PACKAGE_DIR}"
  tar -C "${ROOT_DIR}" -cf - \
    src \
    README.md \
    RUN_DELTA_4DOF.sh \
    RUN_DELTA_4DOF.desktop \
    run_delta_control.sh \
    run_gazebo.sh |
    tar -C "${PACKAGE_DIR}" -xf -
  chmod +x \
    "${PACKAGE_DIR}/RUN_DELTA_4DOF.sh" \
    "${PACKAGE_DIR}/run_delta_control.sh" \
    "${PACKAGE_DIR}/run_gazebo.sh"
  echo "[OK] Portable source package created at ${PACKAGE_DIR}"
  echo "     First run: ./RUN_DELTA_4DOF.sh --install-deps"
  exit 0
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] ROS 2 Jazzy was not found at ${ROS_SETUP}."
  exit 1
fi

# ROS setup scripts may read unset variables.
set +u
source "${ROS_SETUP}"
set -u

APT_PACKAGES=(
  python3-colcon-common-extensions
  python3-tk
  python3-numpy
  python3-scipy
  python3-matplotlib
  ros-jazzy-ros-gz
  gz-harmonic
  libgz-sim8-dev
  libgz-plugin2-dev
  libgz-transport13-dev
  libgz-msgs10-dev
)

missing_packages=()
for package in "${APT_PACKAGES[@]}"; do
  if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null |
      grep -q "install ok installed"; then
    missing_packages+=("${package}")
  fi
done

missing_commands=()
for command in colcon ros2 gz python3; do
  command -v "${command}" >/dev/null 2>&1 || missing_commands+=("${command}")
done

if ((${#missing_packages[@]})); then
  echo "[WARN] Missing apt packages:"
  printf '  - %s\n' "${missing_packages[@]}"
  if ((INSTALL_DEPS)); then
    sudo apt-get update
    sudo apt-get install -y "${missing_packages[@]}"
  else
    echo "[ERROR] Run again with --install-deps to install them."
    exit 1
  fi
fi

if ((${#missing_commands[@]})); then
  echo "[ERROR] Missing commands: ${missing_commands[*]}"
  exit 1
fi

MPLCONFIGDIR="${TMPDIR:-/tmp}/delta_4dof-matplotlib-${UID}" python3 - <<'PY'
import tkinter
import matplotlib
import numpy
import scipy
print("[OK] Python GUI/scientific modules are available.")
PY

echo "[OK] ROS 2, Gazebo, colcon and Python dependencies are available."

if ((CHECK_ONLY)); then
  exit 0
fi

cd "${ROOT_DIR}"

if ((FORCE_BUILD == 2)); then
  echo "[INFO] Removing old build outputs..."
  rm -rf build install log
fi

NEED_BUILD=0
if ((FORCE_BUILD > 0)) || [[ ! -f install/setup.bash ]]; then
  NEED_BUILD=1
elif find src -type f -newer install/setup.bash -print -quit | grep -q .; then
  NEED_BUILD=1
fi

if ((NEED_BUILD)); then
  echo "[INFO] Building ROS 2 workspace..."
  colcon build --symlink-install --event-handlers console_direct+
else
  echo "[OK] Build is current."
fi

if [[ ! -f install/setup.bash ]]; then
  echo "[ERROR] Build did not create install/setup.bash."
  exit 1
fi

chmod +x \
  "${ROOT_DIR}/RUN_DELTA_4DOF.sh" \
  "${ROOT_DIR}/run_delta_control.sh" \
  "${ROOT_DIR}/run_gazebo.sh"

if ((INSTALL_DESKTOP)); then
  if command -v xdg-user-dir >/dev/null 2>&1; then
    DESKTOP_DIR="$(xdg-user-dir DESKTOP)"
  else
    DESKTOP_DIR="${HOME}/Desktop"
  fi
  DESKTOP_FILE="${DESKTOP_DIR}/Delta_4DOF_Control.desktop"
  mkdir -p "${DESKTOP_DIR}"

  cat > "${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Delta 4DOF Control
Comment=Check, build and run Delta 4DOF
Path=${ROOT_DIR}
Exec="${ROOT_DIR}/RUN_DELTA_4DOF.sh"
Icon=utilities-terminal
Terminal=true
Categories=Development;Robotics;
StartupNotify=true
EOF
  chmod +x "${DESKTOP_FILE}"
  gio set "${DESKTOP_FILE}" metadata::trusted true 2>/dev/null || true
  echo "[OK] Desktop shortcut: ${DESKTOP_FILE}"
fi

if ((DO_RUN)); then
  echo "[INFO] Starting Gazebo and Delta control GUI..."
  exec "${ROOT_DIR}/run_delta_control.sh"
fi

echo "[OK] Setup completed."
