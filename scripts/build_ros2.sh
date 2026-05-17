#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${REPO_ROOT}/ros2_ws"
VENV_PYTHON="${REPO_ROOT}/.venv/bin/python"
VENV_ACTIVATE="${REPO_ROOT}/.venv/bin/activate"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "error: ${VENV_PYTHON} does not exist." >&2
  echo "create the repo virtualenv first:" >&2
  echo "  uv venv --python /usr/bin/python3 --system-site-packages .venv" >&2
  echo "  uv sync" >&2
  exit 1
fi

if [[ ! -f "${VENV_ACTIVATE}" ]]; then
  echo "error: ${VENV_ACTIVATE} does not exist." >&2
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

# The ament_python entry points inherit the interpreter used during colcon build.
# Building through this script avoids silently generating /usr/bin/python3 shebangs.
source "${VENV_ACTIVATE}"
set +u
source "${ROS_SETUP}"
set -u

cd "${WORKSPACE_DIR}"
echo "Using Python interpreter: ${VENV_PYTHON}"
"${VENV_PYTHON}" -m colcon build \
  --symlink-install \
  "$@" \
  --cmake-args -DPython3_EXECUTABLE="${VENV_PYTHON}"

echo
echo "Build completed."
echo "Next step:"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
