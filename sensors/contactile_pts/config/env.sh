#!/bin/bash
# Contactile PTS Lab 环境变量
# 使用: source config/env.sh

# 项目根目录 (monorepo top)
export TACTILE_GRASP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# C++ SDK 路径
export CONTACTILE_CPP_INCLUDE="${TACTILE_GRASP_ROOT}/vendor/contactile_pts/C++LIN/Include"
export CONTACTILE_CPP_LIB="${TACTILE_GRASP_ROOT}/vendor/contactile_pts/C++LIN/Library"

# Python 虚拟环境 (如果存在)
if [ -d "${TACTILE_GRASP_ROOT}/sensors/contactile_pts/python/.venv" ]; then
    export VIRTUAL_ENV="${TACTILE_GRASP_ROOT}/sensors/contactile_pts/python/.venv"
    export PATH="${VIRTUAL_ENV}/bin:${PATH}"
fi

# ROS2 工作空间 (如果已编译)
if [ -f "${TACTILE_GRASP_ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${TACTILE_GRASP_ROOT}/ros2_ws/install/setup.bash"
fi

echo "[env.sh] Contactile PTS Lab 环境已加载"
echo "  ROOT: ${CONTACTILE_PTS_ROOT}"
echo "  C++ Include: ${CONTACTILE_CPP_INCLUDE}"
echo "  C++ Lib: ${CONTACTILE_CPP_LIB}"
