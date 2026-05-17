#!/bin/bash
# 运行 ROS2 Contactile 驱动 (3DFBS)
# 用法: bash sensors/contactile_3dfbs/scripts/run_ros2.sh [--real|--mock]

set -e
SENSOR_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MONOREPO_ROOT="$(cd "${SENSOR_ROOT}/../.." && pwd)"
ROS2_WS="$MONOREPO_ROOT/ros2_ws"

# 需要 source ROS2 环境
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "未找到 ROS2 Jazzy，请确认安装路径"
    exit 1
fi
source /opt/ros/jazzy/setup.bash
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi

MODE="${1:---mock}"

case "$MODE" in
    --real)
        echo "=== 启动真实硬件模式 ==="
        ros2 launch buttonsensor_ros2_v1 buttonsensor.launch.py mock_mode:=false
        ;;
    --mock)
        echo "=== 启动模拟模式（无硬件测试）==="
        ros2 launch buttonsensor_ros2_v1 buttonsensor.launch.py mock_mode:=true
        ;;
    *)
        echo "用法: bash scripts/run_ros2.sh [--real|--mock]"
        ;;
esac
