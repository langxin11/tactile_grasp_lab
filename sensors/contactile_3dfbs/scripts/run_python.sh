#!/bin/bash
# 运行 Python 读取脚本
# 用法: bash sensors/contactile_3dfbs/scripts/run_python.sh [quick_read|stream_csv|live_plot]

set -e
SENSOR_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MONOREPO_ROOT="$(cd "${SENSOR_ROOT}/../.." && pwd)"
VENV_DIR="$SENSOR_ROOT/python/.venv"
SCRIPT="${1:-quick_read}"

# 激活虚拟环境
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "虚拟环境不存在，请先运行 scripts/setup.sh"
    exit 1
fi
source "$VENV_DIR/bin/activate"

# 加载环境变量
source "$SENSOR_ROOT/config/env.sh"

# 运行指定脚本
case "$SCRIPT" in
    quick_read)  python "$SENSOR_ROOT/python/quick_read.py" ;;
    stream_csv)  python "$SENSOR_ROOT/python/stream_csv.py" ;;
    live_plot)   python "$SENSOR_ROOT/python/live_plot.py" ;;
    *)           echo "未知脚本: $SCRIPT (可用: quick_read, stream_csv, live_plot)" ;;
esac
