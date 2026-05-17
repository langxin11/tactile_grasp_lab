#!/bin/bash
# 编译并运行 C++ 读取/记录示例
# 用法: bash sensors/contactile_3dfbs/scripts/run_cpp.sh [-- --confirm-no-load --duration 2 --output /tmp/cpp_rate.csv]

set -euo pipefail
SENSOR_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MONOREPO_ROOT="$(cd "${SENSOR_ROOT}/../.." && pwd)"
BUILD_DIR="$SENSOR_ROOT/cpp/build"

if [[ "${1:-}" == "--" ]]; then
    shift
fi

echo "=== 编译 C++ minimal_reader ==="
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake ..
make -j"$(nproc)"

echo ""
echo "=== 运行 C++ minimal_reader ==="
source "$SENSOR_ROOT/config/env.sh"
exec "$BUILD_DIR/minimal_reader" "$@"
