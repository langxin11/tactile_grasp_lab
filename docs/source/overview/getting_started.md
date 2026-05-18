# 快速上手

本页是 Sphinx 文档内的扩展版上手指南。最简流程仍以仓库根 `README.md` 为准；本页提供更详细的解释、
常见踩坑与延伸阅读链接。

## 环境

仓库统一使用根目录 `.venv`（uv 管理）。

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
```

如果需要重建 Python 环境：

```bash
rm -rf .venv
uv venv --python /usr/bin/python3 --system-site-packages .venv
source .venv/bin/activate
uv sync
```

:::{important}
`--system-site-packages` 是必须的。`.venv` 需要同时拿到项目依赖与 ROS2 消息生成所需的
`em` 模块；缺它会导致 `papillarray_ros2_v2` 等 C++ 包构建失败。
:::

## 构建 ROS2 workspace

```bash
source .venv/bin/activate
./scripts/build_ros2.sh
source ros2_ws/install/setup.bash
```

:::{warning}
不要直接调用裸 `colcon build`。`scripts/build_ros2.sh` 会把当前 `.venv/bin/python` 绑给
`ament_python` 生成的入口脚本；否则真机模式下 `robotiq_2f85_driver` 找不到 `pymodbus`。
:::

## 一键 bringup

仿真 / 默认 bringup：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py
```

真机分阶段 bringup（含传感器归零、夹爪激活、open goal）：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py
```

默认不会自动进入闭环；更稳的方式是 bringup 完成后再手动启动：

```bash
ros2 service call /tactile_grasp/start std_srvs/srv/Trigger "{}"
```

如果已经确认串口、法向符号和限位参数正确，也可以：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py \
  use_fake_gripper:=false \
  auto_grasp:=true
```

更细的真机联调步骤、安全检查与故障处理流程见 [硬件联调](../hardware/bringup.md)。

## 构建本文档

```bash
source .venv/bin/activate
uv sync --extra docs
cd docs
make html              # 输出到 docs/_build/html
make strict            # 把警告升级成错误，CI 用
```

浏览器打开 `docs/_build/html/index.html` 即可。
