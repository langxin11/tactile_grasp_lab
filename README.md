# tactile_grasp_lab

当前仓库已经整理为一个可构建的 ROS2 工作区，用于把：

- Contactile PTS 触觉传感器数据流
- Robotiq 2F-85 夹爪驱动
- 规则型触觉闭环控制器

放到同一个工程里统一管理。

## 当前结构

```text
tactile_grasp_lab/
├── .venv/
├── AGENTS.md
├── docs/
│   ├── ros2_repository_architecture.md
│   └── tactile_grasp_controller_codex_plan.md
├── main.py
├── pyproject.toml
└── ros2_ws/
    ├── src/
    │   ├── sensor_interfaces/
    │   ├── papillarray_ros2_v2/
    │   ├── robotiq_2f85_driver/
    │   ├── tactile_grasp_controller/
    │   ├── contactile_visualizer/
    │   └── buttonsensor_ros2_v1/
    ├── build/
    ├── install/
    └── log/
```

说明：

- 根目录保留少量轻量级历史验证脚本；
- 日常开发以 `ros2_ws/` 为主；
- Python 环境现在只保留一个 `.venv`；
- ROS2 传感器消息、传感器发布节点、夹爪 driver、触觉 controller 都已经纳入当前工作区。

## 包职责

### `sensor_interfaces`

- 定义 `SensorState.msg`、`PillarState.msg` 和相关服务；
- 不放业务控制逻辑。

### `papillarray_ros2_v2`

- C++ 触觉传感器发布节点；
- 默认发布 topic 为 `/hub_<hub_id>/sensor_<sensor_id>`；
- 使用 `sensor_interfaces/msg/SensorState`。

### `robotiq_2f85_driver`

- 当前默认 Robotiq 2F-85 执行层；
- Python `pymodbus` + lifecycle + 单线程硬件 worker；
- 内部按 `Driver` Protocol + `PymodbusDriver`（真硬件）+ `FakeDriver`（内存模拟）三件套组织，由 node 在 `on_configure` 根据 `dry_run` 参数择一注入；
- 对上提供 `/robotiq_gripper_controller/gripper_cmd` action；
- 同时保留 legacy `/robotiq/command/position` 和 Trigger 服务作为兼容入口。

### `tactile_grasp_controller`

- 订阅左右触觉数据；
- 提取特征；
- 执行规则型状态机；
- 向夹爪 action server 发送 `GripperCommand`；
- 提供统一 bringup launch。

## 环境准备

统一使用根目录 `.venv`：

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

说明：

- `--system-site-packages` 是必须的；
- 这样 `.venv` 才能同时拿到项目依赖和 ROS2 消息生成需要的 `em` 模块。

## 构建

```bash
source .venv/bin/activate
./scripts/build_ros2.sh
source ros2_ws/install/setup.bash
```

说明：

- 不要直接用裸 `colcon build`；
- 当前仓库的 ROS2 Python 包需要在构建时绑定根目录 `.venv/bin/python`；
- 否则 `ament_python` 生成的入口脚本可能退回到 `/usr/bin/python3`，真机模式下会找不到 `pymodbus`。

当前已验证可通过构建的包：

- `sensor_interfaces`
- `papillarray_ros2_v2`
- `robotiq_2f85_driver`
- `tactile_grasp_controller`

## 启动方式

### 一键启动整套链路

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py
```

该 launch 会组合启动：

- `papillarray_ros2_v2`
- `robotiq_2f85_driver`
- `tactile_grasp_controller`

当前默认控制语义是：

- 初始化、open goal 和手工单次动作走 `GripperCommand` action
- 触觉闭环主路径走 `/robotiq/command/stream`

### 一键启动真机初始化链路

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py
```

这条真机 launch 会做：

- 启动 `papillarray_ros2_v2`
- 启动 `robotiq_2f85_driver`
- 启动 `tactile_grasp_controller`
- 等待左右触觉流稳定
- 调用 `/robotiq/activate`
- 等待 `/robotiq_gripper_controller/gripper_cmd`
- 发送一次 gripper open action goal
- 等待 `post_open_settle_s` 的机械扰动衰减
- 调用 `/hub_0/send_bias_request`
- 等待触觉连续回到低载基线

默认不会自动调用 `/tactile_grasp/start`，也就是说系统会停在“夹爪已就绪、触觉已清零、controller 仍在 IDLE”的安全状态。

进入真实闭环抓取有两种方式：

1. 更稳的两阶段方式

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  use_fake_gripper:=false

ros2 service call /tactile_grasp/start std_srvs/srv/Trigger "{}"
```

2. 已确认参数无误后的一步自动进入闭环

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  use_fake_gripper:=false \
  auto_grasp:=true
```

首次真机闭环建议始终使用第一种方式。

### 常用参数

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  hub_id:=0 \
  n_sensors:=2 \
  controller_config:=/path/to/tactile_grasp_controller.yaml
```

如果只想启动部分链路：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_gripper_execution:=false
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_sensor:=false
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_controller:=false
```

## 默认 topic 约定

### 触觉传感器

- 左传感器：`/hub_0/sensor_0`
- 右传感器：`/hub_0/sensor_1`

### 夹爪执行层

- action：`/robotiq_gripper_controller/gripper_cmd`
- action 类型：`control_msgs/action/GripperCommand`
- legacy 回退路径：`/robotiq/command/position`、`/robotiq/command/echo`

### 控制器

- debug：`/tactile_grasp/debug`
- state：`/tactile_grasp/state`
- start service：`/tactile_grasp/start`
- stop service：`/tactile_grasp/stop`
- reset fault service：`/tactile_grasp/reset_fault`

## 安全要求

- `tactile_grasp_controller` 默认 `dry_run: true`；
- 真机 gripper execution 需要显式确认串口和 action 链路；
- 没确认法向符号前，不要让 controller 进入真实闭环；
- 没明确授权时，不要把 launch 改成默认真机动作；
- 自动化测试和普通 build 不应访问真实串口。

## 当前状态

当前仓库已经具备：

- 单一 `.venv`
- 可构建的 ROS2 workspace
- 本地 `sensor_interfaces`
- 本地 `papillarray_ros2_v2`
- 本地 `robotiq_2f85_driver`
- 本地 `tactile_grasp_controller`
- 一键 bringup launch

下一步更适合做的事情：

1. 根据实际硬件端口和法向方向更新 YAML 参数。
2. 校准 `gripper_closed_position`、最大速度和最大力参数。
3. 做一次真实传感器和夹爪联调。

## 真机联调

真机分阶段联调、闭环启动步骤与故障排查统一见：

- [HARDWARE_BRINGUP.md](/home/xiaodalaing/project/tactile_grasp_lab/HARDWARE_BRINGUP.md)

## 文档

Sphinx 文档源码在 `docs/source/`，构建产物在 `docs/_build/`（已 gitignore）。

```bash
source .venv/bin/activate
uv sync --extra docs
cd docs
make html       # 浏览器打开 docs/_build/html/index.html
make strict     # 把警告当错误，CI 模式
```
