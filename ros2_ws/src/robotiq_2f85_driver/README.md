# robotiq_2f85_driver

`robotiq_2f85_driver` 是当前仓库中 Robotiq 2F-85 的默认 ROS2 执行层。

它负责：

- 对外提供 `control_msgs/action/GripperCommand`；
- 提供 streaming topic 作为闭环主路径；
- 兼容 legacy 位置命令 topic 和若干 Trigger 服务；
- 隔离 Modbus / 串口访问；
- 为上层 `tactile_grasp_controller` 提供稳定的夹爪执行接口。

这个包不负责触觉特征提取、抓取状态机或策略逻辑。

## 结构说明

- `robotiq_driver_node.py`
  ROS2 lifecycle 节点，负责参数装配、生命周期管理和接口暴露。
- `driver.py`
  定义执行层协议和共享数据结构。
- `pymodbus_driver.py`
  真硬件实现。
- `fake_driver.py`
  内存模拟实现，用于 `dry_run` 和测试。
- `safety.py`
  夹爪位置、速度、力度等安全边界处理。
- `launch/robotiq_2f85_driver.launch.py`
  单包启动入口。
- `config/`
  基础配置与真机覆盖配置。

## 对外接口

- 闭环 streaming：`/robotiq/command/stream`，类型 `sensor_interfaces/msg/GripperStreamingCommand`
- 订阅：`/robotiq/command/position`，类型 `std_msgs/msg/Int32`
- 发布：`/robotiq/command/echo`，类型 `std_msgs/msg/Int32`
- 发布：`/robotiq/driver/status`，类型 `sensor_interfaces/msg/GripperStatus`
- Action：`/robotiq_gripper_controller/gripper_cmd`，类型 `control_msgs/action/GripperCommand`
- 服务：`/robotiq/activate`、`/robotiq/open`、`/robotiq/close`、`/robotiq/stop`、`/robotiq/reconnect`

## 配置分层

### `config/robotiq_2f85_driver.yaml`

基础配置，采用安全默认值。

关键点：

- `dry_run: true`
- `startup_activate: false`
- `autostart: true`
- `default_speed: 16`
- `streaming_update_hz: 50.0`

适合：

- 桌面联调；
- 无真机串口时验证接口链路；
- 默认启动场景。

### `config/robotiq_2f85_driver.hardware.yaml`

真机覆盖配置，只保留与基础配置不同的参数。

当前覆盖项：

- `dry_run: false`

适合：

- 真实串口和真实夹爪执行；
- 在基础配置之上切换到真机模式。

## 启动方式

默认启动：

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py
```

指定基础配置：

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py \
  config:=/path/to/robotiq_2f85_driver.yaml
```

在基础配置上叠加真机覆盖：

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py \
  config:=/path/to/robotiq_2f85_driver.yaml \
  config_override:=/path/to/robotiq_2f85_driver.hardware.yaml
```

直接通过 launch 参数覆盖关键项：

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py \
  serial_port:=/dev/ttyUSB0 \
  dry_run:=false \
  startup_activate:=false
```

## 安全说明

- 默认 `dry_run` 为 `true`；
- 真机模式需要 `pymodbus` 和可访问的串口设备；
- 默认不在启动时自动激活夹爪；
- 闭环推荐走 `/robotiq/command/stream`，action 继续用于 open/close/初始化；
- 发布的 `command_echo` 是最近一次接受的命令位置，不是硬件实测指位；
- 真机模式下通过后台状态轮询维持通信，避免长时间无通信导致夹爪故障。
