# tactile_grasp_controller

`tactile_grasp_controller` 是当前仓库里的触觉抓取控制编排包，负责：

- 订阅左右触觉话题；
- 计算接触、法向力、切向力、摩擦裕度和滑移等特征；
- 运行规则型状态机；
- 闭环模式下通过 streaming topic 持续刷新夹爪目标位置与建议速度；
- 兼容保留 `control_msgs/action/GripperCommand` 初始化与手工命令入口。

这个包不直接访问串口，也不直接调用 `pymodbus`。硬件执行层由 `robotiq_2f85_driver` 负责。

## 目录说明

- `tactile_grasp_controller_node.py`
  主控制节点，负责特征提取、状态机和动作下发。
- `hardware_bringup_coordinator.py`
  真机启动协调器，负责 bias、夹爪激活、开爪和可选的闭环启动。
- `launch/`
  放分层 launch。
- `config/`
  放控制器与启动协调器配置。

## 三个 launch 文件的区别

### `tactile_grasp_controller.launch.py`

只启动 `tactile_grasp_controller_node` 自身。

适合：

- 单独调试控制器逻辑；
- 配合已有传感器节点和夹爪执行层使用；
- 单测之外做最小链路验证。

### `tactile_grasp_bringup.launch.py`

统一装配三层链路：

- `papillarray_ros2_v2`
- `robotiq_2f85_driver`
- `tactile_grasp_controller`

它是“通用 bringup”，支持通过 launch 参数裁剪某一层是否启动。

适合：

- 日常联调；
- 分层排障；
- 假驱动或半实物环境。

### `tactile_grasp_hardware_bringup.launch.py`

在 `tactile_grasp_bringup.launch.py` 的基础上，再额外启动 `hardware_bringup_coordinator`。

协调器会按顺序做：

- 等待触觉流稳定；
- 调用夹爪 activate；
- 发送一次 open goal；
- 等待 open 后机械扰动衰减；
- 调用 bias 服务；
- 等待触觉连续回到低载基线；
- 可选调用 `/tactile_grasp/start` 进入闭环。

适合：

- 真机启动；
- 需要把系统带到“触觉已清零 + 夹爪就绪”状态的场景。

## 三个 YAML 的区别

### `config/tactile_grasp_controller.yaml`

控制器默认配置，面向安全默认值和当前推荐的平滑接触参数。

关键点：

- `dry_run: true`
- `auto_start: false`
- `require_clear_tactile_on_start: true`
- `start_force_threshold_n: 5.0`
- `control_rate_hz: 40.0`
- `approach/preload/compensate_position_step: 1`
- `target_command_rate_bytes_per_s: 80.0`
- `min_speed_byte: 8`
- `max_speed_byte: 32`

适合：

- 逻辑调试；
- 无真机命令输出的联调；
- 默认 launch 场景。

### `config/tactile_grasp_controller.hardware.yaml`

控制器真机覆盖配置，专门用于覆盖基础配置里的少数参数。

关键差异：

- `dry_run: false`
- `control_rate_hz: 40.0`

当前用法是：

- `tactile_grasp_controller.yaml` 作为基础配置
- `tactile_grasp_controller.hardware.yaml` 作为覆盖配置叠加加载

适合：

- 真机抓取；
- 需要真实发夹爪命令的场景。

### `config/hardware_bringup_coordinator.yaml`

这不是控制器参数，而是 `hardware_bringup_coordinator` 的启动流程配置。

内容包括：

- 触觉稳定等待时间；
- open 后机械扰动衰减时间；
- 触觉低载门控阈值和稳定时间；
- bias 服务名；
- activate 服务名；
- 开爪 action；
- 是否自动启动 controller；
- 各类等待超时。

## 是否有重复

有一部分“场景分层上的相似”，但不是完全重复。

### launch 层

三个 launch 不是重复文件，而是三层递进关系：

- `tactile_grasp_controller.launch.py` 只管控制器节点；
- `tactile_grasp_bringup.launch.py` 负责装配整条链路；
- `tactile_grasp_hardware_bringup.launch.py` 在通用 bringup 上增加真机初始化流程。

这部分结构是合理的，没有明显重复实现。

### YAML 层

之前真正重复较多的是：

- `tactile_grasp_controller.yaml`
- `tactile_grasp_controller.hardware.yaml`

现在已经收敛成：

- 一份基础配置
- 一份真机覆盖配置

这样公共参数只维护一份，真机场景只保留差异项。

目前 `hardware_bringup_coordinator.yaml` 不属于这种重复，它描述的是另一类节点。

## 默认话题与接口

- 左触觉：`/hub_0/sensor_0`
- 右触觉：`/hub_0/sensor_1`
- 闭环 streaming 命令：`/robotiq/command/stream`
- 夹爪 action：`/robotiq_gripper_controller/gripper_cmd`
- legacy 位置命令：`/robotiq/command/position`
- legacy echo：`/robotiq/command/echo`
- controller debug：`/tactile_grasp/debug`
- controller state：`/tactile_grasp/state`
- controller start service：`/tactile_grasp/start`

## 启动示例

只启动控制器：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_controller.launch.py
```

启动整套链路：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py
```

启动真机 bringup：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py
```

指定控制器配置：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py \
  controller_config:=/path/to/tactile_grasp_controller.yaml
```

在基础配置之上叠加覆盖配置：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py \
  controller_config:=/path/to/tactile_grasp_controller.yaml \
  controller_config_override:=/path/to/override.yaml
```

只关掉夹爪执行层：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py \
  start_gripper_execution:=false
```

## 依赖与限制

这个包默认依赖 `sensor_interfaces/msg/SensorState`。

当工作区里没有可用的 `sensor_interfaces` 时：

- 节点可以启动；
- debug 和 state 发布仍然可用；
- 触觉订阅会被禁用；
- 真正的触觉闭环控制无法运行。

## 当前建议

当前结构里，launch 文件保留现状比较合理；如果要继续精简，优先收敛的是两份控制器 YAML 的重复参数，而不是先合并 launch 文件。
