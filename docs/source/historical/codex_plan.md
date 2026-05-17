# Historical tactile grasp controller plan

:::{admonition} 历史文档
:class: warning

本文档为早期规划记录，**不代表当前架构**。当前默认架构与状态请看：
[架构总览](../overview/architecture.md)、[快速上手](../overview/getting_started.md)、[硬件联调](../hardware/bringup.md)。
:::

## 0. 文档定位

本文档是早期规划记录，不再代表当前默认架构。

当前仓库的默认夹爪执行层已经收敛到 Python `robotiq_2f85_driver`
包。当前架构和迁移状态优先看：

- [架构总览](../overview/architecture.md)
- [快速上手](../overview/getting_started.md)
- [硬件联调](../hardware/bringup.md)

---

## 1. 当前仓库状态

当前仓库已经具备如下结构：

```text
robotiq2f85_control/
├── .venv/
├── AGENTS.md
├── HARDWARE_BRINGUP.md
├── README.md
├── main.py
├── pyproject.toml
├── uv.lock
├── tactile_grasp_controller_codex_plan.md
└── ros2_ws/
    ├── src/
    │   ├── sensor_interfaces/
    │   ├── papillarray_ros2_v2/
    │   ├── robotiq_2f85_driver/
    │   └── tactile_grasp_controller/
    ├── build/
    ├── install/
    └── log/
```

当前已完成的关键事项：

- 根目录只保留一个 `.venv`；
- `.venv` 同时承载项目依赖和 ROS2 Python 消息生成依赖；
- `sensor_interfaces` 已在当前工作区本地化；
- `papillarray_ros2_v2` 已在当前工作区本地化；
- `robotiq_2f85_driver` 已实现为 ROS2 Python package；
- `tactile_grasp_controller` 已实现为 ROS2 Python package；
- 已提供统一 bringup launch；
- 已验证以下四个包可在当前工作区通过构建：
  - `sensor_interfaces`
  - `papillarray_ros2_v2`
  - `robotiq_2f85_driver`
  - `tactile_grasp_controller`

---

## 2. 当前架构

### 2.1 数据链路

当前链路是：

```text
Contactile PTS
  -> papillarray_ros2_v2
  -> sensor_interfaces/msg/SensorState
  -> tactile_grasp_controller
  -> /robotiq/command/position
  -> robotiq_2f85_driver
  -> Robotiq 2F-85
```

### 2.2 工作区分层

当前仓库实际采用下面的职责分层：

1. `sensor_interfaces`
   只负责消息和服务定义。

2. `papillarray_ros2_v2`
   只负责读取 Contactile PTS 数据并发布 `SensorState`。

3. `robotiq_2f85_driver`
   只负责 Robotiq 串口通信、Modbus、位置命令和安全边界。

4. `tactile_grasp_controller`
   只负责触觉特征、状态机和高层抓取策略。

5. `main.py`
   作为遗留直连验证脚本保留，不再是主干架构的一部分。

---

## 3. 当前包职责

### `sensor_interfaces`

职责：

- 定义 `SensorState.msg`
- 定义 `PillarState.msg`
- 定义 bias / slip detection 相关服务

不负责：

- 传感器采集逻辑
- 夹爪控制逻辑
- 状态机逻辑

### `papillarray_ros2_v2`

职责：

- 连接 Contactile PTS
- 读取全局力、力矩、柱接触、滑移等信息
- 发布 `/hub_<hub_id>/sensor_<sensor_id>`

不负责：

- 夹爪驱动
- 闭环抓取决策

### `robotiq_2f85_driver`

职责：

- 隔离串口和 Modbus
- 提供 `/robotiq/command/position`
- 提供 `/robotiq/open`、`/robotiq/close`、`/robotiq/stop`
- 发布 `/robotiq/command/echo`
- 发布 `/robotiq/driver/status`

不负责：

- 触觉特征提取
- 滑移补夹决策

### `tactile_grasp_controller`

职责：

- 订阅左右触觉 topic
- 提取接触、法向力、剪切力、摩擦裕度、滑移特征
- 运行 `IDLE -> APPROACH -> PRELOAD -> HOLD -> SLIP_COMPENSATE -> FAULT`
- 向 driver 发布高层位置步进命令
- 提供 `/tactile_grasp/start`、`/tactile_grasp/stop`、`/tactile_grasp/reset_fault`

不负责：

- 串口
- Modbus
- 传感器 SDK

---

## 4. 当前默认约定

### 4.1 Python 与 ROS2 环境

当前统一约定：

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
```

当前推荐构建命令：

```bash
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/home/xiaodalaing/project/robotiq2f85_control/.venv/bin/python
```

### 4.2 触觉 topic

当前默认把：

- `/hub_0/sensor_0` 视为左传感器
- `/hub_0/sensor_1` 视为右传感器

注意：

- 这是 controller 当前配置约定；
- 不是 `papillarray_ros2_v2` 内部对“左右”的强语义定义；
- 如果真实安装顺序不同，应修改 controller YAML，而不是改 publisher 的命名规则。

### 4.3 Driver 安全边界

当前 driver 的关键安全约定：

- 默认 `dry_run: true`
- 默认不自动 `activate`
- `/robotiq/command/echo` 是命令回显，不是真实位置反馈

### 4.4 Controller 安全边界

当前 controller 的关键安全约定：

- 默认 `dry_run: true`
- 默认 `auto_start: false`
- 左右触觉消息缺失时不允许发闭合动作
- 触觉超时或安全阈值超限时进入 `FAULT`

---

## 5. 当前已知取舍

当前架构已经可 build、可启动、可联调，但还有明确的阶段性取舍。

### 5.1 已知取舍一：driver 还没有真实位置反馈

当前 `tactile_grasp_controller` 通过 `/robotiq/command/echo` 维护“命令位置估计”。

这意味着：

- controller 知道“自己要求夹爪到哪里”；
- 但还不知道“夹爪真实到了哪里”。

因此，当前闭环仍然是：

- 触觉闭环
- 夹爪位置开环或半开环

这不是最终形态。

### 5.2 已知取舍二：法向方向仍依赖现场标定

当前 `left_normal_sign` 与 `right_normal_sign` 默认值是保守占位值。

真实硬件接入前，必须完成：

- `gfz` 是否为法向轴确认
- 左侧正负方向确认
- 右侧正负方向确认

### 5.3 已知取舍三：状态机是第一版规则控制器

当前状态机已经能支持：

- 接触检测
- 预紧建立
- 保持
- 滑移补夹
- fault 保护

但它仍是：

- 可解释的一版控制器
- 便于调参和联调的一版控制器

不是最终性能上限版本。

---

## 6. 当前不做的事情

在当前阶段，仍然明确不做：

- 不引入强化学习；
- 不引入模仿学习；
- 不在 controller 中直接操作 Modbus；
- 不把所有逻辑重新堆回单脚本；
- 不把 `papillarray_ros2_v2`、driver、controller 的职责混写；
- 不默认开启真机闭环；
- 不在自动化测试里碰真实串口。

---

## 7. 统一启动入口

当前工作区已经提供统一 bringup：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py
```

该入口会组合启动：

- `papillarray_ros2_v2`
- `robotiq_2f85_driver`
- `tactile_grasp_controller`

也支持按层关闭：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_sensor:=false
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_gripper_execution:=false
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_controller:=false
```

真机联调顺序请看：

- [HARDWARE_BRINGUP.md](/home/xiaodalaing/project/robotiq2f85_control/HARDWARE_BRINGUP.md)

---

## 8. 下一阶段路线图

当前最值得推进的不是继续扩包，而是把现有链路做实。

### 8.1 优先级最高

1. 给 `robotiq_2f85_driver` 增加真实状态反馈，而不只依赖命令回显。
2. 完成左右法向方向标定，并把结果固化到 YAML。
3. 按真机联调手册完成第一轮闭环验证。

### 8.2 第二优先级

1. 根据真实噪声和零漂，重调 `tactile_timeout_s`、`min_preload_force_n`、`max_safe_force_n`。
2. 根据真实滑移响应，细化 `slip_state` 的判定语义。
3. 让 debug 输出更适合离线分析。

### 8.3 暂缓事项

1. 更复杂的学习型策略
2. 与机械臂更深的系统级联动
3. 控制器性能优化而非安全与可解释性优化

---

## 9. 文档使用建议

如果你的问题是：

- “怎么启动”
  看 [README.md](/home/xiaodalaing/project/robotiq2f85_control/README.md)

- “怎么真机联调”
  看 [HARDWARE_BRINGUP.md](/home/xiaodalaing/project/robotiq2f85_control/HARDWARE_BRINGUP.md)

- “agent 在这个仓库该遵守什么规则”
  看 [AGENTS.md](/home/xiaodalaing/project/robotiq2f85_control/AGENTS.md)

- “当前架构是什么、下一步该做什么”
  看本文档

一句话总结：

当前仓库已经不是“准备开工”的状态，而是“基础架构已落地，进入真机收敛和闭环打磨阶段”。
