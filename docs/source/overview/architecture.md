# ROS2 Repository Architecture

## 1. 目的

本文档描述本仓库的 ROS2 相关架构，区分：

- 当前已经存在并可工作的基线；
- 中长期目标技术栈；
- 推荐的迁移方向与边界。

它是仓库级架构说明，不替代各 package 自己的 `README.md`。

---

## 2. 当前基线

当前仓库并不是一个从零设计的纯 ROS2 项目，而是从可用脚本逐步整理而来。

当前已知事实：

- 根目录仍保留少量历史遗留验证脚本；
- 仓库统一使用根目录 `.venv`；
- ROS2 构建统一通过 `./scripts/build_ros2.sh`；
- 触觉、夹爪与控制逻辑已经开始进入 `ros2_ws/src/` 下的 package。

当前工作区中的主要 package：

- `sensor_interfaces`
- `papillarray_ros2_v2`
- `robotiq_2f85_driver`
- `tactile_grasp_controller`
- `contactile_visualizer`
- `buttonsensor_ros2_v1`

---

## 3. 当前 ROS2 分层

### `sensor_interfaces`

- 定义触觉消息与服务；
- 作为传感器节点、控制器和可视化节点之间的公共接口层。

### `papillarray_ros2_v2`

- 接入 Contactile PTS SDK；
- 发布 `SensorState`；
- 提供 bias / slip detection 相关服务；
- 负责传感器侧设备访问。

### `robotiq_2f85_driver`

- 当前默认夹爪执行层；
- 基于 Python `pymodbus` + lifecycle + 单线程硬件 worker；
- 对上提供 `/robotiq_gripper_controller/gripper_cmd` action；
- 同时保留 legacy `/robotiq/command/position` 和 Trigger 服务。

### `tactile_grasp_controller`

- 订阅左右触觉；
- 提取特征；
- 执行规则型状态机；
- 向夹爪 driver 发高层命令。

### `contactile_visualizer`

- 用于可视化与调试；
- 不参与主控制闭环。

### `buttonsensor_ros2_v1`

- 接入按钮型传感器侧 ROS2 节点；
- 作为与主触觉链路并列的实验层组件。

---

## 4. 目标技术栈

当前阶段的目标架构不包含 MoveIt2，也不额外引入独立的 research layer。

目标是先把“触觉闭环抓取”这一条链路整理成更标准、更清晰的 ROS2 执行栈：

```text
Contactile tactile nodes
  ├─ papillarray_ros2_v2
  └─ buttonsensor_ros2_v1

tactile_grasp_controller
  ├─ 订阅触觉消息
  ├─ 提取特征与维护状态机
  └─ 输出夹爪高层命令

Gripper execution layer
  ├─ robotiq_2f85_driver
  ├─ control_msgs/action/GripperCommand
  └─ sensor_interfaces/msg/GripperStatus

Robotiq 2F-85 hardware
```

设计意图：

- 当前先聚焦“触觉 -> 控制器 -> 夹爪”这条最小闭环；
- 触觉节点、控制器、夹爪执行层之间通过 ROS2 接口解耦；
- 控制器不直接依赖底层串口或厂商 SDK；
- 未来若接入 MoveIt2 或更高层研究代码，应作为上层扩展，而不是当前迁移前提。

---

## 5. 当前夹爪执行层

当前默认执行层已经收敛回仓库内的 `robotiq_2f85_driver`：

- 真实硬件访问继续留在独立 driver 包内；
- 节点使用 `LifecycleNode` 暴露清晰的 bringup 生命周期；
- 串口访问通过单线程硬件 worker 串行化，避免阻塞 executor；
- 对控制器继续暴露标准 `GripperCommand` action。

---

## 6. 推荐迁移方式

当前采用“目标明确、范围收紧”的一步到位迁移：

1. 目标范围只包含触觉节点、`tactile_grasp_controller` 和夹爪执行层。
2. 暂不把 MoveIt2、机械臂驱动和独立 research layer 纳入迁移前提。
3. 保持 `tactile_grasp_controller` 的现有控制语义，下游继续使用 `GripperCommand` action。
4. 用结构化 `GripperStatus` 替换旧 JSON string 状态发布。

这样做的原因：

- 当前真正需要打通的是触觉闭环，而不是完整机器人规划栈；
- 能减少一次迁移中同时变化的系统数量；
- 能把主要风险收敛在夹爪执行层与控制器接口上。

---

## 7. 当前推荐边界

在新执行层落地前，推荐继续保持以下边界：

- `papillarray_ros2_v2` 与 `buttonsensor_ros2_v1` 负责传感器设备访问；
- `sensor_interfaces` 负责公共接口定义；
- `tactile_grasp_controller` 负责闭环控制；
- 夹爪执行层独立负责设备访问与动作执行；
- 不把实验脚本和硬件适配逻辑混进控制器。

---

## 8. 包级迁移判断

基于当前目标架构，现有主要 package 的处理建议如下：

### 保留

- `sensor_interfaces`
  - 继续作为触觉消息与服务的公共接口层。
- `papillarray_ros2_v2`
  - 继续作为 Contactile PTS 传感器节点。
- `buttonsensor_ros2_v1`
  - 继续作为按钮传感器侧实验节点，与主触觉链路并列存在。
- `contactile_visualizer`
  - 继续作为调试与可视化工具。

### 保留并适配

- `tactile_grasp_controller`
  - 保留其触觉特征、状态机与闭环控制逻辑；
  - 下游输出接口改为 `/robotiq_gripper_controller/gripper_cmd`；
  - legacy 裸位置命令 topic 仅用于旧驱动回退。

### 替换

- `robotiq_2f85_driver`
  - 当前实现基于 Python + `pymodbus`；
  - 它就是默认执行层；
  - legacy topic/service 仅保留为兼容和调试入口。

### 废弃候选

- 根目录与旧执行层强耦合的轻量脚本
  - 在当前 Python driver 主路径稳定后，可再决定是否保留、迁移或移除；
  - 当前阶段不作为迁移阻塞项。

一句话总结：

- 传感器层保留；
- 控制器保留；
- 夹爪执行层收敛到仓库内 Python driver。

---

## 9. 文档维护原则

若未来架构目标发生变化：

- 更新本文档；
- 同步更新受影响 package 的 `README.md`；
- 不要把架构讨论继续堆进 `AGENTS.md`；
- 不要把目标架构表述成已经完成的现实。
