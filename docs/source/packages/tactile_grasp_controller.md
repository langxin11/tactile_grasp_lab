# tactile_grasp_controller

规则型触觉抓取控制器。订阅左右触觉数据 → 提取特征 → 状态机决策 → 通过 action 驱动夹爪。

源码：`ros2_ws/src/tactile_grasp_controller/`

## 节点行为

订阅：

- `/hub_0/sensor_0`、`/hub_0/sensor_1`（[`SensorState`](sensor_interfaces.md#sensorstate)）

发布：

- `/tactile_grasp/debug` — 内部特征与决策快照
- `/tactile_grasp/state` — 当前高层状态

服务：

- `/tactile_grasp/start` — 进入 APPROACH
- `/tactile_grasp/stop` — 回到 IDLE
- `/tactile_grasp/reset_fault` — 从 FAULT 恢复

下游 action：

- `/robotiq_gripper_controller/gripper_cmd`（`control_msgs/GripperCommand`）

## 状态机

```
IDLE ──start──► APPROACH ──双侧接触──► PRELOAD ──预紧力达标──► HOLD
                                                                  │
                                                                  ▼
                                                      SLIP_COMPENSATE
                                                                  │
                                                                  └─► HOLD
任意 ──故障──► FAULT ──reset_fault──► IDLE
```

详见 [`TactileGraspStateMachine`](../api/tactile_grasp_controller.rst)。

## 模块组成

- `feature_extractor` — 接触/滑移/力估计特征
- `state_machine` — 高层状态机
- `command_bridge` — 与 gripper action 桥接
- `fault_guard` — 安全/故障守护
- `hardware_bringup_coordinator` — 真机分阶段初始化编排
- `models` — 数据类与控制动作定义
- `tactile_grasp_controller_node` — 入口节点

## Launch

- `tactile_grasp_bringup.launch.py` — sensor + gripper + controller 三件套
- `tactile_grasp_hardware_bringup.launch.py` — 真机版，含归零、激活、open goal

完整参数与启动方式参考仓库根 `README.md` 的「启动方式」一节，以及
[`hardware_bringup_coordinator`](../api/tactile_grasp_controller.rst) 的 API 文档。

## 安全

- 默认 `dry_run: true`；
- 未确认法向符号前不要进入真实闭环；
- 真机 gripper execution 需要显式确认串口与 action 链路。
