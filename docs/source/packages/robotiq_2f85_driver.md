# robotiq_2f85_driver

Robotiq 2F-85 夹爪的 Python 驱动节点。基于 `pymodbus` + ROS2 lifecycle，单线程硬件 worker。

源码：`ros2_ws/src/robotiq_2f85_driver/`

## 主要接口

### Action（推荐）

- `/robotiq_gripper_controller/gripper_cmd`
- 类型：`control_msgs/action/GripperCommand`

### Legacy 入口（兼容）

- `/robotiq/command/position` — 直接发送 0–255 位置字节；
- `/robotiq/command/echo` — 回显话题；
- `/robotiq/activate` — `std_srvs/Trigger`，激活夹爪。

### 状态

- 扩展状态：[`sensor_interfaces/GripperStatus`](sensor_interfaces.md#gripperstatus)。

## 模块组成

详见 [Python API 参考](../api/robotiq_2f85_driver.rst)。

- `driver` — `Driver` Protocol、共享数据类（`GripperFeedback` / `CommandRecord` / `MotionResult` / `GripperError`）、寄存器打包工具与常量。
- `pymodbus_driver` — `PymodbusDriver`，真硬件实现；`dry_run` 常量为 `False`。
- `fake_driver` — `FakeDriver`，内存模拟实现（替代原 dry-run 路径）；`dry_run` 常量为 `True`。
- `robotiq_driver_node` — ROS2 lifecycle 节点；在 `on_configure` 中根据 `dry_run` 参数择一注入 `Driver` 实现。
- `action_utils` — `GripperCommand` action server 适配。
- `safety` — 字节裁剪、速度/力上限保护。

## 常用参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `com_port` | `/dev/ttyUSB0` | Modbus 串口设备 |
| `dry_run` | `false` | 不实际驱动硬件，仅打印 / 模拟反馈 |
| `gripper_closed_position` | 待标定 | 真机闭合位置字节 |

:::{warning}
真机使用前必须确认串口、`gripper_closed_position`、最大速度/力上限已校准。详见
[硬件联调](../hardware/bringup.md)。
:::
