# sensor_interfaces

纯消息层包，定义触觉传感器、夹爪状态相关的 ROS2 自定义消息。**不放业务逻辑**。

源码：`ros2_ws/src/sensor_interfaces/`

## 消息

### `SensorState`

PTS 触觉传感器单帧状态。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 标准时间戳与帧 ID |
| `tus` | `int64` | 传感器内部时间戳（µs） |
| `pillars` | `PillarState[]` | 9 个触点的逐点状态 |
| `gfx`, `gfy`, `gfz` | `float32` | 全局合力（N） |
| `gtx`, `gty`, `gtz` | `float32` | 全局合力矩（N·m） |
| `friction_est` | `float32` | 摩擦系数估计 |
| `target_grip_force` | `float32` | 目标抓取力 |
| `is_sd_active` | `bool` | 滑移检测是否激活 |
| `is_ref_loaded` | `bool` | 参考数据是否加载 |
| `is_contact` | `bool` | 整体是否有接触 |

### `PillarState`

单个 pillar 的力、位移与接触状态。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `id` | `int64` | pillar 编号（0–8） |
| `dx`, `dy`, `dz` | `float32` | 位移（mm） |
| `fx`, `fy`, `fz` | `float32` | 力（N） |
| `in_contact` | `bool` | 单点接触标志 |
| `slip_state` | `int64` | 滑移状态码 |

### `GripperStatus`

Robotiq 2F-85 的扩展状态。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `state`, `detail` | `string` | 高层状态描述 |
| `dry_run`, `connected`, `synthetic_feedback` | `bool` | 模式标志 |
| `gripper_status`, `fault_status`, `fault_text` | `uint8` / `string` | Modbus 状态字 + 文本 |
| `position_request_echo`, `position` | `uint8` | 命令回显与当前位置 |
| `current`, `current_milliamps` | `uint8` / `int32` | 电流原始字节 + 物理量 |
| `activation_state`, `activation_echo`, `go_to_echo` | `uint8` | 激活/Go-To 回显 |
| `object_status` | `uint8` | 0–3 表示 moving / stopped-open / stopped-closed / detected |
| `is_activation_complete` | `bool` | 上电激活是否完成 |

### `ButtonSensorState`

按钮型力传感器单帧（用于 `buttonsensor_ros2_v1`）。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `tus` | `int64` | 时间戳（µs） |
| `gfx`, `gfy`, `gfz` | `float32` | 三轴力（N） |

## 服务

服务定义见包内 `srv/`（按需补充）。
