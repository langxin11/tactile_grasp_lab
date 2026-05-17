# papillarray_ros2_v2

Contactile PTS 触觉传感器的 C++ 发布节点。当前仓库内 vendor 化，作为本地包构建。

源码：`ros2_ws/src/papillarray_ros2_v2/`

## 角色

- 通过 USB 连接 Contactile Hub；
- 解析 PTS pillar 阵列原始流；
- 转换为 [`sensor_interfaces/msg/SensorState`](sensor_interfaces.md#sensorstate) 发布。

## 默认 topic

约定：`/hub_<hub_id>/sensor_<sensor_id>`。

| 用途 | topic |
| --- | --- |
| 左侧触觉 | `/hub_0/sensor_0` |
| 右侧触觉 | `/hub_0/sensor_1` |

## 常用参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `com_port` | `/dev/ttyACM0` | Hub 串口设备 |
| `hub_id` | `0` | Hub 编号 |
| `n_sensors` | `2` | 单 Hub 上挂载的传感器数量 |

## 服务

- `/hub_<hub_id>/send_bias_request` — 触发归零（联调首步必做）。

## 待补充

- 完整参数列表
- 故障码与 LED 状态对照
- 多 Hub 部署示意图

:::{note}
本页为骨架占位，后续按需补全。原厂手册保存在 `sensors/contactile_pts/docs/manuals/`。
:::
