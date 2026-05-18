# papillarray_ros2_v2

当前包是 Contactile PTS 触觉传感器在本工作区中的本地发布节点实现。

## 作用

- 读取 Contactile PTS 传感器数据；
- 发布 `sensor_interfaces/msg/SensorState`；
- 提供 bias 和 slip detection 相关服务；
- 为 `tactile_grasp_controller` 提供左右触觉输入。

## 默认 topic 规则

该包不会直接写死“left / right”语义，而是按下面的规则发布：

```text
/hub_<hub_id>/sensor_<sensor_id>
```

例如：

- `hub_id=0, sensor_id=0` -> `/hub_0/sensor_0`
- `hub_id=0, sensor_id=1` -> `/hub_0/sensor_1`

当前工作区里 `tactile_grasp_controller` 默认把：

- `/hub_0/sensor_0` 视为左传感器
- `/hub_0/sensor_1` 视为右传感器

如果真实安装顺序不同，应修改 controller 的 YAML 参数，而不是改这个 publisher 的命名规则。

## 启动

单独启动：

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch papillarray_ros2_v2 papillarray.launch.py
```

常用参数：

```bash
ros2 launch papillarray_ros2_v2 papillarray.launch.py \
  hub_id:=0 \
  n_sensors:=2 \
  com_port:=/dev/ttyACM0 \
  sampling_rate:=500
```

## CSV 日志

本节点默认禁用 SDK 内置的 CSV 日志（SDK 生成的日志文件权限为 `000`，不可读），
改为由节点自行写 CSV，可通过以下参数控制：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `log_dir` | string | `””` | CSV 输出目录路径，空字符串表示禁用 |
| `csv_pillar_detail` | bool | `false` | 是否展开逐柱体位移/力列 |

### 简化模式（默认）

`csv_pillar_detail=false`，每个传感器独立一行，列：`T_us, sensor_id, G_FX, G_FY, G_FZ, G_TX, G_TY, G_TZ, isSDActive, isRefLoaded, FRIC`。

```bash
ros2 launch papillarray_ros2_v2 papillarray.launch.py log_dir:=/home/xiaodalaing/ros_logs
```

### 完整模式

`csv_pillar_detail=true`，匹配 SDK 原格式，所有传感器合并为一行，每个传感器包含 9 柱体 × 6 列位移/力 + 接触/滑动状态 + 全局力/力矩/摩擦。

```bash
ros2 launch papillarray_ros2_v2 papillarray.launch.py \
  log_dir:=/home/xiaodalaing/ros_logs \
  csv_pillar_detail:=true
```

日志文件命名：`<log_dir>/LOG_hub<hub_id>_<unix_timestamp>.csv`。

## 服务

当前包提供：

- `/hub_<hub_id>/start_slip_detection`
- `/hub_<hub_id>/stop_slip_detection`
- `/hub_<hub_id>/send_bias_request`

## 说明

- 这是触觉”数据发布层”，不是控制器；
- 它不负责夹爪驱动；
- 它不负责抓取状态机；
- 它的输出消息和 `sensor_interfaces` 必须保持一致。
