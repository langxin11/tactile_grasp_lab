# buttonsensor_ros2_v1

当前包是 Contactile ButtonSensor 触觉传感器在本工作区中的本地发布节点实现。

## 作用

- 读取 Contactile ButtonSensor 传感器数据；
- 发布 `sensor_interfaces/msg/ButtonSensorState`；
- 提供 bias 请求服务。

## 默认 topic 规则

```text
/hub_<hub_id>/sensor_<sensor_id>
```

## 启动

单独启动：

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch buttonsensor_ros2_v1 buttonsensor.launch.py
```

常用参数：

```bash
ros2 launch buttonsensor_ros2_v1 buttonsensor.launch.py \
  hub_id:=0 \
  n_sensors:=10 \
  com_port:=/dev/ttyACM0 \
  sampling_rate:=500
```

## CSV 日志

本节点默认禁用 SDK 内置的 CSV 日志（SDK 生成的日志文件权限为 `000`，不可读），
改为由节点自行写 CSV。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `log_dir` | string | `""` | CSV 输出目录路径，空字符串表示禁用 |

CSV 格式：`T_us, sensor_id, G_FX, G_FY, G_FZ`，每个传感器每周期一行。

启用示例：

```bash
ros2 launch buttonsensor_ros2_v1 buttonsensor.launch.py log_dir:=/home/xiaodalaing/ros_logs
```

日志文件命名：`<log_dir>/LOG_hub<hub_id>_<unix_timestamp>.csv`。

## 服务

当前包提供：

- `/hub_<hub_id>/send_bias_request`

## 说明

- 这是触觉"数据发布层"，不是控制器；
- 它不负责夹爪驱动；
- 它不负责抓取状态机；
- 它的输出消息和 `sensor_interfaces` 必须保持一致。
