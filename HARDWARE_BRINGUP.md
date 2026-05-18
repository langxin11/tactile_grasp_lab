# 真机联调手册

本文档用于指导当前仓库在真实硬件上做分阶段联调。

适用对象：

- Contactile PTS 触觉传感器
- Robotiq 2F-85 夹爪
- 当前工作区中的：
  - `sensor_interfaces`
  - `papillarray_ros2_v2`
  - `robotiq_2f85_driver`
  - `tactile_grasp_controller`

本文默认目标不是“一步到位闭环抓取”，而是按风险从低到高逐层确认：

1. 环境正确
2. 传感器能发消息
3. driver 能安全启动
4. controller 能 dry-run
5. 法向符号标定正确
6. 最后才允许真实闭环动作

---

## 1. 联调前原则

- 默认先用 `dry_run: true`
- 默认先单独调每一层
- 没确认法向方向前，不要让 controller 进入真实闭环
- 没确认串口口径前，不要同时插多个相似 USB 设备盲试
- 任何时候只要行为异常，先停在 driver 层，不要直接怀疑 controller

---

## 2. 硬件检查清单

联调前先人工确认：

- Robotiq 2F-85 供电正常
- Robotiq 通信线和 USB 转串口正常
- PTS 传感器连接正常
- PTS 所在串口和 Robotiq 所在串口不是同一个设备
- 夹爪附近无会被误夹伤的物体
- 初次真机闭环时，夹爪内不要放脆弱或高价值物体

建议确认设备名：

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

通常约定：

- Robotiq driver 使用 `/dev/ttyUSB*`
- PTS publisher 使用 `/dev/ttyACM*`

但这不是强保证，必须以本机实际枚举为准。

---

## 3. 环境准备

在仓库根目录执行：

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
```

如果工作区有改动，先重建：

```bash
cd /home/xiaodalaing/project/tactile_grasp_lab
./scripts/build_ros2.sh
cd ros2_ws
source install/setup.bash
```

注意：

- 不要在当前仓库直接执行裸 `colcon build`；
- 真实夹爪模式依赖 `pymodbus`，如果构建时 shebang 绑到 `/usr/bin/python3`，driver 会在 `dry_run: false` 下直接启动失败。

---

## 3.1 真机一键 bringup

当前仓库已经提供单独的真机 launch：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  use_fake_gripper:=false
```

它会按顺序完成：

1. 启动 `papillarray_ros2_v2`
2. 启动 `robotiq_2f85_driver`
3. 启动 `tactile_grasp_controller`
4. 等待左右触觉 topic 开始稳定刷新
5. 调用 `/robotiq/activate`
6. 等待 `/robotiq_gripper_controller/gripper_cmd`
7. 发送一次 gripper open action goal
8. 等待 open 后的机械扰动衰减
9. 调用 `/hub_0/send_bias_request`
10. 等待触觉连续回到低载基线

相关配置文件：

- [robotiq_2f85_driver.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/robotiq_2f85_driver/config/robotiq_2f85_driver.yaml)
- [robotiq_2f85_driver.hardware.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/robotiq_2f85_driver/config/robotiq_2f85_driver.hardware.yaml)
- [tactile_grasp_controller.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/tactile_grasp_controller/config/tactile_grasp_controller.yaml)
- [tactile_grasp_controller.hardware.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/tactile_grasp_controller/config/tactile_grasp_controller.hardware.yaml)
- [hardware_bringup_coordinator.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/tactile_grasp_controller/config/hardware_bringup_coordinator.yaml)

默认行为是：

- controller 用真机参数启动，但 `auto_start: false`
- bringup 不自动调用 `/tactile_grasp/start`
- 联调结束时系统停在“夹爪已激活并张开、触觉已清零、controller 仍在 IDLE”

这里有一个关键时序约束：

- Robotiq 激活阶段可能经历一次完全闭合校验
- 这段接触力不能直接作为闭环判断依据
- 因此当前推荐流程不是“先 bias 再 activate”，而是“activate -> open -> settle -> bias -> tactile clear -> start”

进入真实闭环抓取推荐两种方式：

1. 两阶段方式：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  use_fake_gripper:=false

ros2 service call /tactile_grasp/start std_srvs/srv/Trigger "{}"
```

2. 参数已确认后的一步自动闭环：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_hardware_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  use_fake_gripper:=false \
  auto_grasp:=true
```

首次真机闭环建议始终用第一种方式。

如果你还在做单层排查，也可以继续按下面的分阶段步骤单独启动每一层。

---

## 4. 第一步：只联调触觉传感器

先不要启动 driver 和 controller。

### 4.1 启动 PTS publisher

```bash
ros2 launch papillarray_ros2_v2 papillarray.launch.py \
  hub_id:=0 \
  n_sensors:=2 \
  com_port:=/dev/ttyACM0 \
  sampling_rate:=500
```

如果传感器串口不是 `/dev/ttyACM0`，改成实际值。

### 4.2 检查 topic 是否出现

```bash
ros2 topic list | grep hub_0
```

预期至少看到：

```text
/hub_0/sensor_0
/hub_0/sensor_1
```

### 4.3 检查消息是否正常刷新

```bash
ros2 topic echo /hub_0/sensor_0
ros2 topic echo /hub_0/sensor_1
```

重点看这些字段：

- `gfx`
- `gfy`
- `gfz`
- `friction_est`
- `is_contact`
- `pillars`
- `is_sd_active`

### 4.4 触觉层通过标准

- 左右 topic 都存在
- 消息持续刷新
- 手压传感器时 `is_contact` 能变化
- `pillars[*].in_contact` 会随接触变化
- `gf*` 不全为固定零值

如果这里不过，不要继续调 driver/controller。

---

## 5. 第二步：只联调 Robotiq driver

先不要启动 controller。

### 5.1 检查 driver 配置

确认文件：

- [robotiq_2f85_driver.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/robotiq_2f85_driver/config/robotiq_2f85_driver.yaml)

首次真机联调建议：

- `dry_run: true`
- `startup_activate: false`

### 5.2 先 dry-run 启动 driver

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py
```

查看状态：

```bash
ros2 topic echo /robotiq/driver/status
```

预期：

- 节点能启动
- 有状态输出
- 不触发真实串口动作

### 5.3 再切到真实 driver

推荐不要直接改基础 YAML，而是用 launch 参数或真机覆盖配置：

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py \
  serial_port:=/dev/ttyUSB0 \
  dry_run:=false
```

或者：

```bash
ros2 launch robotiq_2f85_driver robotiq_2f85_driver.launch.py \
  config_override:=/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/robotiq_2f85_driver/config/robotiq_2f85_driver.hardware.yaml
```

### 5.4 手工发最小命令

先不要直接发全闭。

例如：

```bash
ros2 topic pub --once /robotiq/command/position std_msgs/msg/Int32 "{data: 10}"
ros2 topic pub --once /robotiq/command/position std_msgs/msg/Int32 "{data: 20}"
ros2 topic pub --once /robotiq/command/position std_msgs/msg/Int32 "{data: 0}"
```

### 5.5 检查 driver 服务

```bash
ros2 service call /robotiq/open std_srvs/srv/Trigger "{}"
ros2 service call /robotiq/close std_srvs/srv/Trigger "{}"
ros2 service call /robotiq/stop std_srvs/srv/Trigger "{}"
```

注意：

- 初次测试时手随时准备断电或急停
- 如果 `close` 风险太高，先只测小位置命令，不测全闭

### 5.6 driver 层通过标准

- 能启动
- `status` 正常
- 小位置命令动作可预测
- `stop` 可用
- 没有异常抖动或失控

如果这里不过，不要继续调 controller。

---

## 6. 第三步：controller 先 dry-run

这一层的目标是确认：

- topic 接线正确
- 状态机在跑
- 不触发真实夹爪动作

### 6.1 确认 controller 配置

文件：

- [tactile_grasp_controller.yaml](/home/xiaodalaing/project/tactile_grasp_lab/ros2_ws/src/tactile_grasp_controller/config/tactile_grasp_controller.yaml)

首次联调建议：

- `dry_run: true`
- `auto_start: false`

并确认：

- `left_tactile_topic: "/hub_0/sensor_0"`
- `right_tactile_topic: "/hub_0/sensor_1"`

### 6.2 分别启动传感器和 driver

- PTS publisher 启动
- driver 启动
- driver 可以是 `dry_run: true` 或 `false`

更稳妥的首次组合建议：

- `driver.dry_run: true`
- `controller.dry_run: true`

### 6.3 启动 controller

```bash
ros2 launch tactile_grasp_controller tactile_grasp_controller.launch.py
```

### 6.4 观察 debug / state

```bash
ros2 topic echo /tactile_grasp/debug
ros2 topic echo /tactile_grasp/state
```

### 6.5 手工启动状态机

```bash
ros2 service call /tactile_grasp/start std_srvs/srv/Trigger "{}"
```

需要停止时：

```bash
ros2 service call /tactile_grasp/stop std_srvs/srv/Trigger "{}"
```

故障复位：

```bash
ros2 service call /tactile_grasp/reset_fault std_srvs/srv/Trigger "{}"
```

### 6.6 dry-run 阶段通过标准

- controller 能启动
- 能看到 `wait`、`IDLE`、`APPROACH` 等状态变化
- 压左右传感器时，`left_contact`、`right_contact`、`both_contact` 有合理变化
- `fn_left`、`fn_right`、`friction_margin` 在刷新
- 没有真实夹爪动作

---

## 7. 第四步：法向方向标定

这一步必须做。

### 7.1 观察原始力方向

只开传感器节点：

```bash
ros2 topic echo /hub_0/sensor_0
ros2 topic echo /hub_0/sensor_1
```

手动按压左右指尖，记录：

- `gfx`
- `gfy`
- `gfz`

### 7.2 判断法向轴和符号

当前 controller 默认假设：

```yaml
left_normal_sign: 1.0
right_normal_sign: 1.0
```

如果你确认“按压时 `gfz` 变负”，则对应侧应改为：

```yaml
left_normal_sign: -1.0
```

或：

```yaml
right_normal_sign: -1.0
```

### 7.3 标定通过标准

- 按压左侧时 `fn_left` 增大
- 按压右侧时 `fn_right` 增大
- 不接触时 `fn_left`、`fn_right` 接近零或稳定小值

这一步不过，禁止真机闭环。

---

## 8. 第五步：整套链路联合启动

当前仓库已提供统一 bringup launch：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py
```

常用方式：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  hub_id:=0 \
  n_sensors:=2
```

说明：

- 这里的 `com_port` 是触觉传感器串口
- Robotiq 串口默认来自 launch 参数 `gripper_com_port`

如果需要只启动部分链路：

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_sensor:=false
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_gripper_execution:=false
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_controller:=false
```

---

## 9. 第六步：从 dry-run 切到真机闭环

切换顺序建议：

1. 保持 `controller` 仍走基础配置，确认传感器和 driver 都稳定
2. 把 `driver` 切到真机
3. 先确认 driver 真机没问题
4. 再让 `controller` 叠加真机覆盖配置

不要同时第一次把两边都改成真机。

### 推荐第一次真机闭环条件

- 夹爪内先放软物体
- 物体尺寸适中
- 周围有人随时准备停止
- `approach_position_step` / `preload_position_step` / `compensate_position_step` 保持小值
- `max_safe_force_n` 保守

### 推荐第一次真机闭环流程

1. 启动真机初始化链路
2. 观察触觉消息稳定
3. 观察 driver 状态正常
4. 调用 `/tactile_grasp/start`
5. 让夹爪缓慢接触物体
6. 观察是否进入 `PRELOAD` / `HOLD`
7. 如有异常，立刻 `/robotiq/stop`

---

## 10. 常用检查命令

### topic 列表

```bash
ros2 topic list
```

### 传感器数据

```bash
ros2 topic echo /hub_0/sensor_0
ros2 topic echo /hub_0/sensor_1
```

### driver 状态

```bash
ros2 topic echo /robotiq/driver/status
```

### controller debug / state

```bash
ros2 topic echo /tactile_grasp/debug
ros2 topic echo /tactile_grasp/state
```

### 服务列表

```bash
ros2 service list
```

---

## 11. 故障排查

### 11.1 看不到 `/hub_0/sensor_0`

先检查：

- `papillarray_ros2_v2` 是否真的启动
- `com_port` 是否正确
- `n_sensors` 是否正确
- 传感器是否上电

### 11.2 controller 一直停在 waiting

先检查：

- 左右 topic 名是否和 YAML 一致
- 是否真的有左右两路消息
- `ros2 topic echo /hub_0/sensor_0`
- `ros2 topic echo /hub_0/sensor_1`

### 11.3 `fn_left` / `fn_right` 方向不对

先检查：

- `gfz` 是否真的是法向
- `left_normal_sign`
- `right_normal_sign`

### 11.4 driver 能启动但夹爪不动

先检查：

- `dry_run` 是否仍是 `true`
- 串口是否正确
- 夹爪是否供电
- 是否需要先 `activate`

### 11.5 controller 一启动就进 `FAULT`

先检查：

- 触觉数据是否超时
- `max_safe_force_n` 是否太低
- 传感器零漂是否过大
- `fn_min` 是否因为符号错误被算大

---

## 12. 联调记录建议

每次真机联调建议记录：

- 日期
- 夹爪串口
- 传感器串口
- `left_normal_sign`
- `right_normal_sign`
- `dry_run` 配置
- `approach_position_step`
- `preload_position_step`
- `compensate_position_step`
- `max_safe_force_n`
- 是否成功进入 `HOLD`
- 是否出现滑移误判
- 是否出现异常 FAULT

---

## 13. 最小安全结论

满足下面这些条件前，不要把当前系统视为“可长期自动抓取”：

- 传感器消息稳定
- driver 真机动作稳定
- 法向符号已标定
- `stop` 在真机上可用
- `HOLD` 状态行为符合预期
- 轻微滑移时不会过度补夹

一句话原则：

先证明每一层都可解释、可停止、可回退，再追求自动抓取效果。
