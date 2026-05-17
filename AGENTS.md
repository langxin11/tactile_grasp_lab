# AGENTS.md

## 1. 作用范围

本文件适用于当前仓库 `tactile_grasp_lab/` 及其下所有代码、文档和测试。

它只做三件事：

- 说明当前仓库的真实基线；
- 约束高风险改动，尤其是真机相关路径；
- 约定人类开发者和 agent 的默认工作方式。

关于 ROS2 的目标架构与迁移方向，见 [docs/ros2_repository_architecture.md](/home/xiaodalaing/project/tactile_grasp_lab/docs/ros2_repository_architecture.md)。

---

## 2. 当前基线

开始任何修改前，先接受以下事实：

- 当前仓库根目录是 `tactile_grasp_lab/`；
- 历史文档中仍可能出现 `robotiq2f85_control/` 等旧名字；
- 根目录保留少量历史遗留验证脚本，其中包括 `main.py`；
- 当前仓库统一使用根目录 `.venv`；
- ROS2 构建统一通过根目录脚本 `./scripts/build_ros2.sh`；
- 当前仓库已存在的 ROS2 package 至少包括：
  - `sensor_interfaces`
  - `papillarray_ros2_v2`
  - `robotiq_2f85_driver`
  - `robotiq_2f85_gripper_execution`
  - `tactile_grasp_controller`
  - `contactile_visualizer`
  - `buttonsensor_ros2_v1`

因此：

- 不要把计划中的接口、依赖或架构写成已经存在；
- 不要因为旧文档命名遗留而误判当前仓库结构；
- 修改文档时，先纠正事实，再描述目标。

---

## 3. 总体原则

默认按以下顺序决策：

1. 硬件安全；
2. 真实仓库现状；
3. 清晰边界；
4. 可测试性；
5. 实现便利性。

一句话原则：

先保证工程真实、边界清晰、默认安全，再追求功能完整。

---

## 4. 结构与边界

结构约束：

- 根目录允许保留少量轻量级历史验证脚本；
- 新的 ROS2 节点、launch、配置、消息与服务定义进入 `ros2_ws/src/` 下相应 package；
- launch 文件只负责装配，不写业务逻辑；
- 配置文件放各 package 的 `config/`；
- 与供应商 SDK、原厂示例、实验素材有关的内容应与主控制链路隔离。

包职责边界：

- `robotiq_2f85_driver`
  - 作为 legacy Python/pymodbus 回退路径保留；
  - 不作为默认 bringup 主路径。
- `robotiq_2f85_gripper_execution`
  - 负责默认 Robotiq 2F-85 ros2_control 执行层；
  - 不负责触觉特征提取、状态机或策略。
- `tactile_grasp_controller`
  - 负责触觉特征、状态机、闭环控制编排；
  - 不直接做串口或 Modbus 访问。
- `sensor_interfaces`
  - 负责消息与服务定义；
  - 不放控制逻辑和设备驱动。
- `papillarray_ros2_v2`
  - 负责 Contactile PTS SDK 接入、传感器数据发布及相关服务；
  - 不负责夹爪控制。
- `contactile_visualizer`
  - 负责可视化和调试展示；
  - 不改变主控制行为。
- `buttonsensor_ros2_v1`
  - 负责按钮传感器侧 ROS2 接入；
  - 不扩散其设备侧实现细节到其他 package。

禁止：

- 把新 ROS2 节点继续堆进根目录；
- 把串口访问写进 launch 文件；
- 在 controller 包里直接调用 `pymodbus`；
- 为了省事在多个 package 中复制相同安全逻辑。

---

## 5. 实现规范

ROS2 与环境：

- 统一使用根目录 `.venv`；
- ROS2 构建统一使用 `./scripts/build_ros2.sh`；
- 不要在本仓库直接运行裸 `colcon build`；
- 节点名、topic、service、parameter 保持稳定并尽量参数化；
- 所有硬件相关节点默认支持 `dry_run`。

Python 代码：

- Python 版本以仓库实际环境为准，当前 `pyproject.toml` 为 `>=3.12`；
- 新代码默认带类型标注；
- 公共类、函数、模块写简短 docstring；
- 代码风格以 PEP 8 为基础，优先采用中文 Google Python 风格指南；
- `*_node.py` 负责节点装配，`*_core.py` 负责底层逻辑；
- 纯计算与 side effect 分离；
- 异常处理必须可诊断，不能静默吞错。

测试：

- 优先做纯逻辑单元测试；
- 真机相关测试不得进入默认自动化测试；
- `dry_run` 分支应可测试。

---

## 6. 硬件安全

这是本仓库最高优先级约束。

- 所有新节点默认 `dry_run: true`；
- 未经用户明确要求，不执行真实串口命令；
- agent 不主动访问 `/dev/ttyUSB*`、`/dev/ttyACM*`；
- 无明确授权，不运行会驱动真实夹爪或真实传感器动作的 launch / 脚本；
- 遇到串口、Modbus、超时异常时，必须走安全停止路径。

代码层要求：

- 位置、速度、力度等硬件输入都要 clamp；
- 发送命令前检查连接状态；
- 失败路径必须可观测，不能静默吞错；
- 不把人工确认流程伪装成自动测试。

---

## 7. Agent 工作方式

agent 在本仓库默认遵循以下流程：

1. 先读 `README.md`、`pyproject.toml` 和相关 package 文件；
2. 判断任务属于 legacy 脚本、ROS2 工程、传感器实验层还是文档层；
3. 若涉及硬件控制，先确认是否必须保持 `dry_run`；
4. 先检查仓库现状，再做结构或实现改动；
5. 优先做小步、可验证的修改。

agent 必须：

- 明确将修改哪些文件；
- 不虚构本地不存在的文件、消息、接口或依赖；
- 不把 planned API 说成 existing API；
- 不覆盖用户未授权的真机路径；
- 不无故删除仍在仓库中承担历史兼容作用的轻量脚本。

提交策略：

- 提交尽量小而清晰；
- 文档、结构、实现、测试优先分开提交；
- 若目标架构发生变化，优先更新 `docs/ros2_repository_architecture.md`，不要把设计讨论继续堆进 `AGENTS.md`。
