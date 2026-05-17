# Contactile 3DFBS 三维力按钮传感器

3DFBS (3D Force Button Sensor) 三维力按钮传感器实验层，提供 C++/Python 数据采集及 ROS2 驱动节点。

3DFBS 通过 DEV001 通迅集线器连接，最多支持 10 个传感器（端口 A: SEN0-SEN4，端口 B: SEN5-SEN9），每个传感器报告全局 XYZ 合力。

## 目录结构

```
contactile_3dfbs/
├── cpp/          # C++ 最小读取示例 (CMake, 依赖 vendor/ 中的 libPTSDK.a)
├── python/       # Python 采集与可视化 (uv 管理, matplotlib 可视化)
├── config/       # 运行配置文件 (env.sh, dev001.yaml)
├── scripts/      # 一键运行脚本 (setup, cpp, python, ros2)
├── docs/         # 传感器手册 (PDF + MinerU 解析 markdown)
├── udev/         # USB 设备 udev 规则
└── README.md
```

## 环境要求

| 组件 | 要求 |
|------|------|
| Python | 3.10 (受限于 vendor 预编译 wheel) |
| C++ 编译器 | GCC ≥ 9, CMake ≥ 3.16 |
| ROS2 | Jazzy (可选，仅 ros2 脚本需要) |
| 系统 | Linux x86_64 |

## 快速开始

### Python 采集

```bash
cd sensors/contactile_3dfbs/python
uv sync
uv run python quick_read.py
```

或使用脚本：

```bash
bash sensors/contactile_3dfbs/scripts/setup.sh
bash sensors/contactile_3dfbs/scripts/run_python.sh
```

### C++ 编译运行

```bash
bash sensors/contactile_3dfbs/scripts/run_cpp.sh
```

### ROS2 节点

```bash
bash sensors/contactile_3dfbs/scripts/run_ros2.sh
```

## 配置文件

- `config/env.sh` — 环境变量，定义 monorepo 根路径及 vendor SDK 位置
- `config/dev001.yaml` — DEV001 集线器及传感器配置

## 手册与文档

传感器原厂手册位于 `docs/manuals/`，包含 PDF 原文及 MinerU 解析的 markdown 版本：

| 手册 | 内容 |
|------|------|
| 3DFBSC++LIN_B1.0_MAN_DEC24 | C++ SDK 开发手册 |
| 3DFBSPython_B1.0_MAN_FEB25 | Python SDK 手册 |
| DEV001_1.6_MAN_SEP23 | DEV001 通迅集线器手册 |

## Vendor SDK

本项目通过 `vendor/contactile_3dfbs/` 引用原厂 SDK：

- `C++LIN/Include/` — C++ 头文件
- `C++LIN/Library/` — 静态库 (libPTSDK.a)
- `PythonLIN/` — Python 预编译 wheel (FBS3D_CXX_Pybind)
