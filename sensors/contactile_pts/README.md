# Contactile PTS (PapillArray) 触觉传感器

PapillArray 触觉传感器实验层，提供 C++/Python 数据采集、ROS2 驱动节点及可视化工具。

## 目录结构

```
contactile_pts/
├── cpp/          # C++ 最小读取示例 (CMake, 依赖 vendor/ 中的 libPTSDK.a)
├── python/       # Python 采集与可视化 (uv 管理, dearpygui 可视化)
├── config/       # 运行配置文件 (env.sh, pts.yaml)
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
cd sensors/contactile_pts/python
uv sync
uv run python quick_read.py
```

或使用脚本：

```bash
bash sensors/contactile_pts/scripts/setup.sh
bash sensors/contactile_pts/scripts/run_python.sh
```

### C++ 编译运行

```bash
bash sensors/contactile_pts/scripts/run_cpp.sh
```

### ROS2 节点

```bash
bash sensors/contactile_pts/scripts/run_ros2.sh
```

## 配置文件

- `config/env.sh` — 环境变量，定义 monorepo 根路径及 vendor SDK 位置
- `config/pts.yaml` — 传感器运行时参数

## 手册与文档

传感器原厂手册位于 `docs/manuals/`，包含 PDF 原文及 MinerU 解析的 markdown 版本：

| 手册 | 内容 |
|------|------|
| PTSDK_2.0_MAN_DEC21 | C++ SDK 开发手册 |
| PTSCOM_2.0_SPEC_SEP22 | 通讯协议规范 |
| PTSPython_1.0_MAN_MAR25 | Python SDK 手册 |
| PTSVIS_2.1_MAN_MAR23 | 可视化工具手册 |

## Vendor SDK

本项目通过 `vendor/contactile_pts/` 引用原厂 SDK：

- `C++LIN/Include/` — C++ 头文件
- `C++LIN/Library/` — 静态库 (libPTSDK.a)
- `PythonLIN/` — Python 预编译 wheel (ptsdk_cxx_pybind)
