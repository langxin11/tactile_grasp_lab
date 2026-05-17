# tactile_grasp_lab 文档

`tactile_grasp_lab` 是一个把 Contactile PTS 触觉传感器、Robotiq 2F-85 夹爪与规则型触觉闭环控制器
统一到单一 ROS2 工作区的实验平台。

本文档涵盖架构、硬件联调流程、各 ROS2 包的职责与 Python API 参考。
仓库根的 `README.md` 仍作为 GitHub 着陆页与最简上手手册。

## 总览

```{toctree}
:maxdepth: 2
:caption: 总览

overview/getting_started
overview/architecture
```

## 硬件

```{toctree}
:maxdepth: 2
:caption: 硬件

hardware/bringup
```

## ROS2 包

```{toctree}
:maxdepth: 1
:caption: ROS2 包

packages/sensor_interfaces
packages/papillarray_ros2_v2
packages/robotiq_2f85_driver
packages/tactile_grasp_controller
packages/contactile_visualizer
```

## Python API

```{toctree}
:maxdepth: 1
:caption: API 参考

api/tactile_grasp_controller
api/robotiq_2f85_driver
api/contactile_visualizer
```

## 历史归档

```{toctree}
:maxdepth: 1
:caption: 历史归档

historical/codex_plan
```

## 索引

- {ref}`genindex`
- {ref}`modindex`
- {ref}`search`
