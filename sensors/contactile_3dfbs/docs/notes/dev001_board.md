# DEV001 传感器开发板 — 摘要

**文档**: `docs/manuals/DEV001_Datasheet_Rev1.1.pdf`  
**来源**: Contactile Pty Ltd

## 概述

DEV001 是基于 ESP32-S2-SOLO 的五端口传感器集线器，用于与 Contactile 的 I2C/SPI/UART 传感器通信。

- 尺寸: 74 × 47 × 1.6 mm
- 接口: USB-C（固件枚举为 USB CDC 虚拟串口，同时供电）
- 传感器接口: 5× FPC/FFC 连接器 (FH12-8S-0.5SH)
- 安装孔: 4× M3

## 固件

出厂预装固件提供：
- USB CDC 串口 (虚拟 COM 端口)
- 命令行交互界面
- 通过 `SensorHub.h` 可自定义固件

## 关于 `baud_rate`

数据手册说明 DEV001 固件把集成 USB 外设配置为 USB CDC 设备，主机侧会把它显示为
`/dev/ttyACM*` 一类虚拟串口。

因此应区分两件事：

- 主机软件仍然使用串口兼容 API，需要传入 `baud_rate`、`parity`、`byte_size`；
- 但这并不自动等于“DEV001 和主机之间真的在跑某个物理 UART 波特率”。

当前仓库和原厂资料中同时出现过 `9600` 和 `115200`。更稳妥的表述是：

- 这些值很可能只是 CDC 串口兼容层中的 line-coding 参数；
- 对主机到 DEV001 的 USB 链路来说，它们不应被直接理解为真实传输速率；
- 在没有更明确固件说明前，不应把 `9600` 或 `115200` 写成板卡物理链路事实。

## 串口命令

| 命令 | 说明 |
|------|------|
| `help` / `h` | 打印帮助 |
| `bias` / `b [port]` | 对指定端口传感器置零 |
| `hub mux [i2c/spi]` | 配置通信协议 |
| `info` / `i` | 打印板卡和传感器信息 |
| `mode m [i2c/spi]` | 设置传感器通信模式 |
| `reset r [port]` | 复位指定端口传感器 |
| `scan` | I2C 总线扫描（诊断用） |
| `stream [action] [port] -d [flags] -f [freq] -p [fmt]` | 数据流控制 |

### stream 命令参数

**action**: `start` / `stop`  
**data flags**: `force`, `all`, `fx`, `fy`, `fz`, `temp`（逗号分隔，无空格）  
**frequency**: 25–1000 Hz  
**print format**: `binary` / `ascii` / `none`

示例:
```
stream start -d force -f 1000 -p binary
stream start 0 -d force -f 100 -p ascii
stream stop
```

## 快速上手指南

1. 连接传感器到 DEV001
2. USB-C 连接电脑
3. 打开串口终端
4. `bias` → `stream start -d force -f 1000 -p ascii`

## 固件更新

- 按住 BOOT 按钮同时按 RESET 进入 DFU 模式
- Platform IO 项目: https://github.com/contactile/dev001.git
- 烧录后需手动按 RESET

## 指示灯

| LED | 说明 |
|-----|------|
| 红色 | RESET 引脚状态 |
| 绿色 | INT 引脚状态 |
| 蓝色 (×2) | COMMS MODE 指示 (I2C/SPI) |
