"""Robotiq 2F-85 夹爪驱动包。

提供基于 Modbus RTU 的二指夹爪控制接口。驱动通过 :class:`Driver`
Protocol 与上层解耦，真实硬件实现见 :class:`PymodbusDriver`，内存模拟
实现（替代原 dry-run 路径）见 :class:`FakeDriver`。
"""

from .driver import (
    CommandRecord,
    Driver,
    GripperError,
    GripperFeedback,
    MotionResult,
)
from .fake_driver import FakeDriver
from .pymodbus_driver import PymodbusDriver

__all__ = [
    "CommandRecord",
    "Driver",
    "FakeDriver",
    "GripperError",
    "GripperFeedback",
    "MotionResult",
    "PymodbusDriver",
]
