"""Robotiq 2F-85 驱动接口契约与共享类型。

定义 Driver Protocol、寄存器打包工具、反馈数据类以及命令结果数据类。
真实硬件实现见 :mod:`.pymodbus_driver`；内存模拟实现见 :mod:`.fake_driver`。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .safety import clamp_byte

# =============================================================================
# 共享常量
# =============================================================================

SAFE_SPEED = 10
SAFE_FORCE = 10
RESET_SETTLE_SECONDS = 0.2
ACTIVATION_TIMEOUT_SECONDS = 5.0
STATUS_POLL_INTERVAL_SECONDS = 0.05

FAULT_STATUS_TEXT = {
    0x00: "no fault",
    0x05: "action delayed; activation must complete first",
    0x07: "activation bit must be set before action",
    0x08: "maximum operating temperature exceeded",
    0x09: "no communication during at least 1 second",
    0x0A: "under minimum operating voltage",
    0x0B: "automatic release in progress",
    0x0C: "internal fault",
    0x0D: "activation fault; verify no interference occurred",
    0x0E: "overcurrent triggered",
    0x0F: "automatic release completed",
}

OBJECT_STATUS_MOVING = 0x00
OBJECT_STATUS_OPENING_CONTACT = 0x01
OBJECT_STATUS_CLOSING_CONTACT = 0x02
OBJECT_STATUS_AT_REQUESTED_POSITION = 0x03


# =============================================================================
# 寄存器打包工具
# =============================================================================


def pack_position_register(position: int) -> int:
    """将 Robotiq 目标位置字节打包为寄存器值。

    Args:
        position: 目标位置字节。

    Returns:
        int: 打包后的寄存器值。

    Raises:
        ValueError: 参数无法转换为有效字节时抛出。
    """
    return clamp_byte(position, "position")


def pack_speed_force_register(speed: int, force: int) -> int:
    """按 Robotiq 寄存器字节序打包速度和力度。

    Args:
        speed: 速度字节。
        force: 力度字节。

    Returns:
        int: 打包后的寄存器值。

    Raises:
        ValueError: 参数无法转换为有效字节时抛出。
    """
    speed = clamp_byte(speed, "speed")
    force = clamp_byte(force, "force")
    return (force << 8) | speed


# =============================================================================
# 异常与数据类
# =============================================================================


class GripperError(RuntimeError):
    """夹爪无法安全执行指令时抛出。"""


@dataclass
class CommandRecord:
    """记录最近一次接受的指令。

    Attributes:
        position: 目标位置字节。
        speed: 速度字节。
        force: 力度字节。
        simulated: 是否为模拟指令。
    """

    position: int
    speed: int
    force: int
    simulated: bool


@dataclass
class GripperFeedback:
    """Robotiq 夹爪反馈数据解析结果。

    Attributes:
        gripper_status: 状态字节。
        fault_status: 故障字节。
        position_request_echo: 位置回显字节。
        position: 当前位置信息。
        current: 电流原始值。
    """

    gripper_status: int
    fault_status: int
    position_request_echo: int
    position: int
    current: int

    @property
    def activation_state(self) -> int:
        """返回夹爪激活状态位。"""
        return (self.gripper_status >> 4) & 0x03

    @property
    def activation_echo(self) -> int:
        """返回夹爪激活指令回显位。"""
        return self.gripper_status & 0x01

    @property
    def go_to_echo(self) -> int:
        """返回夹爪运动指令回显位。"""
        return (self.gripper_status >> 3) & 0x01

    @property
    def object_status(self) -> int:
        """返回夹爪对象状态位。"""
        return (self.gripper_status >> 6) & 0x03

    @property
    def is_activation_complete(self) -> bool:
        """返回夹爪激活是否完成。"""
        return self.activation_echo == 1 and self.activation_state == 0x03

    @property
    def current_milliamps(self) -> int:
        """返回夹爪当前电流（毫安）。"""
        return 10 * self.current

    @property
    def fault_text(self) -> str:
        """返回夹爪故障状态的文本描述。"""
        return FAULT_STATUS_TEXT.get(self.fault_status, "unknown fault")


@dataclass
class MotionResult:
    """等待运动完成的结果。

    Attributes:
        target_position: 目标位置字节。
        final_feedback: 最终反馈。
        reached_goal: 是否到达目标。
        stalled: 是否因接触而停滞。
    """

    target_position: int
    final_feedback: GripperFeedback
    reached_goal: bool
    stalled: bool


# =============================================================================
# Driver Protocol
# =============================================================================


@runtime_checkable
class Driver(Protocol):
    """Robotiq 2F-85 驱动接口契约。

    满足该 Protocol 的实现由上层 :class:`RobotiqDriverNode` 通过工厂在启动时择一注入。
    现有实现包括真实硬件 :class:`PymodbusDriver` 与内存模拟 :class:`FakeDriver`；
    未来 sim2real 流水线可补充 ``MujocoDriver`` 等同样满足本契约的实现。
    """

    port: str
    baudrate: int
    slave_address: int
    dry_run: bool
    connected: bool
    last_command: CommandRecord | None
    last_feedback: GripperFeedback | None

    def connect(self) -> None:
        """打开与夹爪的通信。"""
        ...

    def activate(self) -> None:
        """执行夹爪激活流程。"""
        ...

    def ensure_activated(self) -> None:
        """确认夹爪已完成激活，否则抛出异常。"""
        ...

    def move(
        self,
        position: int,
        speed: int = SAFE_SPEED,
        force: int = SAFE_FORCE,
    ) -> None:
        """下发一次运动命令但不等待完成。"""
        ...

    def wait_for_motion_complete(
        self,
        target_position: int,
        timeout_s: float,
        position_tolerance: int,
    ) -> MotionResult:
        """阻塞等待夹爪到位、停滞或故障。"""
        ...

    def read_feedback(self) -> GripperFeedback | None:
        """读取一次反馈寄存器。"""
        ...

    def open_gripper(self) -> None:
        """完全打开夹爪。"""
        ...

    def close_gripper(self) -> None:
        """完全闭合夹爪。"""
        ...

    def stop(self) -> None:
        """保持激活状态下停止当前运动。"""
        ...

    def close(self) -> None:
        """关闭通信链路。"""
        ...
