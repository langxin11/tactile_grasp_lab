"""Robotiq 2F-85 内存模拟驱动实现。

替代原 ``RobotiqDriverCore`` 的 ``dry_run`` 路径，给单元测试与桌面演示
提供一个不依赖串口、不依赖 pymodbus 的驱动实现。行为故意保持简化：
``move`` 后立即「到位」，反馈中的位置直接等于命令位置；不模拟逐步运动、
不模拟物体接触。如未来 RL 需要更真实的仿真，可单独写 ``MujocoDriver``。
"""

from __future__ import annotations

from .driver import (
    OBJECT_STATUS_AT_REQUESTED_POSITION,
    SAFE_FORCE,
    SAFE_SPEED,
    CommandRecord,
    GripperFeedback,
    MotionResult,
)
from .safety import clamp_byte


class FakeDriver:
    """无硬件的内存模拟驱动。"""

    dry_run: bool = True

    def __init__(
        self,
        port: str = "/dev/null",
        baudrate: int = 115200,
        slave_address: int = 9,
    ) -> None:
        """初始化模拟驱动。

        Args:
            port: 串口路径（仅为对齐 Protocol 字段，不实际使用）。
            baudrate: 串口波特率（同上）。
            slave_address: Modbus 从站地址（同上）。
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_address = slave_address
        self.connected = False
        self.last_command: CommandRecord | None = None
        self.last_feedback: GripperFeedback | None = None

    def synthetic_feedback(self) -> GripperFeedback:
        """根据最近一条命令生成模拟反馈。

        Returns:
            GripperFeedback: 模拟反馈对象。
        """
        position = self.last_command.position if self.last_command is not None else 0
        feedback = GripperFeedback(
            gripper_status=0x31 | (OBJECT_STATUS_AT_REQUESTED_POSITION << 6),
            fault_status=0x00,
            position_request_echo=position,
            position=position,
            current=0,
        )
        self.last_feedback = feedback
        return feedback

    def read_feedback(self) -> GripperFeedback | None:
        """返回模拟反馈。"""
        return self.synthetic_feedback()

    def ensure_activated(self) -> None:
        """模拟驱动恒为激活态，no-op。"""
        return

    def connect(self) -> None:
        """模拟打开串口连接。"""
        self.connected = True
        self.synthetic_feedback()

    def activate(self) -> None:
        """模拟夹爪激活流程。"""
        self.connected = True
        self.synthetic_feedback()

    def move(
        self,
        position: int,
        speed: int = SAFE_SPEED,
        force: int = SAFE_FORCE,
    ) -> None:
        """记录命令并刷新模拟反馈。

        Args:
            position: 目标位置字节。
            speed: 速度字节。
            force: 力度字节。
        """
        position = clamp_byte(position, "position")
        speed = clamp_byte(speed, "speed")
        force = clamp_byte(force, "force")
        self.last_command = CommandRecord(
            position=position,
            speed=speed,
            force=force,
            simulated=True,
        )
        self.synthetic_feedback()

    def wait_for_motion_complete(
        self,
        target_position: int,
        timeout_s: float,
        position_tolerance: int,
    ) -> MotionResult:
        """模拟运动立即到位。

        Args:
            target_position: 目标位置字节。
            timeout_s: 超时时间（秒，模拟模式下忽略）。
            position_tolerance: 位置容差字节（模拟模式下忽略）。

        Returns:
            MotionResult: 立即到位的结果。
        """
        del timeout_s, position_tolerance
        target_position = clamp_byte(target_position, "target_position")
        feedback = self.synthetic_feedback()
        return MotionResult(
            target_position=target_position,
            final_feedback=feedback,
            reached_goal=True,
            stalled=False,
        )

    def open_gripper(self) -> None:
        """完全打开夹爪。"""
        self.move(position=0)

    def close_gripper(self) -> None:
        """完全闭合夹爪。"""
        self.move(position=255)

    def stop(self) -> None:
        """模拟停止运动。"""
        self.synthetic_feedback()

    def close(self) -> None:
        """模拟关闭串口连接。"""
        self.connected = False
