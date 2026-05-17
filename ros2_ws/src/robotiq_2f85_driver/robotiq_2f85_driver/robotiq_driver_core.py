"""Robotiq 2F-85 驱动核心，包含 dry-run 支持与运动完成检测。

封装 Modbus 通信、命令打包与反馈解析的核心逻辑。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .safety import clamp_byte

try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.framer import FramerType
except Exception:  # pragma: no cover - optional import at runtime
    ModbusSerialClient = None
    FramerType = None


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


class RobotiqDriverCore:
    """Robotiq Modbus 接口封装。"""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        slave_address: int = 9,
        dry_run: bool = True,
    ) -> None:
        """初始化驱动核心与可选的 Modbus 客户端。

        Args:
            port: 串口设备路径。
            baudrate: 串口波特率。
            slave_address: Modbus 从站地址。
            dry_run: 是否启用 dry-run 模式。

        Raises:
            GripperError: 非 dry-run 且缺少 pymodbus 时抛出。
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_address = slave_address
        self.dry_run = dry_run
        self.connected = False
        self.last_command: CommandRecord | None = None
        self.last_feedback: GripperFeedback | None = None

        self.client = None
        if not self.dry_run:
            if ModbusSerialClient is None or FramerType is None:
                raise GripperError(
                    "pymodbus is not available in the Python interpreter used by this "
                    "ROS 2 node; rebuild with the repo .venv, for example: "
                    "'source .venv/bin/activate && ./scripts/build_ros2.sh "
                    "--packages-select robotiq_2f85_driver'."
                )
            self.client = ModbusSerialClient(
                port=port,
                framer=FramerType.RTU,
                baudrate=baudrate,
                timeout=1,
                stopbits=1,
                bytesize=8,
                parity="N",
            )

    def synthetic_feedback(self) -> GripperFeedback:
        """在 dry-run 模式下生成模拟反馈。

        Returns:
            GripperFeedback: 生成的反馈对象。
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

    def _write_registers(self, address: int, registers: list[int], action: str) -> None:
        """写入 Modbus 寄存器。

        Args:
            address: 寄存器起始地址。
            registers: 待写入的寄存器值列表。
            action: 用于错误消息的操作标识。

        Raises:
            GripperError: 连接不可用或写入失败时抛出。
        """
        if self.dry_run:
            return
        if not self.connected or self.client is None:
            raise GripperError("Gripper is not connected; refusing to send commands.")

        try:
            result = self.client.write_registers(address, registers, device_id=self.slave_address)
        except Exception as exc:  # pragma: no cover - hardware path
            raise GripperError(f"{action} failed due to serial write error: {exc}") from exc

        if result.isError():  # pragma: no cover - hardware path
            raise GripperError(f"{action} failed with Modbus error: {result}")

    def read_feedback(self) -> GripperFeedback | None:
        """读取当前夹爪反馈寄存器。

        Returns:
            GripperFeedback | None: 反馈对象，dry-run 下返回模拟反馈。

        Raises:
            GripperError: 连接不可用或读取失败时抛出。
        """
        if self.dry_run:
            return self.synthetic_feedback()
        if not self.connected or self.client is None:
            raise GripperError("Gripper is not connected; refusing to read feedback.")

        try:
            result = self.client.read_input_registers(0x07D0, count=3, device_id=self.slave_address)
        except Exception as exc:  # pragma: no cover - hardware path
            raise GripperError(f"read feedback failed due to serial read error: {exc}") from exc

        if result.isError():  # pragma: no cover - hardware path
            raise GripperError(f"read feedback failed with Modbus error: {result}")

        if len(result.registers) < 3:  # pragma: no cover - defensive check
            raise GripperError("read feedback returned too few registers.")

        status_register, fault_register, position_register = result.registers[:3]
        feedback = GripperFeedback(
            gripper_status=(status_register >> 8) & 0xFF,
            fault_status=(fault_register >> 8) & 0xFF,
            position_request_echo=fault_register & 0xFF,
            position=(position_register >> 8) & 0xFF,
            current=position_register & 0xFF,
        )
        self.last_feedback = feedback
        return feedback

    def _raise_for_fault(self, feedback: GripperFeedback, context: str) -> None:
        """根据反馈判断是否需要抛出故障异常。

        Args:
            feedback: 当前反馈对象。
            context: 故障上下文描述。

        Raises:
            GripperError: 反馈包含故障码时抛出。
        """
        if feedback.fault_status == 0x00:
            return
        raise GripperError(
            f"{context} fault=0x{feedback.fault_status:02X} ({feedback.fault_text}), "
            f"status=0x{feedback.gripper_status:02X}, position={feedback.position}, "
            f"current={feedback.current_milliamps}mA"
        )

    def _wait_for_activation_complete(self) -> GripperFeedback:
        """等待激活完成并返回最后一次反馈。

        Returns:
            GripperFeedback: 激活完成时的反馈。

        Raises:
            GripperError: 激活超时或反馈异常时抛出。
        """
        deadline = time.monotonic() + ACTIVATION_TIMEOUT_SECONDS
        last_feedback: GripperFeedback | None = None
        while time.monotonic() < deadline:
            feedback = self.read_feedback()
            if feedback is None:  # pragma: no cover - caller guarantees non-dry-run
                raise GripperError("Activation feedback unavailable in dry-run mode.")
            last_feedback = feedback
            self._raise_for_fault(feedback, "activate")
            if feedback.is_activation_complete:
                return feedback
            time.sleep(STATUS_POLL_INTERVAL_SECONDS)

        if last_feedback is None:
            raise GripperError("Activation timed out before any feedback was received.")

        raise GripperError(
            "Activation did not complete before timeout: "
            f"status=0x{last_feedback.gripper_status:02X}, "
            f"fault=0x{last_feedback.fault_status:02X} ({last_feedback.fault_text})"
        )

    def ensure_activated(self) -> None:
        """确保夹爪已完成激活后再执行指令。

        Raises:
            GripperError: 未完成激活或存在故障时抛出。
        """
        feedback = self.read_feedback()
        if feedback is None:  # pragma: no cover - dry-run path already handled
            return
        self._raise_for_fault(feedback, "pre-motion")
        if not feedback.is_activation_complete:
            raise GripperError(
                "Gripper is not activation-complete; "
                f"status=0x{feedback.gripper_status:02X}, "
                f"fault=0x{feedback.fault_status:02X} ({feedback.fault_text})"
            )

    def connect(self) -> None:
        """打开 Modbus 串口连接。

        Raises:
            GripperError: 连接失败或客户端未初始化时抛出。
        """
        if self.dry_run:
            self.connected = True
            self.synthetic_feedback()
            return
        if self.client is None:  # pragma: no cover - guarded by __init__
            raise GripperError("Modbus client is not initialized.")
        self.connected = bool(self.client.connect())
        if not self.connected:
            raise GripperError("Failed to connect to the gripper serial port.")

    def activate(self) -> None:
        """执行 Robotiq 激活流程。

        Raises:
            GripperError: 激活失败或通信异常时抛出。
        """
        if self.dry_run:
            self.connected = True
            self.synthetic_feedback()
            return
        self._write_registers(0x03E8, [0x0000, 0x0000, 0x0000], "reset gripper state")
        time.sleep(RESET_SETTLE_SECONDS)
        activation_speed_force = pack_speed_force_register(SAFE_SPEED, SAFE_FORCE)
        self._write_registers(
            0x03E8,
            [0x0100, 0x0000, activation_speed_force],
            "activate gripper",
        )
        self._wait_for_activation_complete()

    def move(self, position: int, speed: int = SAFE_SPEED, force: int = SAFE_FORCE) -> None:
        """发送运动指令但不等待完成。

        Args:
            position: 目标位置字节。
            speed: 速度字节。
            force: 力度字节。

        Raises:
            GripperError: 发送指令或读取反馈失败时抛出。
        """
        position = clamp_byte(position, "position")
        speed = clamp_byte(speed, "speed")
        force = clamp_byte(force, "force")

        self.last_command = CommandRecord(
            position=position,
            speed=speed,
            force=force,
            simulated=self.dry_run,
        )

        if self.dry_run:
            self.synthetic_feedback()
            return

        self.ensure_activated()
        speed_force = pack_speed_force_register(speed, force)
        registers = [0x0900, pack_position_register(position), speed_force]
        self._write_registers(0x03E8, registers, "send motion command")
        time.sleep(STATUS_POLL_INTERVAL_SECONDS)
        feedback = self.read_feedback()
        if feedback is not None:
            self._raise_for_fault(feedback, "motion")

    def wait_for_motion_complete(
        self,
        target_position: int,
        timeout_s: float,
        position_tolerance: int,
    ) -> MotionResult:
        """等待运动到位、停滞或故障。

        Args:
            target_position: 目标位置字节。
            timeout_s: 超时时间（秒）。
            position_tolerance: 位置容差字节。

        Returns:
            MotionResult: 运动完成结果。

        Raises:
            GripperError: 运动超时或反馈异常时抛出。
        """
        target_position = clamp_byte(target_position, "target_position")
        position_tolerance = max(0, int(position_tolerance))
        if self.dry_run:
            feedback = self.synthetic_feedback()
            return MotionResult(
                target_position=target_position,
                final_feedback=feedback,
                reached_goal=True,
                stalled=False,
            )

        deadline = time.monotonic() + float(timeout_s)
        last_feedback: GripperFeedback | None = None
        while time.monotonic() < deadline:
            feedback = self.read_feedback()
            if feedback is None:  # pragma: no cover - non-dry-run callers only
                raise GripperError("Motion feedback unavailable in dry-run mode.")
            last_feedback = feedback
            self._raise_for_fault(feedback, "motion")

            reached_goal = abs(feedback.position - target_position) <= position_tolerance or (
                feedback.object_status == OBJECT_STATUS_AT_REQUESTED_POSITION
                and abs(feedback.position_request_echo - target_position) <= position_tolerance
            )
            stalled = (
                feedback.object_status
                in (OBJECT_STATUS_OPENING_CONTACT, OBJECT_STATUS_CLOSING_CONTACT)
                and not reached_goal
            )
            if reached_goal or stalled:
                return MotionResult(
                    target_position=target_position,
                    final_feedback=feedback,
                    reached_goal=reached_goal,
                    stalled=stalled,
                )
            time.sleep(STATUS_POLL_INTERVAL_SECONDS)

        if last_feedback is None:
            raise GripperError("Motion timed out before any feedback was received.")
        raise GripperError(
            "Motion did not settle before timeout: "
            f"target={target_position}, position={last_feedback.position}, "
            f"status=0x{last_feedback.gripper_status:02X}, "
            f"fault=0x{last_feedback.fault_status:02X} ({last_feedback.fault_text})"
        )

    def open_gripper(self) -> None:
        """完全打开夹爪。

        Raises:
            GripperError: 指令执行失败时抛出。
        """
        self.move(position=0)

    def close_gripper(self) -> None:
        """完全闭合夹爪。

        Raises:
            GripperError: 指令执行失败时抛出。
        """
        self.move(position=255)

    def stop(self) -> None:
        """保持激活状态下停止夹爪运动。

        Raises:
            GripperError: 停止命令发送失败时抛出。
        """
        if self.dry_run:
            self.synthetic_feedback()
            return
        if not self.connected:
            return
        self._write_registers(0x03E8, [0x0100, 0x0000, 0x0000], "stop gripper motion")
        time.sleep(STATUS_POLL_INTERVAL_SECONDS)
        self.read_feedback()

    def close(self) -> None:
        """关闭 Modbus 串口连接。"""
        if self.client is not None and self.connected:
            self.client.close()
        self.connected = False
