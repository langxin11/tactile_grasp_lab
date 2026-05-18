"""Robotiq 2F-85 真硬件驱动实现（pymodbus + RS-485）。

封装与夹爪的 Modbus RTU 通信：命令打包、寄存器写入、状态读取、
激活流程、运动完成判定与故障处理。``dry_run`` 模拟反馈由
:mod:`.fake_driver` 提供。
"""

from __future__ import annotations

import time

from .driver import (
    ACTIVATION_TIMEOUT_SECONDS,
    OBJECT_STATUS_AT_REQUESTED_POSITION,
    OBJECT_STATUS_CLOSING_CONTACT,
    OBJECT_STATUS_OPENING_CONTACT,
    RESET_SETTLE_SECONDS,
    SAFE_FORCE,
    SAFE_SPEED,
    STATUS_POLL_INTERVAL_SECONDS,
    CommandRecord,
    GripperError,
    GripperFeedback,
    MotionResult,
    pack_position_register,
    pack_speed_force_register,
)
from .safety import clamp_byte

try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.framer import FramerType
except Exception:  # pragma: no cover - optional import at runtime
    ModbusSerialClient = None
    FramerType = None


class PymodbusDriver:
    """通过 pymodbus 与真实 Robotiq 2F-85 夹爪通信的驱动实现。"""

    dry_run: bool = False

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        slave_address: int = 9,
    ) -> None:
        """初始化 Modbus 串口客户端。

        Args:
            port: 串口设备路径。
            baudrate: 串口波特率。
            slave_address: Modbus 从站地址。

        Raises:
            GripperError: 缺少 pymodbus 依赖时抛出。
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_address = slave_address
        self.connected = False
        self.last_command: CommandRecord | None = None
        self.last_feedback: GripperFeedback | None = None

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

    def _write_registers(self, address: int, registers: list[int], action: str) -> None:
        """写入 Modbus 寄存器。

        Args:
            address: 寄存器起始地址。
            registers: 待写入的寄存器值列表。
            action: 用于错误消息的操作标识。

        Raises:
            GripperError: 连接不可用或写入失败时抛出。
        """
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
            GripperFeedback | None: 反馈对象。

        Raises:
            GripperError: 连接不可用或读取失败时抛出。
        """
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

        status_register, position_register, current_register = result.registers[:3]
        feedback = GripperFeedback(
            gripper_status=(status_register >> 8) & 0xFF,
            fault_status=status_register & 0xFF,
            position_request_echo=(position_register >> 8) & 0xFF,
            position=position_register & 0xFF,
            current=(current_register >> 8) & 0xFF,
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
            if feedback is None:  # pragma: no cover - hardware path always returns feedback
                raise GripperError("Activation feedback unavailable.")
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
        if feedback is None:  # pragma: no cover - hardware path always returns feedback
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
            simulated=False,
        )

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

        deadline = time.monotonic() + float(timeout_s)
        last_feedback: GripperFeedback | None = None
        while time.monotonic() < deadline:
            feedback = self.read_feedback()
            if feedback is None:  # pragma: no cover - hardware path always returns feedback
                raise GripperError("Motion feedback unavailable.")
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
