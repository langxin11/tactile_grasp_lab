"""Robotiq 2F-85 夹爪键盘交互控制脚本。

通过 Modbus RTU 串口与 Robotiq 2F-85 通信，提供 :class:`Robotiq2F85` 驱动类，
并在脚本入口提供方向键交互式开合控制（粗调 / 细调 / 全开 / 全闭 / 停止）。

仅在支持 termios 的 POSIX 终端（Linux、macOS）下可用。
"""

import contextlib
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

from pymodbus.client import ModbusSerialClient
from pymodbus.framer import FramerType

SAFE_SPEED = 10
SAFE_FORCE = 10
MIN_VALUE = 0
MAX_VALUE = 255
KEY_STEP_COARSE = 25
KEY_STEP_FINE = 5
ESC_SEQUENCE_TIMEOUT = 0.2
KEYBOARD_POLL_INTERVAL_SECONDS = 0.02
KEEPALIVE_INTERVAL_SECONDS = 0.3
STATUS_POLL_INTERVAL_SECONDS = 0.05
ACTIVATION_TIMEOUT_SECONDS = 5.0


def pack_position_register(position):
    """将目标位置字节打包为写入寄存器 0x03E9 的整数。

    Args:
        position: 期望的位置值，范围 0–255（0 全开，255 全闭）。

    Returns:
        int: 经过范围裁剪后的位置字节。
    """
    return clamp_byte(position, "position")


def pack_speed_force_register(speed, force):
    """按 Robotiq 字节顺序将速度/力度打包为寄存器 0x03EA。

    Robotiq 输出寄存器的低字节为速度、高字节为力度。

    Args:
        speed: 期望速度，范围 0–255。
        force: 期望力度，范围 0–255。

    Returns:
        int: 16 位寄存器值 ``(force << 8) | speed``。
    """
    speed = clamp_byte(speed, "speed")
    force = clamp_byte(force, "force")
    return (force << 8) | speed


class GripperError(RuntimeError):
    """夹爪无法安全完成请求时抛出的异常。"""


@dataclass
class GripperFeedback:
    """Robotiq 输入寄存器反馈解析结果。"""

    gripper_status: int
    fault_status: int
    position_request_echo: int
    position: int
    current: int

    @property
    def activation_state(self):
        """返回 gSTA 激活状态位。"""
        return (self.gripper_status >> 4) & 0x03

    @property
    def activation_echo(self):
        """返回 gACT 激活回显位。"""
        return self.gripper_status & 0x01

    @property
    def is_activation_complete(self):
        """返回是否处于激活完成状态。"""
        return self.activation_echo == 1 and self.activation_state == 0x03


def decode_feedback_registers(registers):
    """将 Robotiq 反馈寄存器解码为结构化状态。"""
    if len(registers) < 3:
        raise ValueError("反馈寄存器数量不足，至少需要 3 个寄存器")

    status_register, position_register, current_register = registers[:3]
    return GripperFeedback(
        gripper_status=(status_register >> 8) & 0xFF,
        fault_status=status_register & 0xFF,
        position_request_echo=(position_register >> 8) & 0xFF,
        position=position_register & 0xFF,
        current=(current_register >> 8) & 0xFF,
    )


def clamp_byte(value, name):
    """将夹爪命令值裁剪到 0–255 字节范围。

    超出范围时不抛错，而是限制到边界并打印警告，便于上层快速调试。

    Args:
        value: 待裁剪的整数值。
        name: 用于警告输出的参数名（如 ``"position"``）。

    Returns:
        int: 裁剪到 ``[MIN_VALUE, MAX_VALUE]`` 区间的整数。

    Raises:
        ValueError: ``value`` 无法转换为整数时抛出。
    """
    try:
        value = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是 0 到 255 之间的整数") from exc

    if value < MIN_VALUE:
        print(f"警告：{name}={value} 低于下限，已限制为 {MIN_VALUE}")
        return MIN_VALUE
    if value > MAX_VALUE:
        print(f"警告：{name}={value} 高于上限，已限制为 {MAX_VALUE}")
        return MAX_VALUE
    return value


@contextlib.contextmanager
def raw_terminal():
    """让 stdin 进入 cbreak 模式以读取单个按键。

    退出上下文时无论是否异常都会用 :func:`termios.tcsetattr` 恢复终端属性，
    避免脚本异常退出后终端失去回显。

    Yields:
        None: 仅作为上下文管理器使用。
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _read_raw_byte(timeout_seconds=None):
    """直接从文件描述符读 1 字节，绕过 Python 的 BufferedReader。

    Args:
        timeout_seconds: select 超时秒数；None 表示阻塞等待。

    Returns:
        bytes: 读到的单字节；超时则返回 b""。
    """
    fd = sys.stdin.fileno()
    if timeout_seconds is not None:
        ready = select.select([fd], [], [], timeout_seconds)[0]
        if not ready:
            return b""
    return os.read(fd, 1)


def _read_escape_sequence():
    r"""读取 ESC 之后的 ANSI 转义序列并归一化为按键名称。"""
    data = _read_raw_byte(timeout_seconds=ESC_SEQUENCE_TIMEOUT)
    if not data:
        return "ESC"

    sequence = b""
    while True:
        sequence += data
        last = chr(sequence[-1])
        if len(sequence) > 1 and (last.isalpha() or last == "~"):
            break
        data = _read_raw_byte(timeout_seconds=ESC_SEQUENCE_TIMEOUT)
        if not data:
            break

    seq = sequence.decode("ascii", errors="replace")
    arrow_map = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT", "H": "HOME", "F": "END"}
    if seq.startswith(("[", "O")):
        if seq[-1] in arrow_map:
            return arrow_map[seq[-1]]
        if seq in ("[1~", "[7~", "OH"):
            return "HOME"
        if seq in ("[4~", "[8~", "OF"):
            return "END"
    return "UNKNOWN"


def read_key():
    """阻塞读取一次按键。

    必须在 :func:`raw_terminal` 上下文内调用，否则会得到行缓冲后的输入。

    Returns:
        str: 普通可打印字符按原样返回;方向键 / Home / End 返回归一化名称
        （见 :func:`_read_escape_sequence`）;单击 Esc 返回 ``"ESC"``。
    """
    ch = _read_raw_byte()
    if ch == b"\x1b":
        return _read_escape_sequence()
    return ch.decode("ascii", errors="replace")


def wait_for_key(timeout_seconds):
    """在给定超时时间内等待一次按键，超时返回 ``None``。

    Args:
        timeout_seconds: 最长等待时间（秒）。

    Returns:
        str | None: 读取到按键时返回按键；超时则返回 ``None``。
    """
    ch = _read_raw_byte(timeout_seconds=timeout_seconds)
    if not ch:
        return None
    if ch == b"\x1b":
        return _read_escape_sequence()
    return ch.decode("ascii", errors="replace")


class Robotiq2F85:
    """Robotiq 2F-85 夹爪的 Modbus RTU 驱动。

    封装连接、激活、运动、停止等基本动作，所有运动指令都通过 Robotiq 官方文档
    定义的输出寄存器 0x03E8–0x03EA 下发。

    Attributes:
        client: 底层 :class:`pymodbus.client.ModbusSerialClient` 实例。
        slave_address: Modbus 从站地址，Robotiq 默认为 9。
        connected: 是否已成功打开串口。仅在 :meth:`connect` 成功后为 ``True``。
    """

    def __init__(self, port="/dev/ttyUSB0"):
        """初始化驱动但不立即打开串口。

        Args:
            port: 串口设备路径，默认 ``/dev/ttyUSB0``。
        """
        # 配置 Modbus RTU 串口连接
        self.client = ModbusSerialClient(
            port=port,
            framer=FramerType.RTU,
            baudrate=115200,
            timeout=1,
            stopbits=1,
            bytesize=8,
            parity="N",
        )
        self.slave_address = 9  # Robotiq 默认的从站地址通常是 9
        self.connected = False

    def _write_registers(self, address, registers, action):
        """写多个寄存器，遇到 Modbus 错误立即抛异常。

        Args:
            address: 起始寄存器地址。
            registers: 待写入的 16 位寄存器值列表。
            action: 用于错误信息的动作名（如 ``"激活夹爪"``）。

        Returns:
            pymodbus 写寄存器响应对象。

        Raises:
            GripperError: 未连接、串口写入异常或 Modbus 返回错误时抛出。
        """
        if not self.connected:
            raise GripperError("夹爪未连接，拒绝发送运动指令。")

        try:
            result = self.client.write_registers(address, registers, device_id=self.slave_address)
        except Exception as exc:
            raise GripperError(f"{action} 失败：串口写入异常：{exc}") from exc

        if result.isError():
            raise GripperError(f"{action} 失败：Modbus 返回错误：{result}")
        return result

    def connect(self):
        """打开串口连接。

        Raises:
            GripperError: 串口无法打开时抛出，常见原因为端口路径错误、
                权限不足（缺少 ``dialout`` 组）或夹爪未供电。
        """
        self.connected = bool(self.client.connect())
        if not self.connected:
            raise GripperError("连接失败，请检查端口、权限和夹爪供电。")
        print("成功连接到夹爪串口！")

    def read_feedback(self):
        """读取一次夹爪状态反馈。"""
        if not self.connected:
            raise GripperError("夹爪未连接，拒绝读取状态反馈。")

        try:
            result = self.client.read_input_registers(0x07D0, count=3, device_id=self.slave_address)
        except Exception as exc:
            raise GripperError(f"读取状态反馈失败：串口读异常：{exc}") from exc

        if result.isError():
            raise GripperError(f"读取状态反馈失败：Modbus 返回错误：{result}")

        return decode_feedback_registers(result.registers)

    def _raise_for_fault(self, feedback, action):
        """检测故障码并抛出可诊断异常。"""
        if feedback.fault_status == 0x00:
            return
        raise GripperError(
            f"{action} 失败：fault=0x{feedback.fault_status:02X}, "
            f"status=0x{feedback.gripper_status:02X}, position={feedback.position}"
        )

    def _wait_for_activation_complete(self):
        """轮询等待激活完成。"""
        deadline = time.monotonic() + ACTIVATION_TIMEOUT_SECONDS
        last_feedback = None
        while time.monotonic() < deadline:
            feedback = self.read_feedback()
            last_feedback = feedback
            self._raise_for_fault(feedback, "激活夹爪")
            if feedback.is_activation_complete:
                return
            time.sleep(STATUS_POLL_INTERVAL_SECONDS)

        if last_feedback is None:
            raise GripperError("激活超时：未读取到任何反馈。")
        raise GripperError(
            "激活超时："
            f"status=0x{last_feedback.gripper_status:02X}, "
            f"fault=0x{last_feedback.fault_status:02X}"
        )

    def ensure_activated(self):
        """确认夹爪仍处于激活完成状态。"""
        feedback = self.read_feedback()
        self._raise_for_fault(feedback, "激活状态检查")
        if not feedback.is_activation_complete:
            raise GripperError(
                "夹爪未处于激活完成状态："
                f"status=0x{feedback.gripper_status:02X}, "
                f"fault=0x{feedback.fault_status:02X}"
            )

    def activate(self):
        """激活夹爪，上电后必须调用一次。

        先清空 rACT 标志位，再写入激活指令（rACT=1, rGTO=1），并以安全速度/力度
        完成初始化校准动作，整个过程约 3 秒。

        Raises:
            GripperError: 任一寄存器写入失败时抛出。
        """
        print("正在清空之前的状态...")
        # 清除 rACT 标志位
        self._write_registers(0x03E8, [0x0000, 0x0000, 0x0000], "清空夹爪状态")
        time.sleep(1)

        print("正在激活夹爪...")
        # 写入激活指令 (rACT=1, rGTO=0)。
        # 激活完成后，发送运动指令时再置 rGTO=1。
        # 同时带上安全速度和力度，避免初始化校准过快或过猛。
        activation_speed_force = pack_speed_force_register(SAFE_SPEED, SAFE_FORCE)
        # 寄存器 0x03E8: 动作请求
        self._write_registers(0x03E8, [0x0100, 0x0000, activation_speed_force], "激活夹爪")
        self._wait_for_activation_complete()
        print("夹爪激活完成。")

    def keepalive(self):
        """在空闲等待期间轮询一次状态，避免硬件因长时间无通信进入故障。"""
        self.ensure_activated()

    def move(self, position, speed=SAFE_SPEED, force=SAFE_FORCE):
        """下发一次位置/速度/力度运动指令。

        Args:
            position: 目标位置，``0`` 全开，``255`` 全闭。
            speed: 运动速度，``0`` 最慢，``255`` 最快。默认 :data:`SAFE_SPEED`。
            force: 夹持力度，``0`` 最小，``255`` 最大。默认 :data:`SAFE_FORCE`。

        Raises:
            GripperError: 串口写入或 Modbus 返回错误时抛出。
            ValueError: 任一入参无法转为整数时抛出。
        """
        position = clamp_byte(position, "position")
        speed = clamp_byte(speed, "speed")
        force = clamp_byte(force, "force")

        # Robotiq 输出寄存器的数据区是小端字节序，这里按官方字节顺序打包。
        speed_force = pack_speed_force_register(speed, force)

        # 寄存器说明：
        # 0x03E8: 动作请求 (0x0900 表示 rACT=1, rGTO=1，允许运动)
        # 0x03E9: 目标位置请求字节
        # 0x03EA: 速度/力度字节，按 Robotiq 字节顺序打包
        registers = [0x0900, pack_position_register(position), speed_force]

        self._write_registers(0x03E8, registers, "发送运动指令")
        print(f"指令已发送 -> 位置: {position}, 速度: {speed}, 力: {force}")

    def open_gripper(self):
        """以安全速度/力度将夹爪打开到 ``position=0``。

        Raises:
            GripperError: 写入失败时抛出。
        """
        print("执行：打开夹爪")
        self.move(position=0, speed=SAFE_SPEED, force=SAFE_FORCE)

    def close_gripper(self):
        """以安全速度/力度将夹爪闭合到 ``position=255``。

        Raises:
            GripperError: 写入失败时抛出。
        """
        print("执行：关闭夹爪")
        self.move(position=255, speed=SAFE_SPEED, force=SAFE_FORCE)

    def stop(self):
        """停止当前运动但保持激活状态。

        清零目标位置和速度/力度字节，rACT 保持 1。未连接时静默跳过；
        写入失败仅打印错误，不抛异常，避免淹没主流程的退出逻辑。
        """
        if not self.connected:
            return
        try:
            self._write_registers(0x03E8, [0x0100, 0x0000, 0x0000], "停止夹爪运动")
            print("已发送停止指令。")
        except GripperError as exc:
            print(f"停止指令发送失败：{exc}")

    def close(self):
        """关闭串口连接并将 :attr:`connected` 置为 ``False``。

        未连接时为空操作，可安全重复调用，方便放在 ``finally`` 块里。
        """
        if self.connected:
            self.client.close()
            self.connected = False
            print("连接已关闭。")


if __name__ == "__main__":
    # 1. 实例化并连接
    gripper = Robotiq2F85(port="/dev/ttyUSB0")

    try:
        gripper.connect()

        # 2. 激活夹爪 (如果是刚上电，必须执行)
        gripper.activate()

        # 3. 键盘方向键交互控制
        print(
            f"键盘控制就绪：↑↓ 粗调 (±{KEY_STEP_COARSE})  ←→ 细调 (±{KEY_STEP_FINE})  "
            "Home 全开  End 全闭  Space 停止  q 退出"
        )
        current_position = 0
        next_keepalive_deadline = time.monotonic() + KEEPALIVE_INTERVAL_SECONDS

        with raw_terminal():
            while True:
                now = time.monotonic()
                timeout_seconds = min(
                    KEYBOARD_POLL_INTERVAL_SECONDS,
                    max(0.0, next_keepalive_deadline - now),
                )
                key = wait_for_key(timeout_seconds)
                if key is None:
                    if time.monotonic() >= next_keepalive_deadline:
                        gripper.keepalive()
                        next_keepalive_deadline = time.monotonic() + KEEPALIVE_INTERVAL_SECONDS
                    continue
                if key in ("q", "Q"):
                    print("退出键盘控制。")
                    break
                if key == "UP":
                    current_position = max(MIN_VALUE, current_position - KEY_STEP_COARSE)
                elif key == "DOWN":
                    current_position = min(MAX_VALUE, current_position + KEY_STEP_COARSE)
                elif key == "LEFT":
                    current_position = max(MIN_VALUE, current_position - KEY_STEP_FINE)
                elif key == "RIGHT":
                    current_position = min(MAX_VALUE, current_position + KEY_STEP_FINE)
                elif key == "HOME":
                    current_position = MIN_VALUE
                elif key == "END":
                    current_position = MAX_VALUE
                elif key == " ":
                    gripper.stop()
                    next_keepalive_deadline = time.monotonic() + KEEPALIVE_INTERVAL_SECONDS
                    continue
                else:
                    continue
                gripper.move(current_position)
                next_keepalive_deadline = time.monotonic() + KEEPALIVE_INTERVAL_SECONDS

    except KeyboardInterrupt:
        print("程序被用户中断")
        gripper.stop()
    except (GripperError, ValueError) as exc:
        print(f"安全停止：{exc}")
        gripper.stop()
    finally:
        gripper.close()
