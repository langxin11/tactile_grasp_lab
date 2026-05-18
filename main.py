"""Robotiq 2F-85 夹爪键盘交互控制脚本。

通过 Modbus RTU 串口与 Robotiq 2F-85 通信，提供 :class:`Robotiq2F85` 驱动类，
并在脚本入口提供方向键交互式开合控制（粗调 / 细调 / 全开 / 全闭 / 停止）。

仅在支持 termios 的 POSIX 终端（Linux、macOS）下可用。
"""

import contextlib
import select
import sys
import termios
import time
import tty

from pymodbus.client import ModbusSerialClient
from pymodbus.framer import FramerType

SAFE_SPEED = 10
SAFE_FORCE = 10
MIN_VALUE = 0
MAX_VALUE = 255
KEY_STEP_COARSE = 25
KEY_STEP_FINE = 5
ESC_SEQUENCE_TIMEOUT = 0.05


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


def _read_escape_sequence():
    r"""读取 ESC 之后的 ANSI 转义序列并归一化为按键名称。

    由 :func:`read_key` 在收到 ``\x1b`` 之后调用。使用短超时区分单击 Esc
    与方向键序列，并兼容 ``ESC[1~`` / ``ESC[4~`` 等 Home/End 变体。

    Returns:
        str: 归一化的按键名,如 ``"UP" "DOWN" "LEFT" "RIGHT" "HOME" "END"``;
        无后续字符返回 ``"ESC"``;无法识别的序列返回 ``"UNKNOWN"``。
    """
    if not select.select([sys.stdin], [], [], ESC_SEQUENCE_TIMEOUT)[0]:
        return "ESC"

    second = sys.stdin.read(1)
    if second != "[":
        return "ESC"

    third = sys.stdin.read(1)
    arrow_map = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT", "H": "HOME", "F": "END"}
    if third in arrow_map:
        return arrow_map[third]

    # 处理 ESC[1~ / ESC[4~ 这类带波浪号的序列（部分终端的 Home/End）
    if third.isdigit():
        tail = sys.stdin.read(1)
        if third == "1" and tail == "~":
            return "HOME"
        if third == "4" and tail == "~":
            return "END"
    return "UNKNOWN"


def read_key():
    """阻塞读取一次按键。

    必须在 :func:`raw_terminal` 上下文内调用，否则会得到行缓冲后的输入。

    Returns:
        str: 普通可打印字符按原样返回;方向键 / Home / End 返回归一化名称
        （见 :func:`_read_escape_sequence`）;单击 Esc 返回 ``"ESC"``。
    """
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        return _read_escape_sequence()
    return ch


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
        # 写入激活指令 (rACT=1, rGTO=1, rATR=0, rPRQ=0)
        # 同时带上安全速度和力度，避免初始化校准过快或过猛
        activation_speed_force = pack_speed_force_register(SAFE_SPEED, SAFE_FORCE)
        # 寄存器 0x03E8: 动作请求
        self._write_registers(0x03E8, [0x0100, 0x0000, activation_speed_force], "激活夹爪")
        time.sleep(2)  # 等待夹爪完成初始化校准动作

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
        gripper.move(current_position)

        with raw_terminal():
            while True:
                key = read_key()
                if key in ("q", "Q", "ESC"):
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
                    continue
                else:
                    continue
                gripper.move(current_position)

    except KeyboardInterrupt:
        print("程序被用户中断")
        gripper.stop()
    except (GripperError, ValueError) as exc:
        print(f"安全停止：{exc}")
        gripper.stop()
    finally:
        gripper.close()
