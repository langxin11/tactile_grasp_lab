import time

from pymodbus.client import ModbusSerialClient
from pymodbus.framer import FramerType

SAFE_SPEED = 10
SAFE_FORCE = 10
MIN_VALUE = 0
MAX_VALUE = 255
MOVE_TIMEOUT_SECONDS = 5


def pack_position_register(position):
    """Pack the requested position byte into register 0x03E9."""
    return clamp_byte(position, "position")


def pack_speed_force_register(speed, force):
    """Pack speed/force bytes into register 0x03EA using Robotiq byte order."""
    speed = clamp_byte(speed, "speed")
    force = clamp_byte(force, "force")
    return (force << 8) | speed


class GripperError(RuntimeError):
    """Raised when the gripper cannot safely complete a requested command."""


def clamp_byte(value, name):
    """Clamp gripper command values to the supported 0-255 byte range."""
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


class Robotiq2F85:
    def __init__(self, port="/dev/ttyUSB0"):
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
        """Write registers and fail fast if the Modbus device reports an error."""
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
        """建立连接"""
        self.connected = bool(self.client.connect())
        if not self.connected:
            raise GripperError("连接失败，请检查端口、权限和夹爪供电。")
        print("成功连接到夹爪串口！")

    def activate(self):
        """激活夹爪 (上电后必须执行一次)"""
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
        """
        核心运动函数
        :param position: 0 (全开) 到 255 (全闭)
        :param speed: 0 (最慢) 到 255 (最快)
        :param force: 0 (最小力) 到 255 (最大力)
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
        """打开夹爪"""
        print("执行：打开夹爪")
        self.move(position=0, speed=SAFE_SPEED, force=SAFE_FORCE)

    def close_gripper(self):
        """关闭夹爪"""
        print("执行：关闭夹爪")
        self.move(position=255, speed=SAFE_SPEED, force=SAFE_FORCE)

    def stop(self):
        """停止当前运动，保持激活状态。"""
        if not self.connected:
            return
        try:
            self._write_registers(0x03E8, [0x0100, 0x0000, 0x0000], "停止夹爪运动")
            print("已发送停止指令。")
        except GripperError as exc:
            print(f"停止指令发送失败：{exc}")

    def close(self):
        """关闭串口连接。"""
        if self.connected:
            self.client.close()
            self.connected = False
            print("连接已关闭。")


# ==========================================
# 用户执行区 (这就是你的单一控制流程)
# ==========================================
if __name__ == "__main__":
    # 1. 实例化并连接
    gripper = Robotiq2F85(port="/dev/ttyUSB0")

    try:
        gripper.connect()

        # 2. 激活夹爪 (如果是刚上电，必须执行)
        gripper.activate()

        # 3. 发送控制指令
        gripper.close_gripper()
        time.sleep(MOVE_TIMEOUT_SECONDS)  # 等待动作完成

        gripper.open_gripper()
        time.sleep(MOVE_TIMEOUT_SECONDS)

        # 你可以自定义半开状态，例如 position=128
        # gripper.move(position=128, speed=100, force=50)

    except KeyboardInterrupt:
        print("程序被用户中断")
        gripper.stop()
    except (GripperError, ValueError) as exc:
        print(f"安全停止：{exc}")
        gripper.stop()
    finally:
        gripper.close()
