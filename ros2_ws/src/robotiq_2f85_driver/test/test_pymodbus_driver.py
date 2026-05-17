"""PymodbusDriver 单元测试。

通过 monkeypatch 注入仿真的 Modbus 客户端，验证寄存器编解码、命令载荷
打包以及运动完成判定逻辑，覆盖原 RobotiqDriverCore 的 non-dry-run 路径。
"""

from types import SimpleNamespace

import pytest
from robotiq_2f85_driver import driver as driver_types
from robotiq_2f85_driver import pymodbus_driver
from robotiq_2f85_driver.pymodbus_driver import PymodbusDriver

# =============================================================================
# 测试辅助类：仿真 Modbus 客户端
# =============================================================================


class _FakeResult:
    """仿真 Modbus 操作结果。"""

    def __init__(self, registers: list[int]) -> None:
        """初始化仿真结果。

        Args:
            registers: 模拟返回的寄存器值列表。
        """
        self.registers = registers

    def isError(self) -> bool:
        """模拟无错误返回。"""
        return False


class _FakeClient:
    """仿真 Modbus 串口客户端。

    不执行实际串口通信，返回预设的反馈寄存器值。
    """

    def __init__(self, feedback_registers: list[int]) -> None:
        """初始化仿真客户端。

        Args:
            feedback_registers: 读取输入寄存器时返回的预设值。
        """
        self.feedback_registers = feedback_registers

    def connect(self) -> bool:
        """模拟连接成功。"""
        return True

    def close(self) -> None:
        """模拟关闭连接。"""
        return None

    def write_registers(self, address: int, registers: list[int], device_id: int) -> _FakeResult:
        """模拟写入寄存器（不执行实际操作）。"""
        del address, registers, device_id
        return _FakeResult([])

    def read_input_registers(self, address: int, count: int, device_id: int) -> _FakeResult:
        """模拟读取输入寄存器，返回预设的反馈值。"""
        del address, count, device_id
        return _FakeResult(self.feedback_registers)


class _SequenceClient(_FakeClient):
    """Return a configurable sequence of feedback register snapshots."""

    def __init__(self, feedback_sequence: list[list[int]]) -> None:
        super().__init__(feedback_sequence[-1])
        self.feedback_sequence = list(feedback_sequence)
        self.calls = 0

    def read_input_registers(self, address: int, count: int, device_id: int) -> _FakeResult:
        del address, count, device_id
        index = min(self.calls, len(self.feedback_sequence) - 1)
        self.calls += 1
        return _FakeResult(self.feedback_sequence[index])

# =============================================================================
# 测试用例
# =============================================================================


def test_pack_speed_force_register_uses_robotiq_byte_order() -> None:
    """验证 pack_speed_force_register() 按 Robotiq 字节序打包。

    Robotiq 协议要求高字节为力控值，低字节为速度值。
    输入 speed=0xC8, force=0x3C 应得到 0x3CC8。
    """
    assert driver_types.pack_speed_force_register(speed=0xC8, force=0x3C) == 0x3CC8


def test_read_feedback_decodes_hardware_status(monkeypatch) -> None:
    """验证 read_feedback() 能正确解码硬件状态寄存器。

    使用预设的寄存器值 [0x3100, 0x0D14, 0x0910] 测试解码逻辑：
    - gripper_status = 0x31 → activation 完成
    - fault_status = 0x0D → 激活故障
    - position_request_echo = 0x14
    - position = 0x09
    - current = 0x10 = 16 → 160 mA
    """
    fake_client = _FakeClient([0x3100, 0x0D14, 0x0910])
    monkeypatch.setattr(pymodbus_driver, "ModbusSerialClient", lambda **kwargs: fake_client)
    monkeypatch.setattr(pymodbus_driver, "FramerType", SimpleNamespace(RTU="rtu"))

    driver = PymodbusDriver()
    driver.connect()
    feedback = driver.read_feedback()

    assert feedback is not None
    assert feedback.is_activation_complete is True
    assert feedback.fault_status == 0x0D
    assert feedback.fault_text == "activation fault; verify no interference occurred"
    assert feedback.position_request_echo == 0x14
    assert feedback.position == 0x09
    assert feedback.current_milliamps == 160


def test_move_writes_expected_register_payload(monkeypatch) -> None:
    """验证 move() 发送的寄存器载荷与预期一致。

    使用自定义的 _CaptureClient 记录实际写入的寄存器值，确认：
    - 目标地址为 0x03E8
    - 寄存器值为 [0x0900, 0x0005, 0x140A]（位置=5, 速度=10, 力控=20）
    - 设备 ID 为 9
    """
    class _CaptureClient(_FakeClient):
        """带调用记录功能的仿真客户端。"""

        def __init__(self) -> None:
            super().__init__([0x3100, 0x0005, 0x0300])
            self.calls = []

        def write_registers(
            self, address: int, registers: list[int], device_id: int
        ) -> _FakeResult:
            self.calls.append((address, registers, device_id))
            return _FakeResult([])

    fake_client = _CaptureClient()
    monkeypatch.setattr(pymodbus_driver, "ModbusSerialClient", lambda **kwargs: fake_client)
    monkeypatch.setattr(pymodbus_driver, "FramerType", SimpleNamespace(RTU="rtu"))

    driver = PymodbusDriver()
    driver.connect()
    driver.move(position=5, speed=10, force=20)

    assert fake_client.calls[-1] == (0x03E8, [0x0900, 0x0005, 0x140A], 9)


def test_wait_for_motion_complete_reports_reached_goal(monkeypatch) -> None:
    """Return reached_goal when the hardware position settles near the target."""
    fake_client = _SequenceClient(
        [
            [0x3900, 0x0005, 0x0100],
            [0xF100, 0x0005, 0x0500],
        ]
    )
    monkeypatch.setattr(pymodbus_driver, "ModbusSerialClient", lambda **kwargs: fake_client)
    monkeypatch.setattr(pymodbus_driver, "FramerType", SimpleNamespace(RTU="rtu"))
    monkeypatch.setattr(pymodbus_driver.time, "sleep", lambda _: None)

    driver = PymodbusDriver()
    driver.connect()
    result = driver.wait_for_motion_complete(target_position=5, timeout_s=0.2, position_tolerance=1)

    assert result.reached_goal is True
    assert result.stalled is False
    assert result.final_feedback.position == 5


def test_wait_for_motion_complete_reports_stall(monkeypatch) -> None:
    """Return stalled when the gripper stops on object contact before the target."""
    fake_client = _SequenceClient(
        [
            [0x3900, 0x000A, 0x0400],
            [0xB100, 0x000A, 0x0400],
        ]
    )
    monkeypatch.setattr(pymodbus_driver, "ModbusSerialClient", lambda **kwargs: fake_client)
    monkeypatch.setattr(pymodbus_driver, "FramerType", SimpleNamespace(RTU="rtu"))
    monkeypatch.setattr(pymodbus_driver.time, "sleep", lambda _: None)

    driver = PymodbusDriver()
    driver.connect()
    result = driver.wait_for_motion_complete(
        target_position=10, timeout_s=0.2, position_tolerance=1
    )

    assert result.reached_goal is False
    assert result.stalled is True


def test_wait_for_motion_complete_times_out(monkeypatch) -> None:
    """Raise GripperError when motion feedback never settles before the timeout."""
    fake_client = _SequenceClient([[0x3900, 0x000A, 0x0400]])
    monkeypatch.setattr(pymodbus_driver, "ModbusSerialClient", lambda **kwargs: fake_client)
    monkeypatch.setattr(pymodbus_driver, "FramerType", SimpleNamespace(RTU="rtu"))
    monkeypatch.setattr(pymodbus_driver.time, "sleep", lambda _: None)
    monotonic_values = iter([0.0, 0.01, 0.02, 0.03])
    monkeypatch.setattr(pymodbus_driver.time, "monotonic", lambda: next(monotonic_values))

    driver = PymodbusDriver()
    driver.connect()
    with pytest.raises(driver_types.GripperError, match="Motion did not settle before timeout"):
        driver.wait_for_motion_complete(target_position=10, timeout_s=0.02, position_tolerance=1)
