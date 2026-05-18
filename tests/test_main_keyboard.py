"""`main.py` 键盘输入解析测试。"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "main.py"
MODULE_SPEC = importlib.util.spec_from_file_location("legacy_main", MODULE_PATH)
if MODULE_SPEC is None or MODULE_SPEC.loader is None:
    raise RuntimeError(f"无法从 {MODULE_PATH} 加载 main.py")
main = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(main)


@dataclass
class FakeStdin:
    """提供最小 `read()` 接口的伪 stdin。"""

    buffer: str
    offset: int = 0

    def read(self, size: int = 1) -> str:
        """按顺序返回缓冲区字符。"""
        start = self.offset
        end = min(len(self.buffer), self.offset + size)
        self.offset = end
        return self.buffer[start:end]

    def read_bytes(self, size: int = 1) -> bytes:
        """按顺序返回缓冲区字节。"""
        return self.read(size).encode("ascii")

    def fileno(self) -> int:
        """返回占位文件描述符，供 monkeypatch 后的 `os.read` 使用。"""
        return 0

    def has_pending(self) -> bool:
        """返回是否仍有未消费字符。"""
        return self.offset < len(self.buffer)


def _patch_stdin(monkeypatch, sequence: str) -> FakeStdin:
    """为 `main` 模块安装一个可控的 stdin、select.select 和 os.read。"""
    fake = FakeStdin(sequence)

    def fake_select(readers, _writers, _errors, _timeout):
        return (readers, [], []) if fake.has_pending() else ([], [], [])

    def fake_os_read(_fd: int, size: int) -> bytes:
        return fake.read_bytes(size)

    monkeypatch.setattr(main.sys, "stdin", fake)
    monkeypatch.setattr(main.select, "select", fake_select)
    monkeypatch.setattr(main.os, "read", fake_os_read)
    return fake


def test_read_escape_sequence_supports_csi_arrows(monkeypatch) -> None:
    """`ESC [` 形式的方向键应被识别。"""
    _patch_stdin(monkeypatch, "[A")
    assert main._read_escape_sequence() == "UP"


def test_read_escape_sequence_supports_ss3_arrows(monkeypatch) -> None:
    """`ESC O` 形式的方向键不应误判为退出。"""
    _patch_stdin(monkeypatch, "OA")
    assert main._read_escape_sequence() == "UP"


def test_read_escape_sequence_supports_ss3_home_end(monkeypatch) -> None:
    """`ESC O H/F` 形式的 Home/End 应被识别。"""
    _patch_stdin(monkeypatch, "OH")
    assert main._read_escape_sequence() == "HOME"

    _patch_stdin(monkeypatch, "OF")
    assert main._read_escape_sequence() == "END"


def test_read_key_returns_escape_for_single_escape(monkeypatch) -> None:
    """单独按 Esc 仍应返回退出键。"""
    _patch_stdin(monkeypatch, "\x1b")
    assert main.read_key() == "ESC"


def test_read_key_routes_escape_sequences(monkeypatch) -> None:
    """`read_key()` 应把方向键序列转给转义解析逻辑。"""
    _patch_stdin(monkeypatch, "\x1bOB")
    assert main.read_key() == "DOWN"


def test_wait_for_key_returns_none_on_timeout(monkeypatch) -> None:
    """超时未读到输入时应返回 `None`，供上层发送 keepalive。"""
    fake = FakeStdin("")

    def fake_select(_readers, _writers, _errors, _timeout):
        return ([], [], [])

    monkeypatch.setattr(main.sys, "stdin", fake)
    monkeypatch.setattr(main.select, "select", fake_select)
    monkeypatch.setattr(main.os, "read", lambda _fd, _size: b"")
    assert main.wait_for_key(0.2) is None


def test_wait_for_key_returns_key_when_input_is_ready(monkeypatch) -> None:
    """有输入时应继续按原逻辑返回按键。"""
    _patch_stdin(monkeypatch, "q")
    assert main.wait_for_key(0.2) == "q"


def test_decode_feedback_registers_recovers_status_fields() -> None:
    """反馈寄存器应按 Robotiq 字节布局正确解码。"""
    feedback = main.decode_feedback_registers([0x310D, 0x2233, 0x4400])
    assert feedback.gripper_status == 0x31
    assert feedback.fault_status == 0x0D
    assert feedback.position_request_echo == 0x22
    assert feedback.position == 0x33
    assert feedback.current == 0x44


def test_activation_complete_requires_echo_and_complete_state() -> None:
    """只有 gACT=1 且 gSTA=3 才算激活完成。"""
    feedback = main.GripperFeedback(
        gripper_status=0x31,
        fault_status=0x00,
        position_request_echo=0,
        position=0,
        current=0,
    )
    assert feedback.is_activation_complete is True
