"""FakeDriver 单元测试。

验证内存模拟驱动在不依赖串口与 pymodbus 时的命令记录与反馈生成行为。
"""

from robotiq_2f85_driver.fake_driver import FakeDriver


def test_fake_driver_move_records_last_command() -> None:
    """验证 FakeDriver.move() 能正确记录最后一条命令。

    测试要点：
    - connect 后 connected 为 True
    - move 后 last_command 记录请求值且 simulated 为 True
    - 反馈位置等于命令位置，且 is_activation_complete 为 True
    """
    driver = FakeDriver()
    driver.connect()
    driver.move(position=64, speed=11, force=12)

    assert driver.dry_run is True
    assert driver.connected is True
    assert driver.last_command is not None
    assert driver.last_command.position == 64
    assert driver.last_command.simulated is True
    assert driver.last_feedback is not None
    assert driver.last_feedback.position == 64
    assert driver.last_feedback.is_activation_complete is True


def test_fake_driver_wait_for_motion_complete_reports_reached_goal() -> None:
    """FakeDriver 的 wait_for_motion_complete 应立即返回到达目标。"""
    driver = FakeDriver()
    driver.connect()
    driver.move(position=100)
    result = driver.wait_for_motion_complete(
        target_position=100, timeout_s=0.1, position_tolerance=1
    )

    assert result.reached_goal is True
    assert result.stalled is False
    assert result.target_position == 100
