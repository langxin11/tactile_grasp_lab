"""fault_guard 模块的单元测试。

测试连续超限计数器的触发和复位逻辑。
"""

from tactile_grasp_controller.fault_guard import update_consecutive_limit_counter


def test_update_consecutive_limit_counter_requires_consecutive_frames() -> None:
    """验证需要连续超限达到 required_count 后才触发。"""
    count, tripped = update_consecutive_limit_counter(21.0, 20.0, 0, 3)
    assert count == 1
    assert tripped is False

    count, tripped = update_consecutive_limit_counter(21.0, 20.0, count, 3)
    assert count == 2
    assert tripped is False

    count, tripped = update_consecutive_limit_counter(21.0, 20.0, count, 3)
    assert count == 3
    assert tripped is True


def test_update_consecutive_limit_counter_resets_when_signal_recovers() -> None:
    """验证信号恢复正常时计数器归零。"""
    count, tripped = update_consecutive_limit_counter(21.0, 20.0, 0, 3)
    assert count == 1
    assert tripped is False

    count, tripped = update_consecutive_limit_counter(19.0, 20.0, count, 3)
    assert count == 0
    assert tripped is False


def test_update_consecutive_limit_counter_trips_immediately_when_disabled() -> None:
    """验证 required_count=1 时单帧即触发。"""
    count, tripped = update_consecutive_limit_counter(21.0, 20.0, 0, 1)
    assert count == 1
    assert tripped is True
