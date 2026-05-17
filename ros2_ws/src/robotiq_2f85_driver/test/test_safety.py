"""safety 模块单元测试。

测试 clamp_byte 函数的范围钳位和输入校验行为。
"""

from robotiq_2f85_driver.safety import clamp_byte

# =============================================================================
# 测试用例
# =============================================================================


def test_clamp_byte_limits_range() -> None:
    """验证 clamp_byte 能正确地将越界值钳位到 [0, 255] 范围内。

    测试用例：
    - 负数（-1）应钳位到下限 0
    - 超大值（512）应钳位到上限 255
    - 范围内的值（42）应直接原样返回
    """
    assert clamp_byte(-1, "position") == 0
    assert clamp_byte(512, "position") == 255
    assert clamp_byte(42, "position") == 42


def test_clamp_byte_rejects_non_integer_input() -> None:
    """验证 clamp_byte 对非整数输入抛出 ValueError。

    输入字符串 "bad" 应触发 ValueError，因为无法转换为整数。
    """
    try:
        clamp_byte("bad", "position")
    except ValueError:
        return
    raise AssertionError("Expected ValueError for invalid input")
