"""安全辅助函数，供驱动包内各模块共享使用。

提供数值范围钳位等基础校验功能，确保发送给夹爪的参数始终在有效范围内。
"""

# =============================================================================
# 范围常量
# =============================================================================

MIN_POSITION = 0  # 夹爪最小位置（全开）
MAX_POSITION = 255  # 夹爪最大位置（全闭）

# =============================================================================
# 钳位函数
# =============================================================================


def clamp_byte(value: int, name: str) -> int:
    """将命令值钳位到 0-255 的有效字节范围内。

    任何超出 [0, 255] 的值都会被钳位到边界值，确保不会向硬件发送
    无效参数。非整数输入会触发 ValueError。

    Args:
        value: 待钳位的数值。
        name: 参数名，用于错误消息。

    Returns:
        int: 钳位后的值（保证在 0-255 范围内）。

    Raises:
        ValueError: 当 value 无法转换为整数时抛出。
    """
    try:
        value = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer in the range [0, 255].") from exc

    if value < MIN_POSITION:
        return MIN_POSITION
    if value > MAX_POSITION:
        return MAX_POSITION
    return value
