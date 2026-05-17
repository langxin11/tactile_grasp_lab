"""动作接口辅助函数。

用于将 ROS gripper action 的位置与力度参数映射为 Robotiq 指令字节。
"""

from __future__ import annotations

from .safety import clamp_byte


def clamp_action_position(position: float, closed_position: float) -> float:
    """将 action 空间位置钳位到夹爪行程范围内。

    Args:
        position: 目标位置。
        closed_position: 夹爪全闭对应的行程长度，必须为正。

    Returns:
        float: 钳位后的目标位置。

    Raises:
        ValueError: closed_position 非正数时抛出。
    """
    if closed_position <= 0.0:
        raise ValueError("closed_position must be positive.")
    return max(0.0, min(float(position), float(closed_position)))


def map_action_position_to_command(position: float, closed_position: float) -> int:
    """将 action 目标位置映射为 Robotiq 位置字节。

    Args:
        position: action 目标位置。
        closed_position: 夹爪全闭对应的行程长度。

    Returns:
        int: 位置指令字节，范围为 0-255。

    Raises:
        ValueError: closed_position 非正数或参数无法转换时抛出。
    """
    clamped = clamp_action_position(position, closed_position)
    normalized = clamped / float(closed_position)
    return clamp_byte(round(normalized * 255.0), "position")


def map_effort_to_force_byte(max_effort: float, action_max_force: float, default_force: int) -> int:
    """将 action 的 effort 映射为 Robotiq 力度字节。

    Args:
        max_effort: action 目标的最大力度。
        action_max_force: action 允许的最大力度标定值。
        default_force: max_effort 不可用时的默认力度字节。

    Returns:
        int: 力度字节，范围为 0-255。

    Raises:
        ValueError: action_max_force 非正数或参数无法转换时抛出。
    """
    if max_effort <= 0.0:
        return clamp_byte(default_force, "default_force")
    if action_max_force <= 0.0:
        raise ValueError("action_max_force must be positive.")
    normalized = min(abs(float(max_effort)) / float(action_max_force), 1.0)
    return clamp_byte(round(normalized * 255.0), "force")
