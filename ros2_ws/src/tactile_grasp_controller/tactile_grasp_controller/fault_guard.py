"""故障保护辅助函数。

提供连续超限计数器，用于消抖的故障判断逻辑。
"""

from __future__ import annotations


def update_consecutive_limit_counter(
    value: float,
    limit: float,
    current_count: int,
    required_count: int,
) -> tuple[int, bool]:
    """跟踪连续超限样本数，并在达到阈值时报告触发。

    连续超限指的是 value > limit 的连续帧数。如果中间任一帧不超限，
    计数器立即归零。当 required_count <= 1 时，直接在单帧超限时触发。

    Args:
        value: 当前样本值。
        limit: 上限阈值。
        current_count: 当前已连续超限的帧数。
        required_count: 触发故障所需的连续超限帧数。

    Returns:
        一个 (next_count, tripped) 元组：
        - next_count: 更新后的连续超限计数。
        - tripped: 是否达到触发阈值。
    """

    if required_count <= 1:
        return (1 if value > limit else 0), value > limit

    if value > limit:
        next_count = current_count + 1
        return next_count, next_count >= required_count

    return 0, False
