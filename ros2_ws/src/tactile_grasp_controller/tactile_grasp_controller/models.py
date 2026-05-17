"""触觉抓取控制器的共享数据结构。

定义状态机输出的 ControlAction 数据类。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ControlAction:
    """状态机返回的控制命令。

    Attributes:
        name: 动作名称，如 "close_step"、"hold"、"open"、"fault"。
        step: 闭合步长（仅 close_step 动作有效）。
        reason: 执行该动作的原因描述。
    """

    name: str
    step: int = 0
    reason: str = ""
