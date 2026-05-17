"""基于规则的触觉抓取状态机。

实现高层触觉抓取逻辑，包含 APPROACH、PRELOAD、HOLD、
SLIP_COMPENSATE、FAULT 等状态及其转换规则。
"""

from __future__ import annotations

import time
from typing import Any

from .models import ControlAction


class TactileGraspStateMachine:
    """跟踪高层触觉抓取状态。

    状态流转：
    - IDLE → APPROACH（手动 start）
    - APPROACH → PRELOAD（双侧接触检测到）
    - PRELOAD → HOLD（预紧力达到阈值）
    - HOLD → SLIP_COMPENSATE（检测到滑移或摩擦裕度不足）
    - SLIP_COMPENSATE → HOLD（补偿后回到保持）
    - 任意状态 → FAULT（外部故障注入或未知状态）
    """

    def __init__(self, params: dict[str, Any]) -> None:
        """初始化状态机。

        Args:
            params: 参数字典，包含各阶段的步长、力阈值等配置。
        """
        self.params = params
        self.state = "APPROACH" if params.get("auto_start", False) else "IDLE"
        self.last_reason = "startup"
        self._last_compensate_time = 0.0

    def start(self) -> None:
        """手动启动控制器，从 IDLE 进入 APPROACH 状态。"""
        if self.state == "IDLE":
            self.state = "APPROACH"
            self.last_reason = "manual start"

    def stop(self) -> None:
        """手动停止控制器，回到 IDLE 状态。"""
        self.state = "IDLE"
        self.last_reason = "manual stop"

    def reset_fault(self) -> None:
        """复位故障，回到 IDLE 状态。"""
        self.state = "IDLE"
        self.last_reason = "fault reset"

    def enter_fault(self, reason: str) -> ControlAction:
        """进入 FAULT 状态并返回故障动作。

        Args:
            reason: 进入故障状态的原因描述。

        Returns:
            包含故障信息的 ControlAction 对象。
        """
        self.state = "FAULT"
        self.last_reason = reason
        return ControlAction(name="fault", reason=reason)

    def update(self, features: dict[str, Any], now_s: float | None = None) -> ControlAction:
        """根据当前触觉特征更新状态并返回控制动作。

        这是状态机的主入口，每个控制周期调用一次。

        Args:
            features: 由 extract_tactile_features 提取的特征字典。
            now_s: 当前时间戳（秒），用于冷却计时。不提供则自动获取。

        Returns:
            当前状态对应的 ControlAction。
        """
        now_s = time.monotonic() if now_s is None else now_s

        # ==== IDLE: 空闲状态，不执行任何动作 ====
        if self.state == "IDLE":
            return ControlAction(name="none", reason=self.last_reason)

        # ==== APPROACH: 接近阶段，逐步闭合直到双侧接触 ====
        if self.state == "APPROACH":
            if features["both_contact"]:
                self.state = "PRELOAD"
                self.last_reason = "both sensors contacted"
                return ControlAction(name="hold", reason=self.last_reason)
            return ControlAction(
                name="close_step",
                step=int(self.params["approach_position_step"]),
                reason="approach until both contacts are seen",
            )

        # ==== PRELOAD: 预紧阶段，逐步增加力直到达到目标预紧力 ====
        if self.state == "PRELOAD":
            if features["fn_min"] < float(self.params["min_preload_force_n"]):
                return ControlAction(
                    name="close_step",
                    step=int(self.params["preload_position_step"]),
                    reason="build preload",
                )
            self.state = "HOLD"
            self.last_reason = "preload established"
            return ControlAction(name="hold", reason=self.last_reason)

        # ==== HOLD: 保持阶段，监控滑移和力下降 ====
        if self.state == "HOLD":
            if features["slip_detected"]:
                self.state = "SLIP_COMPENSATE"
                self.last_reason = "slip detected"
                return ControlAction(name="hold", reason=self.last_reason)
            if features["friction_margin"] < float(self.params["friction_margin_threshold_n"]):
                self.state = "SLIP_COMPENSATE"
                self.last_reason = "friction margin low"
                return ControlAction(name="hold", reason=self.last_reason)
            if features["fn_min"] < float(self.params["min_hold_force_n"]):
                self.state = "PRELOAD"
                self.last_reason = "hold force dropped"
                return ControlAction(
                    name="close_step",
                    step=int(self.params["preload_position_step"]),
                    reason=self.last_reason,
                )
            return ControlAction(name="hold", reason="holding position")

        # ==== SLIP_COMPENSATE: 滑移补偿，冷却后闭合一步并回到 HOLD ====
        if self.state == "SLIP_COMPENSATE":
            cooldown = float(self.params["slip_compensate_cooldown_s"])
            if now_s - self._last_compensate_time < cooldown:
                return ControlAction(name="hold", reason="cooldown active")
            self._last_compensate_time = now_s
            self.state = "HOLD"
            self.last_reason = "compensating slip"
            return ControlAction(
                name="close_step",
                step=int(self.params["compensate_position_step"]),
                reason=self.last_reason,
            )

        # ==== FAULT: 故障状态，根据配置执行保持或打开 ====
        if self.state == "FAULT":
            if self.params.get("fault_action", "hold") == "open":
                return ControlAction(name="open", reason=self.last_reason)
            return ControlAction(name="hold", reason=self.last_reason)

        return self.enter_fault(f"unknown state {self.state}")
