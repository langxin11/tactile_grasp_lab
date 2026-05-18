"""触觉特征提取辅助函数。

从两个触觉传感器消息中提取第一版接触与力特征，
包括法向力、切向力、摩擦估计、滑移检测等。
"""

from __future__ import annotations

import math
from typing import Any


def _pillars(msg: Any) -> list[Any]:
    """安全获取消息中的 pillars 字段列表。

    Args:
        msg: 传感器消息对象。

    Returns:
        pillars 列表，如果字段不存在或为 None 则返回空列表。
    """
    pillars = getattr(msg, "pillars", [])
    return list(pillars) if pillars is not None else []


def _safe_min(values: list[float], fallback: float) -> float:
    """计算有限值的最小值，若无有限值则返回备用值。

    Args:
        values: 候选数值列表。
        fallback: 当没有有限值时返回的默认值。

    Returns:
        有限值中的最小值，或 fallback。
    """
    finite = [value for value in values if math.isfinite(value)]
    return min(finite) if finite else fallback


def extract_normal_force_features(
    left_msg: Any, right_msg: Any, params: dict[str, Any]
) -> dict[str, float]:
    """Extract normal-force features using the controller's sign convention.

    Args:
        left_msg: 左侧传感器消息。
        right_msg: 右侧传感器消息。
        params: 参数字典，至少包含 `left_normal_sign` 与 `right_normal_sign`。

    Returns:
        包含 `fn_left`、`fn_right`、`fn_min` 与 `fn_avg` 的字典。
    """
    fn_left = max(0.0, float(params["left_normal_sign"]) * float(getattr(left_msg, "gfz", 0.0)))
    fn_right = max(
        0.0,
        float(params["right_normal_sign"]) * float(getattr(right_msg, "gfz", 0.0)),
    )
    return {
        "fn_left": fn_left,
        "fn_right": fn_right,
        "fn_min": min(fn_left, fn_right),
        "fn_avg": 0.5 * (fn_left + fn_right),
    }


def is_tactile_clear(
    left_msg: Any,
    right_msg: Any,
    params: dict[str, Any],
    threshold_n: float,
) -> tuple[bool, dict[str, float]]:
    """Check whether both fingertips are below the allowed start-force threshold.

    Args:
        left_msg: 左侧传感器消息。
        right_msg: 右侧传感器消息。
        params: 触觉特征参数字典。
        threshold_n: 允许的 `fn_min` 上限。

    Returns:
        `(是否清零, 法向力特征字典)`。
    """
    normal_features = extract_normal_force_features(left_msg, right_msg, params)
    return normal_features["fn_min"] <= float(threshold_n), normal_features


def extract_tactile_features(
    left_msg: Any, right_msg: Any, params: dict[str, Any]
) -> dict[str, Any]:
    """从两个触觉消息中提取第一版接触与力特征。

    特征包括：
    - left_contact / right_contact: 左右传感器是否接触
    - both_contact: 双侧是否同时接触
    - fn_left / fn_right / fn_min / fn_avg: 法向力
    - ft_left / ft_right / ft_max: 切向力
    - mu_est: 估计摩擦系数
    - friction_margin: 摩擦裕度 (mu * fn_min - ft_max)
    - slip_detected: 是否检测到滑移

    Args:
        left_msg: 左侧传感器消息。
        right_msg: 右侧传感器消息。
        params: 参数字典，包含符号校正和摩擦系数默认值等。

    Returns:
        包含所有提取特征的字典。
    """
    left_pillars = _pillars(left_msg)
    right_pillars = _pillars(right_msg)

    # ==== 接触检测 ====
    left_contact = bool(getattr(left_msg, "is_contact", False)) or any(
        bool(getattr(pillar, "in_contact", False)) for pillar in left_pillars
    )
    right_contact = bool(getattr(right_msg, "is_contact", False)) or any(
        bool(getattr(pillar, "in_contact", False)) for pillar in right_pillars
    )

    # ==== 法向力（允许符号反转） ====
    normal_features = extract_normal_force_features(left_msg, right_msg, params)
    fn_left = normal_features["fn_left"]
    fn_right = normal_features["fn_right"]
    fn_min = normal_features["fn_min"]
    fn_avg = normal_features["fn_avg"]

    # ==== 切向力 ====
    ft_left = math.hypot(float(getattr(left_msg, "gfx", 0.0)), float(getattr(left_msg, "gfy", 0.0)))
    ft_right = math.hypot(
        float(getattr(right_msg, "gfx", 0.0)), float(getattr(right_msg, "gfy", 0.0))
    )
    ft_max = max(ft_left, ft_right)

    # ==== 摩擦估计 ====
    mu_est = _safe_min(
        [
            float(getattr(left_msg, "friction_est", math.nan)),
            float(getattr(right_msg, "friction_est", math.nan)),
        ],
        float(params["friction_mu_default"]),
    )
    if (not math.isfinite(mu_est)) or mu_est < float(params["friction_mu_min"]):
        mu_est = float(params["friction_mu_default"])

    friction_margin = mu_est * fn_min - ft_max

    # ==== 滑移检测 ====
    slip_detected = bool(getattr(left_msg, "is_sd_active", False)) or bool(
        getattr(right_msg, "is_sd_active", False)
    )
    slip_detected = slip_detected or any(
        int(getattr(pillar, "slip_state", 0)) != 0 for pillar in left_pillars
    )
    slip_detected = slip_detected or any(
        int(getattr(pillar, "slip_state", 0)) != 0 for pillar in right_pillars
    )

    return {
        "left_contact": left_contact,
        "right_contact": right_contact,
        "both_contact": left_contact and right_contact,
        "fn_left": fn_left,
        "fn_right": fn_right,
        "fn_min": fn_min,
        "fn_avg": fn_avg,
        "ft_left": ft_left,
        "ft_right": ft_right,
        "ft_max": ft_max,
        "mu_est": mu_est,
        "friction_margin": friction_margin,
        "slip_detected": slip_detected,
    }
