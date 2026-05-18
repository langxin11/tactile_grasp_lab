"""Shared tactile-clear gating helpers for bringup and controller start."""

from __future__ import annotations

from typing import Any

from .feature_extractor import is_tactile_clear


def evaluate_start_tactile_gate(
    left_msg: Any | None,
    right_msg: Any | None,
    params: dict[str, Any],
) -> tuple[bool, str]:
    """Validate whether controller start is allowed under current tactile load."""
    if left_msg is None or right_msg is None:
        return False, "tactile data unavailable"
    if not bool(params["require_clear_tactile_on_start"]):
        return True, "controller start allowed"

    tactile_clear, normal_features = is_tactile_clear(
        left_msg,
        right_msg,
        params,
        float(params["start_force_threshold_n"]),
    )
    if tactile_clear:
        return True, "controller start allowed"

    message = (
        "tactile not clear for controller start: "
        f"fn_left={normal_features['fn_left']:.3f}, "
        f"fn_right={normal_features['fn_right']:.3f}, "
        f"fn_min={normal_features['fn_min']:.3f}, "
        f"threshold={float(params['start_force_threshold_n']):.3f}"
    )
    return False, message


def tactile_clear_status(
    left_msg: Any | None,
    right_msg: Any | None,
    params: dict[str, Any],
    *,
    clear_since_s: float | None,
    clear_deadline_s: float | None,
    now_s: float,
) -> tuple[bool, dict[str, float] | None, str, float | None, float | None]:
    """Evaluate bringup tactile-clear state and updated stability timers."""
    if left_msg is None or right_msg is None:
        return False, None, "waiting for tactile messages after bias", None, None

    tactile_clear, normal_features = is_tactile_clear(
        left_msg,
        right_msg,
        params,
        float(params["start_force_threshold_n"]),
    )
    if clear_deadline_s is None:
        clear_deadline_s = now_s + float(params["start_force_timeout_s"])

    if tactile_clear:
        if clear_since_s is None:
            clear_since_s = now_s
        stable_elapsed = now_s - clear_since_s
        stable_target = float(params["start_force_stable_s"])
        if stable_elapsed >= stable_target:
            return (
                True,
                normal_features,
                "tactile baseline is clear for controller start",
                clear_since_s,
                clear_deadline_s,
            )
        detail = (
            "tactile clear candidate; waiting for stable low-force window: "
            f"fn_left={normal_features['fn_left']:.3f}, "
            f"fn_right={normal_features['fn_right']:.3f}, "
            f"fn_min={normal_features['fn_min']:.3f}, "
            f"threshold={float(params['start_force_threshold_n']):.3f}, "
            f"stable_target={stable_target:.1f}s"
        )
        return False, normal_features, detail, clear_since_s, clear_deadline_s

    detail = (
        "waiting for tactile force to clear after open/bias: "
        f"fn_left={normal_features['fn_left']:.3f}, "
        f"fn_right={normal_features['fn_right']:.3f}, "
        f"fn_min={normal_features['fn_min']:.3f}, "
        f"threshold={float(params['start_force_threshold_n']):.3f}"
    )
    return False, normal_features, detail, None, clear_deadline_s
