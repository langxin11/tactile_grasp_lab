"""Tests for startup tactile-clear gating."""

from dataclasses import dataclass

from tactile_grasp_controller.startup_gating import (
    evaluate_start_tactile_gate,
    tactile_clear_status,
)


@dataclass
class Sensor:
    """Minimal sensor message surface used by startup-gating tests."""

    gfz: float = 0.0


def make_params() -> dict[str, object]:
    """Return the minimum tactile params needed by the startup gates."""
    return {
        "left_normal_sign": 1.0,
        "right_normal_sign": 1.0,
        "require_clear_tactile_on_start": True,
        "start_force_threshold_n": 5.0,
        "start_force_stable_s": 0.5,
        "start_force_timeout_s": 5.0,
    }


def test_evaluate_start_tactile_gate_rejects_missing_data() -> None:
    """Controller start should be rejected until both tactile messages exist."""
    allowed, message = evaluate_start_tactile_gate(None, None, make_params())

    assert allowed is False
    assert message == "tactile data unavailable"


def test_evaluate_start_tactile_gate_rejects_high_load() -> None:
    """Controller start should be rejected when fn_min exceeds the threshold."""
    left = Sensor(gfz=6.0)
    right = Sensor(gfz=7.0)

    allowed, message = evaluate_start_tactile_gate(left, right, make_params())

    assert allowed is False
    assert "tactile not clear for controller start" in message
    assert "fn_min=6.000" in message


def test_evaluate_start_tactile_gate_accepts_low_load() -> None:
    """Controller start should pass once fn_min is below the threshold."""
    left = Sensor(gfz=1.0)
    right = Sensor(gfz=2.0)

    allowed, message = evaluate_start_tactile_gate(left, right, make_params())

    assert allowed is True
    assert message == "controller start allowed"


def test_tactile_clear_status_requires_stable_window() -> None:
    """Coordinator should require a continuous low-force window before passing."""
    first_clear, _, _, clear_since_s, clear_deadline_s = tactile_clear_status(
        Sensor(gfz=1.0),
        Sensor(gfz=2.0),
        make_params(),
        clear_since_s=None,
        clear_deadline_s=None,
        now_s=10.0,
    )
    second_clear, _, _, _, _ = tactile_clear_status(
        Sensor(gfz=1.0),
        Sensor(gfz=2.0),
        make_params(),
        clear_since_s=clear_since_s,
        clear_deadline_s=clear_deadline_s,
        now_s=10.6,
    )

    assert first_clear is False
    assert second_clear is True


def test_tactile_clear_status_resets_when_force_returns() -> None:
    """A force rebound should reset the stable low-force window."""
    _, _, _, clear_since_s, clear_deadline_s = tactile_clear_status(
        Sensor(gfz=1.0),
        Sensor(gfz=2.0),
        make_params(),
        clear_since_s=None,
        clear_deadline_s=None,
        now_s=10.0,
    )
    rebound_clear, _, _, clear_since_s, clear_deadline_s = tactile_clear_status(
        Sensor(gfz=6.0),
        Sensor(gfz=7.0),
        make_params(),
        clear_since_s=clear_since_s,
        clear_deadline_s=clear_deadline_s,
        now_s=10.2,
    )
    after_reset_clear, _, _, clear_since_s, _ = tactile_clear_status(
        Sensor(gfz=1.0),
        Sensor(gfz=2.0),
        make_params(),
        clear_since_s=clear_since_s,
        clear_deadline_s=clear_deadline_s,
        now_s=10.3,
    )

    assert rebound_clear is False
    assert after_reset_clear is False
    assert clear_since_s == 10.3
