"""state_machine 模块的单元测试。

测试触觉抓取状态机的状态转换逻辑和故障行为。
"""

from tactile_grasp_controller.state_machine import TactileGraspStateMachine


def make_params() -> dict[str, object]:
    """创建测试用参数字典。"""
    return {
        "auto_start": True,
        "approach_position_step": 1,
        "preload_position_step": 1,
        "compensate_position_step": 2,
        "min_preload_force_n": 3.0,
        "min_hold_force_n": 2.0,
        "friction_margin_threshold_n": 0.5,
        "slip_compensate_cooldown_s": 0.3,
        "fault_action": "hold",
    }


def test_state_machine_reaches_hold_after_contact_and_preload() -> None:
    """验证状态机完整走完 APPROACH → PRELOAD → HOLD 流程。"""
    sm = TactileGraspStateMachine(make_params())

    action = sm.update(
        {"both_contact": False, "fn_min": 0.0, "friction_margin": 1.0, "slip_detected": False}
    )
    assert action.name == "close_step"
    assert sm.state == "APPROACH"

    action = sm.update(
        {"both_contact": True, "fn_min": 0.0, "friction_margin": 1.0, "slip_detected": False}
    )
    assert action.name == "hold"
    assert sm.state == "PRELOAD"

    action = sm.update(
        {"both_contact": True, "fn_min": 3.5, "friction_margin": 1.0, "slip_detected": False}
    )
    assert action.name == "hold"
    assert sm.state == "HOLD"


def test_state_machine_fault_action_holds_by_default() -> None:
    """验证故障状态下默认动作为保持。"""
    sm = TactileGraspStateMachine(make_params())
    sm.enter_fault("timeout")
    action = sm.update(
        {"both_contact": False, "fn_min": 0.0, "friction_margin": 1.0, "slip_detected": False}
    )
    assert action.name == "hold"
