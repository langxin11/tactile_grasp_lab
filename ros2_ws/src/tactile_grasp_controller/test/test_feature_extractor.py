"""feature_extractor 模块的单元测试。

测试触觉特征提取函数对接触、力和滑移的检测逻辑。
"""

from dataclasses import dataclass, field

from tactile_grasp_controller.feature_extractor import extract_tactile_features


@dataclass
class Pillar:
    in_contact: bool = False
    slip_state: int = 0


@dataclass
class Sensor:
    gfx: float = 0.0
    gfy: float = 0.0
    gfz: float = 0.0
    friction_est: float = 0.4
    is_sd_active: bool = False
    is_contact: bool = False
    pillars: list[Pillar] = field(default_factory=list)


def test_extract_tactile_features_detects_contact_and_slip() -> None:
    """验证特征提取能正确检测双侧接触和滑移状态。"""
    params = {
        "left_normal_sign": 1.0,
        "right_normal_sign": 1.0,
        "friction_mu_default": 0.4,
        "friction_mu_min": 0.2,
    }
    left = Sensor(gfx=0.1, gfy=0.2, gfz=3.0, is_contact=True)
    right = Sensor(gfx=0.2, gfy=0.2, gfz=4.0, pillars=[Pillar(in_contact=True, slip_state=1)])

    features = extract_tactile_features(left, right, params)

    assert features["both_contact"] is True
    assert features["fn_min"] == 3.0
    assert features["slip_detected"] is True
