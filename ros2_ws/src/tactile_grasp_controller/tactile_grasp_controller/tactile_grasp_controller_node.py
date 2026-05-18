"""ROS 2 触觉抓取控制器节点。

协调触觉特征提取、状态机转换和命令发布的主节点。
接收左右传感器消息，提取特征后送入状态机，并将状态机输出的
动作通过 CommandBridge 转换为夹爪 action goal。
"""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from .command_bridge import CommandBridge
from .fault_guard import update_consecutive_limit_counter
from .feature_extractor import extract_tactile_features
from .startup_gating import evaluate_start_tactile_gate
from .state_machine import TactileGraspStateMachine

# 可选导入：sensor_interfaces 未加入工作空间或底层 overlay 时降级。
try:
    from sensor_interfaces.msg import SensorState

    SENSOR_INTERFACES_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于外部工作空间内容
    SensorState = Any
    SENSOR_INTERFACES_AVAILABLE = False


class TactileGraspControllerNode(Node):
    """协调触觉特征、状态转换和命令发布。

    核心控制循环 (control_loop) 按固定频率运行，完成以下步骤：
    1. 检查传感器数据是否就绪和新鲜（超时保护）。
    2. 提取触觉特征。
    3. 对 IDLE 以外的状态进行过力保护检测。
    4. 将特征送入状态机，获取控制动作。
    5. 通过 CommandBridge 执行动作。
    6. 发布调试和状态信息。
    """

    def __init__(self) -> None:
        super().__init__("tactile_grasp_controller")

        self.params = self._declare_and_load_params()
        self.left_msg: Any | None = None
        self.right_msg: Any | None = None
        self.left_time_s: float | None = None
        self.right_time_s: float | None = None
        self.overforce_counter = 0

        self.debug_pub = self.create_publisher(String, self.params["debug_topic"], 10)
        self.state_pub = self.create_publisher(String, self.params["state_topic"], 10)

        # controller 只发布高层动作，不直接碰串口或 Modbus。
        self.command_bridge = CommandBridge(self, self.params)
        self.state_machine = TactileGraspStateMachine(self.params)

        self.create_subscription(
            Int32,
            self.params["gripper_command_echo_topic"],
            self.on_command_echo,
            10,
        )

        self.create_service(Trigger, "/tactile_grasp/start", self.on_start)
        self.create_service(Trigger, "/tactile_grasp/reset_fault", self.on_reset_fault)
        self.create_service(Trigger, "/tactile_grasp/stop", self.on_stop)

        if SENSOR_INTERFACES_AVAILABLE:
            # 默认 topic 约定和 papillarray_ros2_v2 保持一致：
            # /hub_<hub_id>/sensor_<sensor_id>
            # 使用 BEST_EFFORT QoS 与 papillarray_ros2_v2 的发布者兼容
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
            self.create_subscription(
                SensorState,
                self.params["left_tactile_topic"],
                self.on_left_tactile,
                qos_profile=qos,
            )
            self.create_subscription(
                SensorState,
                self.params["right_tactile_topic"],
                self.on_right_tactile,
                qos_profile=qos,
            )
        else:
            self.get_logger().warning(
                "sensor_interfaces is not available. Tactile subscriptions are disabled."
            )

        period = 1.0 / float(self.params["control_rate_hz"])
        self.timer = self.create_timer(period, self.control_loop)

    def _declare_and_load_params(self) -> dict[str, Any]:
        """声明并加载节点参数。

        Returns:
            包含所有参数名到值的映射字典。
        """
        defaults = {
            "left_tactile_topic": "/hub_0/sensor_0",
            "right_tactile_topic": "/hub_0/sensor_1",
            "gripper_command_action": "/robotiq_gripper_controller/gripper_cmd",
            "streaming_command_topic": "/robotiq/command/stream",
            "gripper_open_position": 0.0,
            "gripper_closed_position": 0.8,
            "gripper_max_effort": 10.0,
            "wait_for_action_server_s": 2.0,
            "gripper_command_topic": "/robotiq/command/position",
            "gripper_command_echo_topic": "/robotiq/command/echo",
            "debug_topic": "/tactile_grasp/debug",
            "state_topic": "/tactile_grasp/state",
            "control_rate_hz": 40.0,
            "target_command_rate_bytes_per_s": 80.0,
            "min_speed_byte": 8,
            "max_speed_byte": 32,
            "auto_start": False,
            "dry_run": True,
            "require_clear_tactile_on_start": True,
            "start_force_threshold_n": 5.0,
            "tactile_timeout_s": 0.2,
            "left_normal_sign": 1.0,
            "right_normal_sign": 1.0,
            "approach_position_step": 1,
            "preload_position_step": 1,
            "compensate_position_step": 1,
            "release_position_step": 1,
            "min_position": 0,
            "max_position": 255,
            "initial_position": 0,
            "min_preload_force_n": 3.0,
            "min_hold_force_n": 2.0,
            "target_hold_force_n": 10.0,
            "hold_force_deadband_n": 1.0,
            "max_safe_force_n": 20.0,
            "max_safe_force_consecutive_frames": 3,
            "friction_mu_default": 0.4,
            "friction_mu_min": 0.2,
            "friction_margin_threshold_n": 0.5,
            "slip_compensate_cooldown_s": 0.3,
            "fault_action": "hold",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        params: dict[str, Any] = {}
        for name in defaults:
            value = self.get_parameter(name).value
            params[name] = value
        return params

    def on_left_tactile(self, msg: Any) -> None:
        """左侧传感器消息回调，缓存最新消息和时间戳。"""
        self.left_msg = msg
        self.left_time_s = time.monotonic()

    def on_right_tactile(self, msg: Any) -> None:
        """右侧传感器消息回调，缓存最新消息和时间戳。"""
        self.right_msg = msg
        self.right_time_s = time.monotonic()

    def on_command_echo(self, msg: Int32) -> None:
        """驱动器命令回读回调，同步 CommandBridge 的位置估计。"""
        self.command_bridge.sync_echo(msg.data)

    def _response(
        self, success: bool, message: str, response: Trigger.Response
    ) -> Trigger.Response:
        """构建服务响应。

        Args:
            success: 调用是否成功。
            message: 响应消息。
            response: 待填充的响应对象。

        Returns:
            填充后的 Trigger.Response 对象。
        """
        response.success = success
        response.message = message
        return response

    def on_start(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """启动控制器服务回调。"""
        del request
        allowed, message = evaluate_start_tactile_gate(
            self.left_msg,
            self.right_msg,
            self.params,
        )
        if not allowed:
            return self._response(False, message, response)
        self.state_machine.start()
        return self._response(True, "controller started", response)

    def on_reset_fault(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """复位故障服务回调。"""
        del request
        self.state_machine.reset_fault()
        return self._response(True, "fault reset to IDLE", response)

    def on_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """停止控制器服务回调。"""
        del request
        self.state_machine.stop()
        return self._response(True, "controller stopped", response)

    def is_tactile_timeout(self) -> bool:
        """检查触觉数据是否超时。

        Returns:
            如果任一传感器的最新数据超过设定的超时时间，返回 True。
        """
        now_s = time.monotonic()
        timeout_s = float(self.params["tactile_timeout_s"])
        if self.left_time_s is None or self.right_time_s is None:
            return True
        return (now_s - self.left_time_s > timeout_s) or (now_s - self.right_time_s > timeout_s)

    def execute_action(self, action_name: str, step: int) -> bool:
        """根据动作名执行对应的 CommandBridge 操作。

        Args:
            action_name: 动作名称 ("close_step", "open_step", "open", 或其他)。
            step: 位置步长，仅 step 类动作使用。

        Returns:
            动作是否成功发起。
        """
        if action_name == "close_step":
            return self.command_bridge.close_step(step)
        if action_name == "open_step":
            return self.command_bridge.open_step(step)
        if action_name == "open":
            return self.command_bridge.open()
        return self.command_bridge.hold()

    def publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        """将字典序列化为 JSON 并发布到指定话题。

        Args:
            publisher: ROS 2 发布器。
            payload: 待发布的字典载荷。
        """
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        publisher.publish(msg)

    def control_loop(self) -> None:
        """主控制循环，每个周期由定时器触发。"""
        if not SENSOR_INTERFACES_AVAILABLE:
            self.publish_json(
                self.state_pub,
                {"state": self.state_machine.state, "detail": "sensor_interfaces unavailable"},
            )
            return

        if self.left_msg is None or self.right_msg is None:
            # 没有左右触觉输入时，不允许 controller 发闭合动作。
            self.publish_json(
                self.debug_pub,
                {
                    "state": self.state_machine.state,
                    "action": "wait",
                    "detail": "waiting for tactile data",
                },
            )
            return

        if self.is_tactile_timeout():
            action = self.state_machine.enter_fault("tactile timeout")
            self.execute_action(action.name, action.step)
            self.publish_json(
                self.state_pub, {"state": self.state_machine.state, "detail": action.reason}
            )
            return

        # ==== 触觉特征提取 ====
        features = extract_tactile_features(self.left_msg, self.right_msg, self.params)

        # ==== 过力保护检测 ====
        if self.state_machine.state == "IDLE":
            # 激活、open、人工摆放工件阶段都可能出现较大接触力；
            # 只有 controller 真正 start 进入主动抓取后，才启用超力保护。
            self.overforce_counter = 0
            overforce_fault = False
        else:
            self.overforce_counter, overforce_fault = update_consecutive_limit_counter(
                value=float(features["fn_min"]),
                limit=float(self.params["max_safe_force_n"]),
                current_count=self.overforce_counter,
                required_count=int(self.params["max_safe_force_consecutive_frames"]),
            )
        if overforce_fault:
            # 安全阈值优先级最高，连续超限后直接进入 FAULT。
            action = self.state_machine.enter_fault("force exceeds max_safe_force_n")
            self.get_logger().warning(
                "overforce fault: "
                f"fn_left={float(features['fn_left']):.3f} "
                f"fn_right={float(features['fn_right']):.3f} "
                f"fn_min={float(features['fn_min']):.3f} "
                f"threshold={float(self.params['max_safe_force_n']):.3f} "
                f"overforce_counter={self.overforce_counter} "
                f"both_contact={bool(features['both_contact'])}"
            )
            self.publish_json(
                self.debug_pub,
                {
                    "state": self.state_machine.state,
                    "action": action.name,
                    "reason": action.reason,
                    "current_command_position": self.command_bridge.current_command_position,
                    "overforce_counter": self.overforce_counter,
                    **features,
                },
            )
            command_ok = self.execute_action(action.name, action.step)
            if not command_ok:
                detail = self.command_bridge.last_error or "gripper command failed"
                self.state_machine.enter_fault(detail)
                self.publish_json(
                    self.state_pub, {"state": self.state_machine.state, "detail": detail}
                )
                return
            self.publish_json(
                self.state_pub, {"state": self.state_machine.state, "detail": action.reason}
            )
            return

        # ==== 状态机更新并执行动作 ====
        action = self.state_machine.update(features)
        command_ok = self.execute_action(action.name, action.step)
        if not command_ok:
            detail = self.command_bridge.last_error or "gripper command failed"
            self.state_machine.enter_fault(detail)
            self.publish_json(self.state_pub, {"state": self.state_machine.state, "detail": detail})
            return

        self.publish_json(
            self.debug_pub,
            {
                "state": self.state_machine.state,
                "action": action.name,
                "reason": action.reason,
                "current_command_position": self.command_bridge.current_command_position,
                "overforce_counter": self.overforce_counter,
                **features,
            },
        )
        self.publish_json(
            self.state_pub,
            {"state": self.state_machine.state, "action": action.name, "reason": action.reason},
        )


def main(args: list[str] | None = None) -> None:
    """触觉抓取控制器主入口函数。

    Args:
        args: 命令行参数列表，传递给 rclpy.init。
    """
    rclpy.init(args=args)
    node = TactileGraspControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
