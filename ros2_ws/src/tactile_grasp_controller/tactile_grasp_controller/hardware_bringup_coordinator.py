"""硬件启动协调器：负责触觉传感器降噪校准和夹爪初始打开。

按照固定流程执行：
1. 等待左右触觉传感器数据流稳定。
2. 发起传感器降噪校准请求。
3. 等待夹爪 action server 可用并发送打开命令。
4. 可选：启动触觉闭环控制器。
"""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_interfaces.msg import SensorState
from sensor_interfaces.srv import BiasRequest
from std_msgs.msg import String
from std_srvs.srv import Trigger


class HardwareBringupCoordinator(Node):
    """协调触觉降噪校准、夹爪激活和初始打开动作。

    通过有限状态机驱动硬件启动流程，每个阶段通过定时器轮询推进。
    使用异步服务调用，支持超时检测和失败处理。
    """

    def __init__(self) -> None:
        super().__init__("hardware_bringup_coordinator")

        self.params = self._declare_and_load_params()
        self.status_pub = self.create_publisher(String, self.params["status_topic"], 10)

        self.left_seen = False
        self.right_seen = False
        self.left_time_s: float | None = None
        self.right_time_s: float | None = None
        self.stable_since_s: float | None = None
        self.wait_until_s: float | None = None

        self.phase = "wait_tactile"
        self.pending_future: Any | None = None
        self.pending_client: Client | None = None
        self.pending_action_name: str | None = None
        self.pending_kind: str | None = None
        self.pending_deadline_s: float | None = None
        self.last_status_detail: str | None = None
        tactile_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.create_subscription(
            SensorState,
            self.params["left_tactile_topic"],
            self.on_left_tactile,
            qos_profile=tactile_qos,
        )
        self.create_subscription(
            SensorState,
            self.params["right_tactile_topic"],
            self.on_right_tactile,
            qos_profile=tactile_qos,
        )

        self.bias_client = self.create_client(BiasRequest, self.params["bias_service"])
        self.activate_gripper_client = self.create_client(
            Trigger, self.params["gripper_activate_service"]
        )
        self.gripper_action_client = ActionClient(
            self, GripperCommand, self.params["gripper_command_action"]
        )
        self.start_controller_client = self.create_client(
            Trigger, self.params["controller_start_service"]
        )

        self.timer = self.create_timer(0.1, self.on_timer)

    def _declare_and_load_params(self) -> dict[str, Any]:
        """声明并加载节点参数。

        Returns:
            包含所有参数名到值的映射字典。
        """
        defaults = {
            "left_tactile_topic": "/hub_0/sensor_0",
            "right_tactile_topic": "/hub_0/sensor_1",
            "status_topic": "/tactile_grasp/hardware_bringup_status",
            "perform_bias": True,
            "tactile_stable_s": 1.0,
            "bias_settle_s": 0.5,
            "bias_service": "/hub_0/send_bias_request",
            "gripper_activate_service": "/robotiq/activate",
            "gripper_command_action": "/robotiq_gripper_controller/gripper_cmd",
            "gripper_open_position": 0.0,
            "gripper_max_effort": 10.0,
            "wait_for_action_server_s": 2.0,
            "service_timeout_s": 5.0,
            "action_timeout_s": 5.0,
            "call_controller_start": False,
            "post_open_settle_s": 0.5,
            "controller_start_service": "/tactile_grasp/start",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    def on_left_tactile(self, msg: SensorState) -> None:
        """左侧传感器消息回调，标记已收到并记录时间。"""
        del msg
        self.left_seen = True
        self.left_time_s = time.monotonic()

    def on_right_tactile(self, msg: SensorState) -> None:
        """右侧传感器消息回调，标记已收到并记录时间。"""
        del msg
        self.right_seen = True
        self.right_time_s = time.monotonic()

    def publish_status(self, phase: str, detail: str) -> None:
        """发布当前启动状态（JSON 格式）。

        Args:
            phase: 当前阶段名称。
            detail: 详细状态描述。
        """
        payload = {"phase": phase, "detail": detail}
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        if detail != self.last_status_detail:
            self.get_logger().info(f"{phase}: {detail}")
            self.last_status_detail = detail

    def set_phase(self, phase: str, detail: str) -> None:
        """设置当前阶段并发布状态。

        Args:
            phase: 新阶段名称。
            detail: 阶段描述。
        """
        self.phase = phase
        self.publish_status(phase, detail)

    def tactile_ready(self) -> bool:
        """检查双侧触觉数据是否都已就绪。

        Returns:
            如果左右两侧均有传感器数据到达，返回 True。
        """
        if not self.left_seen or not self.right_seen:
            return False
        if self.left_time_s is None or self.right_time_s is None:
            return False
        return True

    def start_wait(self, phase: str, delay_s: float, detail: str) -> None:
        """进入等待阶段，延迟指定秒数后继续。

        Args:
            phase: 等待阶段名称。
            delay_s: 等待时长（秒）。
            detail: 状态描述。
        """
        self.wait_until_s = time.monotonic() + delay_s
        self.set_phase(phase, detail)

    def call_trigger(self, client: Client, action_name: str, next_phase: str) -> bool:
        """异步调用 Trigger 类型服务。

        Args:
            client: ROS 2 服务客户端。
            action_name: 动作名称（用于日志和超时报错）。
            next_phase: 调用成功后的下一阶段名称。

        Returns:
            如果服务就绪并已发起调用，返回 True；否则返回 False。
        """
        if not client.service_is_ready():
            self.publish_status(next_phase, f"waiting for service {client.srv_name}")
            return False
        request = Trigger.Request()
        self.pending_future = client.call_async(request)
        self.pending_client = client
        self.pending_action_name = action_name
        self.pending_kind = "service"
        self.pending_deadline_s = time.monotonic() + float(self.params["service_timeout_s"])
        self.set_phase(next_phase, f"calling {action_name}")
        return True

    def call_bias(self) -> bool:
        """异步调用触觉降噪校准服务。

        Returns:
            如果服务就绪并已发起调用，返回 True；否则返回 False。
        """
        if not self.bias_client.service_is_ready():
            self.publish_status("wait_bias", f"waiting for service {self.bias_client.srv_name}")
            return False
        request = BiasRequest.Request()
        self.pending_future = self.bias_client.call_async(request)
        self.pending_client = self.bias_client
        self.pending_action_name = "bias"
        self.pending_kind = "service"
        self.pending_deadline_s = time.monotonic() + float(self.params["service_timeout_s"])
        self.set_phase("wait_bias", "calling tactile bias service")
        return True

    def send_gripper_open(self) -> bool:
        """Send an opening GripperCommand goal through the new execution layer."""

        if not self.gripper_action_client.wait_for_server(
            timeout_sec=float(self.params["wait_for_action_server_s"])
        ):
            self.publish_status(
                "wait_gripper_action",
                f"waiting for action {self.params['gripper_command_action']}",
            )
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(self.params["gripper_open_position"])
        goal.command.max_effort = float(self.params["gripper_max_effort"])
        self.pending_future = self.gripper_action_client.send_goal_async(goal)
        self.pending_client = None
        self.pending_action_name = "open"
        self.pending_kind = "action_goal"
        self.pending_deadline_s = time.monotonic() + float(self.params["action_timeout_s"])
        self.set_phase("wait_open", "sending gripper open action goal")
        return True

    def finish_success(self, detail: str) -> None:
        """标记启动流程成功完成。

        Args:
            detail: 完成描述。
        """
        self.set_phase("complete", detail)

    def fail(self, detail: str) -> None:
        """标记启动流程失败。

        Args:
            detail: 失败原因描述。
        """
        self.set_phase("error", detail)

    def handle_pending_future(self) -> None:
        """检查并处理挂起的异步服务调用结果。

        处理三种情况：
        - 调用超时：记录失败。
        - 调用未完成：不处理，等待下次轮询。
        - 调用完成：根据动作名称推进到下一阶段。
        """
        if self.pending_future is None:
            return

        now_s = time.monotonic()
        # ==== 超时检测 ====
        if self.pending_deadline_s is not None and now_s > self.pending_deadline_s:
            action_name = self.pending_action_name or "service call"
            self.pending_future = None
            self.pending_client = None
            self.pending_action_name = None
            self.pending_kind = None
            self.pending_deadline_s = None
            self.fail(f"{action_name} timed out")
            return

        if not self.pending_future.done():
            return

        action_name = self.pending_action_name or "service call"
        pending_kind = self.pending_kind or "service"
        # ==== 结果获取与异常处理 ====
        try:
            response = self.pending_future.result()
        except Exception as exc:  # pragma: no cover - ROS future 异常仅运行时出现
            self.pending_future = None
            self.pending_client = None
            self.pending_action_name = None
            self.pending_kind = None
            self.pending_deadline_s = None
            self.fail(f"{action_name} failed: {exc}")
            return

        self.pending_future = None
        self.pending_client = None
        self.pending_action_name = None
        self.pending_kind = None
        self.pending_deadline_s = None

        if pending_kind == "action_goal":
            goal_handle = response
            if not goal_handle.accepted:
                self.fail("gripper open action goal was rejected")
                return
            self.pending_future = goal_handle.get_result_async()
            self.pending_action_name = action_name
            self.pending_kind = "action_result"
            self.pending_deadline_s = time.monotonic() + float(self.params["action_timeout_s"])
            self.set_phase("wait_open_result", "waiting for gripper open result")
            return

        if pending_kind == "action_result":
            result = response.result
            if not result.reached_goal and not result.stalled:
                self.fail("gripper open action did not reach goal")
                return
            if bool(self.params["call_controller_start"]):
                settle_s = float(self.params["post_open_settle_s"])
                if settle_s > 0.0:
                    self.start_wait(
                        "wait_open_settle",
                        settle_s,
                        "gripper opened; waiting before controller start",
                    )
                    return
                if self.call_trigger(
                    self.start_controller_client, "controller start", "wait_controller_start"
                ):
                    return
                return
            self.finish_success("hardware bringup completed; controller remains in IDLE")
            return

        # ==== 根据动作类型推进流程 ====
        if action_name == "bias":
            if not response.result:
                self.fail("tactile bias request returned false")
                return
            self.start_wait(
                "wait_bias_settle",
                float(self.params["bias_settle_s"]),
                "tactile bias completed; waiting for readings to settle",
            )
            return

        if not response.success:
            self.fail(f"{action_name} failed: {response.message}")
            return

        if action_name == "gripper activate":
            self.send_gripper_open()
            return

        if action_name == "controller start":
            self.finish_success("hardware bringup completed and controller started")
            return

        self.finish_success(f"{action_name} completed")

    def on_timer(self) -> None:
        """定时器回调，驱动硬件启动流程的主状态机。

        每 0.1 秒调用一次，根据当前阶段推进流程。
        """
        if self.phase in {"complete", "error"}:
            return

        # ==== 处理挂起的异步调用 ====
        if self.pending_future is not None:
            self.handle_pending_future()
            return

        now_s = time.monotonic()

        # ==== wait_tactile: 等待触觉流 ====
        if self.phase == "wait_tactile":
            if not self.tactile_ready():
                self.publish_status("wait_tactile", "waiting for left/right tactile streams")
                return
            if self.stable_since_s is None:
                self.stable_since_s = now_s
            stable_elapsed = now_s - self.stable_since_s
            stable_target = float(self.params["tactile_stable_s"])
            if stable_elapsed < stable_target:
                self.publish_status(
                    "wait_tactile",
                    f"tactile streams detected; stabilizing for {stable_target:.1f}s",
                )
                return
            if bool(self.params["perform_bias"]):
                self.call_bias()
                return
            self.call_trigger(
                self.activate_gripper_client, "gripper activate", "wait_gripper_activate"
            )
            return

        # ==== wait_bias_settle: 等待降噪校准稳定 ====
        if self.phase == "wait_bias_settle":
            if self.wait_until_s is None or now_s < self.wait_until_s:
                self.publish_status(
                    "wait_bias_settle",
                    "waiting after tactile bias before gripper activation",
                )
                return
            self.wait_until_s = None
            self.call_trigger(
                self.activate_gripper_client, "gripper activate", "wait_gripper_activate"
            )
            return

        # ==== wait_open_settle: 等待夹爪打开稳定 ====
        if self.phase == "wait_open_settle":
            if self.wait_until_s is None or now_s < self.wait_until_s:
                self.publish_status(
                    "wait_open_settle",
                    "waiting after gripper open before controller start",
                )
                return
            self.wait_until_s = None
            self.call_trigger(
                self.start_controller_client, "controller start", "wait_controller_start"
            )
            return

        self.publish_status(self.phase, "waiting for next bringup step")


def main(args: list[str] | None = None) -> None:
    """硬件启动协调器主入口函数。

    Args:
        args: 命令行参数列表，传递给 rclpy.init。
    """
    rclpy.init(args=args)
    node = HardwareBringupCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
