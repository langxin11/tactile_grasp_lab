"""Robotiq 2F-85 夹爪的 ROS 2 生命周期节点。

该节点封装动作、服务与话题接口，并通过工作线程串行化硬件访问。
"""

from __future__ import annotations

import os
import queue
import sys
import threading
import time
from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_interfaces.msg import GripperStatus
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from .action_utils import map_action_position_to_command, map_effort_to_force_byte
from .driver import Driver, GripperError, MotionResult
from .fake_driver import FakeDriver
from .pymodbus_driver import PymodbusDriver
from .safety import clamp_byte


@dataclass
class HardwareRequest:
    """工作线程拥有的硬件操作请求。

    Attributes:
        label: 操作标签，用于日志与诊断。
        operation: 实际硬件操作函数。
        future: 返回结果的 Future。
    """

    label: str
    operation: Callable[[], Any]
    future: Future[Any]


def _find_repo_venv_python() -> Path | None:
    """查找仓库内的虚拟环境 Python 解释器路径。

    Returns:
        Path | None: 解释器路径，未找到时返回 None。
    """
    for parent in Path(__file__).resolve().parents:
        candidate = parent / ".venv" / "bin" / "python"
        if candidate.is_file():
            return candidate
    return None


def _ensure_repo_venv_python() -> None:
    """确保当前进程运行在仓库虚拟环境中。

    Returns:
        None: 无返回值。
    """
    if os.environ.get("ROBOTIQ_DRIVER_SKIP_VENV_REEXEC") == "1":
        return

    venv_python = _find_repo_venv_python()
    if venv_python is None:
        return

    current_python = Path(sys.executable).resolve()
    if current_python == venv_python.resolve():
        return

    env = os.environ.copy()
    env["ROBOTIQ_DRIVER_SKIP_VENV_REEXEC"] = "1"
    os.execve(str(venv_python), [str(venv_python)] + sys.argv, env)


class RobotiqDriverNode(LifecycleNode):
    """通过单一工作线程串行化硬件访问的生命周期节点。"""

    def __init__(self) -> None:
        """初始化生命周期节点与内部状态。

        Returns:
            None: 无返回值。
        """
        super().__init__("robotiq_2f85_driver")
        self._declare_parameters()

        self.params: dict[str, Any] = {}
        self.core: Driver | None = None

        self.command_echo_pub = None
        self.status_pub = None
        self.position_subscription = None
        self.action_server: ActionServer | None = None
        self.activate_service = None
        self.open_service = None
        self.close_service = None
        self.stop_service = None
        self.reconnect_service = None

        self._publishers_active = False
        self._worker_thread: threading.Thread | None = None
        self._worker_stop = threading.Event()
        self._worker_queue: queue.Queue[HardwareRequest | None] = queue.Queue()
        self._command_lock = threading.Lock()
        self._command_busy = False
        self._ready_for_motion = False
        self._last_feedback_signature: tuple[int, int, int, int, int] | None = None
        self._last_status_key: tuple[str, str, bool, bool, bool] | None = None
        self._last_error_detail: str | None = None

    def _declare_parameters(self) -> None:
        """声明默认参数。

        Returns:
            None: 无返回值。
        """
        defaults = {
            "serial_port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "slave_address": 9,
            "default_speed": 10,
            "default_force": 10,
            "startup_activate": False,
            "dry_run": True,
            "command_topic": "/robotiq/command/position",
            "command_echo_topic": "/robotiq/command/echo",
            "status_topic": "/robotiq/driver/status",
            "feedback_poll_hz": 10.0,
            "gripper_action_name": "/robotiq_gripper_controller/gripper_cmd",
            "gripper_closed_position": 0.8,
            "action_max_force": 235.0,
            "action_timeout_s": 5.0,
            "action_position_tolerance": 2,
            "autostart": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_params(self) -> dict[str, Any]:
        """读取并规范化参数配置。

        Returns:
            dict[str, Any]: 归一化后的参数字典。
        """
        names = [
            "serial_port",
            "baudrate",
            "slave_address",
            "default_speed",
            "default_force",
            "startup_activate",
            "dry_run",
            "command_topic",
            "command_echo_topic",
            "status_topic",
            "feedback_poll_hz",
            "gripper_action_name",
            "gripper_closed_position",
            "action_max_force",
            "action_timeout_s",
            "action_position_tolerance",
            "autostart",
        ]
        params = {name: self.get_parameter(name).value for name in names}
        params["default_speed"] = clamp_byte(params["default_speed"], "default_speed")
        params["default_force"] = clamp_byte(params["default_force"], "default_force")
        params["action_position_tolerance"] = max(0, int(params["action_position_tolerance"]))
        params["feedback_poll_hz"] = max(0.0, float(params["feedback_poll_hz"]))
        return params

    def on_configure(self, state: Any) -> TransitionCallbackReturn:
        """配置发布者、服务、动作服务器与驱动核心。

        Args:
            state: 生命周期状态对象（未使用）。

        Returns:
            TransitionCallbackReturn: 生命周期配置结果。
        """
        del state
        try:
            self.params = self._load_params()
            driver_port = str(self.params["serial_port"])
            driver_baudrate = int(self.params["baudrate"])
            driver_slave_address = int(self.params["slave_address"])
            if bool(self.params["dry_run"]):
                self.core = FakeDriver(
                    port=driver_port,
                    baudrate=driver_baudrate,
                    slave_address=driver_slave_address,
                )
            else:
                self.core = PymodbusDriver(
                    port=driver_port,
                    baudrate=driver_baudrate,
                    slave_address=driver_slave_address,
                )
            self.command_echo_pub = self.create_lifecycle_publisher(
                Int32, str(self.params["command_echo_topic"]), 10
            )
            self.status_pub = self.create_lifecycle_publisher(
                GripperStatus, str(self.params["status_topic"]), 10
            )
            self.position_subscription = self.create_subscription(
                Int32,
                str(self.params["command_topic"]),
                self.on_position_command,
                10,
            )
            self.activate_service = self.create_service(
                Trigger, "/robotiq/activate", self.on_activate_service
            )
            self.open_service = self.create_service(Trigger, "/robotiq/open", self.on_open)
            self.close_service = self.create_service(Trigger, "/robotiq/close", self.on_close)
            self.stop_service = self.create_service(Trigger, "/robotiq/stop", self.on_stop)
            self.reconnect_service = self.create_service(
                Trigger, "/robotiq/reconnect", self.on_reconnect
            )
            self.action_server = ActionServer(
                self,
                GripperCommand,
                str(self.params["gripper_action_name"]),
                execute_callback=self.execute_action_goal,
                goal_callback=self.on_action_goal,
                handle_accepted_callback=self.on_action_accepted,
                cancel_callback=self.on_action_cancel,
            )
            self._start_worker()
        except Exception as exc:
            self.get_logger().error(f"configure failed: {exc}")
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: Any) -> TransitionCallbackReturn:
        """激活发布者并按需连接夹爪。

        Args:
            state: 生命周期状态对象。

        Returns:
            TransitionCallbackReturn: 生命周期激活结果。
        """
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        self._publishers_active = True
        self.publish_status("initializing", "driver lifecycle activated", force=True)
        connect_label = "connect+activate" if self.params["startup_activate"] else "connect"
        connect_detail = "connecting to gripper"
        if self.params["startup_activate"]:
            connect_detail = "connecting to gripper and activating hardware"
        accepted, message, future = self._submit_background_command(
            connect_label,
            lambda: self._connect_sequence(bool(self.params["startup_activate"])),
            require_motion_ready=False,
        )
        del future
        if accepted:
            self.publish_status("initializing", connect_detail, force=True)
        else:
            self.publish_status("error", message, force=True)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: Any) -> TransitionCallbackReturn:
        """停用发布者并重置运动就绪状态。

        Args:
            state: 生命周期状态对象。

        Returns:
            TransitionCallbackReturn: 生命周期停用结果。
        """
        self.publish_status("inactive", "driver lifecycle deactivating", force=True)
        self._publishers_active = False
        self._ready_for_motion = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: Any) -> TransitionCallbackReturn:
        """释放 ROS 实体并重置驱动资源。

        Args:
            state: 生命周期状态对象（未使用）。

        Returns:
            TransitionCallbackReturn: 生命周期清理结果。
        """
        del state
        self._publishers_active = False
        self._ready_for_motion = False
        self._stop_worker()
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.position_subscription is not None:
            self.destroy_subscription(self.position_subscription)
            self.position_subscription = None
        for service_name in (
            "activate_service",
            "open_service",
            "close_service",
            "stop_service",
            "reconnect_service",
        ):
            service = getattr(self, service_name)
            if service is not None:
                self.destroy_service(service)
                setattr(self, service_name, None)
        if self.command_echo_pub is not None:
            self.destroy_lifecycle_publisher(self.command_echo_pub)
            self.command_echo_pub = None
        if self.status_pub is not None:
            self.destroy_lifecycle_publisher(self.status_pub)
            self.status_pub = None
        if self.core is not None:
            self.core.close()
            self.core = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: Any) -> TransitionCallbackReturn:
        """停止后台线程并关闭驱动连接。

        Args:
            state: 生命周期状态对象（未使用）。

        Returns:
            TransitionCallbackReturn: 生命周期关闭结果。
        """
        del state
        self._stop_worker()
        if self.core is not None:
            self.core.close()
        return TransitionCallbackReturn.SUCCESS

    def _start_worker(self) -> None:
        """启动硬件工作线程。

        Returns:
            None: 无返回值。
        """
        self._worker_stop.clear()
        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name="robotiq_2f85_driver_worker",
            daemon=True,
        )
        self._worker_thread.start()

    def _stop_worker(self) -> None:
        """停止硬件工作线程并清空队列。

        Returns:
            None: 无返回值。
        """
        self._worker_stop.set()
        self._worker_queue.put(None)
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
            self._worker_thread = None
        while True:
            try:
                self._worker_queue.get_nowait()
            except queue.Empty:
                break

    def _worker_loop(self) -> None:
        """后台线程主循环，处理硬件请求与反馈轮询。

        Returns:
            None: 无返回值。
        """
        next_poll_s = time.monotonic()
        while not self._worker_stop.is_set():
            timeout = None
            if self.params:
                poll_hz = float(self.params["feedback_poll_hz"])
                if poll_hz > 0.0:
                    timeout = max(0.0, next_poll_s - time.monotonic())
            try:
                request = self._worker_queue.get(timeout=timeout)
            except queue.Empty:
                self._poll_feedback()
                next_poll_s = time.monotonic() + self._poll_period_s()
                continue
            if request is None:
                return
            try:
                result = request.operation()
            except Exception as exc:
                self.publish_status("error", str(exc), force=True)
                request.future.set_exception(exc)
            else:
                request.future.set_result(result)
            finally:
                next_poll_s = time.monotonic() + self._poll_period_s()

    def _poll_period_s(self) -> float:
        """计算反馈轮询周期。

        Returns:
            float: 轮询周期（秒）。
        """
        poll_hz = float(self.params.get("feedback_poll_hz", 0.0))
        return (1.0 / poll_hz) if poll_hz > 0.0 else 1.0

    def _poll_feedback(self) -> None:
        """轮询并发布当前反馈状态。

        Returns:
            None: 无返回值。
        """
        if not self._publishers_active or self.core is None:
            return
        if not self.core.connected and not self.core.dry_run:
            return
        try:
            feedback = self.core.read_feedback()
        except GripperError as exc:
            detail = str(exc)
            if detail != self._last_error_detail:
                self.publish_status("error", detail, force=True)
            return

        if feedback is None:
            return
        self._ready_for_motion = bool(self.core.dry_run or feedback.is_activation_complete)
        signature = self._feedback_signature(feedback)
        if feedback.fault_status != 0x00:
            detail = (
                f"background fault=0x{feedback.fault_status:02X} ({feedback.fault_text}), "
                f"status=0x{feedback.gripper_status:02X}, position={feedback.position}, "
                f"current={feedback.current_milliamps}mA"
            )
            if detail != self._last_error_detail:
                self.publish_status("error", detail, force=True)
            self._last_feedback_signature = signature
            return
        if signature != self._last_feedback_signature:
            if self._ready_for_motion:
                detail = "activation complete and communication alive"
                state = "ready"
            else:
                detail = "connected; waiting for explicit activation"
                state = "not_ready"
            self.publish_status(state, detail, force=True)
            self._last_feedback_signature = signature

    def _reserve_command_slot(self) -> bool:
        """尝试占用硬件命令槽位。

        Returns:
            bool: 是否成功占用。
        """
        with self._command_lock:
            if self._command_busy:
                return False
            self._command_busy = True
            return True

    def _release_command_slot(self) -> None:
        """释放硬件命令槽位。

        Returns:
            None: 无返回值。
        """
        with self._command_lock:
            self._command_busy = False

    def _submit_background_command(
        self,
        label: str,
        operation: Callable[[], Any],
        *,
        require_motion_ready: bool,
    ) -> tuple[bool, str, Future[Any] | None]:
        """提交后台硬件命令。

        Args:
            label: 命令标签。
            operation: 实际执行的硬件操作。
            require_motion_ready: 是否要求夹爪已就绪。

        Returns:
            tuple[bool, str, Future[Any] | None]:
                是否接受、描述消息与执行 Future。
        """
        if self.core is None:
            return False, "driver is not configured", None
        if not self._publishers_active:
            return False, "driver lifecycle node is not active", None
        if require_motion_ready and not self._ready_for_motion:
            return False, "gripper is not ready; activate the hardware first", None
        if not self._reserve_command_slot():
            return False, "driver is busy with another hardware command", None

        future: Future[Any] = Future()
        future.add_done_callback(lambda _: self._release_command_slot())
        self._worker_queue.put(HardwareRequest(label=label, operation=operation, future=future))
        return True, f"{label} queued", future

    def _connect_sequence(self, activate_after_connect: bool) -> str:
        """连接夹爪并按需激活。

        Args:
            activate_after_connect: 是否连接后立即激活。

        Returns:
            str: 连接流程结果标识。

        Raises:
            GripperError: 连接或激活失败时抛出。
        """
        if self.core is None:
            raise GripperError("driver core is unavailable")
        if not self.core.connected:
            self.core.connect()
        if activate_after_connect:
            self.core.activate()
        feedback = self.core.read_feedback()
        self._ready_for_motion = bool(
            self.core.dry_run or (feedback and feedback.is_activation_complete)
        )
        if self._ready_for_motion:
            self.publish_status("ready", "gripper connected and ready", force=True)
        else:
            self.publish_status(
                "not_ready", "connected; waiting for explicit activation", force=True
            )
        return "connected"

    def _activate_sequence(self) -> str:
        """执行激活流程。

        Returns:
            str: 激活流程结果标识。

        Raises:
            GripperError: 激活失败时抛出。
        """
        if self.core is None:
            raise GripperError("driver core is unavailable")
        if not self.core.connected:
            self.core.connect()
        self.core.activate()
        feedback = self.core.read_feedback()
        self._ready_for_motion = bool(
            self.core.dry_run or (feedback and feedback.is_activation_complete)
        )
        self.publish_status("ready", "activation completed", force=True)
        return "activated"

    def _reconnect_sequence(self) -> str:
        """执行重连流程。

        Returns:
            str: 重连流程结果标识。

        Raises:
            GripperError: 重连失败时抛出。
        """
        if self.core is None:
            raise GripperError("driver core is unavailable")
        self.publish_status("initializing", "reconnecting to gripper", force=True)
        self.core.close()
        self._ready_for_motion = False
        return self._connect_sequence(bool(self.params["startup_activate"]))

    def _move_sequence(self, target: int, force: int) -> MotionResult:
        """执行一次运动指令并等待完成。

        Args:
            target: 目标位置字节。
            force: 力度字节。

        Returns:
            MotionResult: 运动完成结果。

        Raises:
            GripperError: 运动失败或反馈异常时抛出。
        """
        if self.core is None:
            raise GripperError("driver core is unavailable")
        target = clamp_byte(target, "position")
        force = clamp_byte(force, "force")
        self.core.move(
            position=target,
            speed=int(self.params["default_speed"]),
            force=force,
        )
        self.publish_echo(target)
        result = self.core.wait_for_motion_complete(
            target_position=target,
            timeout_s=float(self.params["action_timeout_s"]),
            position_tolerance=int(self.params["action_position_tolerance"]),
        )
        if result.reached_goal:
            detail = f"target position {target} reached"
        elif result.stalled:
            detail = f"target position {target} stalled on object contact"
        else:
            detail = f"target position {target} finished"
        self.publish_status("commanded", detail, force=True)
        return result

    def _stop_sequence(self) -> str:
        """执行停止动作。

        Returns:
            str: 停止流程结果标识。

        Raises:
            GripperError: 停止失败时抛出。
        """
        if self.core is None:
            raise GripperError("driver core is unavailable")
        self.core.stop()
        self.publish_status("commanded", "stop command completed", force=True)
        return "stopped"

    def publish_echo(self, position: int) -> None:
        """发布指令位置回显话题。

        Args:
            position: 指令位置字节。

        Returns:
            None: 无返回值。
        """
        if not self._publishers_active or self.command_echo_pub is None:
            return
        msg = Int32()
        msg.data = int(position)
        self.command_echo_pub.publish(msg)

    def _feedback_signature(self, feedback: Any) -> tuple[int, int, int, int, int]:
        """生成反馈签名用于去重判断。

        Args:
            feedback: 夹爪反馈对象。

        Returns:
            tuple[int, int, int, int, int]: 反馈签名。
        """
        return (
            int(feedback.gripper_status),
            int(feedback.fault_status),
            int(feedback.position_request_echo),
            int(feedback.position),
            int(feedback.current),
        )

    def publish_status(self, state: str, detail: str, *, force: bool = False) -> None:
        """发布驱动状态与反馈快照。

        Args:
            state: 状态标识。
            detail: 状态描述。
            force: 是否强制发布。

        Returns:
            None: 无返回值。
        """
        if not self._publishers_active or self.status_pub is None or self.core is None:
            return
        feedback = self.core.last_feedback
        synthetic = bool(self.core.dry_run and feedback is not None)
        status_key = (state, detail, self.core.connected, self._ready_for_motion, synthetic)
        if not force and status_key == self._last_status_key:
            return
        msg = GripperStatus()
        msg.state = state
        msg.detail = detail
        msg.dry_run = bool(self.core.dry_run)
        msg.connected = bool(self.core.connected)
        msg.synthetic_feedback = synthetic
        if feedback is not None:
            msg.gripper_status = int(feedback.gripper_status)
            msg.fault_status = int(feedback.fault_status)
            msg.fault_text = str(feedback.fault_text)
            msg.position_request_echo = int(feedback.position_request_echo)
            msg.position = int(feedback.position)
            msg.current = int(feedback.current)
            msg.current_milliamps = int(feedback.current_milliamps)
            msg.activation_state = int(feedback.activation_state)
            msg.activation_echo = int(feedback.activation_echo)
            msg.go_to_echo = int(feedback.go_to_echo)
            msg.object_status = int(feedback.object_status)
            msg.is_activation_complete = bool(feedback.is_activation_complete)
        self.status_pub.publish(msg)
        self._last_status_key = status_key
        self._last_error_detail = detail if state == "error" else None

    def on_position_command(self, msg: Int32) -> None:
        """处理来自订阅话题的位置指令。

        Args:
            msg: 位置指令消息。

        Returns:
            None: 无返回值。
        """
        target = clamp_byte(msg.data, "position")
        accepted, message, future = self._submit_background_command(
            "topic_move",
            lambda: self._move_sequence(target, int(self.params["default_force"])),
            require_motion_ready=True,
        )
        del future
        if not accepted:
            self.publish_status("error", message, force=True)
            self.get_logger().error(message)

    def _handle_service_queue(
        self,
        response: Trigger.Response,
        label: str,
        operation: Callable[[], Any],
        *,
        require_motion_ready: bool,
    ) -> Trigger.Response:
        """统一处理服务队列逻辑。

        Args:
            response: 服务响应对象。
            label: 操作标签。
            operation: 实际执行的硬件操作。
            require_motion_ready: 是否要求夹爪已就绪。

        Returns:
            Trigger.Response: 填充后的响应对象。
        """
        accepted, message, future = self._submit_background_command(
            label,
            operation,
            require_motion_ready=require_motion_ready,
        )
        if not accepted:
            response.success = False
            response.message = message
            self.publish_status("error", message, force=True)
            return response

        self.publish_status("initializing", message, force=True)
        try:
            future.result()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.publish_status("error", str(exc), force=True)
            return response

        response.success = True
        response.message = f"{label} completed"
        return response

    def on_activate_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """处理激活服务请求。

        Args:
            request: 服务请求对象。
            response: 服务响应对象。

        Returns:
            Trigger.Response: 填充后的响应对象。
        """
        del request
        return self._handle_service_queue(
            response,
            "activate",
            self._activate_sequence,
            require_motion_ready=False,
        )

    def on_open(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """处理打开服务请求。

        Args:
            request: 服务请求对象。
            response: 服务响应对象。

        Returns:
            Trigger.Response: 填充后的响应对象。
        """
        del request
        return self._handle_service_queue(
            response,
            "open",
            lambda: self._move_sequence(0, int(self.params["default_force"])),
            require_motion_ready=True,
        )

    def on_close(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """处理关闭服务请求。

        Args:
            request: 服务请求对象。
            response: 服务响应对象。

        Returns:
            Trigger.Response: 填充后的响应对象。
        """
        del request
        return self._handle_service_queue(
            response,
            "close",
            lambda: self._move_sequence(255, int(self.params["default_force"])),
            require_motion_ready=True,
        )

    def on_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """处理停止服务请求。

        Args:
            request: 服务请求对象。
            response: 服务响应对象。

        Returns:
            Trigger.Response: 填充后的响应对象。
        """
        del request
        return self._handle_service_queue(
            response,
            "stop",
            self._stop_sequence,
            require_motion_ready=False,
        )

    def on_reconnect(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """处理重连服务请求。

        Args:
            request: 服务请求对象。
            response: 服务响应对象。

        Returns:
            Trigger.Response: 填充后的响应对象。
        """
        del request
        return self._handle_service_queue(
            response,
            "reconnect",
            self._reconnect_sequence,
            require_motion_ready=False,
        )

    def on_action_goal(self, goal_request: GripperCommand.Goal) -> GoalResponse:
        """校验动作目标是否可接受。

        Args:
            goal_request: 动作目标请求。

        Returns:
            GoalResponse: 是否接受该目标。
        """
        if self.core is None or not self._publishers_active:
            return GoalResponse.REJECT
        if not self._ready_for_motion:
            return GoalResponse.REJECT
        if not self._reserve_command_slot():
            return GoalResponse.REJECT
        try:
            map_action_position_to_command(
                float(goal_request.command.position),
                float(self.params["gripper_closed_position"]),
            )
            map_effort_to_force_byte(
                float(goal_request.command.max_effort),
                float(self.params["action_max_force"]),
                int(self.params["default_force"]),
            )
        except ValueError:
            self._release_command_slot()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_action_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """拒绝动作取消请求（当前不支持）。

        Args:
            goal_handle: 动作句柄。

        Returns:
            CancelResponse: 取消请求的处理结果。
        """
        del goal_handle
        return CancelResponse.REJECT

    def on_action_accepted(self, goal_handle: ServerGoalHandle) -> None:
        """为已接受的动作目标启动执行线程。

        Args:
            goal_handle: 动作句柄。

        Returns:
            None: 无返回值。
        """
        thread = threading.Thread(
            target=goal_handle.execute,
            args=(self.execute_action_goal,),
            daemon=True,
            name="robotiq_2f85_action_goal",
        )
        thread.start()

    def execute_action_goal(self, goal_handle: ServerGoalHandle) -> GripperCommand.Result:
        """在线程队列中执行动作目标。

        Args:
            goal_handle: 动作句柄。

        Returns:
            GripperCommand.Result: 动作执行结果。
        """
        try:
            target = map_action_position_to_command(
                float(goal_handle.request.command.position),
                float(self.params["gripper_closed_position"]),
            )
            force = map_effort_to_force_byte(
                float(goal_handle.request.command.max_effort),
                float(self.params["action_max_force"]),
                int(self.params["default_force"]),
            )
            future: Future[MotionResult] = Future()
            self._worker_queue.put(
                HardwareRequest(
                    label="action_move",
                    operation=lambda: self._move_sequence(target, force),
                    future=future,
                )
            )
            result_data = future.result()
            result = GripperCommand.Result()
            result.position = float(goal_handle.request.command.position)
            result.effort = float(goal_handle.request.command.max_effort)
            result.stalled = bool(result_data.stalled)
            result.reached_goal = bool(result_data.reached_goal)
            goal_handle.succeed()
            return result
        except Exception as exc:
            goal_handle.abort()
            self.publish_status("error", str(exc), force=True)
            result = GripperCommand.Result()
            result.position = float(goal_handle.request.command.position)
            result.effort = float(goal_handle.request.command.max_effort)
            result.stalled = False
            result.reached_goal = False
            return result
        finally:
            self._release_command_slot()

    def destroy_node(self) -> bool:
        """销毁节点前停止线程并关闭驱动。

        Returns:
            bool: 销毁结果。
        """
        self._stop_worker()
        if self.core is not None:
            self.core.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Robotiq 2F-85 驱动节点入口。

    Args:
        args: 传入的命令行参数。

    Returns:
        None: 无返回值。
    """
    _ensure_repo_venv_python()
    rclpy.init(args=args)
    node = RobotiqDriverNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if bool(node.get_parameter("autostart").value):
            configure_result = node.trigger_configure()
            if configure_result != TransitionCallbackReturn.SUCCESS:
                node.get_logger().error("Lifecycle configure transition failed.")
            else:
                activate_result = node.trigger_activate()
                if activate_result != TransitionCallbackReturn.SUCCESS:
                    node.get_logger().error("Lifecycle activate transition failed.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
