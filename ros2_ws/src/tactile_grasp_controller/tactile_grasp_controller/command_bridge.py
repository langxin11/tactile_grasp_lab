"""Map controller actions to streaming or action-based gripper commands."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

try:
    from control_msgs.action import GripperCommand
except Exception:  # pragma: no cover - allows pure unit tests outside ROS env

    class _DummyCommand:
        def __init__(self) -> None:
            self.position = 0.0
            self.max_effort = 0.0

    class _DummyGoal:
        def __init__(self) -> None:
            self.command = _DummyCommand()

    class GripperCommand:  # type: ignore[override]
        """Fallback stub used when control_msgs is unavailable in unit tests."""

        Goal = _DummyGoal


try:
    from rclpy.action import ActionClient
except Exception:  # pragma: no cover - allows pure unit tests outside ROS env
    ActionClient = None

try:
    from sensor_interfaces.msg import GripperStreamingCommand
except Exception:  # pragma: no cover - allows pure unit tests outside ROS env
    GripperStreamingCommand = None


def clamp_command_position(position: int, min_position: int, max_position: int) -> int:
    """Clamp a byte-like gripper command position."""
    return max(min_position, min(max_position, int(position)))


def map_command_to_action_position(
    command_position: int,
    min_position: int,
    max_position: int,
    open_position: float,
    closed_position: float,
) -> float:
    """Map the controller's 0-255 command estimate to a gripper action position."""
    if max_position <= min_position:
        raise ValueError("max_position must be greater than min_position")

    clamped = clamp_command_position(command_position, min_position, max_position)
    normalized = (clamped - min_position) / (max_position - min_position)
    return open_position + normalized * (closed_position - open_position)


def clamp_speed_byte(speed: int, min_speed: int, max_speed: int) -> int:
    """Clamp a suggested speed byte into the configured streaming limits."""
    if max_speed < min_speed:
        raise ValueError("max_speed must be greater than or equal to min_speed")
    return max(min_speed, min(max_speed, int(speed)))


def compute_suggested_speed(
    step: int,
    control_rate_hz: float,
    target_command_rate_bytes_per_s: float,
    min_speed_byte: int,
    max_speed_byte: int,
) -> int:
    """Map `step * control_rate_hz` into a bounded Robotiq speed byte.

    `target_command_rate_bytes_per_s` is the command-byte rate where the mapping
    saturates to `max_speed_byte`.
    """
    if target_command_rate_bytes_per_s <= 0.0:
        raise ValueError("target_command_rate_bytes_per_s must be positive")

    requested_rate = max(0.0, float(step)) * max(0.0, float(control_rate_hz))
    normalized = min(requested_rate / float(target_command_rate_bytes_per_s), 1.0)
    suggested = round(min_speed_byte + normalized * (max_speed_byte - min_speed_byte))
    return clamp_speed_byte(suggested, min_speed_byte, max_speed_byte)


class CommandBridge:
    """Own the streaming publisher, action client, and tracked command estimate."""

    def __init__(
        self,
        node: Any,
        params: dict[str, Any],
        action_client_factory: Callable[[Any, type, str], Any] | None = None,
        streaming_publisher_factory: Callable[[Any, type, str, int], Any] | None = None,
        streaming_message_factory: Callable[[], Any] | None = None,
    ) -> None:
        """Initialize the command bridge.

        Args:
            node: Parent ROS 2 node.
            params: Parameter dictionary with limits and speed-mapping config.
            action_client_factory: Test injection hook for action clients.
            streaming_publisher_factory: Test injection hook for streaming publishers.
            streaming_message_factory: Test injection hook for streaming messages.
        """
        self.node = node
        self.params = params
        self.dry_run = bool(params["dry_run"])
        self.min_position = int(params["min_position"])
        self.max_position = int(params["max_position"])
        self.current_command_position = int(params["initial_position"])
        self.open_position = float(params["gripper_open_position"])
        self.closed_position = float(params["gripper_closed_position"])
        self.max_effort = float(params["gripper_max_effort"])
        self.wait_for_action_server_s = float(params["wait_for_action_server_s"])
        self.control_rate_hz = float(params["control_rate_hz"])
        self.target_command_rate_bytes_per_s = float(params["target_command_rate_bytes_per_s"])
        self.min_speed_byte = int(params["min_speed_byte"])
        self.max_speed_byte = int(params["max_speed_byte"])
        self.streaming_topic = str(params["streaming_command_topic"])
        self.action_name = str(params["gripper_command_action"])
        self.last_error: str | None = None
        self._command_id = 0
        self._streaming_message_factory = streaming_message_factory or GripperStreamingCommand

        factory = action_client_factory or ActionClient
        if factory is None and not self.dry_run:
            raise RuntimeError("rclpy.action.ActionClient is unavailable")
        self.action_client = (
            None if self.dry_run else factory(node, GripperCommand, self.action_name)
        )

        publisher_factory = streaming_publisher_factory
        if publisher_factory is None and not self.dry_run:
            publisher_factory = getattr(node, "create_publisher", None)

        if self.dry_run or publisher_factory is None or self._streaming_message_factory is None:
            self.streaming_publisher = None
        else:
            self.streaming_publisher = publisher_factory(
                self._streaming_message_factory,
                self.streaming_topic,
                10,
            )

    def sync_echo(self, position: int) -> None:
        """Update the command estimate from the legacy driver echo."""
        self.current_command_position = clamp_command_position(
            int(position), self.min_position, self.max_position
        )

    def suggested_speed_for_step(self, step: int) -> int:
        """Return a bounded speed suggestion for the given close step."""
        return compute_suggested_speed(
            step=step,
            control_rate_hz=self.control_rate_hz,
            target_command_rate_bytes_per_s=self.target_command_rate_bytes_per_s,
            min_speed_byte=self.min_speed_byte,
            max_speed_byte=self.max_speed_byte,
        )

    def close_step(self, step: int) -> bool:
        """Close by a byte-like step from the current command estimate."""
        suggested_speed = self.suggested_speed_for_step(step)
        return self.publish_position(
            self.current_command_position + int(step),
            suggested_speed=suggested_speed,
        )

    def open_step(self, step: int) -> bool:
        """Open by a byte-like step from the current command estimate."""
        suggested_speed = self.suggested_speed_for_step(step)
        return self.publish_position(
            self.current_command_position - int(step),
            suggested_speed=suggested_speed,
        )

    def hold(self) -> bool:
        """Hold the current command and send no new goal."""
        return True

    def open(self) -> bool:
        """Send an open command at the minimum command position."""
        return self.publish_position(self.min_position, suggested_speed=self.min_speed_byte)

    def action_position_for_command(self, position: int) -> float:
        """Return the mapped action-space target for a command position."""
        return map_command_to_action_position(
            position,
            self.min_position,
            self.max_position,
            self.open_position,
            self.closed_position,
        )

    def _next_command_id(self) -> int:
        self._command_id += 1
        return self._command_id

    def _publish_streaming_command(self, position: int, suggested_speed: int) -> bool:
        """Publish a non-blocking streaming command if the publisher is available."""
        if self.streaming_publisher is None or self._streaming_message_factory is None:
            return False

        message = self._streaming_message_factory()
        if hasattr(self.node, "get_clock") and hasattr(message, "stamp"):
            message.stamp = self.node.get_clock().now().to_msg()
        message.command_id = self._next_command_id()
        message.target_position = int(position)
        message.speed = int(
            clamp_speed_byte(suggested_speed, self.min_speed_byte, self.max_speed_byte)
        )
        message.max_effort = float(self.max_effort)
        self.streaming_publisher.publish(message)
        return True

    def publish_position(self, position: int, suggested_speed: int | None = None) -> bool:
        """Clamp a command position and send it as streaming or action command."""
        self.last_error = None
        clamped = clamp_command_position(int(position), self.min_position, self.max_position)
        self.current_command_position = clamped
        if self.dry_run:
            return True

        if suggested_speed is None:
            suggested_speed = self.min_speed_byte

        if self._publish_streaming_command(clamped, suggested_speed):
            return True

        if self.action_client is None:
            self.last_error = "gripper action client is not initialized"
            return False

        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_action_server_s):
            self.last_error = f"gripper action server unavailable: {self.action_name}"
            self.node.get_logger().error(self.last_error)
            return False

        goal = GripperCommand.Goal()
        goal.command.position = self.action_position_for_command(clamped)
        goal.command.max_effort = self.max_effort
        self.action_client.send_goal_async(goal)
        return True
