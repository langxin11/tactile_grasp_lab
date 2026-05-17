"""Map controller actions to standard gripper action commands.

The state machine still tracks a 0-255 command estimate. This bridge maps that
estimate to the position field used by control_msgs/action/GripperCommand.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from control_msgs.action import GripperCommand

try:
    from rclpy.action import ActionClient
except Exception:  # pragma: no cover - allows pure unit tests outside ROS env
    ActionClient = None


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


class CommandBridge:
    """Own the gripper action client and tracked command position estimate.

    Besides sending commands, the bridge keeps a command-position estimate for
    dry-run operation and for cases where measured gripper feedback is absent.

    Attributes:
        node: Parent ROS 2 node.
        params: Parameter dictionary.
        dry_run: Whether commands are simulated.
        min_position: Minimum byte-like command position.
        max_position: Maximum byte-like command position.
        current_command_position: Current command-position estimate.
        action_client: GripperCommand action client.
    """

    def __init__(
        self,
        node: Any,
        params: dict[str, Any],
        action_client_factory: Callable[[Any, type, str], Any] | None = None,
    ) -> None:
        """Initialize the command bridge.

        Args:
            node: Parent ROS 2 node.
            params: Parameter dictionary with dry-run mode and limits.
            action_client_factory: Test injection hook for action clients.
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
        self.action_name = str(params["gripper_command_action"])
        self.last_error: str | None = None

        factory = action_client_factory or ActionClient
        if factory is None and not self.dry_run:
            raise RuntimeError("rclpy.action.ActionClient is unavailable")
        self.action_client = None if self.dry_run else factory(node, GripperCommand, self.action_name)

    def sync_echo(self, position: int) -> None:
        """Update the command estimate from the legacy driver echo.

        Args:
            position: Echoed byte-like position from the legacy driver.
        """
        self.current_command_position = clamp_command_position(
            int(position), self.min_position, self.max_position
        )

    def close_step(self, step: int) -> bool:
        """Close by a byte-like step from the current command estimate.

        Args:
            step: Positive closing step in command units.
        """
        return self.publish_position(self.current_command_position + int(step))

    def hold(self) -> bool:
        """Hold the current command and send no new goal."""
        return True

    def open(self) -> bool:
        """Send an open command at the minimum command position."""
        return self.publish_position(self.min_position)

    def action_position_for_command(self, position: int) -> float:
        """Return the mapped action-space target for a command position."""
        return map_command_to_action_position(
            position,
            self.min_position,
            self.max_position,
            self.open_position,
            self.closed_position,
        )

    def publish_position(self, position: int) -> bool:
        """Clamp a command position and send it as a GripperCommand goal.

        Args:
            position: Raw command position clamped to [min_position, max_position].

        Returns:
            Whether the command was started. Always True in dry-run mode.
        """
        self.last_error = None
        clamped = clamp_command_position(int(position), self.min_position, self.max_position)
        self.current_command_position = clamped
        if self.dry_run:
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
