"""Unit tests for gripper action command bridging."""

import pytest
from tactile_grasp_controller.command_bridge import (
    CommandBridge,
    map_command_to_action_position,
)


class FakeLogger:
    """Collect error logs without requiring a real ROS node."""

    def __init__(self) -> None:
        """Initialize an empty error collection."""
        self.errors: list[str] = []

    def error(self, message: str) -> None:
        """Record an error message."""
        self.errors.append(message)


class FakeNode:
    """Minimal node surface used by CommandBridge tests."""

    def __init__(self) -> None:
        """Initialize a fake logger."""
        self.logger = FakeLogger()

    def get_logger(self) -> FakeLogger:
        """Return the fake logger."""
        return self.logger


class FakeActionClient:
    """Capture GripperCommand goals sent by CommandBridge."""

    def __init__(self, available: bool = True) -> None:
        """Initialize fake server availability and captured goals."""
        self.available = available
        self.goals: list[object] = []
        self.timeout_sec: float | None = None

    def wait_for_server(self, timeout_sec: float) -> bool:
        """Return configured server availability."""
        self.timeout_sec = timeout_sec
        return self.available

    def send_goal_async(self, goal: object) -> object:
        """Capture a goal and return a dummy future."""
        self.goals.append(goal)
        return object()


def make_params(dry_run: bool = False) -> dict[str, object]:
    """Return a complete parameter dictionary for CommandBridge tests."""
    return {
        "dry_run": dry_run,
        "min_position": 0,
        "max_position": 255,
        "initial_position": 0,
        "gripper_command_action": "/robotiq_gripper_controller/gripper_cmd",
        "gripper_open_position": 0.0,
        "gripper_closed_position": 0.8,
        "gripper_max_effort": 10.0,
        "wait_for_action_server_s": 2.0,
    }


def test_maps_command_position_to_action_position() -> None:
    """Verify open, closed, and midpoint mapping."""
    assert map_command_to_action_position(0, 0, 255, 0.0, 0.8) == pytest.approx(0.0)
    assert map_command_to_action_position(255, 0, 255, 0.0, 0.8) == pytest.approx(0.8)
    assert map_command_to_action_position(127, 0, 255, 0.0, 0.8) == pytest.approx(
        127 / 255 * 0.8
    )


def test_open_sends_open_goal() -> None:
    """Opening sends a GripperCommand goal at the configured open position."""
    fake_client = FakeActionClient()
    bridge = CommandBridge(
        FakeNode(),
        make_params(),
        action_client_factory=lambda node, action_type, action_name: fake_client,
    )

    assert bridge.open()
    assert bridge.current_command_position == 0
    assert len(fake_client.goals) == 1
    goal = fake_client.goals[0]
    assert goal.command.position == pytest.approx(0.0)
    assert goal.command.max_effort == pytest.approx(10.0)


def test_close_step_updates_position_and_sends_goal() -> None:
    """close_step advances the internal byte estimate and sends mapped action position."""
    fake_client = FakeActionClient()
    bridge = CommandBridge(
        FakeNode(),
        make_params(),
        action_client_factory=lambda node, action_type, action_name: fake_client,
    )

    assert bridge.close_step(10)
    assert bridge.current_command_position == 10
    goal = fake_client.goals[0]
    assert goal.command.position == pytest.approx(10 / 255 * 0.8)


def test_dry_run_does_not_send_goal() -> None:
    """dry_run still updates the estimate but does not create an action goal."""
    bridge = CommandBridge(FakeNode(), make_params(dry_run=True))

    assert bridge.close_step(10)
    assert bridge.current_command_position == 10
    assert bridge.action_client is None


def test_action_server_unavailable_is_diagnostic() -> None:
    """Unavailable action server returns False and records an error."""
    fake_client = FakeActionClient(available=False)
    node = FakeNode()
    bridge = CommandBridge(
        node,
        make_params(),
        action_client_factory=lambda node, action_type, action_name: fake_client,
    )

    assert not bridge.close_step(1)
    assert bridge.last_error == "gripper action server unavailable: /robotiq_gripper_controller/gripper_cmd"
    assert node.logger.errors == [bridge.last_error]
