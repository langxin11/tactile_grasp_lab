"""Unit tests for gripper command bridging."""

import pytest
from tactile_grasp_controller.command_bridge import (
    CommandBridge,
    compute_suggested_speed,
    map_command_to_action_position,
)


class FakeLogger:
    """Collect error logs without requiring a real ROS node."""

    def __init__(self) -> None:
        self.errors: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)


class FakeClock:
    """Provide a deterministic ROS-like clock surface."""

    def now(self) -> "FakeClock":
        return self

    def to_msg(self) -> object:
        return object()


class FakeNode:
    """Minimal node surface used by CommandBridge tests."""

    def __init__(self) -> None:
        self.logger = FakeLogger()
        self.clock = FakeClock()

    def get_logger(self) -> FakeLogger:
        return self.logger

    def get_clock(self) -> FakeClock:
        return self.clock


class FakeActionClient:
    """Capture GripperCommand goals sent by CommandBridge."""

    def __init__(self, available: bool = True) -> None:
        self.available = available
        self.goals: list[object] = []
        self.timeout_sec: float | None = None

    def wait_for_server(self, timeout_sec: float) -> bool:
        self.timeout_sec = timeout_sec
        return self.available

    def send_goal_async(self, goal: object) -> object:
        self.goals.append(goal)
        return object()


class FakeStreamingPublisher:
    """Capture streaming messages published by CommandBridge."""

    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FakeStreamingMessage:
    """Simple mutable object that mimics a ROS message."""

    def __init__(self) -> None:
        self.stamp = None
        self.command_id = 0
        self.target_position = 0
        self.speed = 0
        self.max_effort = 0.0


def make_params(dry_run: bool = False) -> dict[str, object]:
    """Return a complete parameter dictionary for CommandBridge tests."""
    return {
        "dry_run": dry_run,
        "min_position": 0,
        "max_position": 255,
        "initial_position": 0,
        "control_rate_hz": 40.0,
        "target_command_rate_bytes_per_s": 80.0,
        "min_speed_byte": 8,
        "max_speed_byte": 32,
        "streaming_command_topic": "/robotiq/command/stream",
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
    assert map_command_to_action_position(127, 0, 255, 0.0, 0.8) == pytest.approx(127 / 255 * 0.8)


def test_compute_suggested_speed_scales_with_rate_and_step() -> None:
    """Suggested speed should rise with command-byte rate and respect configured bounds."""
    assert compute_suggested_speed(1, 40.0, 80.0, 8, 32) == 20
    assert compute_suggested_speed(2, 40.0, 80.0, 8, 32) == 32


def test_open_publishes_streaming_command_when_available() -> None:
    """Opening prefers the streaming path and uses the configured minimum speed."""
    publisher = FakeStreamingPublisher()
    bridge = CommandBridge(
        FakeNode(),
        make_params(),
        action_client_factory=lambda node, action_type, action_name: FakeActionClient(),
        streaming_publisher_factory=lambda msg_type, topic, qos: publisher,
        streaming_message_factory=FakeStreamingMessage,
    )

    assert bridge.open()
    assert bridge.current_command_position == 0
    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.target_position == 0
    assert message.speed == 8
    assert message.max_effort == pytest.approx(10.0)


def test_close_step_publishes_streaming_command_with_suggested_speed() -> None:
    """close_step should stream the updated target and derived speed byte."""
    publisher = FakeStreamingPublisher()
    bridge = CommandBridge(
        FakeNode(),
        make_params(),
        action_client_factory=lambda node, action_type, action_name: FakeActionClient(),
        streaming_publisher_factory=lambda msg_type, topic, qos: publisher,
        streaming_message_factory=FakeStreamingMessage,
    )

    assert bridge.close_step(1)
    assert bridge.current_command_position == 1
    message = publisher.messages[0]
    assert message.target_position == 1
    assert message.speed == 20


def test_open_step_publishes_streaming_command_with_suggested_speed() -> None:
    """open_step should stream the updated target in the opening direction."""
    publisher = FakeStreamingPublisher()
    bridge = CommandBridge(
        FakeNode(),
        make_params(),
        action_client_factory=lambda node, action_type, action_name: FakeActionClient(),
        streaming_publisher_factory=lambda msg_type, topic, qos: publisher,
        streaming_message_factory=FakeStreamingMessage,
    )
    bridge.sync_echo(10)

    assert bridge.open_step(1)
    assert bridge.current_command_position == 9
    message = publisher.messages[0]
    assert message.target_position == 9
    assert message.speed == 20


def test_dry_run_does_not_send_goal_or_stream() -> None:
    """dry_run still updates the estimate but does not create outbound commands."""
    bridge = CommandBridge(FakeNode(), make_params(dry_run=True))

    assert bridge.close_step(10)
    assert bridge.current_command_position == 10
    assert bridge.action_client is None
    assert bridge.streaming_publisher is None


def test_action_server_unavailable_is_diagnostic_when_streaming_unavailable() -> None:
    """Unavailable action server returns False and records an error."""
    fake_client = FakeActionClient(available=False)
    node = FakeNode()
    bridge = CommandBridge(
        node,
        make_params(),
        action_client_factory=lambda node, action_type, action_name: fake_client,
        streaming_publisher_factory=lambda msg_type, topic, qos: None,
    )

    assert not bridge.close_step(1)
    assert (
        bridge.last_error
        == "gripper action server unavailable: /robotiq_gripper_controller/gripper_cmd"
    )
    assert node.logger.errors == [bridge.last_error]
