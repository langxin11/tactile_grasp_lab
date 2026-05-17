"""Unit tests for action-space to Robotiq-byte mapping helpers."""

import pytest

from robotiq_2f85_driver.action_utils import (
    map_action_position_to_command,
    map_effort_to_force_byte,
)


def test_map_action_position_to_command() -> None:
    """Map open/closed action positions to the expected Robotiq byte values."""
    assert map_action_position_to_command(0.0, 0.8) == 0
    assert map_action_position_to_command(0.8, 0.8) == 255
    assert map_action_position_to_command(0.4, 0.8) == 128


def test_map_action_position_clamps_out_of_range_values() -> None:
    """Clamp action positions before converting them to byte commands."""
    assert map_action_position_to_command(-1.0, 0.8) == 0
    assert map_action_position_to_command(9.9, 0.8) == 255


def test_map_effort_to_force_byte() -> None:
    """Map physical effort values onto the Robotiq 0-255 force byte range."""
    assert map_effort_to_force_byte(0.0, 235.0, 10) == 10
    assert map_effort_to_force_byte(117.5, 235.0, 10) == 128
    assert map_effort_to_force_byte(235.0, 235.0, 10) == 255


def test_map_effort_rejects_invalid_force_range() -> None:
    """Require a positive physical force range when mapping max_effort."""
    with pytest.raises(ValueError):
        map_effort_to_force_byte(1.0, 0.0, 10)
