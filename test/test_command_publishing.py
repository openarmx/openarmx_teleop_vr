import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "openarmx_teleop_vr"))

from openarmx_teleop_vr.command_publishing import publish_active_commands


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_left_only_publishes_left_command():
    left = FakePublisher()
    right = FakePublisher()

    publish_active_commands(left, right, "left", "right", True, False)

    assert left.messages == ["left"]
    assert right.messages == []


def test_right_only_publishes_right_command():
    left = FakePublisher()
    right = FakePublisher()

    publish_active_commands(left, right, "left", "right", False, True)

    assert left.messages == []
    assert right.messages == ["right"]


def test_both_active_publish_both_commands():
    left = FakePublisher()
    right = FakePublisher()

    publish_active_commands(left, right, "left", "right", True, True)

    assert left.messages == ["left"]
    assert right.messages == ["right"]


def test_no_active_hand_publishes_no_commands():
    left = FakePublisher()
    right = FakePublisher()

    publish_active_commands(left, right, "left", "right", False, False)

    assert left.messages == []
    assert right.messages == []
