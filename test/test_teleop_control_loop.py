"""Exercise real adapter methods without ROS, hardware or the closed IK binary."""
import ast
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "openarmx_teleop_vr"))
from openarmx_teleop_vr.command_publishing import publish_active_commands


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(list(msg.data))


@pytest.fixture
def node():
    # Load the unchanged method bodies; stub only unavailable external services.
    source = Path(__file__).parents[1] / "openarmx_teleop_vr/openarmx_teleop_vr/openarmx_teleop_vr_node.py"
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = {"_control_loop", "_limit_joint_step", "_publish_joint_commands"}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    cls.bases = []
    env = {"np": np, "Float64MultiArray": SimpleNamespace, "publish_active_commands": publish_active_commands}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), "exec"), env)
    obj = env[cls.name]()
    obj.control_lock = threading.Lock()
    obj._process_button_events = lambda: None
    obj.button_motion_active = False
    obj._current_relative_input = lambda: None
    obj._current_absolute_input = lambda: None
    obj.current_mode = "relative"
    obj.relative_is_full_speed = False  # emulate the old APK's 10% state
    obj.absolute_is_full_speed = False
    obj._get_current_joint_positions = lambda: np.zeros(14)
    obj.fast_max_step_rad_per_joint = np.full(14, 0.2)
    obj.max_step_rad_per_joint = np.full(14, 0.02)
    obj.step_limit_enable_threshold_rad_per_joint = np.zeros(14)
    obj.relative_left_trigger = obj.absolute_left_trigger = 0.25
    obj.relative_right_trigger = obj.absolute_right_trigger = 0.75
    obj._map_trigger_to_gripper = lambda x: x * 0.04
    obj._get_current_gripper_command = lambda side: 0.0
    obj.left_cmd_pub, obj.right_cmd_pub = Publisher(), Publisher()
    obj.publish_gripper_command = True
    obj.result = SimpleNamespace(
        active_mode="relative", waiting_for_joint_states=False,
        calibration_required=False, target_q=np.ones(14),
        left_active=True, right_active=True,
    )
    obj.core = SimpleNamespace(step=lambda *args: obj.result)
    return obj


@pytest.mark.parametrize("mode", ["relative", "absolute"])
@pytest.mark.parametrize("left,right", [(True, False), (False, True), (True, True), (False, False)])
@pytest.mark.parametrize("gripper", [True, False])
def test_fixed_full_rate_and_independent_outputs(node, mode, left, right, gripper):
    node.current_mode = node.result.active_mode = mode
    node.result.left_active, node.result.right_active = left, right
    node.publish_gripper_command = gripper
    node._control_loop()
    expected_left = [0.2] * 7 + ([0.01] if gripper else [])
    expected_right = [0.2] * 7 + ([0.03] if gripper else [])
    assert node.left_cmd_pub.messages == ([expected_left] if left else [])
    assert node.right_cmd_pub.messages == ([expected_right] if right else [])
    assert not node.control_lock.locked()


@pytest.mark.parametrize("reason", ["waiting_for_joint_states", "calibration_required", "target_q"])
def test_not_ready_produces_no_commands(node, reason):
    setattr(node.result, reason, None if reason == "target_q" else True)
    node._control_loop()
    assert node.left_cmd_pub.messages == node.right_cmd_pub.messages == []


def test_explicit_button_step_limit_still_applies(node):
    result = node._limit_joint_step(np.ones(14), np.zeros(14), False, 0.0, 0.01)
    np.testing.assert_allclose(result, np.full(14, 0.01))
