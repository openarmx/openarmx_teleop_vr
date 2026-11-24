import importlib.util
import os
import sys
from pathlib import Path

import pytest


LAUNCH_PATH = Path(__file__).parents[1] / "openarmx_teleop_vr" / "launch" / "teleop_vr.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("teleop_vr_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeContext:
    def __init__(self, values):
        self.values = values

    def perform_substitution(self, substitution):
        return self.values[substitution]


def test_gripper_uses_upstream_description_without_importing_hand_package(monkeypatch):
    module = load_launch_module()
    upstream = "/robot/share/openarmx_description"

    monkeypatch.setattr(module, "get_package_share_directory", lambda name: upstream)

    def reject_hand_import(name, *args, **kwargs):
        if name == "openarmx_hand_description":
            raise AssertionError("gripper mode must not import openarmx_hand_description")
        return __import__(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", reject_hand_import)
    path = module._resolve_urdf_path(
        FakeContext({"urdf_path": "", "controller_pose_mode": "gripper", "robot_namespace": ""}),
        "urdf_path",
        "controller_pose_mode",
        "robot_namespace",
    )

    assert path == os.path.join(upstream, "urdf", "robot", "openarmx_robot.urdf")


def test_explicit_urdf_path_is_used_for_both_modes():
    module = load_launch_module()
    explicit = "/tmp/custom.urdf"
    context = FakeContext(
        {"urdf_path": explicit, "controller_pose_mode": "hand", "robot_namespace": ""}
    )

    assert module._resolve_urdf_path(
        context, "urdf_path", "controller_pose_mode", "robot_namespace"
    ) == explicit


def test_hand_mode_uses_hand_description_builder(monkeypatch):
    module = load_launch_module()
    calls = []

    class HandDescription:
        @staticmethod
        def build_robot_description(**kwargs):
            calls.append(kwargs)
            return "<robot name='o6'/>"

    monkeypatch.setitem(sys.modules, "openarmx_hand_description", HandDescription)
    context = FakeContext({"urdf_path": "", "controller_pose_mode": "hand", "robot_namespace": ""})

    path = module._resolve_urdf_path(
        context, "urdf_path", "controller_pose_mode", "robot_namespace"
    )

    assert Path(path).read_text(encoding="utf-8") == "<robot name='o6'/>"
    assert calls == [{"use_fake_hardware": "true", "node_namespace": ""}]
    os.unlink(path)


def test_hand_mode_reports_missing_optional_dependency(monkeypatch):
    module = load_launch_module()
    monkeypatch.delitem(sys.modules, "openarmx_hand_description", raising=False)

    real_import = __import__

    def reject_hand_import(name, *args, **kwargs):
        if name == "openarmx_hand_description":
            raise ModuleNotFoundError("No module named 'openarmx_hand_description'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", reject_hand_import)
    context = FakeContext({"urdf_path": "", "controller_pose_mode": "hand", "robot_namespace": ""})

    with pytest.raises(RuntimeError, match="controller_pose_mode=hand requires openarmx_hand_description"):
        module._resolve_urdf_path(
            context, "urdf_path", "controller_pose_mode", "robot_namespace"
        )
