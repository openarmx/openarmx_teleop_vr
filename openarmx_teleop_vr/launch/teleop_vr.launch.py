#!/usr/bin/env python3
"""Launch OpenArmX VR teleoperation with the composable O6 robot model."""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from openarmx_hand_description import build_robot_description


_temporary_urdfs = set()


def _launch_node(context, config_file, urdf_path, controller_pose_mode, robot_namespace):
    path = context.perform_substitution(urdf_path).strip()
    namespace = context.perform_substitution(robot_namespace).strip().strip("/") or None
    if not path:
        description = build_robot_description(use_fake_hardware="true")
        fd, path = tempfile.mkstemp(
            prefix=f"openarmx_vr_o6_{namespace or 'default'}_",
            suffix=".urdf",
        )
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(description)
        _temporary_urdfs.add(path)

    return [Node(
        package="openarmx_teleop_vr",
        executable="openarmx_teleop_vr_node",
        name="openarmx_teleop_vr_node",
        namespace=namespace,
        output="screen",
        parameters=[
            context.perform_substitution(config_file),
            {
                "urdf_path": path,
                "controller_pose_mode": context.perform_substitution(controller_pose_mode),
            },
        ],
    )]


def _cleanup(_context):
    for path in list(_temporary_urdfs):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        _temporary_urdfs.discard(path)
    return []


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    urdf_path = LaunchConfiguration("urdf_path")
    controller_pose_mode = LaunchConfiguration("controller_pose_mode")
    robot_namespace = LaunchConfiguration("robot_namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("openarmx_teleop_vr"),
                "config",
                "teleop_params.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value="",
            description="Optional URDF path; empty uses openarmx_hand_description.",
        ),
        DeclareLaunchArgument(
            "controller_pose_mode",
            default_value="gripper",
            choices=["gripper", "hand"],
        ),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        OpaqueFunction(
            function=_launch_node,
            args=[config_file, urdf_path, controller_pose_mode, robot_namespace],
        ),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
    ])
