#!/usr/bin/env python3
"""Launch file for OpenArmX VR teleoperation with Pinocchio IK."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('openarmx_teleop_vr'),
            'config',
            'teleop_params.yaml'
        ]),
        description='Path to configuration file'
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('openarmx_description'),
            'urdf',
            'robot',
            'openarmx_robot.urdf'
        ]),
        description='Path to robot URDF file'
    )

    # Teleoperation node
    teleop_node = Node(
        package='openarmx_teleop_vr',
        executable='openarmx_teleop_vr_node',
        name='openarmx_teleop_vr_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'urdf_path': LaunchConfiguration('urdf_path')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        urdf_path_arg,
        teleop_node
    ])
