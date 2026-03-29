#!/usr/bin/env python3

# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# To view a copy of this license, visit:
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

'''
@File    :   GUI_MultiRobot.py
@Time    :   2026/02/15 18:43:53
@Author  :   Yan HaiYang 
@Version :   1.0
@Desc    :   VR 遥操作 节点启动文件
'''

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for VR teleoperation"""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_xacro',
            default_value='true',
            description='Use xacro to generate URDF dynamically (default: true)'
        ),
        DeclareLaunchArgument(
            'rate_topic',
            default_value='/vr_right_controller/rate',
            description='Topic providing controller rate (0.1 or 1.0) for speed mode'
        ),
        DeclareLaunchArgument(
            'slow_max_step_deg',
            default_value='1.0',
            description='Max joint step (degrees) per control cycle when in slow mode (rate=0.1)'
        ),
        DeclareLaunchArgument(
            'fast_max_step_deg',
            default_value='12.0',
            description='Optional max joint step (degrees) per control cycle when in fast mode (rate=1.0); 0=disabled'
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='100.0',
            description='Control loop rate in Hz (default: 100.0, higher = lower latency)'
        ),
        DeclareLaunchArgument(
            'ik_iterations',
            default_value='3',
            description='Number of IK solver iterations per control loop (default: 3, lower = faster but less accurate)'
        ),
        DeclareLaunchArgument(
            'publish_visualization_tf',
            default_value='true',
            description='Publish TF transforms for RViz visualization (default: false for better performance)'
        ),
        DeclareLaunchArgument(
            'print_performance',
            default_value='false',
            description='Print performance metrics to console (default: false)'
        ),
        DeclareLaunchArgument(
            'use_link4_ext',
            default_value='true',
            description='Use link4_ext frames for elbow IK tasks (default: true, requires _ext.urdf)'
        ),
        DeclareLaunchArgument(
            'constraint_mode',
            default_value='joint',
            choices=['joint', 'link'],
            description='Constraint mode: "joint"  or "link" (link4 position) (default: link)'
        ),
        
        DeclareLaunchArgument(
            'grip_threshold',
            default_value='0.5',
            description='Grip value threshold: > enables teleop, <= disables (default: 0.5)'
        ),
        DeclareLaunchArgument(
            'left_grip_topic',
            default_value='/vr_left_controller/grip',
            description='Topic for left controller grip value (deadman switch)'
        ),
        DeclareLaunchArgument(
            'right_grip_topic',
            default_value='/vr_right_controller/grip',
            description='Topic for right controller grip value (deadman switch)'
        ),
        DeclareLaunchArgument(
            'arm_prefix',
            default_value='',
            description='Prefix for the arm for topic namespacing (e.g., robot1, robot2)'
        ),
    ]

    # Initialize launch configurations
    use_xacro = LaunchConfiguration('use_xacro')
    rate_topic = LaunchConfiguration('rate_topic')
    slow_max_step_deg = LaunchConfiguration('slow_max_step_deg')
    fast_max_step_deg = LaunchConfiguration('fast_max_step_deg')
    control_rate = LaunchConfiguration('control_rate')
    ik_iterations = LaunchConfiguration('ik_iterations')
    publish_visualization_tf = LaunchConfiguration('publish_visualization_tf')
    print_performance = LaunchConfiguration('print_performance')
    use_link4_ext = LaunchConfiguration('use_link4_ext')
    constraint_mode = LaunchConfiguration('constraint_mode')
    grip_threshold = LaunchConfiguration('grip_threshold')
    left_grip_topic = LaunchConfiguration('left_grip_topic')
    right_grip_topic = LaunchConfiguration('right_grip_topic')
    arm_prefix = LaunchConfiguration('arm_prefix')

    # Teleoperation node
    teleop_node = Node(
        package='openarmx_teleop_vr',
        executable='openarmx_teleop_vr_node',
        name='openarmx_teleop_vr_node',
        output='screen',
        parameters=[{
            'use_xacro': use_xacro,
            'rate_topic': rate_topic,
            'slow_max_step_deg': slow_max_step_deg,
            'fast_max_step_deg': fast_max_step_deg,
            'control_rate': control_rate,
            'ik_iterations': ik_iterations,
            'publish_visualization_tf': publish_visualization_tf,
            'print_performance': print_performance,
            'use_link4_ext': use_link4_ext,
            'constraint_mode': constraint_mode,
            'grip_threshold': grip_threshold,
            'left_grip_topic': left_grip_topic,
            'right_grip_topic': right_grip_topic,
            'arm_prefix': arm_prefix,
        }]
    )

    return LaunchDescription(
        declared_arguments + [
            teleop_node,
        ]
    )
