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
@Desc    :   VR 遥操作 节点
'''

import os
import tempfile
import threading
import time

import numpy as np
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    from openarmx_arm_driver import (
        OpenArmTeleopController,
        RelativePose,
        TeleopConfig,
        TeleopInputFrame,
    )
except ImportError as exc:
    raise ImportError(
        "openarmx_arm_driver is required but not installed or not on PYTHONPATH. "
        "Install it "
        "before running teleop_node."
    ) from exc


class OpenArmTeleopNode(Node):
    """Teleoperation node for OpenArm robot using VR controllers."""

    def __init__(self):
        super().__init__("openarmx_teleop_vr_node")
        self.cb_group = ReentrantCallbackGroup()

        self._declare_parameters()
        urdf_path = self._resolve_urdf_path()

        control_rate = float(self.get_parameter("control_rate").value)
        self.print_performance = bool(self.get_parameter("print_performance").value)
        self.publish_viz_tf = bool(self.get_parameter("publish_visualization_tf").value)

        core_config = TeleopConfig(
            urdf_path=urdf_path,
            constraint_mode=str(self.get_parameter("constraint_mode").value),
            use_link4_ext=bool(self.get_parameter("use_link4_ext").value),
            grip_threshold=float(self.get_parameter("grip_threshold").value),
            slow_max_step_deg=float(self.get_parameter("slow_max_step_deg").value),
            fast_max_step_deg=float(self.get_parameter("fast_max_step_deg").value),
            ik_iterations=int(self.get_parameter("ik_iterations").value),
            print_performance=self.print_performance,
            enable_placo_viewer=bool(self.get_parameter("enable_placo_viewer").value),
        )

        self.controller = OpenArmTeleopController(
            config=core_config,
            log_info=lambda msg: self.get_logger().info(msg),
            log_warn=lambda msg: self.get_logger().warn(msg),
            log_error=lambda msg: self.get_logger().error(msg),
        )

        self.get_logger().info(f"Control rate: {control_rate} Hz")
        self.get_logger().info(f"Constraint mode: {core_config.constraint_mode}")

        self.rate_topic = str(self.get_parameter("rate_topic").value)

        self.left_pose = None
        self.right_pose = None
        self.pose_mutex = threading.Lock()
        self.control_lock = threading.Lock()

        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.left_grip = 0.0
        self.right_grip = 0.0
        self.is_full_speed = True

        self.tf_broadcaster = TransformBroadcaster(self)

        self.control_loop_times = []
        self.max_samples = 100
        self.performance_log_interval = 50
        self.loop_counter = 0

        self._setup_ros_io()
        self.timer = self.create_timer(
            1.0 / control_rate, self._control_loop, callback_group=self.cb_group
        )

        self.get_logger().info("OpenArm teleoperation node initialized successfully")

    def _declare_parameters(self):
        # URDF-related
        self.declare_parameter("robot_description", "")
        self.declare_parameter("use_xacro", True)

        # Teleop/core behavior
        self.declare_parameter("control_rate", 60.0)
        self.declare_parameter("use_link4_ext", True)
        self.declare_parameter("constraint_mode", "joint")
        self.declare_parameter("slow_max_step_deg", 3.0)
        self.declare_parameter("fast_max_step_deg", 0.0)
        self.declare_parameter("ik_iterations", 3)
        self.declare_parameter("grip_threshold", 0.5)

        # Namespace prefix
        self.declare_parameter("arm_prefix", "")

        # ROS topics
        self.declare_parameter("rate_topic", "/vr_right_controller/rate")
        self.declare_parameter("left_grip_topic", "/vr_left_controller/grip")
        self.declare_parameter("right_grip_topic", "/vr_right_controller/grip")

        # Diagnostics and visualization
        self.declare_parameter("print_performance", False)
        self.declare_parameter("publish_visualization_tf", False)
        self.declare_parameter("enable_placo_viewer", False)

    def _setup_ros_io(self):
        left_grip_topic = str(self.get_parameter("left_grip_topic").value)
        right_grip_topic = str(self.get_parameter("right_grip_topic").value)

        # Get arm_prefix parameter and build topic prefix
        arm_prefix = str(self.get_parameter("arm_prefix").value).strip()
        if arm_prefix:
            # Ensure prefix starts with / and doesn't end with /
            if not arm_prefix.startswith('/'):
                arm_prefix = '/' + arm_prefix
            if arm_prefix.endswith('/'):
                arm_prefix = arm_prefix[:-1]
            topic_prefix = arm_prefix
        else:
            topic_prefix = ''

        self.left_sub = self.create_subscription(
            PoseStamped,
            "/vr_left_controller/pose",
            self._left_pose_callback,
            5,
            callback_group=self.cb_group,
        )
        self.right_sub = self.create_subscription(
            PoseStamped,
            "/vr_right_controller/pose",
            self._right_pose_callback,
            5,
            callback_group=self.cb_group,
        )

        self.left_trigger_sub = self.create_subscription(
            Float32,
            "/vr_left_controller/trigger",
            self._left_trigger_callback,
            5,
            callback_group=self.cb_group,
        )
        self.right_trigger_sub = self.create_subscription(
            Float32,
            "/vr_right_controller/trigger",
            self._right_trigger_callback,
            5,
            callback_group=self.cb_group,
        )

        self.left_grip_sub = self.create_subscription(
            Float32,
            left_grip_topic,
            self._left_grip_callback,
            5,
            callback_group=self.cb_group,
        )
        self.right_grip_sub = self.create_subscription(
            Float32,
            right_grip_topic,
            self._right_grip_callback,
            5,
            callback_group=self.cb_group,
        )

        self.rate_sub = self.create_subscription(
            Float32,
            self.rate_topic,
            self._rate_callback,
            5,
            callback_group=self.cb_group,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            f"{topic_prefix}/joint_states" if topic_prefix else "/joint_states",
            self._joint_state_callback,
            5,
            callback_group=self.cb_group,
        )

        left_cmd_topic = f"{topic_prefix}/left_forward_position_controller/commands"
        right_cmd_topic = f"{topic_prefix}/right_forward_position_controller/commands"

        self.left_cmd_pub = self.create_publisher(Float64MultiArray, left_cmd_topic, 10)
        self.right_cmd_pub = self.create_publisher(Float64MultiArray, right_cmd_topic, 10)

        self.get_logger().info(
            f"Publishing commands to: left={left_cmd_topic}, right={right_cmd_topic}"
        )

    # ====================
    # ROS callbacks
    # ====================

    def _left_pose_callback(self, msg: PoseStamped):
        if not hasattr(self, "_left_callback_count"):
            self._left_callback_count = 0
            self.get_logger().info("Left controller connected")
        self._left_callback_count += 1
        with self.pose_mutex:
            self.left_pose = msg

    def _right_pose_callback(self, msg: PoseStamped):
        if not hasattr(self, "_right_callback_count"):
            self._right_callback_count = 0
            self.get_logger().info("Right controller connected")
        self._right_callback_count += 1
        with self.pose_mutex:
            self.right_pose = msg

    def _left_trigger_callback(self, msg: Float32):
        if not hasattr(self, "_left_trigger_callback_count"):
            self._left_trigger_callback_count = 0
            self.get_logger().info("Left trigger connected")
        self._left_trigger_callback_count += 1
        self.left_trigger = max(0.0, min(1.0, float(msg.data)))

    def _right_trigger_callback(self, msg: Float32):
        if not hasattr(self, "_right_trigger_callback_count"):
            self._right_trigger_callback_count = 0
            self.get_logger().info("Right trigger connected")
        self._right_trigger_callback_count += 1
        self.right_trigger = max(0.0, min(1.0, float(msg.data)))

    def _left_grip_callback(self, msg: Float32):
        if not hasattr(self, "_left_grip_callback_count"):
            self._left_grip_callback_count = 0
            self.get_logger().info(
                f"Left grip connected (topic: {self.get_parameter('left_grip_topic').value})"
            )
        self._left_grip_callback_count += 1
        self.left_grip = max(0.0, min(1.0, float(msg.data)))

    def _right_grip_callback(self, msg: Float32):
        if not hasattr(self, "_right_grip_callback_count"):
            self._right_grip_callback_count = 0
            self.get_logger().info(
                f"Right grip connected (topic: {self.get_parameter('right_grip_topic').value})"
            )
        self._right_grip_callback_count += 1
        self.right_grip = max(0.0, min(1.0, float(msg.data)))

    def _rate_callback(self, msg: Float32):
        rate_val = float(msg.data)
        new_is_full_speed = rate_val >= 0.999
        if new_is_full_speed != self.is_full_speed:
            mode = "FAST (no limit)" if new_is_full_speed else "SAFE (step limited)"
            self.get_logger().info(f"Speed mode changed to: {mode} (rate={rate_val:.2f})")
        self.is_full_speed = new_is_full_speed

    def _joint_state_callback(self, msg: JointState):
        self.controller.update_joint_states(msg.name, msg.position)
        if not hasattr(self, "_joint_state_once_logged"):
            self._joint_state_once_logged = True
            self.get_logger().info("Received joint_states - step limiting and sync enabled")

    # ====================
    # Main loop
    # ====================

    def _control_loop(self):
        loop_start_time = time.perf_counter()

        with self.pose_mutex:
            left_pose_msg = self.left_pose
            right_pose_msg = self.right_pose

        left_pose = self._to_relative_pose(left_pose_msg)
        right_pose = self._to_relative_pose(right_pose_msg)

        if not self.control_lock.acquire(blocking=False):
            return

        try:
            frame = TeleopInputFrame(
                left_pose=left_pose,
                right_pose=right_pose,
                left_grip=self.left_grip,
                right_grip=self.right_grip,
                left_trigger=self.left_trigger,
                right_trigger=self.right_trigger,
                is_full_speed=self.is_full_speed,
            )
            result = self.controller.step(frame)

            if result.waiting_for_poses:
                if not hasattr(self, "_waiting_logged"):
                    self._waiting_logged = True
                    self.get_logger().info(
                        "Waiting for controllers - Left: waiting, Right: waiting"
                    )
                return

            if hasattr(self, "_waiting_logged"):
                delattr(self, "_waiting_logged")

            if result.error:
                self.get_logger().error(f"Core returned error: {result.error}")
                return

            if result.should_publish_left:
                left_cmd = Float64MultiArray()
                left_cmd.data = result.left_command
                self.left_cmd_pub.publish(left_cmd)

            if result.should_publish_right:
                right_cmd = Float64MultiArray()
                right_cmd.data = result.right_command
                self.right_cmd_pub.publish(right_cmd)

            if self.publish_viz_tf:
                self._publish_visualization_tfs(result, left_pose_msg, right_pose_msg)

        except Exception as exc:
            self.get_logger().error(f"Control loop error: {exc}")
            import traceback

            self.get_logger().error(traceback.format_exc())
        finally:
            self.control_lock.release()
            self.loop_counter += 1

            total_loop_time = (time.perf_counter() - loop_start_time) * 1000.0
            self.control_loop_times.append(total_loop_time)
            if len(self.control_loop_times) > self.max_samples:
                self.control_loop_times.pop(0)

            if (
                self.print_performance
                and self.control_loop_times
                and self.loop_counter % self.performance_log_interval == 0
            ):
                avg_loop = sum(self.control_loop_times) / len(self.control_loop_times)
                max_loop = max(self.control_loop_times)
                min_loop = min(self.control_loop_times)
                self.get_logger().info(
                    f"Total Loop | Current: {total_loop_time:.2f}ms | "
                    f"Avg: {avg_loop:.2f}ms | Min: {min_loop:.2f}ms | Max: {max_loop:.2f}ms"
                )

    # ====================
    # ROS layer helpers
    # ====================

    def _to_relative_pose(self, pose_msg: PoseStamped):
        if pose_msg is None:
            return None
        return RelativePose(
            position=(
                float(pose_msg.pose.position.x),
                float(pose_msg.pose.position.y),
                float(pose_msg.pose.position.z),
            ),
            orientation_xyzw=(
                float(pose_msg.pose.orientation.x),
                float(pose_msg.pose.orientation.y),
                float(pose_msg.pose.orientation.z),
                float(pose_msg.pose.orientation.w),
            ),
        )

    def _publish_visualization_tfs(self, result, left_pose_msg, right_pose_msg):
        if result.left_target_transform is not None:
            stamp = (
                left_pose_msg.header.stamp
                if left_pose_msg is not None
                else self.get_clock().now().to_msg()
            )
            self._publish_transform("left_ee_target", result.left_target_transform, stamp)
        if result.right_target_transform is not None:
            stamp = (
                right_pose_msg.header.stamp
                if right_pose_msg is not None
                else self.get_clock().now().to_msg()
            )
            self._publish_transform("right_ee_target", result.right_target_transform, stamp)

        if result.left_current_transform is not None:
            stamp = (
                left_pose_msg.header.stamp
                if left_pose_msg is not None
                else self.get_clock().now().to_msg()
            )
            self._publish_transform("left_ee_current", result.left_current_transform, stamp)
        if result.right_current_transform is not None:
            stamp = (
                right_pose_msg.header.stamp
                if right_pose_msg is not None
                else self.get_clock().now().to_msg()
            )
            self._publish_transform("right_ee_current", result.right_current_transform, stamp)

        identity_rot = np.eye(3)
        if result.left_link4_target is not None:
            stamp = (
                left_pose_msg.header.stamp
                if left_pose_msg is not None
                else self.get_clock().now().to_msg()
            )
            self._publish_hand_tf("left_link4_target", result.left_link4_target, identity_rot, stamp)

        if result.right_link4_target is not None:
            stamp = (
                right_pose_msg.header.stamp
                if right_pose_msg is not None
                else self.get_clock().now().to_msg()
            )
            self._publish_hand_tf(
                "right_link4_target", result.right_link4_target, identity_rot, stamp
            )

    def _publish_transform(self, frame_id: str, T: np.ndarray, stamp):
        self._publish_hand_tf(frame_id, T[:3, 3], T[:3, :3], stamp)

    def _publish_hand_tf(self, frame_id, position, rotation, stamp):
        qx, qy, qz, qw = self._matrix_to_quaternion(rotation)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "world"
        t.child_frame_id = frame_id
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _resolve_urdf_path(self) -> str:
        robot_description = self.get_parameter("robot_description").value
        use_xacro = bool(self.get_parameter("use_xacro").value)

        if robot_description and len(robot_description.strip()) > 0:
            self.get_logger().info("Using robot_description from parameter")
            fd, temp_path = tempfile.mkstemp(suffix=".urdf", prefix="openarmx_teleop_")
            with os.fdopen(fd, "w") as file_obj:
                file_obj.write(robot_description)
            self._temp_urdf_path = temp_path
            return temp_path

        if use_xacro:
            self.get_logger().info("Generating URDF from xacro")
            try:
                description_pkg = get_package_share_directory("openarmx_description")
                xacro_file = os.path.join(description_pkg, "urdf", "robot", "v10.urdf.xacro")
                if not os.path.exists(xacro_file):
                    raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

                self.get_logger().info(f"Processing xacro: {xacro_file}")
                doc = xacro.process_file(xacro_file, mappings={"bimanual": "true"})
                urdf_content = doc.toprettyxml(indent="  ")

                fd, temp_path = tempfile.mkstemp(
                    suffix=".urdf", prefix="openarmx_teleop_xacro_"
                )
                with os.fdopen(fd, "w") as file_obj:
                    file_obj.write(urdf_content)
                self._temp_urdf_path = temp_path
                self.get_logger().info(f"Generated URDF written to: {temp_path}")
                return temp_path
            except Exception as exc:
                self.get_logger().error(f"Failed to process xacro: {exc}")
                self.get_logger().warn("Falling back to static URDF file")

        description_pkg = get_package_share_directory("openarmx_description")
        static_urdf = os.path.join(description_pkg, "urdf", "robot", "openarmx_robot.urdf")
        self.get_logger().info(f"Using static URDF: {static_urdf}")
        return static_urdf

    def _matrix_to_quaternion(self, rot_matrix):
        m00, m01, m02 = rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[0, 2]
        m10, m11, m12 = rot_matrix[1, 0], rot_matrix[1, 1], rot_matrix[1, 2]
        m20, m21, m22 = rot_matrix[2, 0], rot_matrix[2, 1], rot_matrix[2, 2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            s_val = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s_val
            qx = (m21 - m12) / s_val
            qy = (m02 - m20) / s_val
            qz = (m10 - m01) / s_val
        elif (m00 > m11) and (m00 > m22):
            s_val = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / s_val
            qx = 0.25 * s_val
            qy = (m01 + m10) / s_val
            qz = (m02 + m20) / s_val
        elif m11 > m22:
            s_val = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / s_val
            qx = (m01 + m10) / s_val
            qy = 0.25 * s_val
            qz = (m12 + m21) / s_val
        else:
            s_val = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / s_val
            qx = (m02 + m20) / s_val
            qy = (m12 + m21) / s_val
            qz = 0.25 * s_val

        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        norm = np.linalg.norm(q)
        if norm < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        q /= norm
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])

    def destroy_node(self):
        if hasattr(self, "_temp_urdf_path") and self._temp_urdf_path:
            try:
                if os.path.exists(self._temp_urdf_path):
                    os.remove(self._temp_urdf_path)
                    self.get_logger().info(
                        f"Cleaned up temp URDF: {self._temp_urdf_path}"
                    )
            except Exception as exc:
                self.get_logger().warn(f"Failed to clean up temp URDF: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenArmTeleopNode()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
