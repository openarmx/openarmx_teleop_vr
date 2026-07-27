#!/usr/bin/env python3
"""ROS2 adapter layer for the OpenArmX Pinocchio teleop core."""

import ast
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from openarmx_arm_driver import TeleopConfig
from openarmx_arm_driver._lib.teleop_core import (
    AbsoluteTeleopInputFrame,
    PinocchioTeleopCore,
    PoseInput,
    RelativeTeleopInputFrame,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String


class OpenArmXTeleopVRNode(Node):
    """ROS node that wraps the closed teleop core."""

    def __init__(self):
        super().__init__("openarmx_teleop_vr_node")

        self.cb_group = ReentrantCallbackGroup()
        self._declare_parameters()

        urdf_path = self.get_parameter("urdf_path").value
        if not urdf_path:
            self.get_logger().error("URDF path not provided")
            raise ValueError("urdf_path parameter is required")

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.grip_threshold = float(self.get_parameter("grip_threshold").value)
        max_step_joint1 = float(self.get_parameter("max_step_deg_joint1").value)
        max_step_joint2 = float(self.get_parameter("max_step_deg_joint2").value)
        max_step_joint3 = float(self.get_parameter("max_step_deg_joint3").value)
        max_step_joint4 = float(self.get_parameter("max_step_deg_joint4").value)
        max_step_joint5 = float(self.get_parameter("max_step_deg_joint5").value)
        max_step_joint6 = float(self.get_parameter("max_step_deg_joint6").value)
        max_step_joint7 = float(self.get_parameter("max_step_deg_joint7").value)

        fast_max_step_joint1 = float(self.get_parameter("fast_max_step_deg_joint1").value)
        fast_max_step_joint2 = float(self.get_parameter("fast_max_step_deg_joint2").value)
        fast_max_step_joint3 = float(self.get_parameter("fast_max_step_deg_joint3").value)
        fast_max_step_joint4 = float(self.get_parameter("fast_max_step_deg_joint4").value)
        fast_max_step_joint5 = float(self.get_parameter("fast_max_step_deg_joint5").value)
        fast_max_step_joint6 = float(self.get_parameter("fast_max_step_deg_joint6").value)
        fast_max_step_joint7 = float(self.get_parameter("fast_max_step_deg_joint7").value)

        threshold_joint1 = float(self.get_parameter("step_limit_enable_threshold_deg_joint1").value)
        threshold_joint2 = float(self.get_parameter("step_limit_enable_threshold_deg_joint2").value)
        threshold_joint3 = float(self.get_parameter("step_limit_enable_threshold_deg_joint3").value)
        threshold_joint4 = float(self.get_parameter("step_limit_enable_threshold_deg_joint4").value)
        threshold_joint5 = float(self.get_parameter("step_limit_enable_threshold_deg_joint5").value)
        threshold_joint6 = float(self.get_parameter("step_limit_enable_threshold_deg_joint6").value)
        threshold_joint7 = float(self.get_parameter("step_limit_enable_threshold_deg_joint7").value)

        base_max_step_rad = np.array([
            np.deg2rad(max_step_joint1),
            np.deg2rad(max_step_joint2),
            np.deg2rad(max_step_joint3),
            np.deg2rad(max_step_joint4),
            np.deg2rad(max_step_joint5),
            np.deg2rad(max_step_joint6),
            np.deg2rad(max_step_joint7),
        ], dtype=np.float64)
        self.max_step_rad_per_joint = np.concatenate([base_max_step_rad, base_max_step_rad])

        base_fast_max_step_rad = np.array([
            np.deg2rad(fast_max_step_joint1),
            np.deg2rad(fast_max_step_joint2),
            np.deg2rad(fast_max_step_joint3),
            np.deg2rad(fast_max_step_joint4),
            np.deg2rad(fast_max_step_joint5),
            np.deg2rad(fast_max_step_joint6),
            np.deg2rad(fast_max_step_joint7),
        ], dtype=np.float64)
        self.fast_max_step_rad_per_joint = np.concatenate([base_fast_max_step_rad, base_fast_max_step_rad])

        base_threshold_rad = np.array([
            np.deg2rad(threshold_joint1),
            np.deg2rad(threshold_joint2),
            np.deg2rad(threshold_joint3),
            np.deg2rad(threshold_joint4),
            np.deg2rad(threshold_joint5),
            np.deg2rad(threshold_joint6),
            np.deg2rad(threshold_joint7),
        ], dtype=np.float64)
        self.step_limit_enable_threshold_rad_per_joint = np.concatenate(
            [base_threshold_rad, base_threshold_rad]
        )
        self.button_motion_step_deg = float(self.get_parameter("button_motion_step_deg").value)
        self.button_motion_step_rad = np.deg2rad(self.button_motion_step_deg)
        self.button_motion_done_tolerance_deg = float(
            self.get_parameter("button_motion_done_tolerance_deg").value
        )
        self.button_motion_done_tolerance_rad = np.deg2rad(
            self.button_motion_done_tolerance_deg
        )

        self.pose_mutex = threading.Lock()
        self.control_lock = threading.Lock()
        self.button_lock = threading.Lock()

        self.joint_positions = {}
        self.joint_states_received = False

        self.current_mode = None
        self.relative_is_full_speed = False
        self.absolute_is_full_speed = False
        self.ik_enable_override = False
        self.relative_stream_timeout_sec = 0.3
        self._init_pose_state()
        self._init_button_state()

        controller_pose_mode = str(self.get_parameter("controller_pose_mode").value).strip().lower()
        if controller_pose_mode not in ("gripper", "hand"):
            self.get_logger().warn(
                f"Invalid controller_pose_mode '{controller_pose_mode}', falling back to 'gripper'"
            )
            controller_pose_mode = "gripper"
        self.controller_pose_mode = controller_pose_mode
        self.publish_gripper_command = controller_pose_mode == "gripper"
        relative_matrix_prefix = controller_pose_mode
        self.relative_left_axis_matrix = self._matrix_from_parameter(f"{relative_matrix_prefix}_left_axis_matrix")
        self.relative_right_axis_matrix = self._matrix_from_parameter(f"{relative_matrix_prefix}_right_axis_matrix")
        self.relative_left_orientation_matrix = self._matrix_from_parameter(f"{relative_matrix_prefix}_left_orientation_matrix")
        self.relative_right_orientation_matrix = self._matrix_from_parameter(f"{relative_matrix_prefix}_right_orientation_matrix")
        absolute_left_axis_matrix = self._matrix_from_parameter("absolute_left_axis_matrix")
        absolute_right_axis_matrix = self._matrix_from_parameter("absolute_right_axis_matrix")
        absolute_left_orientation_matrix = self._matrix_from_parameter("gripper_left_orientation_matrix")
        absolute_right_orientation_matrix = self._matrix_from_parameter("gripper_right_orientation_matrix")
        self.get_logger().info(f"Controller pose mode: {controller_pose_mode}")
        self.get_logger().info(f"Relative left axis matrix: {self._format_matrix(self.relative_left_axis_matrix)}")
        self.get_logger().info(f"Relative right axis matrix: {self._format_matrix(self.relative_right_axis_matrix)}")
        self.get_logger().info(f"Relative left orientation matrix: {self._format_matrix(self.relative_left_orientation_matrix)}")
        self.get_logger().info(f"Relative right orientation matrix: {self._format_matrix(self.relative_right_orientation_matrix)}")
        self.get_logger().info("Absolute mode uses gripper matrices")
        self.get_logger().info(
            "Command output: 7 arm joints + gripper"
            if self.publish_gripper_command
            else "Command output: 7 arm joints only"
        )

        core_config = TeleopConfig(
            urdf_path=urdf_path,
            grip_threshold=self.grip_threshold,
            body_anchor_offset=np.asarray(
                self.get_parameter("body_anchor_offset").value, dtype=np.float64
            ),
            position_scale_xyz=np.asarray(
                self.get_parameter("position_scale_xyz").value, dtype=np.float64
            ),
            left_axis_matrix=absolute_left_axis_matrix,
            right_axis_matrix=absolute_right_axis_matrix,
            left_orientation_matrix=absolute_left_orientation_matrix,
            right_orientation_matrix=absolute_right_orientation_matrix,
            stream_timeout_sec=0.3,
        )
        self.core = PinocchioTeleopCore(core_config)
        self.get_logger().info("Pinocchio teleop core initialized successfully")

        self._setup_ros_io()
        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self._control_loop,
            callback_group=self.cb_group,
        )
        self.get_logger().info(f"Node initialized - control rate: {self.control_rate} Hz")

    def _declare_parameters(self):
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("grip_threshold", 0.5)
        self.declare_parameter("max_step_deg_joint1", 4.0)
        self.declare_parameter("max_step_deg_joint2", 4.0)
        self.declare_parameter("max_step_deg_joint3", 4.0)
        self.declare_parameter("max_step_deg_joint4", 4.0)
        self.declare_parameter("max_step_deg_joint5", 4.0)
        self.declare_parameter("max_step_deg_joint6", 4.0)
        self.declare_parameter("max_step_deg_joint7", 4.0)

        # 慢速参数，建议不要轻易修改
        # self.declare_parameter("fast_max_step_deg_joint1", 10.0)
        # self.declare_parameter("fast_max_step_deg_joint2", 10.0)
        # self.declare_parameter("fast_max_step_deg_joint3", 10.0)
        # self.declare_parameter("fast_max_step_deg_joint4", 10.0)
        # self.declare_parameter("fast_max_step_deg_joint5", 10.0)
        # self.declare_parameter("fast_max_step_deg_joint6", 10.0)
        # self.declare_parameter("fast_max_step_deg_joint7", 10.0)

        # 快速参数，建议不要轻易修改
        self.declare_parameter("fast_max_step_deg_joint1", 20.0)
        self.declare_parameter("fast_max_step_deg_joint2", 16.0)
        self.declare_parameter("fast_max_step_deg_joint3", 12.0)
        self.declare_parameter("fast_max_step_deg_joint4", 12.0)
        self.declare_parameter("fast_max_step_deg_joint5", 14.0)
        self.declare_parameter("fast_max_step_deg_joint6", 8.0)
        self.declare_parameter("fast_max_step_deg_joint7", 12.0)

        # 慢速参数，建议不要轻易修改
        # self.declare_parameter("step_limit_enable_threshold_deg_joint1", 4.0)
        # self.declare_parameter("step_limit_enable_threshold_deg_joint2", 4.0)
        # self.declare_parameter("step_limit_enable_threshold_deg_joint3", 4.0)
        # self.declare_parameter("step_limit_enable_threshold_deg_joint4", 5.0)
        # self.declare_parameter("step_limit_enable_threshold_deg_joint5", 9.0)
        # self.declare_parameter("step_limit_enable_threshold_deg_joint6", 4.0)
        # self.declare_parameter("step_limit_enable_threshold_deg_joint7", 5.0)
        
        # 快速参数，建议不要轻易修改
        self.declare_parameter("step_limit_enable_threshold_deg_joint1", 20.0)
        self.declare_parameter("step_limit_enable_threshold_deg_joint2", 16.0)
        self.declare_parameter("step_limit_enable_threshold_deg_joint3", 12.0)
        self.declare_parameter("step_limit_enable_threshold_deg_joint4", 12.0)
        self.declare_parameter("step_limit_enable_threshold_deg_joint5", 14.0)
        self.declare_parameter("step_limit_enable_threshold_deg_joint6", 8.0)
        self.declare_parameter("step_limit_enable_threshold_deg_joint7", 12.0)

        self.declare_parameter("button_motion_step_deg", 0.5)
        self.declare_parameter("button_motion_done_tolerance_deg", 1.0)

        self.declare_parameter("vr_head_topic", "vr/head/pose")
        self.declare_parameter("vr_left_pose_topic", "vr/left/pose_absolute")
        self.declare_parameter("vr_right_pose_topic", "vr/right/pose_absolute")
        self.declare_parameter("vr_left_grip_topic", "vr/left/grip")
        self.declare_parameter("vr_right_grip_topic", "vr/right/grip")
        self.declare_parameter("vr_left_trigger_topic", "vr/left/trigger")
        self.declare_parameter("vr_right_trigger_topic", "vr/right/trigger")
        self.declare_parameter("vr_calibrate_topic", "vr/calibrate_done")
        self.declare_parameter("vr_control_mode_topic", "vr/control_mode")
        self.declare_parameter("vr_rate_topic", "vr/rate")
        self.declare_parameter("button_b_topic", "pico_right_controller/button_b")
        self.declare_parameter("button_y_topic", "pico_left_controller/button_y")
        self.declare_parameter("absolute_button_b_topic", "vr/right/button_b")
        self.declare_parameter("absolute_button_x_topic", "vr/left/button_x")
        self.declare_parameter("absolute_button_y_topic", "vr/left/button_y")
        self.declare_parameter("ik_enable_override_topic", "openarmx_teleop_vr/ik_enable_override")

        self.declare_parameter("left_cmd_topic", "left_forward_position_controller/commands")
        self.declare_parameter("right_cmd_topic", "right_forward_position_controller/commands")

        self.declare_parameter("body_anchor_offset", [0.0, -0.18, 0.08])
        self.declare_parameter("position_scale_xyz", [1.0, 1.0, 0.9])
        self.declare_parameter("controller_pose_mode", "gripper")
        gripper_axis_matrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        absolute_axis_matrix = [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        hand_axis_matrix = [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]
        identity_orientation_matrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        hand_orientation_matrix = hand_axis_matrix
        self.declare_parameter(
            "gripper_left_axis_matrix",
            gripper_axis_matrix,
        )
        self.declare_parameter(
            "gripper_right_axis_matrix",
            gripper_axis_matrix,
        )
        self.declare_parameter(
            "absolute_left_axis_matrix",
            absolute_axis_matrix,
        )
        self.declare_parameter(
            "absolute_right_axis_matrix",
            absolute_axis_matrix,
        )
        self.declare_parameter(
            "gripper_left_orientation_matrix",
            identity_orientation_matrix,
        )
        self.declare_parameter(
            "gripper_right_orientation_matrix",
            identity_orientation_matrix,
        )
        self.declare_parameter(
            "hand_left_axis_matrix",
            hand_axis_matrix,
        )
        self.declare_parameter(
            "hand_right_axis_matrix",
            hand_axis_matrix,
        )
        self.declare_parameter(
            "hand_left_orientation_matrix",
            hand_orientation_matrix,
        )
        self.declare_parameter(
            "hand_right_orientation_matrix",
            hand_orientation_matrix,
        )

    def _init_pose_state(self):
        self.absolute_head_pose = None
        self.absolute_left_pose = None
        self.absolute_right_pose = None
        self.relative_left_raw_pose = None
        self.relative_right_raw_pose = None
        self.relative_left_pose = None
        self.relative_right_pose = None
        self.relative_left_reference_pose = None
        self.relative_right_reference_pose = None
        self.relative_left_enabled = False
        self.relative_right_enabled = False
        self.absolute_left_grip = 0.0
        self.absolute_right_grip = 0.0
        self.absolute_left_trigger = 0.0
        self.absolute_right_trigger = 0.0
        self.relative_left_grip = 0.0
        self.relative_right_grip = 0.0
        self.relative_left_trigger = 0.0
        self.relative_right_trigger = 0.0

    def _init_button_state(self):
        self.relative_button_b_pressed = False
        self.relative_button_y_pressed = False
        self.absolute_button_b_pressed = False
        self.absolute_button_x_pressed = False
        self.absolute_button_y_pressed = False
        self.pending_go_home = False
        self.pending_hands_up = False
        self.pending_absolute_capture = False
        self.button_motion_active = False
        self.button_motion_name = None
        self.button_motion_kind = None
        self.button_desired_target_q = None
        self.button_command_target_q = None
        self.button_target_left_transform = None
        self.button_target_right_transform = None
        self.button_target_reached_count = 0
        self.button_target_stable_cycles_required = 5
        self.relative_manual_override_position_threshold_m = 0.01
        self.relative_manual_override_rotation_threshold_rad = np.deg2rad(5.0)

    def _matrix_from_parameter(self, name: str) -> np.ndarray:
        raw_value = self.get_parameter(name).value
        if isinstance(raw_value, str):
            raw_value = ast.literal_eval(raw_value)
        values = np.asarray(list(raw_value), dtype=np.float64)
        if values.size != 9:
            raise ValueError(f"{name} must contain 9 values, got {values.size}")
        return values.reshape(3, 3)

    @staticmethod
    def _format_matrix(matrix: np.ndarray) -> str:
        values = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        return "[" + "; ".join(
            ", ".join(f"{value:.1f}" for value in row) for row in values
        ) + "]"

    def _pose_from_msg(self, msg: PoseStamped) -> PoseInput:
        position = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=np.float64,
        )
        orientation_xyzw = (
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        timestamp = time.monotonic()
        pose = PoseInput()
        pose.position = position
        pose.orientation_xyzw = orientation_xyzw
        pose.timestamp = timestamp
        return pose

    def _copy_pose(self, pose: PoseInput) -> PoseInput:
        copied = PoseInput()
        copied.position = np.asarray(pose.position, dtype=np.float64).copy()
        copied.orientation_xyzw = tuple(float(value) for value in pose.orientation_xyzw)
        copied.timestamp = pose.timestamp
        return copied

    def _identity_relative_pose(self, timestamp=None) -> PoseInput:
        pose = PoseInput()
        pose.position = np.zeros(3, dtype=np.float64)
        pose.orientation_xyzw = (0.0, 0.0, 0.0, 1.0)
        pose.timestamp = time.monotonic() if timestamp is None else timestamp
        return pose

    def _compute_raw_relative_pose(self, reference: PoseInput, current: PoseInput) -> PoseInput:
        reference_rotation = self.core._quaternion_to_rotation_matrix(*reference.orientation_xyzw)
        current_rotation = self.core._quaternion_to_rotation_matrix(*current.orientation_xyzw)
        delta_pose = PoseInput()
        delta_pose.position = reference_rotation.T @ (
            np.asarray(current.position, dtype=np.float64) - np.asarray(reference.position, dtype=np.float64)
        )
        delta_rotation = reference_rotation.T @ current_rotation
        delta_pose.orientation_xyzw = self._rotation_matrix_to_quaternion(delta_rotation)
        delta_pose.timestamp = current.timestamp
        return delta_pose

    def _effective_relative_enable(self, grip_value: float) -> bool:
        return self.ik_enable_override or grip_value > self.grip_threshold

    def _update_relative_pose_state(self, arm: str):
        raw_pose_attr = f"relative_{arm}_raw_pose"
        reference_attr = f"relative_{arm}_reference_pose"
        enabled_attr = f"relative_{arm}_enabled"
        pose_attr = f"relative_{arm}_pose"
        grip_attr = f"relative_{arm}_grip"
        axis_matrix = getattr(self, f"relative_{arm}_axis_matrix")
        orientation_matrix = getattr(self, f"relative_{arm}_orientation_matrix")

        raw_pose = getattr(self, raw_pose_attr)
        enabled = self._effective_relative_enable(getattr(self, grip_attr))
        was_enabled = getattr(self, enabled_attr)

        raw_pose_fresh = (
            raw_pose is not None
            and (time.monotonic() - raw_pose.timestamp) <= self.relative_stream_timeout_sec
        )

        if not enabled or not raw_pose_fresh:
            release_pose = self._identity_relative_pose() if was_enabled else None
            setattr(self, enabled_attr, False)
            setattr(self, reference_attr, None)
            setattr(self, pose_attr, release_pose)
            return

        if not was_enabled or getattr(self, reference_attr) is None:
            setattr(self, reference_attr, self._copy_pose(raw_pose))
            setattr(self, enabled_attr, True)
            relative_pose = self._identity_relative_pose(raw_pose.timestamp)
        else:
            relative_pose = self._compute_raw_relative_pose(getattr(self, reference_attr), raw_pose)

        setattr(
            self,
            pose_attr,
            self._map_relative_pose(relative_pose, axis_matrix, orientation_matrix),
        )

    def _map_relative_pose(
        self,
        pose: PoseInput,
        axis_matrix: np.ndarray,
        orientation_matrix: np.ndarray,
    ) -> PoseInput:
        pose.position = axis_matrix @ pose.position
        rotation = self.core._quaternion_to_rotation_matrix(*pose.orientation_xyzw)
        mapped_rotation = orientation_matrix @ rotation @ orientation_matrix.T
        pose.orientation_xyzw = self._rotation_matrix_to_quaternion(mapped_rotation)
        return pose

    @staticmethod
    def _rotation_matrix_to_quaternion(rotation: np.ndarray):
        matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (matrix[2, 1] - matrix[1, 2]) / scale
            qy = (matrix[0, 2] - matrix[2, 0]) / scale
            qz = (matrix[1, 0] - matrix[0, 1]) / scale
        elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif matrix[1, 1] > matrix[2, 2]:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
        quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if norm > 0.0:
            quaternion /= norm
        return tuple(float(value) for value in quaternion)

    def _setup_ros_io(self):
        p = self.get_parameter

        self.absolute_head_sub = self.create_subscription(
            PoseStamped, p("vr_head_topic").value, self._absolute_head_callback, 10, callback_group=self.cb_group
        )
        self.absolute_left_pose_sub = self.create_subscription(
            PoseStamped, p("vr_left_pose_topic").value, self._absolute_left_pose_callback, 10, callback_group=self.cb_group
        )
        self.absolute_right_pose_sub = self.create_subscription(
            PoseStamped, p("vr_right_pose_topic").value, self._absolute_right_pose_callback, 10, callback_group=self.cb_group
        )
        self.absolute_left_grip_sub = self.create_subscription(
            Float32, p("vr_left_grip_topic").value, self._absolute_left_grip_callback, 10, callback_group=self.cb_group
        )
        self.absolute_right_grip_sub = self.create_subscription(
            Float32, p("vr_right_grip_topic").value, self._absolute_right_grip_callback, 10, callback_group=self.cb_group
        )
        self.absolute_left_trigger_sub = self.create_subscription(
            Float32, p("vr_left_trigger_topic").value, self._absolute_left_trigger_callback, 10, callback_group=self.cb_group
        )
        self.absolute_right_trigger_sub = self.create_subscription(
            Float32, p("vr_right_trigger_topic").value, self._absolute_right_trigger_callback, 10, callback_group=self.cb_group
        )
        self.absolute_calibrate_sub = self.create_subscription(
            Bool, p("vr_calibrate_topic").value, self._absolute_calibrate_callback, 10, callback_group=self.cb_group
        )
        self.absolute_mode_sub = self.create_subscription(
            String, p("vr_control_mode_topic").value, self._absolute_mode_callback, 10, callback_group=self.cb_group
        )
        self.absolute_rate_sub = self.create_subscription(
            Float32, p("vr_rate_topic").value, self._absolute_rate_callback, 10, callback_group=self.cb_group
        )
        self.absolute_button_b_sub = self.create_subscription(
            Bool, p("absolute_button_b_topic").value, self._absolute_button_b_callback, 10, callback_group=self.cb_group
        )
        self.absolute_button_x_sub = self.create_subscription(
            Bool, p("absolute_button_x_topic").value, self._absolute_button_x_callback, 10, callback_group=self.cb_group
        )
        self.absolute_button_y_sub = self.create_subscription(
            Bool, p("absolute_button_y_topic").value, self._absolute_button_y_callback, 10, callback_group=self.cb_group
        )

        self.relative_left_pose_sub = self.create_subscription(
            PoseStamped, "pico_left_controller/pose", self._relative_left_pose_callback, 10, callback_group=self.cb_group
        )
        self.relative_right_pose_sub = self.create_subscription(
            PoseStamped, "pico_right_controller/pose", self._relative_right_pose_callback, 10, callback_group=self.cb_group
        )
        self.relative_left_grip_sub = self.create_subscription(
            Float32, "pico_left_controller/grip", self._relative_left_grip_callback, 10, callback_group=self.cb_group
        )
        self.relative_right_grip_sub = self.create_subscription(
            Float32, "pico_right_controller/grip", self._relative_right_grip_callback, 10, callback_group=self.cb_group
        )
        self.relative_left_trigger_sub = self.create_subscription(
            Float32, "pico_left_controller/trigger", self._relative_left_trigger_callback, 10, callback_group=self.cb_group
        )
        self.relative_right_trigger_sub = self.create_subscription(
            Float32, "pico_right_controller/trigger", self._relative_right_trigger_callback, 10, callback_group=self.cb_group
        )
        self.relative_left_rate_sub = self.create_subscription(
            Float32, "pico_left_controller/rate", self._relative_rate_callback, 10, callback_group=self.cb_group
        )
        self.relative_right_rate_sub = self.create_subscription(
            Float32, "pico_right_controller/rate", self._relative_rate_callback, 10, callback_group=self.cb_group
        )
        self.relative_button_b_sub = self.create_subscription(
            Bool, p("button_b_topic").value, self._relative_button_b_callback, 10, callback_group=self.cb_group
        )
        self.relative_button_y_sub = self.create_subscription(
            Bool, p("button_y_topic").value, self._relative_button_y_callback, 10, callback_group=self.cb_group
        )
        ik_override_qos = QoSProfile(depth=1)
        ik_override_qos.reliability = ReliabilityPolicy.RELIABLE
        ik_override_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ik_enable_override_sub = self.create_subscription(
            Bool,
            p("ik_enable_override_topic").value,
            self._ik_enable_override_callback,
            ik_override_qos,
            callback_group=self.cb_group,
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10, callback_group=self.cb_group
        )

        self.left_cmd_pub = self.create_publisher(
            Float64MultiArray, p("left_cmd_topic").value, 10
        )
        self.right_cmd_pub = self.create_publisher(
            Float64MultiArray, p("right_cmd_topic").value, 10
        )
        self.get_logger().info("ROS2 interfaces configured")

    def _absolute_head_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.absolute_head_pose = self._pose_from_msg(msg)

    def _absolute_left_pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.absolute_left_pose = self._pose_from_msg(msg)

    def _absolute_right_pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.absolute_right_pose = self._pose_from_msg(msg)

    def _absolute_left_grip_callback(self, msg: Float32):
        self.absolute_left_grip = max(0.0, min(1.0, float(msg.data)))

    def _absolute_right_grip_callback(self, msg: Float32):
        self.absolute_right_grip = max(0.0, min(1.0, float(msg.data)))

    def _absolute_left_trigger_callback(self, msg: Float32):
        self.absolute_left_trigger = max(0.0, min(1.0, float(msg.data)))

    def _absolute_right_trigger_callback(self, msg: Float32):
        self.absolute_right_trigger = max(0.0, min(1.0, float(msg.data)))

    def _absolute_mode_callback(self, msg: String):
        self.core.update_absolute_mode(msg.data)

    def _absolute_calibrate_callback(self, msg: Bool):
        if not bool(msg.data):
            return
        abs_input = self._current_absolute_input()
        if self.core.calibrate_absolute(abs_input):
            self.get_logger().info("Absolute calibration completed successfully")
        else:
            self.get_logger().warn("Absolute calibration skipped - absolute poses are incomplete")

    def _absolute_rate_callback(self, msg: Float32):
        self.absolute_is_full_speed = float(msg.data) >= 0.999

    def _absolute_button_b_callback(self, msg: Bool):
        self._handle_button_edge("absolute_button_b_pressed", "pending_go_home", bool(msg.data))

    def _absolute_button_x_callback(self, msg: Bool):
        self._handle_button_edge("absolute_button_x_pressed", "pending_absolute_capture", bool(msg.data))

    def _absolute_button_y_callback(self, msg: Bool):
        self._handle_button_edge("absolute_button_y_pressed", "pending_hands_up", bool(msg.data))

    def _relative_left_pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.relative_left_raw_pose = self._pose_from_msg(msg)

    def _relative_right_pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.relative_right_raw_pose = self._pose_from_msg(msg)

    def _relative_left_grip_callback(self, msg: Float32):
        self.relative_left_grip = max(0.0, min(1.0, float(msg.data)))

    def _relative_right_grip_callback(self, msg: Float32):
        self.relative_right_grip = max(0.0, min(1.0, float(msg.data)))

    def _relative_left_trigger_callback(self, msg: Float32):
        self.relative_left_trigger = max(0.0, min(1.0, float(msg.data)))

    def _relative_right_trigger_callback(self, msg: Float32):
        self.relative_right_trigger = max(0.0, min(1.0, float(msg.data)))

    def _relative_rate_callback(self, msg: Float32):
        self.relative_is_full_speed = float(msg.data) >= 0.999

    def _ik_enable_override_callback(self, msg: Bool):
        self.ik_enable_override = bool(msg.data)

    def _relative_button_b_callback(self, msg: Bool):
        self._handle_button_edge("relative_button_b_pressed", "pending_go_home", bool(msg.data))

    def _relative_button_y_callback(self, msg: Bool):
        self._handle_button_edge("relative_button_y_pressed", "pending_hands_up", bool(msg.data))

    def _handle_button_edge(self, state_attr: str, pending_attr: str, current_state: bool):
        with self.button_lock:
            previous_state = getattr(self, state_attr)
            setattr(self, state_attr, current_state)
            if current_state and not previous_state:
                setattr(self, pending_attr, True)

    def _joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = float(pos)
        self.joint_states_received = True
        self.core.update_joint_states(msg.name, msg.position)

    def _current_relative_input(self) -> RelativeTeleopInputFrame:
        with self.pose_mutex:
            self._update_relative_pose_state("left")
            self._update_relative_pose_state("right")
            left_pose = self.relative_left_pose
            right_pose = self.relative_right_pose
            left_enabled = self.relative_left_enabled
            right_enabled = self.relative_right_enabled
        frame = RelativeTeleopInputFrame()
        frame.left_pose = left_pose
        frame.right_pose = right_pose
        frame.left_grip = 1.0 if left_enabled else 0.0
        frame.right_grip = 1.0 if right_enabled else 0.0
        frame.left_trigger = self.relative_left_trigger
        frame.right_trigger = self.relative_right_trigger
        return frame

    def _current_absolute_input(self) -> AbsoluteTeleopInputFrame:
        with self.pose_mutex:
            head_pose = self.absolute_head_pose
            left_pose = self.absolute_left_pose
            right_pose = self.absolute_right_pose
        frame = AbsoluteTeleopInputFrame()
        frame.head_pose = head_pose
        frame.left_pose = left_pose
        frame.right_pose = right_pose
        frame.left_grip = self.absolute_left_grip
        frame.right_grip = self.absolute_right_grip
        frame.left_trigger = self.absolute_left_trigger
        frame.right_trigger = self.absolute_right_trigger
        frame.mode_selected = self.core.absolute_mode_selected
        return frame

    def _manual_override_requested(self):
        return (
            self._relative_manual_override_requested()
            or self.absolute_left_grip > self.grip_threshold
            or self.absolute_right_grip > self.grip_threshold
        )

    def _relative_manual_override_requested(self):
        if self.ik_enable_override:
            return True
        if self.relative_left_grip > self.grip_threshold:
            return True
        if self.relative_right_grip > self.grip_threshold:
            return True
        return False

    def _relative_pose_has_motion(self, pose: PoseInput):
        if pose is None:
            return False
        position = np.asarray(pose.position, dtype=np.float64).reshape(3)
        if float(np.linalg.norm(position)) > self.relative_manual_override_position_threshold_m:
            return True
        rotation = self.core._quaternion_to_rotation_matrix(*pose.orientation_xyzw)
        trace_value = float(np.trace(rotation))
        cos_angle = max(-1.0, min(1.0, 0.5 * (trace_value - 1.0)))
        angle = float(np.arccos(cos_angle))
        return angle > self.relative_manual_override_rotation_threshold_rad

    def _consume_pending_button_events(self):
        with self.button_lock:
            pending_go_home = self.pending_go_home
            pending_hands_up = self.pending_hands_up
            pending_absolute_capture = self.pending_absolute_capture
            self.pending_go_home = False
            self.pending_hands_up = False
            self.pending_absolute_capture = False
        return pending_go_home, pending_hands_up, pending_absolute_capture

    def _build_zero_joint_target(self):
        return np.zeros(14, dtype=np.float64)

    def _build_hands_up_joint_target(self):
        target = np.zeros(14, dtype=np.float64)
        q_upper = self.core.ik_solver.q_upper
        target[3] = min(2.0, float(q_upper[3]))
        target[10] = min(2.0, float(q_upper[10]))
        return target

    def _start_button_joint_motion(self, *, name: str, target_q):
        self.button_motion_active = True
        self.button_motion_name = name
        self.button_motion_kind = "joint"
        self.button_desired_target_q = np.asarray(target_q, dtype=np.float64).reshape(14)
        self.button_command_target_q = None
        self.button_target_left_transform = None
        self.button_target_right_transform = None
        self.button_target_reached_count = 0
        self.get_logger().info(
            f"Starting button motion '{name}' at {self.button_motion_step_deg:.2f} deg/cycle"
        )

    def _start_button_transform_motion(self, *, name: str, left_target, right_target):
        self.button_motion_active = True
        self.button_motion_name = name
        self.button_motion_kind = "transform"
        self.button_desired_target_q = None
        self.button_command_target_q = None
        self.button_target_left_transform = np.asarray(left_target, dtype=np.float64).reshape(4, 4)
        self.button_target_right_transform = np.asarray(right_target, dtype=np.float64).reshape(4, 4)
        self.button_target_reached_count = 0
        self.get_logger().info(
            f"Starting button motion '{name}' at {self.button_motion_step_deg:.2f} deg/cycle"
        )

    def _cancel_button_motion(self, *, reason: str):
        if not self.button_motion_active:
            return
        self.get_logger().info(
            f"Cancelling button motion '{self.button_motion_name}' because {reason}"
        )
        self.button_motion_active = False
        self.button_motion_name = None
        self.button_motion_kind = None
        self.button_desired_target_q = None
        self.button_command_target_q = None
        self.button_target_left_transform = None
        self.button_target_right_transform = None
        self.button_target_reached_count = 0

    def _process_button_events(self):
        pending_go_home, pending_hands_up, pending_absolute_capture = self._consume_pending_button_events()

        if pending_go_home:
            self._start_button_joint_motion(name="go_home", target_q=self._build_zero_joint_target())
        if pending_hands_up:
            self._start_button_joint_motion(name="hands_up", target_q=self._build_hands_up_joint_target())
        if pending_absolute_capture:
            abs_input = self._current_absolute_input()
            targets = self.core.compute_absolute_capture_targets(abs_input)
            if targets is None:
                self.get_logger().warn(
                    "Ignoring absolute capture because absolute mode is not selected, calibrated, or poses are incomplete"
                )
            else:
                self._start_button_transform_motion(
                    name="absolute_capture",
                    left_target=targets[0],
                    right_target=targets[1],
                )

    def _run_button_motion(self):
        current_q = self._get_current_joint_positions()
        if current_q is None:
            self.get_logger().warn(
                f"Button motion '{self.button_motion_name}' paused because joint_states are incomplete"
            )
            return
        if self._manual_override_requested():
            self._cancel_button_motion(reason="another controller took over")
            return

        if self.button_motion_kind == "joint":
            target_q = np.asarray(self.button_desired_target_q, dtype=np.float64)
            if self.button_command_target_q is None:
                self.button_command_target_q = np.asarray(current_q, dtype=np.float64).copy()
            next_command_q = self._limit_joint_step(
                target_q,
                self.button_command_target_q,
                is_full_speed=False,
                threshold_rad=0.0,
                max_step_override=self.button_motion_step_rad,
            )
        elif self.button_motion_kind == "transform":
            target_q, _ = self.core.ik_solver.solve_ik(
                self.button_target_left_transform,
                self.button_target_right_transform,
                current_q=current_q,
            )
            next_command_q = self._limit_joint_step(
                target_q,
                current_q,
                is_full_speed=False,
                threshold_rad=0.0,
                max_step_override=self.button_motion_step_rad,
            )
        else:
            self._cancel_button_motion(reason="button motion kind is invalid")
            return

        max_error = float(np.max(np.abs(target_q - current_q))) if target_q.size > 0 else 0.0
        if max_error <= self.button_motion_done_tolerance_rad:
            self.button_target_reached_count += 1
        else:
            self.button_target_reached_count = 0

        self._publish_joint_commands(
            next_command_q[:7].tolist(),
            next_command_q[7:14].tolist(),
            self._get_current_gripper_command("left"),
            self._get_current_gripper_command("right"),
        )
        if self.button_motion_kind == "joint":
            self.button_command_target_q = np.asarray(next_command_q, dtype=np.float64).copy()
        if self.button_target_reached_count >= self.button_target_stable_cycles_required:
            self._cancel_button_motion(reason="target reached")

    def _control_loop(self):
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            self._process_button_events()
            if self.button_motion_active:
                self._run_button_motion()
                return

            relative_input = self._current_relative_input()
            absolute_input = self._current_absolute_input()
            result = self.core.step(relative_input, absolute_input)
            if result.active_mode != self.current_mode:
                self.current_mode = result.active_mode
                if self.current_mode is None:
                    self.get_logger().info("Teleop mode idle - no active relative/absolute stream")
                else:
                    self.get_logger().info(f"Teleop mode switched to: {self.current_mode}")

            if result.waiting_for_joint_states or result.calibration_required or result.target_q is None:
                return

            current_q = self._get_current_joint_positions()
            if current_q is None:
                return
            is_full_speed = (
                self.relative_is_full_speed if result.active_mode == "relative" else self.absolute_is_full_speed
            )
            limited_q = self._limit_joint_step(result.target_q, current_q, is_full_speed)

            if result.left_active:
                left_gripper = (
                    self._map_trigger_to_gripper(self.relative_left_trigger)
                    if result.active_mode == "relative"
                    else self._map_trigger_to_gripper(self.absolute_left_trigger)
                )
            else:
                left_gripper = self._get_current_gripper_command("left")

            if result.right_active:
                right_gripper = (
                    self._map_trigger_to_gripper(self.relative_right_trigger)
                    if result.active_mode == "relative"
                    else self._map_trigger_to_gripper(self.absolute_right_trigger)
                )
            else:
                right_gripper = self._get_current_gripper_command("right")

            self._publish_joint_commands(
                limited_q[:7].tolist(),
                limited_q[7:14].tolist(),
                left_gripper,
                right_gripper,
            )
        except Exception as exc:
            # self.get_logger().error(f"Control loop error: {exc}")
            pass
        finally:
            self.control_lock.release()

    def _get_current_joint_positions(self):
        if not self.joint_states_received:
            return None
        joint_names = [
            "openarmx_left_joint1",
            "openarmx_left_joint2",
            "openarmx_left_joint3",
            "openarmx_left_joint4",
            "openarmx_left_joint5",
            "openarmx_left_joint6",
            "openarmx_left_joint7",
            "openarmx_right_joint1",
            "openarmx_right_joint2",
            "openarmx_right_joint3",
            "openarmx_right_joint4",
            "openarmx_right_joint5",
            "openarmx_right_joint6",
            "openarmx_right_joint7",
        ]
        values = []
        for name in joint_names:
            current = self.joint_positions.get(name)
            if current is None:
                return None
            values.append(float(current))
        return np.asarray(values, dtype=np.float64)

    def _limit_joint_step(
        self,
        target_q,
        current_q,
        is_full_speed,
        threshold_rad=None,
        max_step_override=None,
    ):
        if current_q is None:
            return target_q

        if max_step_override is not None:
            max_step = np.full(14, float(max_step_override), dtype=np.float64)
        else:
            max_step = (
                self.fast_max_step_rad_per_joint
                if is_full_speed
                else self.max_step_rad_per_joint
            )
        if threshold_rad is None:
            threshold_rad = self.step_limit_enable_threshold_rad_per_joint
        delta = np.asarray(target_q, dtype=np.float64) - np.asarray(current_q, dtype=np.float64)
        if np.isscalar(threshold_rad):
            threshold = np.full(delta.shape, float(threshold_rad), dtype=np.float64)
        else:
            threshold = np.asarray(threshold_rad, dtype=np.float64)
        apply_limit_mask = np.abs(delta) > threshold
        limited_delta = np.clip(delta, -max_step, max_step)
        delta = np.where(apply_limit_mask, limited_delta, delta)
        return np.asarray(current_q, dtype=np.float64) + delta

    @staticmethod
    def _map_trigger_to_gripper(trigger_value):
        return max(0.0, min(1.0, float(trigger_value))) * 0.04

    def _publish_joint_commands(self, left_joints, right_joints, left_gripper, right_gripper):
        left_cmd = Float64MultiArray()
        left_cmd.data = list(left_joints)
        if self.publish_gripper_command:
            left_cmd.data.append(float(left_gripper))
        self.left_cmd_pub.publish(left_cmd)

        right_cmd = Float64MultiArray()
        right_cmd.data = list(right_joints)
        if self.publish_gripper_command:
            right_cmd.data.append(float(right_gripper))
        self.right_cmd_pub.publish(right_cmd)

    def _get_current_gripper_command(self, arm: str) -> float:
        joint_name = f"openarmx_{arm}_finger_joint1"
        current = self.joint_positions.get(joint_name)
        return 0.0 if current is None else float(current)


def main(args=None):
    rclpy.init(args=args)
    node = OpenArmXTeleopVRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
