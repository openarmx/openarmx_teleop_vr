# openarmx_teleop_vr User Guide

## 1. Package Positioning

`openarmx_teleop_vr` is the ROS 2 VR teleoperation execution node for OpenArmX.

This node subscribes to controller pose, headset pose, grip, trigger, button, and speed-rate topics published by the Pico/VR bridge node, reads robot `/joint_states`, and calls the IK logic inside `openarmx_arm_driver` to compute target joint angles for both arms. It then publishes joint position commands to the left and right `forward_position_controller`.

The overall goal is:

```text
Pico/VR controllers -> ROS 2 bridge topics -> openarmx_teleop_vr_node -> openarmx_arm_driver IK -> dual-arm controllers
```

For illustrated tutorials about VR teleoperation, visit the official documentation: <http://docs.openarmx.com>

## 2. Package Structure

Current package structure:

```text
openarmx_teleop_vr/
├── README.md
├── config/
│   └── teleop_params.yaml          # Teleoperation parameter config
├── launch/
│   └── teleop_vr.launch.py         # Launch entry
├── openarmx_teleop_vr/
│   ├── __init__.py
│   └── openarmx_teleop_vr_node.py  # Main node
├── package.xml
├── resource/
│   └── openarmx_teleop_vr
├── setup.cfg
└── setup.py
```

## 3. System Pipeline

Typical runtime pipeline:

1. The VR device publishes controller data through the bridge package (`openarmx_teleop_bridge_vr`).
2. This node subscribes to input topics and computes joint control commands for the left and right arms.
3. Control commands are published to `forward_position_controller` to drive simulation or real hardware.

## 4. Main Features

1. Supports independent left/right controller mapping to left/right arm end-effector motion.
2. Supports both relative and absolute VR control input.
3. Uses grip as a manual-control enable switch to reduce accidental operations.
4. Uses trigger values to control grippers in standard gripper mode.
5. Supports two joint step-limit modes: slow and fast.
6. Supports button-triggered go-home and hands-up actions.

## 5. Quick Start

### Prerequisites

1. Robot backend (simulation or real hardware) is running, and `forward_position_controller` is available.
2. VR bridge node is running and continuously publishing controller topics.
3. The runtime environment can import `openarmx_arm_driver` (required dependency).

### Option 1: Standard Grippers (Default)

1. Terminal 1: start the upstream bimanual robot with standard grippers.

```bash
cd <your_workspace>
source install/setup.bash

# Simulation
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true

# Real hardware; bring can0/can1 up at bitrate 1000000 first
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

2. Terminal 2: start the Pico/VR bridge.

```bash
cd <your_workspace>
source install/setup.bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

3. Terminal 3: start VR with the standard-gripper model.

```bash
cd <your_workspace>
source install/setup.bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  controller_pose_mode:=gripper \
  urdf_path:="$(ros2 pkg prefix openarmx_description)/share/openarmx_description/urdf/robot/openarmx_robot.urdf"
```

`gripper` is the default control mode. Each side publishes seven arm joints plus one
gripper command, and VR triggers control the standard grippers.

### Option 2: O6 Dexterous Hands

![OpenArmX O6 dexterous-hand VR teleoperation demo](../assets/openarmx_hand.gif)

1. Terminal 1: start the robot backend with O6 hands.

```bash
cd <your_workspace>
source install/setup.bash

# Simulation; fake arm hardware also selects fake O6 and starts no real O6 CAN driver
ros2 launch openarmx_hand_bringup openarmx.bimanual.o6.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true

# Real hardware; bring can0/can1/can2/can3 up at bitrate 1000000 first
ros2 launch openarmx_hand_bringup openarmx.bimanual.o6.launch.py \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

O6 defaults are left `can3/0x28` and right `can2/0x27`. Gravity compensation is off by
default. Enable it with `enable_forward_effort:=true`; it requires `control_mode:=mit`.

2. Terminal 2: start the same VR bridge shown in option 1.

3. Terminal 3: start VR in dexterous-hand mode.

```bash
cd <your_workspace>
source install/setup.bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  controller_pose_mode:=hand
```

`hand` mode publishes only seven arm joints per side and does not control O6 fingers from
VR. Joint Slider, the GUI, or a separate controller controls O6 fingers.

### O6 Multi-Robot Namespace

If O6 bringup uses `robot_namespace:=robot1`, bridge and teleop must use `/robot1` as well:

```bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node \
  --ros-args -r __ns:=/robot1
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  controller_pose_mode:=hand robot_namespace:=robot1
```

## 6. Input and Output Topics

The tables show fully resolved names without a namespace. With
`robot_namespace:=robot1`, the corresponding topics are under `/robot1/...`.
### Relative-Mode Input Topics (default)

| Topic | Type | Description |
|------|------|-------------|
| `/pico_left_controller/pose` | `geometry_msgs/PoseStamped` | Left controller relative pose |
| `/pico_right_controller/pose` | `geometry_msgs/PoseStamped` | Right controller relative pose |
| `/pico_left_controller/grip` | `std_msgs/Float32` | Left grip, enables left-arm control |
| `/pico_right_controller/grip` | `std_msgs/Float32` | Right grip, enables right-arm control |
| `/pico_left_controller/trigger` | `std_msgs/Float32` | Left trigger, controls left gripper |
| `/pico_right_controller/trigger` | `std_msgs/Float32` | Right trigger, controls right gripper |
| `/pico_left_controller/rate` | `std_msgs/Float32` | Relative-mode speed rate |
| `/pico_right_controller/rate` | `std_msgs/Float32` | Relative-mode speed rate |
| `/pico_right_controller/button_b` | `std_msgs/Bool` | Button B, triggers go-home action |
| `/pico_left_controller/button_y` | `std_msgs/Bool` | Button Y, triggers hands-up action |

### Absolute-Mode Input Topics

| Topic | Type | Description |
|------|------|-------------|
| `/vr/head/pose` | `geometry_msgs/PoseStamped` | Headset pose |
| `/vr/left/pose_absolute` | `geometry_msgs/PoseStamped` | Left controller absolute pose |
| `/vr/right/pose_absolute` | `geometry_msgs/PoseStamped` | Right controller absolute pose |
| `/vr/left/grip`, `/vr/right/grip` | `std_msgs/Float32` | Left and right arm enable inputs |
| `/vr/left/trigger`, `/vr/right/trigger` | `std_msgs/Float32` | Standard-gripper trigger inputs |
| `/vr/calibrate_done` | `std_msgs/Bool` | Absolute-mode calibration completion |
| `/vr/control_mode` | `std_msgs/String` | VR control mode |
| `/vr/rate` | `std_msgs/Float32` | Absolute-mode speed rate |

### Robot State Input

| Topic | Type | Description |
|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current robot joint-angle feedback |

### Output Topics (default)

| Topic | Type | Description |
|------|------|-------------|
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Seven joints plus gripper in `gripper`; seven joints in `hand` |
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Seven joints plus gripper in `gripper`; seven joints in `hand` |

## 7. Common Parameters

Parameters are defined in `config/teleop_params.yaml`, and can also be overridden by ROS 2 launch arguments.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `urdf_path` | Injected by launch | Standard grippers use the upstream URDF; empty generates O6 |
| `control_rate` | `100.0` | Control loop frequency (Hz) |
| `grip_threshold` | `0.5` | Grip enable threshold |
| `max_step_deg_joint1..7` | `4.0` | Per-joint maximum step in slow mode |
| `fast_max_step_deg_joint1..7` | See config | Per-joint maximum step in fast mode |
| `step_limit_enable_threshold_deg_joint1..7` | See config | Per-joint error threshold for step limiting |
| `button_motion_step_deg` | `0.5` | Max joint step per cycle for button actions |
| `button_motion_done_tolerance_deg` | `1.0` | Target-reached tolerance for button actions |
| `body_anchor_offset` | `[0.0, -0.18, 0.08]` | Offset from headset to body reference point |
| `position_scale_xyz` | `[1.0, 1.0, 0.9]` | Position mapping scale factors |
| `controller_pose_mode` | `gripper` | `gripper` uses AIM mapping and publishes a gripper; `hand` uses grip mapping and publishes seven joints |
| `gripper_*_axis_matrix` | See config | Standard-gripper relative position mapping |
| `gripper_*_orientation_matrix` | See config | Standard-gripper relative orientation mapping |
| `hand_*_axis_matrix` | See config | O6 relative position mapping |
| `hand_*_orientation_matrix` | See config | O6 relative orientation mapping |

Common topic parameters are also defined in the config file, for example:

```yaml
vr_left_pose_topic: "vr/left/pose_absolute"
vr_right_pose_topic: "vr/right/pose_absolute"
left_cmd_topic: "left_forward_position_controller/commands"
right_cmd_topic: "right_forward_position_controller/commands"
controller_pose_mode: "gripper"
```

## 8. Operation Notes

### Grip

When grip value is greater than `grip_threshold`, the corresponding arm enters manual control. After grip is released, that arm no longer follows controller pose, and the gripper command keeps its current state.

### Trigger

In `gripper` mode, trigger values map to gripper commands. In `hand` mode, triggers do
not generate O6 finger commands; use Joint Slider, the GUI, or a separate O6 controller.

### Speed Rate

When `rate` is close to `1.0`, fast mode is enabled; otherwise slow mode is used. The
modes use `fast_max_step_deg_joint1..7` and `max_step_deg_joint1..7`, respectively.

### Button B

Button B triggers dual-arm go-home action with a 14-dimensional zero joint target. This action uses independent speed limit `button_motion_step_deg`.

### Button Y

Button Y triggers hands-up action. It currently mainly raises the 4th joint of both arms and is constrained by joint upper limits.

## 9. FAQ

1. Node reports `urdf_path parameter is required` at startup

The launch file generates the O6 model automatically in O6 mode. Standard-gripper mode
must pass the upstream model as shown in option 1. When running the node without the
launch file, specify an absolute URDF path manually:

```bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  urdf_path:=/absolute/path/to/openarmx_robot.urdf
```

2. Error: `openarmx_arm_driver` import failed

It means the current Python environment is missing the low-level driver, or it is not correctly installed in the current user environment. Install `openarmx_arm_driver` first, and ensure `source install/setup.bash` has been executed in your terminal.

3. Robot arm does not move after node startup

First verify both `/joint_states` and controller topics have data:

```bash
ros2 topic echo /joint_states
ros2 topic echo /pico_left_controller/pose
ros2 topic echo /pico_right_controller/pose
```

4. Controller data exists but no control commands

Check whether grip exceeds threshold, and inspect control command topics:

```bash
ros2 topic echo /left_forward_position_controller/commands
ros2 topic echo /right_forward_position_controller/commands
```

5. Robot arm jitters or jumps

Reduce the corresponding `max_step_deg_joint1..7` or `fast_max_step_deg_joint1..7`
values, and confirm `/joint_states` is continuous and stable. Low Pico controller battery
or tracking loss may also cause abnormal input poses.

6. Gripper does not respond

In standard gripper mode, check trigger topics and confirm the corresponding grip is
enabled. O6 finger control from VR is intentionally disabled.

## 10. License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, see the project LICENSE file or visit: <http://creativecommons.org/licenses/by-nc-sa/4.0/>

## 11. Author

- **Zhang Li**
- Company: Chengdu Changshu Robot Co., Ltd.
- Website: <https://openarmx.com/>

## 12. Version

**Current version**: 3.0.0

## 13. Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact | Information |
|---------|-------------|
| Email | openarmrobot@gmail.com |
| Phone / WeChat | +86-17746530375 |
| Website | <https://openarmx.com/> |
| Address | Huacheng Machinery Factory, No. 11 Xinye 8th Street, West Zone, Tianjin Economic-Technological Development Area |
| Contact Person | Mr. Wang |
