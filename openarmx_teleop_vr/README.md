# openarmx_teleop_vr User Guide (English)

## 1. Package Positioning

`openarmx_teleop_vr` is the VR teleoperation execution node of OpenArmX.
This node subscribes to ROS topics such as VR controller pose, trigger, and grip, and publishes joint commands to dual-arm controllers, enabling online teleoperation from "VR controllers -> robot dual arms".

For the illustrated tutorial on VR device teleoperation, please visit the official documentation: <http://docs.openarmx.com>

## 2. Package Structure

```text
openarmx_teleop_vr/
├── README_CN.md
├── README.md
├── launch/
│   └── teleop_vr.launch.py         # Launch entry
├── openarmx_teleop_vr/
│   └── openarmx_teleop_vr_node.py  # Main node
├── package.xml
└── setup.py
```

## 3. System Pipeline

The overall data flow is as follows:

1. The VR device publishes controller data through the bridge package (`openarmx_teleop_bridge_vr`).
2. This node subscribes to input topics and computes joint control commands for the left and right arms.
3. Control commands are published to `forward_position_controller` to drive simulation or real hardware.

## 4. Main Functions

1. Use dual-controller poses to control dual-arm end-effector motion.
2. Use index triggers to control opening and closing of the left and right grippers.
3. Use grip as an enable switch (deadman switch) to reduce accidental operation risk.
4. Supports speed mode switching (slow/fast) to reduce safety risks caused by large jumps.
5. Optionally publishes visualization TF for RViz debugging.

## 5. Quick Start

### Prerequisites

1. The robot low-level stack (simulation or real hardware) is running, and `forward_position_controller` is available.
2. The VR bridge node is running and continuously publishing controller topics.
3. `openarmx_arm_driver` can be imported in the runtime environment (required dependency for this node).

### Typical Startup Order

1. Terminal 1: Start the robot

```bash
cd <your workspace>
source install/setup.bash

# Simulation mode
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true

# Real hardware mode: bring up CAN channels first
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

2. Terminal 2: Start VR bridge

```bash
cd <your workspace>
source install/setup.bash

ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

3. Terminal 3: Start this package

```bash
cd <your workspace>
source install/setup.bash

ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 6. Input and Output Topics

### Input Topics (Default)

| Topic | Type | Description |
|------|------|------|
| `/vr_left_controller/pose` | `geometry_msgs/PoseStamped` | Left controller pose input |
| `/vr_right_controller/pose` | `geometry_msgs/PoseStamped` | Right controller pose input |
| `/vr_left_controller/trigger` | `std_msgs/Float32` | Left trigger (gripper) |
| `/vr_right_controller/trigger` | `std_msgs/Float32` | Right trigger (gripper) |
| `/vr_left_controller/grip` | `std_msgs/Float32` | Left grip (enable) |
| `/vr_right_controller/grip` | `std_msgs/Float32` | Right grip (enable) |
| `/vr_right_controller/rate` | `std_msgs/Float32` | Speed mode input (0.1/1.0) |
| `/joint_states` | `sensor_msgs/JointState` | Current joint state feedback |

### Output Topics (Default)

| Topic | Type | Description |
|------|------|------|
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Left arm joint commands |
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Right arm joint commands |

## 7. Common Parameters (Application Layer)

| Parameter | Default | Description |
|------|--------|------|
| `control_rate` | `100.0` | Control loop frequency (Hz) |
| `rate_topic` | `/vr_right_controller/rate` | Speed mode input topic |
| `slow_max_step_deg` | `1.0` | Maximum joint step per cycle in slow mode |
| `fast_max_step_deg` | `12.0` | Maximum joint step per cycle in fast mode |
| `ik_iterations` | `3` | Inverse kinematics iteration count per cycle |
| `grip_threshold` | `0.5` | Grip enable threshold |
| `left_grip_topic` | `/vr_left_controller/grip` | Left grip topic |
| `right_grip_topic` | `/vr_right_controller/grip` | Right grip topic |
| `publish_visualization_tf` | `true` | Whether to publish visualization TF |
| `print_performance` | `false` | Whether to print performance logs |
| `use_xacro` | `true` | Whether to generate URDF from xacro at runtime |
| `use_link4_ext` | `true` | Whether to enable extended constraint frame configuration |
| `constraint_mode` | `joint` | Constraint mode: `joint` or `link` |

Example:

```bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  constraint_mode:=link
```

Note: The robotic arm may jitter when approaching or exceeding the workspace boundary. This is usually because the solver is still attempting to approach an unreachable target. For safety, avoid extreme-pose operations.

`constraint_mode` description:

1. `joint` (default): faster response, but inverse kinematics has no additional pose constraints, so non-intuitive joint poses may occur.
2. `link`: introduces additional constraints on the 2nd and 4th joints, making the arm posture closer to the robot center and better suited for operations in the chest-front workspace.

Please switch between the two modes according to task requirements.

## 8. FAQ

1. Wrist joint response is relatively slow

Default parameters set relatively strict joint step limits to reduce risks from abrupt arm motion. You can moderately increase `slow_max_step_deg` and `fast_max_step_deg` to improve responsiveness, but you need to assess safety risks yourself.

2. Node starts but the robotic arm does not move

First check whether the bridge topics have data:

```bash
ros2 topic echo /vr_left_controller/pose
ros2 topic echo /vr_right_controller/pose
```

3. Controller data exists but there are no control commands

Check the controller command topics:

```bash
ros2 topic echo /left_forward_position_controller/commands
ros2 topic echo /right_forward_position_controller/commands
```

4. Error: `openarmx_arm_driver` import failed

This means the current environment is missing this Python dependency. Please install it first and ensure the environment has been sourced correctly.

5. Gripper does not respond

Please check trigger topic data and whether the grip threshold (`grip_threshold`) matches your actual operation habits.

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to [LICENSE_CN.md](LICENSE) or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Zhang Li** (张力)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

**Current Version**: 1.0.0

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact Method | Information |
|---------|------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Official Website | <https://openarmx.com/> |
| 📍 Address | Huacheng Machinery Factory, No. 11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang |
