# openarmx_teleop_vr

This directory is the OpenArmX **VR teleoperation pipeline**. It contains two ROS 2 packages that together provide:

`VR controller UDP data -> ROS 2 topics -> dual-arm joint control commands`

<p align="center">
  <img src="assets/openarmx_teleop_vr.gif" alt="openarmx_teleop_vr demo" />
</p>

> ⚠️ It is strongly recommended to start in this order: "robot base -> bridge node -> teleop node". This avoids failures caused by unavailable topics.

## ✅ Prerequisite

Before running this ROS 2 teleoperation pipeline, install the VR teleoperation app on your VR device first.  
APK repository:

- `https://github.com/openarmx/openarmx_teleop_vr_apk.git`

> ⚠️ If this app is not installed and running properly on the VR device, the bridge node cannot receive valid controller UDP data.

## 🗂️ Directory Structure

```text
openarmx_teleop_vr/
├── openarmx_teleop_bridge_vr/   # C++ bridge package: UDP -> ROS 2 topics/TF
├── openarmx_teleop_vr/          # Python teleop package: topic input -> IK -> control commands
├── LICENSE
├── README_CN.md                      # This overview (Chinese)
└── README_EN.md                      # This overview (English)
```

## 📦 What Each Subpackage Does

1. `openarmx_teleop_bridge_vr`
- Receives UDP data from VR/OpenXR side (default listen port: `5100`).
- Publishes ROS 2 topics for controller pose, trigger, grip, rate, etc. (optional TF publishing).

2. `openarmx_teleop_vr`
- Subscribes to bridge topics and `/joint_states`.
- Performs IK computation and constraint handling.
- Publishes dual-arm control commands to:
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

## 🚀 Shortest Usage Flow (Recommended Order)

1. Start robot base services (ensure `forward_position_controller` is available)  
2. Start bridge node  
3. Start VR teleoperation node

> ✅ Teleoperation is more stable after all three steps are fully running.

Example:

```bash
cd <your_workspace>
colcon build --packages-select openarmx_teleop_bridge_vr openarmx_teleop_vr
source install/setup.bash

# Terminal 1: robot base (real robot)
# Note: In the current version, performance is better with gravity compensation enabled; keeping it on is recommended.
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
    use_fake_hardware:=false

# Terminal 2: VR bridge
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node

# Terminal 3: VR teleoperation execution node
ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 🔎 Quick Checks

- Check bridge input:
  - `ros2 topic echo /vr_left_controller/pose`
  - `ros2 topic echo /vr_right_controller/pose`
- Check teleop output:
  - `ros2 topic echo /left_forward_position_controller/commands`
  - `ros2 topic echo /right_forward_position_controller/commands`

> ⚠️ If input topics have data but output commands are missing, first check robot controller status and node startup order.

## 🧩 Key Dependencies (Summary)

- Bridge package: `rclcpp`, `geometry_msgs`, `tf2_ros`
- Teleop package: `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`, `xacro`, `openarmx_description`
- Runtime also requires `openarmx_arm_driver` (see subpackage docs)

## 📚 Detailed Documentation Entry Points

- Bridge package CN doc: `openarmx_teleop_bridge_vr/README_CN.md`
- Teleop package CN doc: `openarmx_teleop_vr/README_CN.md`
- Teleop launch file: `openarmx_teleop_vr/launch/teleop_vr.launch.py`

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