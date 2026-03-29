# openarmx_teleop_bridge_vr

## 1. What This Package Does

`openarmx_teleop_bridge_vr` is a ROS 2 bridge package.
It receives VR controller data over UDP and publishes it as ROS 2 topics (with optional TF publishing), so downstream teleoperation or control nodes can use it directly.

In one sentence: **Bring VR controller data into ROS 2.**

## 2. Package Structure

```text
openarmx_teleop_bridge_vr/
├── README_CN.md
├── README.md
├── CMakeLists.txt
├── package.xml
└── src/
    └── openarmx_teleop_bridge_vr_node.cpp
```

## 3. Application-Level Data Flow

VR/OpenXR sender
-> UDP data (default port `5100`)
-> `openarmx_teleop_bridge_vr_node`  
-> ROS 2 topics (pose/trigger/grip/button/rate)  
-> your teleop or control node

## 4. Quick Start

### Build

```bash
cd <your_workspace>
colcon build --packages-select openarmx_teleop_bridge_vr
```

### Run

```bash
source install/setup.bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

### Check Whether Data Is Being Published

```bash
ros2 topic echo /vr_left_controller/pose
ros2 topic echo /vr_right_controller/pose
```

## 5. Topics Published by Default

| No. | Type | Topic Name |
|-----|------|------------|
| 1 | Pose | `/vr_left_controller/pose` |
| 2 | Pose | `/vr_right_controller/pose` |
| 3 | Trigger | `/vr_left_controller/trigger` |
| 4 | Trigger | `/vr_right_controller/trigger` |
| 5 | Grip | `/vr_left_controller/grip` |
| 6 | Grip | `/vr_right_controller/grip` |
| 7 | Rate | `/vr_left_controller/rate` |
| 8 | Rate | `/vr_right_controller/rate` |
| 9 | Button | `vr_right_controller/button_a` |
| 10 | Button | `vr_right_controller/button_b` |
| 11 | Button | `vr_left_controller/button_x` |
| 12 | Button | `vr_left_controller/button_y` |

## 6. Common Parameters (Application Level)

| Parameter | Description | Default |
|-----------|-------------|---------|
| `listen_address` | Listening address | `0.0.0.0` |
| `listen_port` | Listening port | `5100` |
| `publish_tf` | Whether to publish TF | `false` |
| `frame_id` | Parent frame for published poses | `vr_hmd` |

Example (change port and enable TF):

```bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node \
  --ros-args -p listen_port:=5101 -p publish_tf:=true
```

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, see the [LICENSE_CN.md](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Authors

- **Zhang Li**
- Company: Chengdu Changshu Robot Co., Ltd.
- Website: https://openarmx.com/

## Version

**Current Version**: 1.0.0

---

## Contact Us

### Chengdu Changshu Robotics Co., Ltd.

| Contact Method | Information |
|---------------|-------------|
| Email | openarmrobot@gmail.com |
| Phone/WeChat | +86-17746530375 |
| Website | <https://openarmx.com/> |
| Address | Huacheng Machinery Factory, No. 11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| Contact Person | Mr. Wang |
