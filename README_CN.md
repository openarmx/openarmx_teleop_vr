# openarmx_teleop_vr

这个目录是 OpenArmX 的 **VR 遥操作链路**，包含两个 ROS 2 包，组合后可实现：

`VR 手柄 UDP 数据 -> ROS 2 话题 -> 双臂关节控制命令`

<p align="center">
  <img src="assets/openarmx_teleop_vr.gif" alt="openarmx_teleop_vr 演示" />
</p>

> ⚠️ 推荐严格按“机器人底层 -> 桥接节点 -> 遥操作节点”的顺序启动，避免话题未就绪导致控制失败。

## ✅ 前提条件

在开始本包的 ROS 2 链路之前，请先在 VR 设备上安装好 VR 遥操作 App。  
该 App 的 APK 来源仓库：

- `https://github.com/openarmx/openarmx_teleop_vr_apk.git`

> ⚠️ 若 VR 设备未安装并正常运行该 App，桥接节点将无法收到有效的手柄 UDP 数据。

## 🗂️ 目录结构

```text
openarmx_teleop_vr/
├── openarmx_teleop_bridge_vr/   # C++ 桥接包：UDP -> ROS 2 话题/TF
├── openarmx_teleop_vr/          # Python 遥操作包：话题输入 -> IK -> 控制命令
├── LICENSE                           # 许可证
├── README_CN.md                      # 本简介（中文）
└── README_EN.md                      # This overview (English)
```

## 📦 两个子包分别做什么

1. `openarmx_teleop_bridge_vr`
- 接收 VR/OpenXR 侧 UDP 数据（默认监听端口 `5100`）。
- 发布手柄位姿、扳机、握把、速率等 ROS 2 话题（可选发布 TF）。

2. `openarmx_teleop_vr`
- 订阅桥接包发布的话题与 `/joint_states`。
- 进行 IK 计算和约束处理。
- 发布双臂控制命令到：
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

## 🚀 最短使用链路（建议顺序）

1. 启动机器人底层（确保 `forward_position_controller` 可用）  
2. 启动桥接节点  
3. 启动 VR 遥操作节点

> ✅ 三步全部启动后，再进行手柄操作更稳定。

示例：

```bash
cd <你的工作空间>
colcon build --packages-select openarmx_teleop_bridge_vr openarmx_teleop_vr
source install/setup.bash

# 终端 1：机器人控制（真机）
# 提示：当前版本启动重力补偿后效果更佳，建议保持开启状态。
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
    use_fake_hardware:=false

# 终端 2：VR 桥接
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node

# 终端 3：VR 遥操作执行节点
# 当前使用的 openarmx_arm_driver 库必须是 1.3.3 版本即以上
ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 🔎 快速检查

- 桥接输入是否正常：
  - `ros2 topic echo /vr_left_controller/pose`
  - `ros2 topic echo /vr_right_controller/pose`
- 遥操作输出是否正常：
  - `ros2 topic echo /left_forward_position_controller/commands`
  - `ros2 topic echo /right_forward_position_controller/commands`

> ⚠️ 若输入有数据但输出无命令，优先检查机器人控制器和节点启动顺序。

## 🧩 关键依赖（摘要）

- 桥接包：`rclcpp`、`geometry_msgs`、`tf2_ros`
- 遥操作包：`rclpy`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`tf2_ros`、`xacro`、`openarmx_description`
- 运行侧还需要 `openarmx_arm_driver`（详见子包文档）

## 📚 详细文档入口

- 桥接包中文说明：`openarmx_teleop_bridge_vr/README_CN.md`
- 遥操作包中文说明：`openarmx_teleop_vr/README_CN.md`
- 遥操作 launch 文件：`openarmx_teleop_vr/launch/teleop_vr.launch.py`


## 10. 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议（CC BY-NC-SA 4.0）进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅项目 LICENSE 文件或访问：<http://creativecommons.org/licenses/by-nc-sa/4.0/>

## 11. 作者

- **Zhang Li** (张力)
- 公司：Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站：<https://openarmx.com/>

## 12. 版本

**当前版本**：3.0.0

## 13. 联系我们

### 成都长数机器人有限公司

**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 邮箱 | openarmrobot@gmail.com |
| 电话/微信 | +86-17746530375 |
| 官网 | <https://openarmx.com/> |
| 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 联系人 | 王先生 |
