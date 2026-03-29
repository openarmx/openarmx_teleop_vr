# openarmx_teleop_vr 使用说明（中文）

## 1. 包定位

`openarmx_teleop_vr` 是 OpenArmX 的 VR 遥操作执行节点。
该节点订阅 VR 手柄的位姿、扳机、握把等 ROS 话题，并向双臂控制器发布关节命令，实现”VR 手柄 -> 机器人双臂”的在线遥操作。

关于 VR 设备遥操作的图文教程，请访问官方文档：<http://docs.openarmx.com>

## 2. 包结构

```text
openarmx_teleop_vr/
├── README_CN.md
├── README.md
├── launch/
│   └── teleop_vr.launch.py         # 启动入口
├── openarmx_teleop_vr/
│   └── openarmx_teleop_vr_node.py  # 主节点
├── package.xml
└── setup.py
```

## 3. 系统链路

整体数据流如下：

1. VR 设备通过桥接包发布控制器数据（`openarmx_teleop_bridge_vr`）。
2. 本节点订阅输入话题，计算左右臂关节控制命令。
3. 控制命令发布到 `forward_position_controller`，驱动仿真或实机。

## 4. 主要功能

1. 使用双手柄位姿控制双臂末端运动。
2. 使用食指扳机控制左右夹爪开合。
3. 使用握把作为使能开关（deadman switch），降低误操作风险。
4. 支持速度档位切换（慢速/快速），减少大幅跳变带来的安全风险。
5. 可选发布可视化 TF，便于 RViz 联调。

## 5. 快速启动

### 前置条件

1. 机器人底层（仿真或实机）已启动，且 `forward_position_controller` 可用。
2. VR 桥接节点已启动，并持续发布手柄话题。
3. 运行环境中可导入 `openarmx_arm_driver`（本节点必需依赖）。

### 典型启动顺序

1. 终端 1：启动机器人

```bash
cd <你的工作空间>
source install/setup.bash

# 仿真模式
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true

# 实机模式：先启动 CAN 通道
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

2. 终端 2：启动 VR 桥接

```bash
cd <你的工作空间>
source install/setup.bash

ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

3. 终端 3：启动本包

```bash
cd <你的工作空间>
source install/setup.bash

ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 6. 输入与输出话题

### 输入话题（默认）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vr_left_controller/pose` | `geometry_msgs/PoseStamped` | 左手柄位姿输入 |
| `/vr_right_controller/pose` | `geometry_msgs/PoseStamped` | 右手柄位姿输入 |
| `/vr_left_controller/trigger` | `std_msgs/Float32` | 左扳机（夹爪） |
| `/vr_right_controller/trigger` | `std_msgs/Float32` | 右扳机（夹爪） |
| `/vr_left_controller/grip` | `std_msgs/Float32` | 左握把（使能） |
| `/vr_right_controller/grip` | `std_msgs/Float32` | 右握把（使能） |
| `/vr_right_controller/rate` | `std_msgs/Float32` | 速度模式输入（0.1/1.0） |
| `/joint_states` | `sensor_msgs/JointState` | 当前关节状态反馈 |

### 输出话题（默认）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 左臂关节命令 |
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 右臂关节命令 |

## 7. 常用参数（应用层）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_rate` | `100.0` | 控制循环频率（Hz） |
| `rate_topic` | `/vr_right_controller/rate` | 速度档位输入话题 |
| `slow_max_step_deg` | `1.0` | 慢速模式每周期最大关节步进 |
| `fast_max_step_deg` | `12.0` | 快速模式每周期最大关节步进 |
| `ik_iterations` | `3` | 每周期逆解迭代次数 |
| `grip_threshold` | `0.5` | 握把使能阈值 |
| `left_grip_topic` | `/vr_left_controller/grip` | 左握把话题 |
| `right_grip_topic` | `/vr_right_controller/grip` | 右握把话题 |
| `publish_visualization_tf` | `true` | 是否发布可视化 TF |
| `print_performance` | `false` | 是否输出性能日志 |
| `use_xacro` | `true` | 运行时是否由 xacro 生成 URDF |
| `use_link4_ext` | `true` | 是否启用扩展约束帧配置 |
| `constraint_mode` | `joint` | 约束模式：`joint` 或 `link` |

示例：

```bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  constraint_mode:=link
```

注意：机械臂在接近或超出工作边界时可能出现颤抖。这通常是因为求解器仍在尝试逼近不可达目标。为保障安全，请避免极限位姿操作。

`constraint_mode` 模式说明：

1. `joint`（默认）：响应更快，但逆解缺少额外姿态约束，可能出现不符合直觉的关节姿态。
2. `link`：会在第二、第四关节引入附加约束，手臂姿态更靠近机身中心、更适合胸前作业区。

请根据任务需求在两种模式间切换。

## 8. 常见问题

1. 腕部关节响应较慢

默认参数对关节步长有较严格限制，以降低手臂突变带来的风险。可适当增大 `slow_max_step_deg` 和 `fast_max_step_deg` 提升响应速度，但需自行评估安全风险。

2. 节点已启动但机械臂不动

先检查桥接话题是否有数据：

```bash
ros2 topic echo /vr_left_controller/pose
ros2 topic echo /vr_right_controller/pose
```

3. 有手柄数据但没有控制命令

检查控制器命令话题：

```bash
ros2 topic echo /left_forward_position_controller/commands
ros2 topic echo /right_forward_position_controller/commands
```

4. 报错：`openarmx_arm_driver` 导入失败

说明当前环境缺少该 Python 依赖，请先安装并确认环境已正确 `source`。

5. 夹爪不响应

请检查扳机话题数据，以及握把阈值（`grip_threshold`）是否匹配实际操作习惯。

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **Zhang Li** (张力)
- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：1.0.0

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
