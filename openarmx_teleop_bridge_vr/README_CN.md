# openarmx_teleop_bridge_vr

## 1. 这个包是做什么的

`openarmx_teleop_bridge_vr` 是一个 ROS 2 桥接包。  
它接收 Pico VR 控制器通过 UDP 发送的数据，并发布为 ROS 2 话题（可选发布 TF），方便下游遥操作或控制节点直接使用。

一句话：**把 Pico 手柄数据接进 ROS 2。**

## 2. 包结构

```text
openarmx_teleop_bridge_vr/
├── README_CN.md   
├── README.md                                  
├── CMakeLists.txt
├── package.xml
└── src/
    └── openarmx_teleop_bridge_vr_node.cpp
```

## 3. 应用层数据流

Pico/OpenXR 发送端  
-> UDP 数据（默认端口 `5100`）  
-> `openarmx_teleop_bridge_vr_node`  
-> ROS 2 话题（pose/trigger/grip/button/rate）  
-> 你的 teleop 或控制节点

## 4. 快速使用

### 构建

```bash
cd <你的工作空间>
colcon build --packages-select openarmx_teleop_bridge_vr
```

### 运行

```bash
source install/setup.bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

### 检查是否有数据

```bash
ros2 topic echo /pico_left_controller/pose
ros2 topic echo /pico_right_controller/pose
```

## 5. 默认发布的话题

| 序号 | 类型 | 话题名 |
|------|------|--------|
| 1 | 位姿 | `/pico_left_controller/pose` |
| 2 | 位姿 | `/pico_right_controller/pose` |
| 3 | 扳机 | `/pico_left_controller/trigger` |
| 4 | 扳机 | `/pico_right_controller/trigger` |
| 5 | 握把 | `/pico_left_controller/grip` |
| 6 | 握把 | `/pico_right_controller/grip` |
| 7 | 倍率 | `/pico_left_controller/rate` |
| 8 | 倍率 | `/pico_right_controller/rate` |
| 9 | 按键 | `pico_right_controller/button_a` |
| 10 | 按键 | `pico_right_controller/button_b` |
| 11 | 按键 | `pico_left_controller/button_x` |
| 12 | 按键 | `pico_left_controller/button_y` |

## 6. 常用参数（应用层）

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `listen_address` | 监听地址 | `0.0.0.0` |
| `listen_port` | 监听端口 | `5100` |
| `publish_tf` | 是否发布 TF | `false` |
| `frame_id` | 发布姿态时的父坐标系 | `pico_hmd` |

示例（修改端口并开启 TF）：

```bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node \
  --ros-args -p listen_port:=5101 -p publish_tf:=true
```

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
