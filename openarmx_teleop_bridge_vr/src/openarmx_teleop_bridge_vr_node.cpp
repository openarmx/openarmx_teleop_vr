// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
// 4.0 International License (CC BY-NC-SA 4.0).
//
// To view a copy of this license, visit:
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxDatagramSize = 512;

enum HandIndex { LEFT = 0, RIGHT = 1, HAND_COUNT = 2 };

struct PoseSample {
  HandIndex hand = LEFT;
  double position[3]{0.0, 0.0, 0.0};
  double orientation[4]{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  double trigger_value = 0.0;  // 食指扳机值 (0-1)
  double grip_value = 0.0;     // 握把扳机值 (0-1)
  bool button_a = false;       // A键状态（右手柄）
  bool button_b = false;       // B键状态（右手柄）
  bool button_x = false;       // X键状态（左手柄）
  bool button_y = false;       // Y键状态（左手柄）
  double rate = 0.1;           // 倍率（0.1或1.0）
  int64_t timestamp_ns = 0;
};

bool stringEqualsIgnoreCase(const std::string &a, const std::string &b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (std::tolower(a[i]) != std::tolower(b[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

class PoseBridgeNode : public rclcpp::Node {
 public:
  PoseBridgeNode() : Node("openarmx_teleop_bridge_vr_node"), running_(true) {
    listen_address_ = declare_parameter<std::string>("listen_address", "0.0.0.0");
    listen_port_ = declare_parameter<int>("listen_port", 5100);
    frame_id_ = declare_parameter<std::string>("frame_id", "vr_hmd");
    child_frame_ids_[LEFT] =
        declare_parameter<std::string>("left_child_frame_id", "vr_left_controller");
    child_frame_ids_[RIGHT] =
        declare_parameter<std::string>("right_child_frame_id", "vr_right_controller");
    pose_topics_[LEFT] =
        declare_parameter<std::string>("left_pose_topic", "/vr_left_controller/pose");
    pose_topics_[RIGHT] =
        declare_parameter<std::string>("right_pose_topic", "/vr_right_controller/pose");
    trigger_topics_[LEFT] =
        declare_parameter<std::string>("left_trigger_topic", "/vr_left_controller/trigger");
    trigger_topics_[RIGHT] =
        declare_parameter<std::string>("right_trigger_topic", "/vr_right_controller/trigger");
    grip_topics_[LEFT] =  declare_parameter<std::string>("left_grip_topic", "/vr_left_controller/grip");
    grip_topics_[RIGHT] = declare_parameter<std::string>("right_grip_topic", "/vr_right_controller/grip");
    button_a_topic_ = declare_parameter<std::string>("button_a_topic", "vr_right_controller/button_a");
    button_b_topic_ = declare_parameter<std::string>("button_b_topic", "vr_right_controller/button_b");
    button_x_topic_ = declare_parameter<std::string>("button_x_topic", "vr_left_controller/button_x");
    button_y_topic_ = declare_parameter<std::string>("button_y_topic", "vr_left_controller/button_y");
    // Use absolute topic names by default so they appear as /vr_* instead of being prefixed by the node name
    rate_topics_[LEFT] = declare_parameter<std::string>("left_rate_topic", "/vr_left_controller/rate");
    rate_topics_[RIGHT] = declare_parameter<std::string>("right_rate_topic", "/vr_right_controller/rate");
    RCLCPP_INFO(get_logger(), "DEBUG: Rate topics - LEFT: '%s', RIGHT: '%s'", 
                rate_topics_[LEFT].c_str(), rate_topics_[RIGHT].c_str());
    // 默认只发送左右手姿态四元数、扳机和按键，不再发布 TF（如需 TF，可通过参数开启）
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    for (int i = 0; i < HAND_COUNT; ++i) {
      // 仅发布位姿（位置 + 四元数）、扳机，不再发布欧拉角 rpy
      pose_publishers_[i] = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topics_[i], 10);
      trigger_publishers_[i] =
          create_publisher<std_msgs::msg::Float32>(trigger_topics_[i], 10);
      grip_publishers_[i] =
          create_publisher<std_msgs::msg::Float32>(grip_topics_[i], 10);
      RCLCPP_INFO(get_logger(), "DEBUG: Creating rate publisher for '%s'", rate_topics_[i].c_str());
      rate_publishers_[i] = create_publisher<std_msgs::msg::Float32>(rate_topics_[i], 10);
      // 检查是否创建成功
    if (rate_publishers_[i]) {
        RCLCPP_INFO(get_logger(), "DEBUG: ✓ Rate publisher created successfully");
    } else {
        RCLCPP_ERROR(get_logger(), "DEBUG: ✗ Rate publisher creation FAILED!");
    }
}
    button_a_publisher_ = create_publisher<std_msgs::msg::Bool>(button_a_topic_, 10);
    button_b_publisher_ = create_publisher<std_msgs::msg::Bool>(button_b_topic_, 10);
    button_x_publisher_ = create_publisher<std_msgs::msg::Bool>(button_x_topic_, 10);
    button_y_publisher_ = create_publisher<std_msgs::msg::Bool>(button_y_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    receiver_thread_ = std::thread(&PoseBridgeNode::receiverLoop, this);
    
    // 添加调试输出
    RCLCPP_INFO(get_logger(), "========== Publisher Creation Summary ==========");
    for (int i = 0; i < HAND_COUNT; ++i) {
        std::string hand_name = (i == LEFT) ? "LEFT" : "RIGHT";
        RCLCPP_INFO(get_logger(), "%s Hand:", hand_name.c_str());
        RCLCPP_INFO(get_logger(), "  Pose: %s - %s", 
                    pose_topics_[i].c_str(), 
                    pose_publishers_[i] ? "CREATED" : "FAILED");
        RCLCPP_INFO(get_logger(), "  Rate: %s - %s", 
                    rate_topics_[i].c_str(),
                    rate_publishers_[i] ? "CREATED" : "FAILED");
    }
    RCLCPP_INFO(get_logger(), "================================================");
  }

  ~PoseBridgeNode() override {
    running_.store(false);
    if (socket_fd_ >= 0) {
      ::shutdown(socket_fd_, SHUT_RDWR);
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }

 private:
  void receiverLoop() {
    if (!openSocket()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize UDP socket. Receiver thread exiting.");
      return;
    }

    while (rclcpp::ok() && running_.load()) {
      sockaddr_in remote_addr{};
      socklen_t addr_len = sizeof(remote_addr);
      char buffer[kMaxDatagramSize];
      const ssize_t received = ::recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                          reinterpret_cast<sockaddr *>(&remote_addr), &addr_len);

      if (received < 0) {
        if (!running_.load()) {
          break;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "recvfrom failed: %s", std::strerror(errno));
        continue;
      }

      buffer[received] = '\0';
      PoseSample sample;
      if (!parseDatagram(std::string(buffer, received), sample)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Failed to parse datagram: '%s'", buffer);
        continue;
      }
      
      // 调试：显示原始接收到的位置数据（每 60 帧一次）
      static int raw_frame_count = 0;
      raw_frame_count++;
      if (raw_frame_count % 60 == 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                            "RAW UDP data - Hand: %s, pos=(%.4f,%.4f,%.4f) (no transform applied)",
                            sample.hand == LEFT ? "LEFT" : "RIGHT",
                            sample.position[0], sample.position[1], sample.position[2]);
      }
      
      publishSample(sample);
    }
  }

  bool openSocket() {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Unable to create UDP socket: %s", std::strerror(errno));
      return false;
    }

    const int reuse = 1;
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      RCLCPP_WARN(get_logger(), "Failed to set SO_REUSEADDR: %s", std::strerror(errno));
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
      RCLCPP_WARN(get_logger(), "Failed to set SO_RCVTIMEO: %s", std::strerror(errno));
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(listen_port_));
    if (listen_address_ == "0.0.0.0") {
      addr.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, listen_address_.c_str(), &addr.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid listen_address '%s'", listen_address_.c_str());
      return false;
    }

    if (::bind(socket_fd_, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind UDP socket to %s:%d -> %s",
                   listen_address_.c_str(), listen_port_, std::strerror(errno));
      return false;
    }

    RCLCPP_INFO(get_logger(), "Listening for controller poses on %s:%d",
                listen_address_.c_str(), listen_port_);
    return true;
  }

  bool parseDatagram(const std::string &payload, PoseSample &out_sample) const {
    std::istringstream iss(payload);
    std::string hand_token;

    if (!(iss >> hand_token)) {
      return false;
    }

    if (stringEqualsIgnoreCase(hand_token, "L") ||
        stringEqualsIgnoreCase(hand_token, "LEFT")) {
      out_sample.hand = LEFT;
    } else if (stringEqualsIgnoreCase(hand_token, "R") ||
               stringEqualsIgnoreCase(hand_token, "RIGHT")) {
      out_sample.hand = RIGHT;
    } else {
      return false;
    }

    for (double &component : out_sample.position) {
      if (!(iss >> component)) {
        return false;
      }
    }
    for (double &component : out_sample.orientation) {
      if (!(iss >> component)) {
        return false;
      }
    }

    // 解析扳机值（如果存在）
    if (!(iss >> out_sample.trigger_value)) {
      out_sample.trigger_value = 0.0;
    }
    if (!(iss >> out_sample.grip_value)) {
      out_sample.grip_value = 0.0;
    }

    // 解析按键状态（如果存在）
    int button_a_int = 0, button_b_int = 0, button_x_int = 0, button_y_int = 0;
    if (!(iss >> button_a_int)) {
      button_a_int = 0;
    }
    if (!(iss >> button_b_int)) {
      button_b_int = 0;
    }
    if (!(iss >> button_x_int)) {
      button_x_int = 0;
    }
    if (!(iss >> button_y_int)) {
      button_y_int = 0;
    }
    out_sample.button_a = (button_a_int != 0);
    out_sample.button_b = (button_b_int != 0);
    out_sample.button_x = (button_x_int != 0);
    out_sample.button_y = (button_y_int != 0);
    // 解析倍率和时间戳
    double temp_value;
    if (!(iss >> temp_value)) {
        out_sample.rate = 0.1;
        out_sample.timestamp_ns = 0;
    } else {
    // 严格验证：rate只能是0.1或1.0，其他都视为时间戳
    if (temp_value == 0.1 || temp_value == 1.0) {
        out_sample.rate = temp_value;
        // 继续解析时间戳
        if (!(iss >> out_sample.timestamp_ns)) {
            out_sample.timestamp_ns = 0;
        }
    } else {
        // 不是有效的rate，可能是时间戳或错误数据
        out_sample.rate = 0.1;  // 强制设为安全值
        out_sample.timestamp_ns = static_cast<int64_t>(temp_value);
        
        // 记录警告
        RCLCPP_WARN(get_logger(), "Invalid rate value detected: %.2f, resetting to 0.1", temp_value);
        }
    }

    // 调试：显示解析结果
    RCLCPP_INFO(get_logger(), "DEBUG: Parsed - hand=%s, rate=%.2f, timestamp=%ld, payload=%s",
                out_sample.hand == LEFT ? "LEFT" : "RIGHT",
                out_sample.rate,
                out_sample.timestamp_ns,
            payload.substr(0, 30).c_str());

    RCLCPP_INFO(get_logger(), "DEBUG: Parsed trigger=%.3f grip=%.3f buttons[A=%d B=%d X=%d Y=%d]",
                out_sample.trigger_value,
                out_sample.grip_value,
                out_sample.button_a ? 1 : 0,
                out_sample.button_b ? 1 : 0,
                out_sample.button_x ? 1 : 0,
                out_sample.button_y ? 1 : 0);

    return true;
  }

  void publishSample(const PoseSample &sample) {
     // 调试：显示发布信息
     RCLCPP_INFO(get_logger(), "DEBUG: Publishing - hand=%s, rate=%.2f, rate_publishers valid: L=%d R=%d",
              sample.hand == LEFT ? "LEFT" : "RIGHT",
              sample.rate,
              rate_publishers_[LEFT] ? 1 : 0,
              rate_publishers_[RIGHT] ? 1 : 0);
    const rclcpp::Time stamp =
        sample.timestamp_ns > 0 ? rclcpp::Time(sample.timestamp_ns) : now();
	
    // 使用系统默认坐标系，不进行转换
    // 直接使用原始数据：OpenXR 手柄坐标系
    //   x：向右为正
    //   y：向上为正
    //   z：向前为正
    const double pos_x = sample.position[0];
    const double pos_y = sample.position[1];
    const double pos_z = sample.position[2];

    // 直接使用原始四元数，不进行转换
    const double q_x = sample.orientation[0];
    const double q_y = sample.orientation[1];
    const double q_z = sample.orientation[2];
    const double q_w = sample.orientation[3];

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = pos_x;
    pose_msg.pose.position.y = pos_y;
    pose_msg.pose.position.z = pos_z;
    pose_msg.pose.orientation.x = q_x;
    pose_msg.pose.orientation.y = q_y;
    pose_msg.pose.orientation.z = q_z;
    pose_msg.pose.orientation.w = q_w;

    pose_publishers_[sample.hand]->publish(pose_msg);

    // 调试输出：每 20 帧输出一次位置和姿态信息（更频繁，便于观察）
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 20 == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                          "Controller %s (raw): pos=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f)",
                          sample.hand == LEFT ? "LEFT" : "RIGHT",
                          pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
                          pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                          pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
    }

    // 发布扳机状态
    auto trigger_msg = std_msgs::msg::Float32();
    trigger_msg.data = static_cast<float>(sample.trigger_value);
    trigger_publishers_[sample.hand]->publish(trigger_msg);

    // 按需求，仅保留扳机（trigger）；如需握把值（grip），可取消下面代码注释
    auto grip_msg = std_msgs::msg::Float32();
    grip_msg.data = static_cast<float>(sample.grip_value);
    grip_publishers_[sample.hand]->publish(grip_msg);
    
    // 发布倍率（两个手柄都发布相同的rate值）
    auto rate_msg = std_msgs::msg::Float32();
    rate_msg.data = static_cast<float>(sample.rate);
    RCLCPP_INFO(get_logger(), "DEBUG: Publishing rate=%.2f to both hands", sample.rate);
    rate_publishers_[LEFT]->publish(rate_msg);   // 左手rate
    rate_publishers_[RIGHT]->publish(rate_msg);  // 右手rate
    
    // 发布按键状态（根据手柄类型只发布对应的按键）
    if (sample.hand == RIGHT) {
      // 右手柄：只发布 A 和 B 按键
      static bool last_a = false, last_b = false;
      if (sample.button_a != last_a || sample.button_b != last_b) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Right hand button state changed: A=%d B=%d",
                            sample.button_a ? 1 : 0, sample.button_b ? 1 : 0);
        last_a = sample.button_a;
        last_b = sample.button_b;
      }
      
      auto button_a_msg = std_msgs::msg::Bool();
      button_a_msg.data = sample.button_a;
      button_a_publisher_->publish(button_a_msg);

      auto button_b_msg = std_msgs::msg::Bool();
      button_b_msg.data = sample.button_b;
      button_b_publisher_->publish(button_b_msg);
    } else {
      // 左手柄：只发布 X 和 Y 按键
      static bool last_x = false, last_y = false;
      if (sample.button_x != last_x || sample.button_y != last_y) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Left hand button state changed: X=%d Y=%d",
                            sample.button_x ? 1 : 0, sample.button_y ? 1 : 0);
        last_x = sample.button_x;
        last_y = sample.button_y;
      }
      
      auto button_x_msg = std_msgs::msg::Bool();
      button_x_msg.data = sample.button_x;
      button_x_publisher_->publish(button_x_msg);

      auto button_y_msg = std_msgs::msg::Bool();
      button_y_msg.data = sample.button_y;
      button_y_publisher_->publish(button_y_msg);
    }

    if (publish_tf_) {
      // 原有 TF：父坐标系为 frame_id_（默认 vr_hmd），子坐标系为手柄自身坐标系
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = pose_msg.header;
      tf_msg.child_frame_id = child_frame_ids_[sample.hand];
      tf_msg.transform.translation.x = pose_msg.pose.position.x;
      tf_msg.transform.translation.y = pose_msg.pose.position.y;
      tf_msg.transform.translation.z = pose_msg.pose.position.z;
      tf_msg.transform.rotation = pose_msg.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);

      // 新增 TF：以 world 作为父坐标系，发布左右手相对 world 的运动量
      geometry_msgs::msg::TransformStamped world_tf_msg;
      world_tf_msg.header.stamp = stamp;
      world_tf_msg.header.frame_id = "world";
      world_tf_msg.child_frame_id =
          (sample.hand == LEFT) ? "left_ee_from_world" : "right_ee_from_world";
      world_tf_msg.transform.translation.x = pos_x;
      world_tf_msg.transform.translation.y = pos_y;
      world_tf_msg.transform.translation.z = pos_z;
      world_tf_msg.transform.rotation.x = q_x;
      world_tf_msg.transform.rotation.y = q_y;
      world_tf_msg.transform.rotation.z = q_z;
      world_tf_msg.transform.rotation.w = q_w;
      tf_broadcaster_->sendTransform(world_tf_msg);
    }
  }

  std::string listen_address_;
  int listen_port_{5100};
  std::string frame_id_;
  std::array<std::string, HAND_COUNT> child_frame_ids_;
  std::array<std::string, HAND_COUNT> pose_topics_;
  std::array<std::string, HAND_COUNT> trigger_topics_;
  std::array<std::string, HAND_COUNT> grip_topics_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, HAND_COUNT>
      pose_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      trigger_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      grip_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      rate_publishers_;  // 添加这行
  std::string button_a_topic_;
  std::string button_b_topic_;
  std::string button_x_topic_;
  std::string button_y_topic_;
  std::array<std::string, HAND_COUNT> rate_topics_;  // 添加这行
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_a_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_b_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_x_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_y_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_{true};

  std::thread receiver_thread_;
  std::atomic<bool> running_;
  int socket_fd_{-1};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
