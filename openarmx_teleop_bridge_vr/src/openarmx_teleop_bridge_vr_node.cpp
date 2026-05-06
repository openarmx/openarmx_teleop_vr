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
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxDatagramSize = 512;

enum HandIndex { LEFT = 0, RIGHT = 1, HAND_COUNT = 2 };
enum PacketKind {
  RELATIVE_HAND = 0,
  ABSOLUTE_HAND = 1,
  HEAD = 2,
  MODE = 3,
  CALIBRATE_DONE = 4,
};

struct PoseSample {
  PacketKind kind = RELATIVE_HAND;
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
  std::string control_mode{};
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
    frame_id_ = declare_parameter<std::string>("frame_id", "pico_hmd");
    child_frame_ids_[LEFT] =
        declare_parameter<std::string>("left_child_frame_id", "pico_left_controller");
    child_frame_ids_[RIGHT] =
        declare_parameter<std::string>("right_child_frame_id", "pico_right_controller");
    pose_topics_[LEFT] =
        declare_parameter<std::string>("left_pose_topic", "/pico_left_controller/pose");
    pose_topics_[RIGHT] =
        declare_parameter<std::string>("right_pose_topic", "/pico_right_controller/pose");
    absolute_pose_topics_[LEFT] = declare_parameter<std::string>(
        "left_absolute_pose_topic", "/vr/left/pose_absolute");
    absolute_pose_topics_[RIGHT] = declare_parameter<std::string>(
        "right_absolute_pose_topic", "/vr/right/pose_absolute");
    head_pose_topic_ = declare_parameter<std::string>("head_pose_topic", "/vr/head/pose");
    control_mode_topic_ = declare_parameter<std::string>(
        "control_mode_topic", "/vr/control_mode");
    calibrate_done_topic_ = declare_parameter<std::string>(
        "calibrate_done_topic", "/vr/calibrate_done");
    absolute_trigger_topics_[LEFT] = declare_parameter<std::string>(
        "left_absolute_trigger_topic", "/vr/left/trigger");
    absolute_trigger_topics_[RIGHT] = declare_parameter<std::string>(
        "right_absolute_trigger_topic", "/vr/right/trigger");
    absolute_grip_topics_[LEFT] = declare_parameter<std::string>(
        "left_absolute_grip_topic", "/vr/left/grip");
    absolute_grip_topics_[RIGHT] = declare_parameter<std::string>(
        "right_absolute_grip_topic", "/vr/right/grip");
    absolute_button_a_topic_ = declare_parameter<std::string>(
        "absolute_button_a_topic", "/vr/right/button_a");
    absolute_button_b_topic_ = declare_parameter<std::string>(
        "absolute_button_b_topic", "/vr/right/button_b");
    absolute_button_x_topic_ = declare_parameter<std::string>(
        "absolute_button_x_topic", "/vr/left/button_x");
    absolute_button_y_topic_ = declare_parameter<std::string>(
        "absolute_button_y_topic", "/vr/left/button_y");
    absolute_rate_topic_ = declare_parameter<std::string>(
        "absolute_rate_topic", "/vr/rate");
    trigger_topics_[LEFT] =
        declare_parameter<std::string>("left_trigger_topic", "/pico_left_controller/trigger");
    trigger_topics_[RIGHT] =
        declare_parameter<std::string>("right_trigger_topic", "/pico_right_controller/trigger");
    grip_topics_[LEFT] =  declare_parameter<std::string>("left_grip_topic", "/pico_left_controller/grip");
    grip_topics_[RIGHT] = declare_parameter<std::string>("right_grip_topic", "/pico_right_controller/grip");
    button_a_topic_ = declare_parameter<std::string>("button_a_topic", "pico_right_controller/button_a");
    button_b_topic_ = declare_parameter<std::string>("button_b_topic", "pico_right_controller/button_b");
    button_x_topic_ = declare_parameter<std::string>("button_x_topic", "pico_left_controller/button_x");
    button_y_topic_ = declare_parameter<std::string>("button_y_topic", "pico_left_controller/button_y");
    // Use absolute topic names by default so they appear as /pico_* instead of being prefixed by the node name
    rate_topics_[LEFT] = declare_parameter<std::string>("left_rate_topic", "/pico_left_controller/rate");
    rate_topics_[RIGHT] = declare_parameter<std::string>("right_rate_topic", "/pico_right_controller/rate");
    RCLCPP_INFO(get_logger(), "DEBUG: Rate topics - LEFT: '%s', RIGHT: '%s'", 
                rate_topics_[LEFT].c_str(), rate_topics_[RIGHT].c_str());
    // 默认只发送左右手姿态四元数、扳机和按键，不再发布 TF（如需 TF，可通过参数开启）
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    for (int i = 0; i < HAND_COUNT; ++i) {
      // 仅发布位姿（位置 + 四元数）、扳机，不再发布欧拉角 rpy
      pose_publishers_[i] = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topics_[i], 10);
      absolute_pose_publishers_[i] =
          create_publisher<geometry_msgs::msg::PoseStamped>(absolute_pose_topics_[i], 10);
      trigger_publishers_[i] =
          create_publisher<std_msgs::msg::Float32>(trigger_topics_[i], 10);
      absolute_trigger_publishers_[i] =
          create_publisher<std_msgs::msg::Float32>(absolute_trigger_topics_[i], 10);
      grip_publishers_[i] =
          create_publisher<std_msgs::msg::Float32>(grip_topics_[i], 10);
      absolute_grip_publishers_[i] =
          create_publisher<std_msgs::msg::Float32>(absolute_grip_topics_[i], 10);
      RCLCPP_INFO(get_logger(), "DEBUG: Creating rate publisher for '%s'", rate_topics_[i].c_str());
      rate_publishers_[i] = create_publisher<std_msgs::msg::Float32>(rate_topics_[i], 10);
      // 检查是否创建成功
    if (rate_publishers_[i]) {
        RCLCPP_INFO(get_logger(), "DEBUG: ✓ Rate publisher created successfully");
    } else {
        RCLCPP_ERROR(get_logger(), "DEBUG: ✗ Rate publisher creation FAILED!");
    }
}
    head_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(head_pose_topic_, 10);
    control_mode_publisher_ =
        create_publisher<std_msgs::msg::String>(control_mode_topic_, 10);
    calibrate_done_publisher_ =
        create_publisher<std_msgs::msg::Bool>(calibrate_done_topic_, 10);
    absolute_rate_publisher_ =
        create_publisher<std_msgs::msg::Float32>(absolute_rate_topic_, 10);
    button_a_publisher_ = create_publisher<std_msgs::msg::Bool>(button_a_topic_, 10);
    button_b_publisher_ = create_publisher<std_msgs::msg::Bool>(button_b_topic_, 10);
    button_x_publisher_ = create_publisher<std_msgs::msg::Bool>(button_x_topic_, 10);
    button_y_publisher_ = create_publisher<std_msgs::msg::Bool>(button_y_topic_, 10);
    absolute_button_a_publisher_ =
        create_publisher<std_msgs::msg::Bool>(absolute_button_a_topic_, 10);
    absolute_button_b_publisher_ =
        create_publisher<std_msgs::msg::Bool>(absolute_button_b_topic_, 10);
    absolute_button_x_publisher_ =
        create_publisher<std_msgs::msg::Bool>(absolute_button_x_topic_, 10);
    absolute_button_y_publisher_ =
        create_publisher<std_msgs::msg::Bool>(absolute_button_y_topic_, 10);
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
    std::string token;

    if (!(iss >> token)) {
      return false;
    }

    if (stringEqualsIgnoreCase(token, "LEFT") ||
        stringEqualsIgnoreCase(token, "L")) {
      out_sample.kind = RELATIVE_HAND;
      out_sample.hand = LEFT;
      return parseHandPayload(iss, out_sample);
    }

    if (stringEqualsIgnoreCase(token, "RIGHT") ||
        stringEqualsIgnoreCase(token, "R")) {
      out_sample.kind = RELATIVE_HAND;
      out_sample.hand = RIGHT;
      return parseHandPayload(iss, out_sample);
    }

    if (stringEqualsIgnoreCase(token, "LEFT_ABS")) {
      out_sample.kind = ABSOLUTE_HAND;
      out_sample.hand = LEFT;
      return parseHandPayload(iss, out_sample);
    }

    if (stringEqualsIgnoreCase(token, "RIGHT_ABS")) {
      out_sample.kind = ABSOLUTE_HAND;
      out_sample.hand = RIGHT;
      return parseHandPayload(iss, out_sample);
    }

    if (stringEqualsIgnoreCase(token, "HEAD")) {
      out_sample.kind = HEAD;
      return parseHeadPayload(iss, out_sample);
    }

    if (stringEqualsIgnoreCase(token, "MODE")) {
      out_sample.kind = MODE;
      if (!(iss >> out_sample.control_mode)) {
        return false;
      }
      if (!(iss >> out_sample.timestamp_ns)) {
        out_sample.timestamp_ns = 0;
      }
      return true;
    }

    if (stringEqualsIgnoreCase(token, "CALIBRATE_DONE")) {
      out_sample.kind = CALIBRATE_DONE;
      if (!(iss >> out_sample.timestamp_ns)) {
        out_sample.timestamp_ns = 0;
      }
      return true;
    }

    return false;
  }

  bool parseHandPayload(std::istringstream &iss, PoseSample &out_sample) const {
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

    if (!(iss >> out_sample.trigger_value)) {
      out_sample.trigger_value = 0.0;
    }
    if (!(iss >> out_sample.grip_value)) {
      out_sample.grip_value = 0.0;
    }

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

    double temp_value;
    if (!(iss >> temp_value)) {
      out_sample.rate = 0.1;
      out_sample.timestamp_ns = 0;
    } else if (temp_value == 0.1 || temp_value == 1.0) {
      out_sample.rate = temp_value;
      if (!(iss >> out_sample.timestamp_ns)) {
        out_sample.timestamp_ns = 0;
      }
    } else {
      out_sample.rate = 0.1;
      out_sample.timestamp_ns = static_cast<int64_t>(temp_value);
      RCLCPP_WARN(
          get_logger(), "Invalid rate value detected: %.2f, resetting to 0.1",
          temp_value);
    }

    return true;
  }

  bool parseHeadPayload(std::istringstream &iss, PoseSample &out_sample) const {
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

    if (!(iss >> out_sample.timestamp_ns)) {
      out_sample.timestamp_ns = 0;
    }

    return true;
  }

  geometry_msgs::msg::PoseStamped makePoseMessage(const PoseSample &sample) const {
    const rclcpp::Time stamp =
        sample.timestamp_ns > 0 ? rclcpp::Time(sample.timestamp_ns) : now();

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = sample.position[0];
    pose_msg.pose.position.y = sample.position[1];
    pose_msg.pose.position.z = sample.position[2];
    pose_msg.pose.orientation.x = sample.orientation[0];
    pose_msg.pose.orientation.y = sample.orientation[1];
    pose_msg.pose.orientation.z = sample.orientation[2];
    pose_msg.pose.orientation.w = sample.orientation[3];
    return pose_msg;
  }

  void publishRelativeHandInputs(const PoseSample &sample) {
    auto trigger_msg = std_msgs::msg::Float32();
    trigger_msg.data = static_cast<float>(sample.trigger_value);
    trigger_publishers_[sample.hand]->publish(trigger_msg);

    auto grip_msg = std_msgs::msg::Float32();
    grip_msg.data = static_cast<float>(sample.grip_value);
    grip_publishers_[sample.hand]->publish(grip_msg);

    auto rate_msg = std_msgs::msg::Float32();
    rate_msg.data = static_cast<float>(sample.rate);
    rate_publishers_[LEFT]->publish(rate_msg);
    rate_publishers_[RIGHT]->publish(rate_msg);

    if (sample.hand == RIGHT) {
      auto button_a_msg = std_msgs::msg::Bool();
      button_a_msg.data = sample.button_a;
      button_a_publisher_->publish(button_a_msg);

      auto button_b_msg = std_msgs::msg::Bool();
      button_b_msg.data = sample.button_b;
      button_b_publisher_->publish(button_b_msg);
    } else {
      auto button_x_msg = std_msgs::msg::Bool();
      button_x_msg.data = sample.button_x;
      button_x_publisher_->publish(button_x_msg);

      auto button_y_msg = std_msgs::msg::Bool();
      button_y_msg.data = sample.button_y;
      button_y_publisher_->publish(button_y_msg);
    }
  }

  void publishAbsoluteHandInputs(const PoseSample &sample) {
    auto trigger_msg = std_msgs::msg::Float32();
    trigger_msg.data = static_cast<float>(sample.trigger_value);
    absolute_trigger_publishers_[sample.hand]->publish(trigger_msg);

    auto grip_msg = std_msgs::msg::Float32();
    grip_msg.data = static_cast<float>(sample.grip_value);
    absolute_grip_publishers_[sample.hand]->publish(grip_msg);

    auto rate_msg = std_msgs::msg::Float32();
    rate_msg.data = static_cast<float>(sample.rate);
    absolute_rate_publisher_->publish(rate_msg);

    if (sample.hand == RIGHT) {
      auto button_a_msg = std_msgs::msg::Bool();
      button_a_msg.data = sample.button_a;
      absolute_button_a_publisher_->publish(button_a_msg);

      auto button_b_msg = std_msgs::msg::Bool();
      button_b_msg.data = sample.button_b;
      absolute_button_b_publisher_->publish(button_b_msg);
    } else {
      auto button_x_msg = std_msgs::msg::Bool();
      button_x_msg.data = sample.button_x;
      absolute_button_x_publisher_->publish(button_x_msg);

      auto button_y_msg = std_msgs::msg::Bool();
      button_y_msg.data = sample.button_y;
      absolute_button_y_publisher_->publish(button_y_msg);
    }
  }

  void publishHandTf(const geometry_msgs::msg::PoseStamped &pose_msg,
                     HandIndex hand) {
    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = pose_msg.header;
    tf_msg.child_frame_id = child_frame_ids_[hand];
    tf_msg.transform.translation.x = pose_msg.pose.position.x;
    tf_msg.transform.translation.y = pose_msg.pose.position.y;
    tf_msg.transform.translation.z = pose_msg.pose.position.z;
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishSample(const PoseSample &sample) {
    switch (sample.kind) {
      case RELATIVE_HAND:
      case ABSOLUTE_HAND: {
        if (sample.kind == ABSOLUTE_HAND && !absolute_packet_seen_[sample.hand]) {
          absolute_packet_seen_[sample.hand] = true;
          RCLCPP_INFO(
              get_logger(), "First absolute hand packet received: %s",
              sample.hand == LEFT ? "LEFT_ABS" : "RIGHT_ABS");
        }
        auto pose_msg = makePoseMessage(sample);
        if (sample.kind == RELATIVE_HAND) {
          pose_publishers_[sample.hand]->publish(pose_msg);
          publishRelativeHandInputs(sample);
        } else {
          absolute_pose_publishers_[sample.hand]->publish(pose_msg);
          publishAbsoluteHandInputs(sample);
        }
        publishHandTf(pose_msg, sample.hand);
        break;
      }
      case HEAD: {
        if (!head_packet_seen_) {
          head_packet_seen_ = true;
          RCLCPP_INFO(get_logger(), "First HEAD packet received");
        }
        auto pose_msg = makePoseMessage(sample);
        head_pose_publisher_->publish(pose_msg);
        break;
      }
      case MODE: {
        RCLCPP_INFO(get_logger(), "Mode packet received: %s", sample.control_mode.c_str());
        auto mode_msg = std_msgs::msg::String();
        mode_msg.data = sample.control_mode;
        control_mode_publisher_->publish(mode_msg);
        break;
      }
      case CALIBRATE_DONE: {
        RCLCPP_INFO(get_logger(), "CALIBRATE_DONE packet received");
        auto calibrate_msg = std_msgs::msg::Bool();
        calibrate_msg.data = true;
        calibrate_done_publisher_->publish(calibrate_msg);
        break;
      }
    }
  }

  std::string listen_address_;
  int listen_port_{5100};
  std::string frame_id_;
  std::array<std::string, HAND_COUNT> child_frame_ids_;
  std::array<std::string, HAND_COUNT> pose_topics_;
  std::array<std::string, HAND_COUNT> absolute_pose_topics_;
  std::string head_pose_topic_;
  std::string control_mode_topic_;
  std::string calibrate_done_topic_;
  std::array<std::string, HAND_COUNT> absolute_trigger_topics_;
  std::array<std::string, HAND_COUNT> absolute_grip_topics_;
  std::string absolute_button_a_topic_;
  std::string absolute_button_b_topic_;
  std::string absolute_button_x_topic_;
  std::string absolute_button_y_topic_;
  std::string absolute_rate_topic_;
  std::array<std::string, HAND_COUNT> trigger_topics_;
  std::array<std::string, HAND_COUNT> grip_topics_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, HAND_COUNT>
      pose_publishers_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, HAND_COUNT>
      absolute_pose_publishers_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr head_pose_publisher_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      trigger_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      absolute_trigger_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      grip_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      absolute_grip_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, HAND_COUNT>
      rate_publishers_;  // 添加这行
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr absolute_rate_publisher_;
  std::string button_a_topic_;
  std::string button_b_topic_;
  std::string button_x_topic_;
  std::string button_y_topic_;
  std::array<std::string, HAND_COUNT> rate_topics_;  // 添加这行
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibrate_done_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_a_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_b_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_x_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_y_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr absolute_button_a_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr absolute_button_b_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr absolute_button_x_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr absolute_button_y_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_{true};
  std::array<bool, HAND_COUNT> absolute_packet_seen_{{false, false}};
  bool head_packet_seen_{false};

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
