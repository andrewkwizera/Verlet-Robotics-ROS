// ROS 2 live teleoperation: physical can0 -> ROS joint states -> can1 position controller.
// No mock/synthetic leader and no direct CAN transmission from this node.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class YAMLiveBridge final : public rclcpp::Node {
public:
  YAMLiveBridge() : Node("yam_live_bridge") {
    actuation_authorized_ = declare_parameter<bool>("actuation_authorized", false);
    auto_track_ = declare_parameter<bool>("auto_track", false);
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { record(m, leader_); });
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/follower/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { record(m, follower_); });
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/yam_position_controller/commands", rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>("/yam_teleop/status", rclcpp::QoS(1));
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "/yam_teleop/enable_tracking",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if (!req->data) {
          disarm("Operator requested tracking disabled");
          auto_armed_once_ = true;
          res->success = true;
          res->message = "Tracking disabled; follower holds last measured joint position";
          return;
        }
        std::string reason;
        if (!arm_tracking(reason)) {
          res->success = false;
          res->message = reason;
          return;
        }
        auto_armed_once_ = true;
        res->success = true;
        res->message = "TRACKING ARMED: move the physical leader slowly";
      });
    start_ = Clock::now();
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    status_timer_ = create_wall_timer(1s, [this] { publish_status(); });
    RCLCPP_WARN(get_logger(),
      "Bridge ready; auto_track=%s, actuation_authorized=%s; tracking initially OFF",
      auto_track_ ? "true" : "false", actuation_authorized_ ? "true" : "false");
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t N = 6;
  static constexpr auto STATE_TIMEOUT = 250ms;
  static constexpr double ARM_TOLERANCE = 0.10;  // rad, all joints
  static constexpr double SEPARATION_LIMIT = 0.35; // rad; disarm on unexpectedly large gap
  static constexpr double COMMAND_RATE = 0.20; // rad/s; lower than hardware 0.25 rad/s
  static constexpr double STEP = COMMAND_RATE / 50.0;

  struct State {
    std::array<double,N> q{};
    Clock::time_point received{};
    bool valid = false;
  };

  static bool decode(const sensor_msgs::msg::JointState &msg, std::array<double,N> &q) {
    if (msg.name.size() != N || msg.position.size() != N) return false;
    for (std::size_t i=0; i<N; ++i) {
      const auto requested = "joint_" + std::to_string(i+1);
      const auto match = std::find(msg.name.begin(), msg.name.end(), requested);
      if (match == msg.name.end()) return false;
      const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(),match));
      const double value = msg.position[index];
      if (!std::isfinite(value)) return false;
      q[i] = value;
    }
    return true;
  }

  void record(const sensor_msgs::msg::JointState::ConstSharedPtr &msg, State &out) {
    std::array<double,N> q{};
    if (!decode(*msg,q)) return;
    out.q = q;
    out.received = Clock::now();
    out.valid = true;
  }

  bool fresh(const State &s) const {
    return s.valid && (Clock::now() - s.received) < STATE_TIMEOUT;
  }

  bool arm_tracking(std::string &reason) {
    if (!actuation_authorized_) {
      reason = "ROS 2 launch is read-only; motor actuation is not authorized";
      return false;
    }
    if (!fresh(leader_) || !fresh(follower_)) {
      reason = "Physical leader and follower joint-state streams must both be fresh";
      return false;
    }
    if (cmd_pub_->get_subscription_count() == 0) {
      reason = "Follower ROS 2 position controller is not subscribed";
      return false;
    }
    for (std::size_t i=0; i<N; ++i) {
      if (std::abs(leader_.q[i] - follower_.q[i]) > ARM_TOLERANCE) {
        reason = "Initial alignment outside 0.10 rad at joint " + std::to_string(i+1);
        return false;
      }
    }
    command_ = follower_.q;
    have_command_ = true;
    tracking_ = true;
    reason = "TRACKING ARMED";
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
    return true;
  }

  void disarm(const std::string &why) {
    if (tracking_) RCLCPP_ERROR(get_logger(), "Tracking disarmed: %s", why.c_str());
    tracking_ = false;
    // A latched hold target prevents an uncommanded arm from drifting when idle.
    if (fresh(follower_)) {
      command_ = follower_.q;
      have_command_ = true;
    }
  }

  void tick() {
    if (!fresh(follower_)) {
      if (tracking_) disarm("Follower feedback lost");
      return; // Hardware will independently fault on missing CAN/servo cycles.
    }
    if (!have_command_) {
      command_ = follower_.q; // initialize to physical pose, NEVER all-zero target
      have_command_ = true;
    }
    if (!tracking_ && auto_track_ && !auto_armed_once_ &&
        Clock::now() - start_ > 2s) {
      std::string reason;
      if (arm_tracking(reason)) auto_armed_once_ = true;
    }
    if (tracking_) {
      if (!fresh(leader_)) {
        disarm("Leader stream stale");
        auto_armed_once_ = true;
      } else {
        bool separated = false;
        for (std::size_t i=0; i<N; ++i) {
          if (std::abs(leader_.q[i] - follower_.q[i]) > SEPARATION_LIMIT) {
            separated = true;
          }
        }
        if (separated) {
          disarm("Leader/follower separation above 0.35 rad");
          auto_armed_once_ = true;
        } else {
          for (std::size_t i=0; i<N; ++i) {
            const auto difference = leader_.q[i] - command_[i];
            command_[i] += std::clamp(difference, -STEP, STEP);
          }
        }
      }
    }
    std_msgs::msg::Float64MultiArray output;
    output.data.assign(command_.begin(), command_.end());
    cmd_pub_->publish(output);
  }

  void publish_status() {
    std_msgs::msg::String s;
    s.data = std::string("tracking=") + (tracking_ ? "ENABLED" : "DISABLED") +
      " leader=" + (fresh(leader_) ? "FRESH" : "STALE") +
      " follower=" + (fresh(follower_) ? "FRESH" : "STALE") +
      " actuation_authorized=" + (actuation_authorized_ ? "true" : "false");
    status_pub_->publish(s);
  }

  State leader_, follower_;
  std::array<double,N> command_{};
  bool actuation_authorized_ = false;
  bool auto_track_ = false;
  bool tracking_ = false;
  bool have_command_ = false;
  bool auto_armed_once_ = false;
  Clock::time_point start_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_,follower_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_,status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YAMLiveBridge>());
  rclcpp::shutdown();
  return 0;
}
