// ROS 2 live teleoperation: physical can0 -> ROS joint states -> can1 position controller.
// No mock/synthetic leader and no direct CAN transmission from this node.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <memory>
#include <string>
#include <stdexcept>
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
    auto_reconnect_ = declare_parameter<bool>("auto_reconnect", true);
    command_rate_ = declare_parameter<double>("follower_speed", 0.25);
    if (!std::isfinite(command_rate_) || command_rate_ < 0.05 || command_rate_ > 0.50) {
      throw std::runtime_error("follower_speed must be in [0.05, 0.50] rad/s");
    }
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
          reconnect_pending_ = false;
          reconnect_candidate_valid_ = false;
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
        reconnect_pending_ = false;
        reconnect_candidate_valid_ = false;
        res->message = "TRACKING ARMED: move the physical leader slowly";
      });
    start_ = Clock::now();
    last_tick_ = start_;
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    status_timer_ = create_wall_timer(1s, [this] { publish_status(); });
    RCLCPP_WARN(get_logger(),
      "Bridge ready; auto_track=%s, auto_reconnect=%s, "
      "actuation_authorized=%s; tracking initially OFF",
      auto_track_ ? "true" : "false",
      auto_reconnect_ ? "true" : "false",
      actuation_authorized_ ? "true" : "false");
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t N = 6;
  static constexpr auto STATE_TIMEOUT = 250ms;
  static constexpr double ARM_TOLERANCE = 0.10;  // rad, all joints
  static constexpr double SEPARATION_LIMIT = 0.35; // rad; disarm on unexpectedly large gap
  // Auto-rejoin gate: five degrees per joint, stable for 750 ms.
  static constexpr double REJOIN_TOLERANCE = 0.08726646259971647;
  static constexpr double REJOIN_LEADER_STABILITY = 0.008726646259971648;
  static constexpr double REJOIN_FOLLOWER_STABILITY = 0.017453292519943295;
  static constexpr auto REJOIN_DWELL = 750ms;

  struct State {
    std::array<double,N> q{};
    Clock::time_point received{};
    bool valid = false;
    std::uint64_t received_messages = 0;
    double max_arrival_gap_ms = 0.0;
    double max_source_stamp_gap_ms = 0.0;
    std::int64_t last_source_stamp_ns = 0;
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
    const auto now = Clock::now();
    if (out.valid) {
      const double arrival_gap_ms =
        std::chrono::duration<double, std::milli>(now - out.received).count();
      out.max_arrival_gap_ms = std::max(out.max_arrival_gap_ms, arrival_gap_ms);
    }
    const std::int64_t source_stamp_ns =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);
    if (out.last_source_stamp_ns != 0 && source_stamp_ns > out.last_source_stamp_ns) {
      const double source_gap_ms =
        static_cast<double>(source_stamp_ns - out.last_source_stamp_ns) / 1000000.0;
      out.max_source_stamp_gap_ms = std::max(out.max_source_stamp_gap_ms, source_gap_ms);
    }
    out.last_source_stamp_ns = source_stamp_ns;
    out.q = q;
    out.received = now;
    out.valid = true;
    ++out.received_messages;
  }

  double age_ms(const State &s) const {
    if (!s.valid) return -1.0;
    return std::chrono::duration<double, std::milli>(Clock::now() - s.received).count();
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
    if (tracking_) {
      last_disarm_reason_ = why;
      RCLCPP_ERROR(get_logger(),
        "Tracking disarmed: %s | leader_age_ms=%.1f follower_age_ms=%.1f "
        "leader_rx=%llu follower_rx=%llu leader_max_callback_gap_ms=%.1f "
        "follower_max_callback_gap_ms=%.1f leader_max_source_gap_ms=%.1f "
        "follower_max_source_gap_ms=%.1f",
        why.c_str(), age_ms(leader_), age_ms(follower_),
        static_cast<unsigned long long>(leader_.received_messages),
        static_cast<unsigned long long>(follower_.received_messages),
        leader_.max_arrival_gap_ms, follower_.max_arrival_gap_ms,
        leader_.max_source_stamp_gap_ms, follower_.max_source_stamp_gap_ms);
    }
    tracking_ = false;
    // A latched hold target prevents an uncommanded arm from drifting when idle.
    if (fresh(follower_)) {
      command_ = follower_.q;
      have_command_ = true;
    }
  }

  // Only a LEADER stream timeout during an armed session permits auto-rejoin.
  // Manual disarm, large separation and missing follower feedback never do.
  void maybe_rejoin(const Clock::time_point now) {
    if (!reconnect_pending_ || tracking_) return;
    if (!auto_reconnect_ || !actuation_authorized_ || !fresh(follower_) ||
        cmd_pub_->get_subscription_count() == 0) {
      reconnect_pending_ = false;
      reconnect_candidate_valid_ = false;
      return;
    }
    if (!fresh(leader_)) {
      reconnect_candidate_valid_ = false;
      return;
    }
    for (std::size_t i = 0; i < N; ++i) {
      if (std::abs(leader_.q[i] - follower_.q[i]) > REJOIN_TOLERANCE) {
        reconnect_candidate_valid_ = false;
        return;
      }
    }
    if (!reconnect_candidate_valid_) {
      reconnect_candidate_valid_ = true;
      reconnect_candidate_since_ = now;
      reconnect_leader_pose_ = leader_.q;
      reconnect_follower_pose_ = follower_.q;
      return;
    }
    for (std::size_t i = 0; i < N; ++i) {
      if (std::abs(leader_.q[i] - reconnect_leader_pose_[i]) >
            REJOIN_LEADER_STABILITY ||
          std::abs(follower_.q[i] - reconnect_follower_pose_[i]) >
            REJOIN_FOLLOWER_STABILITY) {
        reconnect_candidate_valid_ = false;
        return;
      }
    }
    if (now - reconnect_candidate_since_ < REJOIN_DWELL) return;
    std::string reason;
    if (arm_tracking(reason)) {
      reconnect_pending_ = false;
      reconnect_candidate_valid_ = false;
      RCLCPP_WARN(get_logger(),
        "AUTO-REJOIN: all 6 joints within 5 deg, stable 750 ms; tracking restored");
    } else {
      reconnect_candidate_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Auto-rejoin blocked: %s", reason.c_str());
    }
  }

  void tick() {
    const auto now = Clock::now();
    const double dt = std::clamp(std::chrono::duration<double>(now-last_tick_).count(),
                                 0.001, 0.04);
    last_tick_ = now;
    if (!fresh(follower_)) {
      if (tracking_) disarm("Follower feedback lost");
      reconnect_pending_ = false;
      reconnect_candidate_valid_ = false;
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
        reconnect_pending_ = auto_reconnect_;
        reconnect_candidate_valid_ = false;
        auto_armed_once_ = true;
      } else {
        double max_gap = 0.0;
        std::size_t gap_joint = 0;
        for (std::size_t i=0; i<N; ++i) {
          const double gap = std::abs(leader_.q[i] - follower_.q[i]);
          if (gap > max_gap) { max_gap = gap; gap_joint = i; }
        }
        if (max_gap > SEPARATION_LIMIT) {
          std::ostringstream reason;
          reason << "Leader/follower separation " << max_gap
                 << " rad on joint " << gap_joint + 1
                 << " exceeds " << SEPARATION_LIMIT
                 << " rad (configured command speed " << command_rate_ << " rad/s)";
          disarm(reason.str());
          reconnect_pending_ = false;
          reconnect_candidate_valid_ = false;
          auto_armed_once_ = true;
        } else {
          for (std::size_t i=0; i<N; ++i) {
            const auto difference = leader_.q[i] - command_[i];
            command_[i] += std::clamp(difference,
              -command_rate_ * dt, command_rate_ * dt);
          }
        }
      }
    }
    maybe_rejoin(now);
    std_msgs::msg::Float64MultiArray output;
    output.data.assign(command_.begin(), command_.end());
    cmd_pub_->publish(output);
  }

  void publish_status() {
    double max_gap = 0.0;
    double max_command_gap = 0.0;
    std::size_t gap_joint = 0;
    if (fresh(leader_) && fresh(follower_)) {
      for (std::size_t i=0; i<N; ++i) {
        const double gap = std::abs(leader_.q[i] - follower_.q[i]);
        if (gap > max_gap) { max_gap = gap; gap_joint = i + 1; }
        if (have_command_) {
          max_command_gap = std::max(max_command_gap,
            std::abs(command_[i] - follower_.q[i]));
        }
      }
    }
    std::ostringstream status;
    status << std::fixed << std::setprecision(3)
      << "tracking=" << (tracking_ ? "ENABLED" : "DISABLED")
      << " leader=" << (fresh(leader_) ? "FRESH" : "STALE")
      << " follower=" << (fresh(follower_) ? "FRESH" : "STALE")
      << " actuation_authorized=" << (actuation_authorized_ ? "true" : "false")
      << " max_separation_rad=" << max_gap
      << " worst_joint=" << gap_joint
      << " command_gap_rad=" << max_command_gap
      << " follower_speed_rad_s=" << command_rate_
      << " reconnect=" << (reconnect_pending_
           ? (!fresh(leader_) ? "WAITING_FOR_LINK"
              : (reconnect_candidate_valid_ ? "STABILIZING" : "WAITING_FOR_ALIGNMENT"))
           : "IDLE")
      << " leader_age_ms=" << age_ms(leader_)
      << " follower_age_ms=" << age_ms(follower_)
      << " leader_rx=" << leader_.received_messages
      << " follower_rx=" << follower_.received_messages
      << " leader_max_callback_gap_ms=" << leader_.max_arrival_gap_ms
      << " follower_max_callback_gap_ms=" << follower_.max_arrival_gap_ms
      << " leader_max_source_gap_ms=" << leader_.max_source_stamp_gap_ms
      << " follower_max_source_gap_ms=" << follower_.max_source_stamp_gap_ms
      << " last_disarm_reason=" << (last_disarm_reason_.empty() ? "none" : last_disarm_reason_);
    std_msgs::msg::String s;
    s.data = status.str();
    status_pub_->publish(s);
  }

  State leader_, follower_;
  std::array<double,N> command_{};
  double command_rate_ = 0.25;
  bool actuation_authorized_ = false;
  bool auto_track_ = false;
  bool auto_reconnect_ = true;
  bool tracking_ = false;
  bool reconnect_pending_ = false;
  bool reconnect_candidate_valid_ = false;
  Clock::time_point reconnect_candidate_since_{};
  std::array<double,N> reconnect_leader_pose_{};
  std::array<double,N> reconnect_follower_pose_{};
  bool have_command_ = false;
  bool auto_armed_once_ = false;
  std::string last_disarm_reason_;
  Clock::time_point start_{};
  Clock::time_point last_tick_{};
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
