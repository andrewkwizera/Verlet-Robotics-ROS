#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class YAMLiveBridge : public rclcpp::Node {
 public:
  YAMLiveBridge():rclcpp::Node("yam_live_bridge") {
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){record(m,leader_);});
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/follower/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){record(m,follower_);});
    cmd_pub_=create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/yam_position_controller/commands",rclcpp::QoS(1));
    enable_srv_=create_service<std_srvs::srv::SetBool>(
      "/yam_teleop/enable_tracking",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if(!req->data) {
          tracking_=false;
          res->success=true;
          res->message="Tracking disabled; follower holds measured pose";
          return;
        }
        if(!fresh(leader_) || !fresh(follower_)) {
          res->success=false;res->message="Both physical joint-state streams must be fresh";return;
        }
        for(std::size_t i=0;i<6;++i) {
          if(std::abs(leader_.q[i]-follower_.q[i])>0.10) {
            res->success=false;
            res->message="Leader and follower must be within 0.10 rad at ALL joints before enabling";
            return;
          }
        }
        tracking_=true;
        res->success=true;
        res->message="TRACKING ARMED: move the leader slowly; use enable_tracking false to stop";
        RCLCPP_WARN(get_logger(),"Physical YAM leader-follower tracking ARMED");
      });
    timer_=create_wall_timer(20ms,[this]{tick();});
    RCLCPP_WARN(get_logger(),
      "Bridge ready; track is DISABLED. Service: /yam_teleop/enable_tracking (std_srvs/SetBool)");
  }
 private:
  using Clock=std::chrono::steady_clock;
  struct State {
    std::array<double,6> q{};
    Clock::time_point stamp{};
    bool valid=false;
  };
  static bool names_ok(const std::vector<std::string> &names) {
    if(names.size()!=6) return false;
    for(std::size_t i=0;i<6;++i) if(names[i]!="joint_"+std::to_string(i+1)) return false;
    return true;
  }
  void record(sensor_msgs::msg::JointState::ConstSharedPtr msg,State &out) {
    if(!names_ok(msg->name)||msg->position.size()!=6) return;
    for(std::size_t i=0;i<6;++i) if(!std::isfinite(msg->position[i])) return;
    for(std::size_t i=0;i<6;++i) out.q[i]=msg->position[i];
    out.stamp=Clock::now();out.valid=true;
  }
  bool fresh(const State &s) const {
    return s.valid && Clock::now()-s.stamp<200ms;
  }
  void tick() {
    if(!fresh(follower_)) {
      if(tracking_) RCLCPP_ERROR(get_logger(),"Follower feedback lost: tracking DISARMED");
      tracking_=false;
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(follower_.q.begin(),follower_.q.end());
    if(tracking_) {
      if(!fresh(leader_)) {
        tracking_=false;
        RCLCPP_ERROR(get_logger(),"Leader feedback stale: tracking DISARMED");
      } else {
        bool gap=false;
        for(std::size_t i=0;i<6;++i) {
          const double delta=leader_.q[i]-follower_.q[i];
          if(std::abs(delta)>0.35) gap=true;
          msg.data[i]=follower_.q[i]+std::clamp(delta,-0.08,0.08);
        }
        if(gap) {
          tracking_=false;
          msg.data.assign(follower_.q.begin(),follower_.q.end());
          RCLCPP_ERROR(get_logger(),"Tracking separation over 0.35 rad: DISARMED");
        }
      }
    }
    cmd_pub_->publish(msg);
  }
  State leader_,follower_;
  bool tracking_=false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_,follower_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<YAMLiveBridge>());
  rclcpp::shutdown();
  return 0;
}
