#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class YAMLeader : public rclcpp::Node
{
public:
  YAMLeader() : Node("yam_leader")
  {
    publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>(
        "/yam/leader/joint_states", 10);

    start_time_ = this->now();

    timer_ = this->create_wall_timer(
      20ms,
      std::bind(&YAMLeader::publish_joint_states, this));

    RCLCPP_INFO(
      this->get_logger(),
      "YAM Leader Publisher started at 50 Hz");
  }

private:
  void publish_joint_states()
  {
    double t = (this->now() - start_time_).seconds();

    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->now();

    msg.name = {
      "joint_1",
      "joint_2",
      "joint_3",
      "joint_4",
      "joint_5",
      "joint_6"
    };

    // Simulated joint positions in radians
    msg.position = {
      0.1 * std::sin(t),
      0.2 * std::sin(t * 0.5),
      0.1 * std::cos(t),
      0.0,
      0.0,
      0.0
    };

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<YAMLeader>());

  rclcpp::shutdown();

  return 0;
}