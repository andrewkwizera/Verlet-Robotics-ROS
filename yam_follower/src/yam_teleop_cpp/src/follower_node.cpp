
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class YAMFollower : public rclcpp::Node
{
public:
  YAMFollower() : Node("yam_follower")
  {
    // Subscribe to the leader's joint positions.
    subscription_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/yam/leader/joint_states",
        10,
        std::bind(&YAMFollower::joint_callback, this, _1));

    // Send commands to the mock ros2_control position controller.
    command_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/yam_position_controller/commands",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "YAM Follower connected to mock position controller");
  }

private:
  void joint_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::vector<std::string> expected_names = {
      "joint_1",
      "joint_2",
      "joint_3",
      "joint_4",
      "joint_5",
      "joint_6"
    };

    // Verify joint names and position count.
    if (msg->name != expected_names ||
        msg->position.size() != 6)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid joint names or position count");
      return;
    }

    // Verify all positions are finite.
    for (double position : msg->position)
    {
      if (!std::isfinite(position))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid joint position received");
        return;
      }
    }

    // Forward all six positions to the controller.
    std_msgs::msg::Float64MultiArray command;
    command.data = msg->position;

    command_publisher_->publish(command);

    message_count_++;

    if (message_count_ % 50 == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Forwarded joint commands: "
        "[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        command.data[0],
        command.data[1],
        command.data[2],
        command.data[3],
        command.data[4],
        command.data[5]);
    }
  }

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr subscription_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;

  size_t message_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<YAMFollower>());

  rclcpp::shutdown();

  return 0;
}

