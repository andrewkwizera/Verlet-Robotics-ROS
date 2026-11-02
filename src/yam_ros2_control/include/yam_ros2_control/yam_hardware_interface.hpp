#ifndef YAM_ROS2_CONTROL__YAM_HARDWARE_INTERFACE_HPP_
#define YAM_ROS2_CONTROL__YAM_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "yam_ros2_control/yam_can_driver.hpp"

namespace yam_ros2_control
{

// CAN settings for one joint, read from the URDF.
struct JointMotorConfig
{
  uint32_t motor_id = 0;     // CAN ID we send commands to
  uint32_t response_id = 0;  // CAN ID the motor replies with
  MotorType type = MotorType::DM4310;
  double kp = 0.0;           // position gain
  double kd = 0.0;           // damping gain
};

class YAMHardwareInterface
  : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Values shared with ros2_control controllers.
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;

  // true  = no motors, state simply copies the command.
  // false = talk to the real motors over CAN.
  bool use_mock_hardware_ = true;
  std::string can_interface_ = "can0";
  std::vector<JointMotorConfig> motors_;

  // Only created when use_mock_hardware_ is false.
  std::unique_ptr<YAMCanDriver> driver_;

  // Latest reply from each motor (filled in write()).
  std::vector<MotorState> latest_feedback_;
};

}  // namespace yam_ros2_control

#endif
