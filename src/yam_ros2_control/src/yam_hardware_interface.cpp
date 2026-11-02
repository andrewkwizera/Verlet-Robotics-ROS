#include "yam_ros2_control/yam_hardware_interface.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace yam_ros2_control
{

hardware_interface::CallbackReturn
YAMHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(
      rclcpp::get_logger("YAMHardwareInterface"),
      "Expected six YAM joints, received %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_states_.assign(6, 0.0);
  velocity_states_.assign(6, 0.0);
  position_commands_.assign(6, 0.0);

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name !=
          hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("YAMHardwareInterface"),
        "Joint %s must have a position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position = false;
    bool has_velocity = false;

    for (const auto & interface : joint.state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      }
      if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity = true;
      }
    }

    if (joint.state_interfaces.size() != 2 ||
        !has_position || !has_velocity)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("YAMHardwareInterface"),
        "Joint %s needs position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("YAMHardwareInterface"),
    "Mock YAM hardware initialized with six joints");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
YAMHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_states_[i]);

    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[i]);
  }

  return interfaces;
}


std::vector<hardware_interface::CommandInterface>
YAMHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_commands_[i]);
  }

  return interfaces;
}


hardware_interface::CallbackReturn
YAMHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Begin with commands equal to current positions.
  position_commands_ = position_states_;

  RCLCPP_INFO(
    rclcpp::get_logger("YAMHardwareInterface"),
    "Mock YAM hardware activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
YAMHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("YAMHardwareInterface"),
    "Mock YAM hardware deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type
YAMHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  if (dt <= 0.0) {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < 6; ++i) {
    const double previous = position_states_[i];

    // Mock hardware: instantly track the last command.
    position_states_[i] = position_commands_[i];

    velocity_states_[i] =
      (position_states_[i] - previous) / dt;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type
YAMHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // No physical motors are connected.
  // Position commands are applied in read().

  for (double command : position_commands_) {
    if (!std::isfinite(command)) {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace yam_ros2_control


PLUGINLIB_EXPORT_CLASS(
  yam_ros2_control::YAMHardwareInterface,
  hardware_interface::SystemInterface)