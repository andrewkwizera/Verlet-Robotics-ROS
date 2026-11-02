#include "yam_ros2_control/yam_hardware_interface.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace yam_ros2_control
{

namespace
{

using ParamMap = std::unordered_map<std::string, std::string>;

// Returns a <param> value from the URDF, or throws if it is missing.
std::string get_param(
  const ParamMap & params,
  const std::string & name)
{
  auto it = params.find(name);
  if (it == params.end()) {
    throw std::runtime_error("Missing URDF param: " + name);
  }
  return it->second;
}

// Converts "DM4340" / "DM4310" into a MotorType.
MotorType parse_motor_type(const std::string & text)
{
  if (text == "DM4340") {
    return MotorType::DM4340;
  }
  if (text == "DM4310") {
    return MotorType::DM4310;
  }
  throw std::runtime_error("Unknown motor_type: " + text);
}

}  // namespace


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
  latest_feedback_.assign(6, MotorState{});

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

  // Mock mode is the default, so real motors are never
  // used unless the URDF explicitly asks for them.
  auto mock_param =
    info_.hardware_parameters.find("use_mock_hardware");
  if (mock_param != info_.hardware_parameters.end()) {
    use_mock_hardware_ = (mock_param->second != "false");
  }

  if (use_mock_hardware_) {
    RCLCPP_INFO(
      rclcpp::get_logger("YAMHardwareInterface"),
      "Mock YAM hardware initialized with six joints");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Real hardware: read the CAN settings for every joint.
  try {
    can_interface_ =
      get_param(info_.hardware_parameters, "can_interface");

    motors_.clear();
    for (const auto & joint : info_.joints) {
      JointMotorConfig motor;
      // Base 0 lets IDs be written as "1" or "0x01".
      motor.motor_id = static_cast<uint32_t>(
        std::stoul(get_param(joint.parameters, "motor_id"), nullptr, 0));
      motor.response_id = static_cast<uint32_t>(
        std::stoul(get_param(joint.parameters, "response_id"), nullptr, 0));
      motor.type =
        parse_motor_type(get_param(joint.parameters, "motor_type"));
      motor.kp = std::stod(get_param(joint.parameters, "kp"));
      motor.kd = std::stod(get_param(joint.parameters, "kd"));
      motors_.push_back(motor);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("YAMHardwareInterface"),
      "Bad hardware config: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("YAMHardwareInterface"),
    "Real YAM hardware initialized on %s",
    can_interface_.c_str());

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
YAMHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (use_mock_hardware_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Open the CAN socket. Motors are not enabled yet.
  try {
    driver_ = std::make_unique<YAMCanDriver>(can_interface_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("YAMHardwareInterface"),
      "Could not open CAN: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
YAMHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // Closes the CAN socket (if it was open).
  driver_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
YAMHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!use_mock_hardware_) {
    // Enable each motor and read where it currently is.
    try {
      for (size_t i = 0; i < motors_.size(); ++i) {
        latest_feedback_[i] = driver_->enable_motor(
          motors_[i].motor_id,
          motors_[i].response_id,
          motors_[i].type);

        position_states_[i] = latest_feedback_[i].position;
        velocity_states_[i] = latest_feedback_[i].velocity;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        rclcpp::get_logger("YAMHardwareInterface"),
        "Motor enable failed: %s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Begin with commands equal to current positions,
  // so the arm holds still instead of jumping.
  position_commands_ = position_states_;

  RCLCPP_INFO(
    rclcpp::get_logger("YAMHardwareInterface"),
    "YAM hardware activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
YAMHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // We do not disable the motors here: a disabled
  // motor goes limp and the arm would fall.
  RCLCPP_INFO(
    rclcpp::get_logger("YAMHardwareInterface"),
    "YAM hardware deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type
YAMHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!use_mock_hardware_) {
    // The motors reply to every command we send in write(),
    // so the newest state is already stored. Just copy it.
    for (size_t i = 0; i < 6; ++i) {
      position_states_[i] = latest_feedback_[i].position;
      velocity_states_[i] = latest_feedback_[i].velocity;
    }
    return hardware_interface::return_type::OK;
  }

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
  for (double command : position_commands_) {
    if (!std::isfinite(command)) {
      return hardware_interface::return_type::ERROR;
    }
  }

  if (use_mock_hardware_) {
    // No physical motors are connected.
    // Position commands are applied in read().
    return hardware_interface::return_type::OK;
  }

  // Send each motor its target position and save its reply.
  // This talks to one motor at a time.
  try {
    for (size_t i = 0; i < motors_.size(); ++i) {
      MotorCommand command;
      command.position = position_commands_[i];
      command.velocity = 0.0;
      command.kp = motors_[i].kp;
      command.kd = motors_[i].kd;
      command.torque = 0.0;

      latest_feedback_[i] = driver_->exchange(
        motors_[i].motor_id,
        motors_[i].response_id,
        motors_[i].type,
        command);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("YAMHardwareInterface"),
      "CAN write failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace yam_ros2_control


PLUGINLIB_EXPORT_CLASS(
  yam_ros2_control::YAMHardwareInterface,
  hardware_interface::SystemInterface)
