
#include "yam_ros2_control/yam_physical_hardware.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace yam_ros2_control
{

hardware_interface::CallbackReturn
YAMPhysicalHardware::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != JOINT_COUNT)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Expected exactly six YAM joints");

        return hardware_interface::CallbackReturn::ERROR;
    }

    try
    {
        // Read CAN interface and robot role.
        can_channel_ =
            info_.hardware_parameters.at("can_interface");

        role_ =
            info_.hardware_parameters.at("role");

        if (role_ != "leader" && role_ != "follower")
        {
            throw std::runtime_error(
                "Hardware role must be leader or follower");
        }

        // Verify CAN interface matches the intended robot.
        if (role_ == "leader" && can_channel_ != "can0")
        {
            throw std::runtime_error(
                "Leader must use can0");
        }

        if (role_ == "follower" && can_channel_ != "can1")
        {
            throw std::runtime_error(
                "Follower must use can1");
        }

        // YAM V1 motor directions, gains, and response IDs
        // are defined in yam_physical_hardware.hpp.
        //
        // Only software offsets are loaded here.
        // They must be established by calibration.

        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            offsets_[i] = std::stod(
                info_.hardware_parameters.at(
                    "offset_" + std::to_string(i + 1)));

            if (!std::isfinite(offsets_[i]))
            {
                throw std::runtime_error(
                    "Invalid motor offset for joint " +
                    std::to_string(i + 1));
            }

            // Validate fixed YAM motor configuration.
            if ((directions_[i] != 1.0 &&
                 directions_[i] != -1.0) ||
                !std::isfinite(kp_[i]) ||
                !std::isfinite(kd_[i]) ||
                kp_[i] < 0.0 ||
                kp_[i] > 500.0 ||
                kd_[i] < 0.0 ||
                kd_[i] > 5.0 ||
                response_ids_[i] > 0x7FF)
            {
                throw std::runtime_error(
                    "Invalid YAM motor configuration for joint " +
                    std::to_string(i + 1));
            }
        }

        // Validate six ROS 2 joint interfaces.
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            const auto &joint = info_.joints[i];

            const std::string expected_name =
                "joint_" + std::to_string(i + 1);

            if (joint.name != expected_name)
            {
                throw std::runtime_error(
                    "Unexpected joint name: " + joint.name);
            }

            if (joint.command_interfaces.size() != 1 ||
                joint.command_interfaces[0].name != "position")
            {
                throw std::runtime_error(
                    "Joint must have one position command interface: " +
                    joint.name);
            }

            if (joint.state_interfaces.size() != 2)
            {
                throw std::runtime_error(
                    "Joint must have two state interfaces: " +
                    joint.name);
            }

            bool has_position = false;
            bool has_velocity = false;

            for (const auto &interface :
                 joint.state_interfaces)
            {
                if (interface.name == "position")
                {
                    has_position = true;
                }

                if (interface.name == "velocity")
                {
                    has_velocity = true;
                }
            }

            if (!has_position || !has_velocity)
            {
                throw std::runtime_error(
                    "Missing position or velocity state: " +
                    joint.name);
            }
        }

        // Open native Linux SocketCAN connection.
        // This does not enable the motors.
        can_driver_ =
            std::make_unique<YAMCanDriver>(can_channel_);

        positions_.fill(0.0);
        velocities_.fill(0.0);
        commands_.fill(0.0);

        enabled_ = false;
        initialized_ = true;

        RCLCPP_INFO(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Physical YAM hardware initialized");

        RCLCPP_INFO(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Role: %s | CAN interface: %s",
            role_.c_str(),
            can_channel_.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Using YAM V1 motor configuration");

        RCLCPP_INFO(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Motor activation remains disabled");
    }
    catch (const std::exception &e)
    {
        initialized_ = false;
        enabled_ = false;

        RCLCPP_ERROR(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Initialization failed: %s",
            e.what());

        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// EXPORT JOINT STATE INTERFACES
// ============================================================

std::vector<hardware_interface::StateInterface>
YAMPhysicalHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface>
        interfaces;

    for (size_t i = 0; i < JOINT_COUNT; ++i)
    {
        const auto &name = info_.joints[i].name;

        interfaces.emplace_back(
            name,
            "position",
            &positions_[i]);

        interfaces.emplace_back(
            name,
            "velocity",
            &velocities_[i]);
    }

    return interfaces;
}


// ============================================================
// EXPORT JOINT COMMAND INTERFACES
// ============================================================

std::vector<hardware_interface::CommandInterface>
YAMPhysicalHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface>
        interfaces;

    for (size_t i = 0; i < JOINT_COUNT; ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            "position",
            &commands_[i]);
    }

    return interfaces;
}


// ============================================================
// ACTIVATE HARDWARE
// ============================================================

hardware_interface::CallbackReturn
YAMPhysicalHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    if (!initialized_ || !can_driver_)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Cannot activate uninitialized hardware");

        return hardware_interface::CallbackReturn::ERROR;
    }

    // Physical motor activation must remain disabled
    // until startup validation, real feedback,
    // gravity compensation, command alignment,
    // and fault handling are implemented.

    enabled_ = false;

    RCLCPP_ERROR(
        rclcpp::get_logger("YAMPhysicalHardware"),
        "Activation blocked: physical startup and "
        "safe control integration are incomplete");

    return hardware_interface::CallbackReturn::ERROR;
}


// ============================================================
// DEACTIVATE HARDWARE
// ============================================================

hardware_interface::CallbackReturn
YAMPhysicalHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    enabled_ = false;

    RCLCPP_INFO(
        rclcpp::get_logger("YAMPhysicalHardware"),
        "Physical YAM control deactivated");

    // This does not send a motor disable or hold command.
    // Physical shutdown handling is still required.

    return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// READ ACTUAL MOTOR FEEDBACK
// ============================================================

hardware_interface::return_type
YAMPhysicalHardware::read(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    if (!enabled_)
    {
        return hardware_interface::return_type::ERROR;
    }

    // motor_states_ must contain validated, fresh
    // motor feedback before this method can operate.

    for (size_t i = 0; i < JOINT_COUNT; ++i)
    {
        if (!std::isfinite(motor_states_[i].position) ||
            !std::isfinite(motor_states_[i].velocity))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("YAMPhysicalHardware"),
                "Invalid motor feedback for joint %zu",
                i + 1);

            enabled_ = false;

            return hardware_interface::return_type::ERROR;
        }

        positions_[i] =
            (motor_states_[i].position - offsets_[i]) *
            directions_[i];

        velocities_[i] =
            motor_states_[i].velocity *
            directions_[i];
    }

    return hardware_interface::return_type::OK;
}


// ============================================================
// WRITE MOTOR COMMANDS
// ============================================================

hardware_interface::return_type
YAMPhysicalHardware::write(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    if (!enabled_)
    {
        return hardware_interface::return_type::ERROR;
    }

    // The leader requires gravity-compensation control,
    // not follower-style position control.
    if (role_ != "follower")
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Position-control write is not valid "
            "for the gravity-compensated leader");

        enabled_ = false;

        return hardware_interface::return_type::ERROR;
    }

    try
    {
        // Reject invalid commands before sending
        // any CAN frames during this cycle.
        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            if (!std::isfinite(commands_[i]))
            {
                throw std::runtime_error(
                    "Non-finite position command for joint " +
                    std::to_string(i + 1));
            }
        }

        for (size_t i = 0; i < JOINT_COUNT; ++i)
        {
            MotorCommand cmd;

            // Convert ROS joint position to
            // the motor coordinate system.
            cmd.position =
                commands_[i] * directions_[i] +
                offsets_[i];

            cmd.velocity = 0.0;

            // Actual YAM V1 position-controller gains.
            cmd.kp = kp_[i];
            cmd.kd = kd_[i];

            cmd.torque = 0.0;

            // Actual SocketCAN motor communication.
            motor_states_[i] =
                can_driver_->exchange(
                    motor_ids_[i],
                    response_ids_[i],
                    motor_types_[i],
                    cmd,
                    10);
        }
    }
    catch (const std::exception &e)
    {
        enabled_ = false;

        RCLCPP_ERROR(
            rclcpp::get_logger("YAMPhysicalHardware"),
            "Motor communication failed: %s",
            e.what());

        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

} // namespace yam_ros2_control


PLUGINLIB_EXPORT_CLASS(
    yam_ros2_control::YAMPhysicalHardware,
    hardware_interface::SystemInterface)

