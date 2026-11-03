
#include "yam_ros2_control/yam_physical_hardware.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace yam_ros2_control
{

namespace
{

rclcpp::Logger logger()
{
    return rclcpp::get_logger("YAMPhysicalHardware");
}

} // namespace

// ============================================================
// INITIALIZATION
// ============================================================

hardware_interface::CallbackReturn
YAMPhysicalHardware::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try
    {
        if (info_.joints.size() != JOINT_COUNT)
        {
            throw std::runtime_error(
                "Expected six YAM arm joints");
        }

        can_channel_ =
            info_.hardware_parameters.at("can_interface");

        role_ =
            info_.hardware_parameters.at("role");

        // This integration stage is follower diagnostics only.
        if (role_ != "follower" || can_channel_ != "can1")
        {
            throw std::runtime_error(
                "Read-only diagnostics support follower on can1 only");
        }

        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
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
                    "Joint requires one position command interface");
            }

            if (joint.state_interfaces.size() != 2)
            {
                throw std::runtime_error(
                    "Joint requires position and velocity states");
            }

            bool position_found = false;
            bool velocity_found = false;

            for (const auto &interface : joint.state_interfaces)
            {
                if (interface.name == "position")
                {
                    position_found = true;
                }

                if (interface.name == "velocity")
                {
                    velocity_found = true;
                }
            }

            if (!position_found || !velocity_found)
            {
                throw std::runtime_error(
                    "Missing joint state interface");
            }
        }

        register_reader_ =
            std::make_unique<YAMCanRegisterReader>(
                can_channel_);

        const double nan =
            std::numeric_limits<double>::quiet_NaN();

        positions_.fill(nan);
        velocities_.fill(nan);
        commands_.fill(nan);

        initialized_ = true;
        active_ = false;
        read_only_mode_ = true;
        has_previous_read_ = false;

        RCLCPP_WARN(
            logger(),
            "YAM follower connected on %s in READ-ONLY mode. "
            "No motor commands can be transmitted. "
            "Positions are raw xout telemetry, "
            "NOT calibrated URDF joint angles.",
            can_channel_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            logger(),
            "Hardware initialization failed: %s",
            e.what());

        initialized_ = false;
        active_ = false;

        return hardware_interface::CallbackReturn::ERROR;
    }
}

// ============================================================
// STATE INTERFACES
// ============================================================

std::vector<hardware_interface::StateInterface>
YAMPhysicalHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> result;

    for (std::size_t i = 0; i < JOINT_COUNT; ++i)
    {
        result.emplace_back(
            info_.joints[i].name,
            "position",
            &positions_[i]);

        result.emplace_back(
            info_.joints[i].name,
            "velocity",
            &velocities_[i]);
    }

    return result;
}

// ============================================================
// COMMAND INTERFACES
// ============================================================

std::vector<hardware_interface::CommandInterface>
YAMPhysicalHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> result;

    for (std::size_t i = 0; i < JOINT_COUNT; ++i)
    {
        result.emplace_back(
            info_.joints[i].name,
            "position",
            &commands_[i]);
    }

    // Exported for interface compatibility.
    // write() intentionally never acts on these commands.

    return result;
}

// ============================================================
// ACQUIRE RAW MOTOR POSITIONS
// ============================================================

bool YAMPhysicalHardware::acquire_positions(
    std::array<double, JOINT_COUNT> &measurements)
{
    if (!register_reader_)
    {
        return false;
    }

    try
    {
        for (std::size_t i = 0; i < JOINT_COUNT; ++i)
        {
            const float value =
                register_reader_->read_float_register(
                    motor_ids_[i],
                    XOUT_REGISTER);

            if (!std::isfinite(value))
            {
                throw std::runtime_error(
                    "Invalid xout position");
            }

            measurements[i] =
                static_cast<double>(value);
        }

        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            logger(),
            "Motor feedback acquisition failed: %s",
            e.what());

        return false;
    }
}

// ============================================================
// READ-ONLY ACTIVATION
// ============================================================

hardware_interface::CallbackReturn
YAMPhysicalHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    if (!initialized_ ||
        !register_reader_ ||
        !read_only_mode_)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::array<double, JOINT_COUNT> measurements{};

    if (!acquire_positions(measurements))
    {
        RCLCPP_ERROR(
            logger(),
            "Cannot activate diagnostics without "
            "valid feedback from all six motors");

        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize from actual measured raw positions.
    positions_ = measurements;

    velocities_.fill(0.0);

    // This initializes command buffers only.
    // It does NOT command the motors to hold.
    commands_ = positions_;

    active_ = true;
    has_previous_read_ = false;

    RCLCPP_WARN(
        logger(),
        "READ-ONLY YAM follower diagnostics activated. "
        "All six motors responded. "
        "Motor enable and position control remain BLOCKED.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// DEACTIVATION
// ============================================================

hardware_interface::CallbackReturn
YAMPhysicalHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    active_ = false;
    has_previous_read_ = false;

    RCLCPP_INFO(
        logger(),
        "YAM read-only diagnostics deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// READ MOTOR FEEDBACK
// ============================================================

hardware_interface::return_type
YAMPhysicalHardware::read(
    const rclcpp::Time &time,
    const rclcpp::Duration &)
{
    if (!active_ || !register_reader_)
    {
        return hardware_interface::return_type::ERROR;
    }

    std::array<double, JOINT_COUNT> measurements{};

    if (!acquire_positions(measurements))
    {
        active_ = false;
        has_previous_read_ = false;

        return hardware_interface::return_type::ERROR;
    }

    const double dt = has_previous_read_
        ? (time - last_read_time_).seconds()
        : 0.0;

    for (std::size_t i = 0; i < JOINT_COUNT; ++i)
    {
        // Numerical derivative of raw xout.
        // Not yet a calibrated joint velocity.
        if (has_previous_read_ &&
            std::isfinite(dt) &&
            dt > 0.0)
        {
            velocities_[i] =
                (measurements[i] - positions_[i]) / dt;
        }
        else
        {
            velocities_[i] = 0.0;
        }

        positions_[i] = measurements[i];
    }

    last_read_time_ = time;
    has_previous_read_ = true;

    return hardware_interface::return_type::OK;
}

// ============================================================
// WRITE: DELIBERATELY NON-ACTUATING
// ============================================================

hardware_interface::return_type
YAMPhysicalHardware::write(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    if (!active_ || !read_only_mode_)
    {
        return hardware_interface::return_type::ERROR;
    }

    // SAFETY INTERLOCK:
    //
    // Do not send any CAN commands here.
    //
    // Position controllers may update commands_, but
    // motor enable, torque commands and position tracking
    // are NOT implemented in this diagnostic stage.

    return hardware_interface::return_type::OK;
}

} // namespace yam_ros2_control

PLUGINLIB_EXPORT_CLASS(
    yam_ros2_control::YAMPhysicalHardware,
    hardware_interface::SystemInterface)