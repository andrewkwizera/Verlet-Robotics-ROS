
#ifndef YAM_ROS2_CONTROL__YAM_PHYSICAL_HARDWARE_HPP_
#define YAM_ROS2_CONTROL__YAM_PHYSICAL_HARDWARE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "yam_ros2_control/yam_can_register_reader.hpp"

namespace yam_ros2_control
{

class YAMPhysicalHardware
    : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

private:
    static constexpr std::size_t JOINT_COUNT = 6;

    static constexpr uint8_t XOUT_REGISTER = 81;

    std::unique_ptr<YAMCanRegisterReader> register_reader_;

    std::array<double, JOINT_COUNT> positions_{};
    std::array<double, JOINT_COUNT> velocities_{};
    std::array<double, JOINT_COUNT> commands_{};

    std::array<uint8_t, JOINT_COUNT> motor_ids_ = {
        1, 2, 3, 4, 5, 6
    };

    std::string can_channel_;
    std::string role_;

    bool initialized_ = false;
    bool active_ = false;

    // The current stage provides raw diagnostic telemetry only.
    // It does not enable motors or execute position commands.
    bool read_only_mode_ = true;

    rclcpp::Time last_read_time_;
    bool has_previous_read_ = false;

    bool acquire_positions(
        std::array<double, JOINT_COUNT> &measurements);
};

} // namespace yam_ros2_control

#endif