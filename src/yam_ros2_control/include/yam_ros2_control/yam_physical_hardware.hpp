
#ifndef YAM_ROS2_CONTROL__YAM_PHYSICAL_HARDWARE_HPP_
#define YAM_ROS2_CONTROL__YAM_PHYSICAL_HARDWARE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "yam_ros2_control/yam_can_driver.hpp"

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
        static constexpr size_t JOINT_COUNT = 6;

        std::unique_ptr<YAMCanDriver> can_driver_;

        std::array<double, JOINT_COUNT> positions_{};
        std::array<double, JOINT_COUNT> velocities_{};
        std::array<double, JOINT_COUNT> commands_{};

        std::array<MotorState, JOINT_COUNT> motor_states_{};

        const std::array<MotorType, JOINT_COUNT> motor_types_ = {
            MotorType::DM4340,
            MotorType::DM4340,
            MotorType::DM4340,
            MotorType::DM4310,
            MotorType::DM4310,
            MotorType::DM4310};

        const std::array<uint32_t, JOINT_COUNT> motor_ids_ = {
            1, 2, 3, 4, 5, 6};

        const std::array<uint32_t, JOINT_COUNT> response_ids_ = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

        // YAM V1 hardware configuration
        const std::array<double, JOINT_COUNT> directions_ = {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        };

        // Software offsets must be established from
        // actual joint calibration before enabling motors.
        std::array<double, JOINT_COUNT> offsets_{};

        const std::array<double, JOINT_COUNT> kp_ = {
            80.0, 80.0, 80.0, 10.0, 10.0, 10.0
        };

        const std::array<double, JOINT_COUNT> kd_ = {
            5.0, 5.0, 5.0, 1.5, 1.5, 1.5
        };

        // Gravity-compensation scaling
        const std::array<double, JOINT_COUNT> gravity_comp_factor_ = {
            1.0, 1.1, 1.1, 1.2, 1.0, 1.0
        };

        // Damping during gravity-compensation mode
        const std::array<double, JOINT_COUNT> grav_comp_kd_ = {
            0.1, 0.1, 0.1, 0.3, 0.05, 0.05
        };

        std::string can_channel_;
        std::string role_;

        bool initialized_ = false;
        bool enabled_ = false;
    };

} // namespace yam_ros2_control

#endif