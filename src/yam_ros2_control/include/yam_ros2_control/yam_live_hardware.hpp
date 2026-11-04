#ifndef YAM_ROS2_CONTROL__YAM_LIVE_HARDWARE_HPP_
#define YAM_ROS2_CONTROL__YAM_LIVE_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "yam_ros2_control/yam_can_driver.hpp"
#include "yam_ros2_control/yam_can_register_reader.hpp"
#include <kdl/chaindynparam.hpp>

namespace yam_ros2_control {

// One instance per robot. ROS 2 owns the positions and command interfaces;
// ONLY this plugin can transmit MIT actuation commands to its CAN interface.
class YAMLiveHardware final : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;
  ~YAMLiveHardware() override;

private:
  static constexpr std::size_t N = 6;
  const std::array<MotorType,N> types_ = {
      MotorType::DM4340, MotorType::DM4340, MotorType::DM4340,
      MotorType::DM4310, MotorType::DM4310, MotorType::DM4310};
  // Factory YAM v1 values, not dynamically tuned PID gains.
  const std::array<double,N> kp_ = {80,80,80,10,10,10};
  const std::array<double,N> kd_ = {5,5,5,1.5,1.5,1.5};
  const std::array<double,N> gravity_kd_ = {0.1,0.1,0.1,0.3,0.05,0.05};
  const std::array<double,N> gravity_factor_ = {1.0,1.1,1.1,1.2,1.0,1.0};
  const std::array<double,N> lower_ = {-2.61799,0.0,0.0,-1.69297,-1.5708,-2.0944};
  const std::array<double,N> upper_ = {3.14159,3.66519,3.14159,1.5708,1.5708,2.0944};
  // Software gravity torque envelope, beneath each DM motor's MIT limit.
  const std::array<double,N> torque_caps_ = {12.0,12.0,12.0,5.0,5.0,5.0};

  std::array<double,N> offsets_{};
  std::array<double,N> positions_{};
  std::array<double,N> velocities_{};
  std::array<double,N> commands_{};
  std::array<double,N> startup_hold_{};
  std::array<double,N> raw_unwrapped_{};
  std::array<double,N> raw_last_{};
  std::array<bool,N> unwrapped_ready_{};
  std::array<bool,N> motor_enabled_{};
  std::unique_ptr<YAMCanDriver> control_bus_;
  std::unique_ptr<YAMCanRegisterReader> registers_;
  std::unique_ptr<KDL::ChainDynParam> gravity_solver_;
  std::string role_, channel_;
  bool initialized_ = false;
  bool active_ = false;
  bool allow_actuation_ = false;
  bool calibrated_ = false;
  bool estop_confirmed_ = false;
  bool faulted_ = false;
  bool follower_command_seen_ = false;
  bool follower_controller_claimed_ = false;
  std::chrono::steady_clock::time_point follower_command_ready_at_{};
  std::chrono::steady_clock::time_point last_cycle_{};
  std::chrono::steady_clock::time_point last_raw_read_{};

  void verify_registers();
  void read_encoder_snapshot(bool initial);
  void put_feedback(std::size_t joint, const MotorState &state);
  bool within_limits(const std::array<double,N>& q) const;
  std::array<double,N> gravity_torques() const;
  MotorCommand startup_command(std::size_t joint, double measured_motor_position,
                               const std::array<double,N>& gravity) const;
  void turn_off_best_effort() noexcept;
};
}  // namespace yam_ros2_control
#endif
