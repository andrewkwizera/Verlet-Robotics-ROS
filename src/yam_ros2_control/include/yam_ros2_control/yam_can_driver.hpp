
#ifndef YAM_ROS2_CONTROL__YAM_CAN_DRIVER_HPP_
#define YAM_ROS2_CONTROL__YAM_CAN_DRIVER_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace yam_ros2_control
{

enum class MotorType
{
  DM4340,
  DM4310
};

struct MotorState
{
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double temperature_mos = 0.0;
  double temperature_rotor = 0.0;
  uint8_t error_code = 0;
};

struct MotorCommand
{
  double position = 0.0;
  double velocity = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double torque = 0.0;
};

class YAMCanDriver
{
public:
  explicit YAMCanDriver(const std::string & channel);
  ~YAMCanDriver();

  YAMCanDriver(const YAMCanDriver &) = delete;
  YAMCanDriver & operator=(const YAMCanDriver &) = delete;

  static std::array<uint8_t, 8> encode_mit(
    const MotorCommand & command,
    MotorType type);

  static MotorState decode_feedback(
    const std::array<uint8_t, 8> & data,
    MotorType type);

  MotorState exchange(
    uint32_t motor_id,
    uint32_t response_id,
    MotorType type,
    const MotorCommand & command,
    int timeout_ms = 10);

  // Enables an actual DM motor.
  // Must only be called by the hardware plugin
  // after the startup safety checks pass.
  MotorState enable_motor(
    uint32_t motor_id,
    uint32_t response_id,
    MotorType type,
    int timeout_ms = 20);

  // Sends DM MIT motor-off frame (FF ... FD). Only when the arm is supported.
  void disable_motor(uint32_t motor_id);

  // Receives and validates feedback for a specific motor.
  MotorState wait_for_feedback(
    uint32_t response_id,
    MotorType type,
    int timeout_ms);

private:
  int socket_fd_ = -1;
};

}  // namespace yam_ros2_control

#endif