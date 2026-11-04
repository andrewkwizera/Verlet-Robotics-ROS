// Offline unit test: does not open CAN or energize hardware.
#include "yam_ros2_control/yam_can_driver.hpp"
#include <cassert>
#include <cmath>
#include <stdexcept>
using yam_ros2_control::YAMCanDriver;
using yam_ros2_control::MotorType;
using yam_ros2_control::MotorCommand;
int main() {
  MotorCommand normal;
  normal.position = 0.2;
  normal.velocity = 0.0;
  normal.kp = 80.0;
  normal.kd = 5.0;
  normal.torque = 6.0;
  const auto encoded = YAMCanDriver::encode_mit(normal, MotorType::DM4340);
  assert(encoded.size() == 8);
  auto invalid = normal;
  invalid.position = 15.0;
  bool threw = false;
  try { (void)YAMCanDriver::encode_mit(invalid, MotorType::DM4340); }
  catch (const std::runtime_error&) { threw=true; }
  assert(threw);
  threw = false;
  invalid = normal;
  invalid.torque = 11.0; // Above DM4310 +/-10 Nm MIT range.
  try { (void)YAMCanDriver::encode_mit(invalid, MotorType::DM4310); }
  catch (const std::runtime_error&) { threw=true; }
  assert(threw);
  return 0;
}
