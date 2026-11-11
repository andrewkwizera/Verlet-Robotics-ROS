
#include "yam_ros2_control/yam_can_driver.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <algorithm>

namespace yam_ros2_control
{

namespace
{

struct MotorLimits
{
  double velocity;
  double torque;
};

MotorLimits limits_for(MotorType type)
{
  if (type == MotorType::DM4340) {
    return {10.0, 28.0};
  }
  return {30.0, 10.0};
}

uint32_t float_to_uint(
  double value, double minimum,
  double maximum, unsigned int bits)
{
  if (!std::isfinite(value) ||
      value < minimum || value > maximum)
  {
    throw std::runtime_error(
      "Motor command outside permitted encoding range");
  }

  const uint32_t max_int = (1u << bits) - 1u;

  return static_cast<uint32_t>(
    (value - minimum) * max_int /
    (maximum - minimum));
}

double uint_to_float(
  uint32_t value, double minimum,
  double maximum, unsigned int bits)
{
  const uint32_t max_int = (1u << bits) - 1u;

  return static_cast<double>(value) *
    (maximum - minimum) / max_int + minimum;
}

}  // namespace


YAMCanDriver::YAMCanDriver(const std::string & channel)
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (socket_fd_ < 0) {
    throw std::runtime_error(
      "Failed to create CAN socket");
  }

  unsigned int index = if_nametoindex(channel.c_str());

  if (index == 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error(
      "CAN interface not found: " + channel);
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(index);

  if (bind(
      socket_fd_,
      reinterpret_cast<sockaddr *>(&address),
      sizeof(address)) < 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("CAN bind failed");
  }
}


YAMCanDriver::~YAMCanDriver()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}


std::array<uint8_t, 8> YAMCanDriver::encode_mit(
  const MotorCommand & c,
  MotorType type)
{
  auto limits = limits_for(type);

  uint32_t p = float_to_uint(
    c.position, -12.5, 12.5, 16);

  uint32_t v = float_to_uint(
    c.velocity, -limits.velocity,
    limits.velocity, 12);

  uint32_t kp = float_to_uint(
    c.kp, 0.0, 500.0, 12);

  uint32_t kd = float_to_uint(
    c.kd, 0.0, 5.0, 12);

  uint32_t t = float_to_uint(
    c.torque, -limits.torque,
    limits.torque, 12);

  return {{
    static_cast<uint8_t>(p >> 8),
    static_cast<uint8_t>(p & 0xff),
    static_cast<uint8_t>(v >> 4),
    static_cast<uint8_t>(((v & 0xf) << 4) | (kp >> 8)),
    static_cast<uint8_t>(kp & 0xff),
    static_cast<uint8_t>(kd >> 4),
    static_cast<uint8_t>(((kd & 0xf) << 4) | (t >> 8)),
    static_cast<uint8_t>(t & 0xff)
  }};
}


MotorState YAMCanDriver::decode_feedback(
  const std::array<uint8_t, 8> & data,
  MotorType type)
{
  auto limits = limits_for(type);

  uint32_t p =
    (static_cast<uint32_t>(data[1]) << 8) | data[2];

  uint32_t v =
    (static_cast<uint32_t>(data[3]) << 4) |
    (data[4] >> 4);

  uint32_t t =
    ((static_cast<uint32_t>(data[4]) & 0x0f) << 8) |
    data[5];

  MotorState state;

  state.position = uint_to_float(
    p, -12.5, 12.5, 16);

  state.velocity = uint_to_float(
    v, -limits.velocity,
    limits.velocity, 12);

  state.torque = uint_to_float(
    t, -limits.torque,
    limits.torque, 12);

  state.temperature_mos = data[6];
  state.temperature_rotor = data[7];
  state.error_code = (data[0] >> 4) & 0x0f;

  return state;
}


MotorState YAMCanDriver::exchange(
  uint32_t motor_id,
  uint32_t response_id,
  MotorType type,
  const MotorCommand & command,
  int timeout_ms)
{
  if (motor_id > CAN_SFF_MASK ||
      response_id > CAN_SFF_MASK ||
      timeout_ms <= 0)
  {
    throw std::runtime_error(
      "Invalid CAN identifier or timeout");
  }

  can_frame tx{};
  tx.can_id = motor_id;
  tx.can_dlc = 8;

  auto encoded = encode_mit(command, type);

  for (size_t i = 0; i < encoded.size(); ++i) {
    tx.data[i] = encoded[i];
  }

  // Must only be called after safe motor initialization.
  if (write(socket_fd_, &tx, sizeof(tx)) != sizeof(tx)) {
    throw std::runtime_error("CAN transmit failed");
  }

  return wait_for_feedback(
    response_id, type, timeout_ms);
}


MotorState YAMCanDriver::wait_for_feedback(
  uint32_t response_id,
  MotorType type,
  int timeout_ms)
{
  if (response_id > CAN_SFF_MASK || timeout_ms <= 0) {
    throw std::runtime_error(
      "Invalid CAN response ID or timeout");
  }

  using Clock = std::chrono::steady_clock;

  const auto deadline =
    Clock::now() + std::chrono::milliseconds(timeout_ms);

  while (Clock::now() < deadline) {
    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now());

    pollfd descriptor{};
    descriptor.fd = socket_fd_;
    descriptor.events = POLLIN;

    const int result = poll(
      &descriptor,
      1,
      std::max(1, static_cast<int>(remaining.count())));

    if (result == 0) {
      break;
    }

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(
        "CAN poll failed: " +
        std::string(std::strerror(errno)));
    }

    if (!(descriptor.revents & POLLIN)) {
      throw std::runtime_error(
        "CAN socket reported an error");
    }

    can_frame frame{};

    const ssize_t bytes = read(
      socket_fd_, &frame, sizeof(frame));

    if (bytes != sizeof(frame)) {
      throw std::runtime_error(
        "Incomplete CAN feedback frame");
    }

    if ((frame.can_id &
         (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0)
    {
      continue;
    }

    if ((frame.can_id & CAN_SFF_MASK) != response_id ||
        frame.can_dlc != 8)
    {
      continue;
    }

    std::array<uint8_t, 8> data{};

    for (size_t i = 0; i < 8; ++i) {
      data[i] = frame.data[i];
    }

    MotorState state = decode_feedback(data, type);

    if (state.error_code != 1) {
      throw std::runtime_error(
        "Motor reported fault code " +
        std::to_string(state.error_code));
    }

    return state;
  }

  throw std::runtime_error(
    "CAN motor feedback timeout");
}


MotorState YAMCanDriver::enable_motor(
  uint32_t motor_id,
  uint32_t response_id,
  MotorType type,
  int timeout_ms)
{
  if (motor_id == 0 || motor_id > CAN_SFF_MASK ||
      response_id > CAN_SFF_MASK || timeout_ms <= 0)
  {
    throw std::runtime_error(
      "Invalid motor initialization parameters");
  }

  can_frame frame{};

  frame.can_id = motor_id;
  frame.can_dlc = 8;

  // DM motor MIT enable command.
  for (size_t i = 0; i < 7; ++i) {
    frame.data[i] = 0xFF;
  }
  frame.data[7] = 0xFC;

  const ssize_t written = write(
    socket_fd_, &frame, sizeof(frame));

  if (written != sizeof(frame)) {
    throw std::runtime_error(
      "Failed to transmit motor enable command");
  }

  return wait_for_feedback(
    response_id, type, timeout_ms);
}

void YAMCanDriver::disable_motor(uint32_t motor_id)
{
  if (motor_id < 1 || motor_id > 6) {
    throw std::runtime_error("Invalid motor ID for disable");
  }
  can_frame tx{};
  tx.can_id = motor_id;
  tx.can_dlc = 8;
  for (std::size_t i = 0; i < 7; ++i) { tx.data[i] = 0xFF; }
  tx.data[7] = 0xFD;
  if (::write(socket_fd_, &tx, sizeof(tx)) != sizeof(tx)) {
    throw std::runtime_error("Failed to send motor disable frame");
  }
}

}  // namespace yam_ros2_control