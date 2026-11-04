#include "yam_ros2_control/yam_live_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace yam_ros2_control {
namespace {
auto logger() { return rclcpp::get_logger("YAMLiveHardware"); }
bool yes(const std::string& v) { return v == "true" || v == "1"; }
constexpr double MOTOR_RANGE = 25.0;
constexpr double MAX_ENCODER_STEP = 0.35;  // rad/servo iteration
constexpr double MAX_FOLLOW_VELOCITY = 0.25; // rad/s; deliberately slower than I2RT
constexpr double MAX_FOLLOW_ERROR = 0.20;
constexpr double JOINT_MARGIN = 0.08;
constexpr double MAX_STARTUP_SENSOR_DISAGREEMENT = 0.15; // xout vs MIT feedback
constexpr auto MAX_CYCLE_GAP = std::chrono::milliseconds(150);

double nearest_wrap_delta(double current, double previous) {
  return std::remainder(current - previous, MOTOR_RANGE);
}
}

hardware_interface::CallbackReturn YAMLiveHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    if (info_.joints.size() != N) throw std::runtime_error("Expected exactly 6 arm joints");
    role_ = info_.hardware_parameters.at("role");
    channel_ = info_.hardware_parameters.at("can_interface");
    if (!((role_ == "leader" && channel_ == "can0") ||
          (role_ == "follower" && channel_ == "can1"))) {
      throw std::runtime_error("Unexpected role/CAN mapping; refusing to drive other bus");
    }
    allow_actuation_ = yes(info_.hardware_parameters.at("allow_actuation"));
    calibrated_ = yes(info_.hardware_parameters.at("calibration_verified"));
    estop_confirmed_ = yes(info_.hardware_parameters.at("emergency_stop_verified"));
    for (std::size_t i = 0; i < N; ++i) {
      const auto &j = info_.joints[i];
      if (j.name != "joint_" + std::to_string(i + 1) ||
          j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "position" ||
          j.state_interfaces.size() != 2) {
        throw std::runtime_error("Wrong joint interface: " + j.name);
      }
      bool position = false, velocity = false;
      for (const auto &s : j.state_interfaces) {
        position |= s.name == "position";
        velocity |= s.name == "velocity";
      }
      if (!position || !velocity) throw std::runtime_error("Missing joint state interface");
      offsets_[i] = std::stod(info_.hardware_parameters.at("offset_" + std::to_string(i+1)));
      if (!std::isfinite(offsets_[i])) throw std::runtime_error("Non-finite motor offset");
    }
    positions_.fill(std::numeric_limits<double>::quiet_NaN());
    commands_.fill(std::numeric_limits<double>::quiet_NaN());
    velocities_.fill(0.0);
    unwrapped_ready_.fill(false);
    motor_enabled_.fill(false);
    registers_ = std::make_unique<YAMCanRegisterReader>(channel_);

    if (allow_actuation_) {
      if (!calibrated_ || !estop_confirmed_) {
        throw std::runtime_error(
          "Actuation refused: offsets/axes AND independent E-stop must be verified first");
      }
      const auto model_file = info_.hardware_parameters.at("model_file");
      KDL::Tree tree;
      KDL::Chain chain;
      if (!kdl_parser::treeFromFile(model_file, tree) ||
          !tree.getChain("base", "gripper", chain) || chain.getNrOfJoints() != N) {
        throw std::runtime_error("Cannot build six-joint gravity chain from " + model_file);
      }
      gravity_chain_ = chain;
      gravity_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(gravity_chain_);
      // Reject a broken dynamics model before any actuator is enabled.
      KDL::JntArray neutral(N);
      for (std::size_t i = 0; i < N; ++i) neutral(i) = 0.0;
      (void)gravitational_potential(neutral);
    }
    initialized_ = true;
    RCLCPP_WARN(logger(), "%s on %s initialized. Physical actuation: %s",
      role_.c_str(), channel_.c_str(), allow_actuation_ ? "PERMITTED" : "BLOCKED");
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "Initialization failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> YAMLiveHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> states;
  for (std::size_t i = 0; i < N; ++i) {
    states.emplace_back(info_.joints[i].name, "position", &positions_[i]);
    states.emplace_back(info_.joints[i].name, "velocity", &velocities_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> YAMLiveHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> cmds;
  for (std::size_t i = 0; i < N; ++i) {
    cmds.emplace_back(info_.joints[i].name, "position", &commands_[i]);
  }
  return cmds;
}

void YAMLiveHardware::verify_registers() {
  for (std::size_t i = 0; i < N; ++i) {
    const auto id = static_cast<uint8_t>(i + 1);
    const double gr = registers_->read_float_register(id, 20);
    const double pmax = registers_->read_float_register(id, 21);
    const double vmax = registers_->read_float_register(id, 22);
    const double tmax = registers_->read_float_register(id, 23);
    const auto mode = registers_->read_uint_register(id, 10);
    const auto timeout = registers_->read_uint_register(id, 9);
    const double expected_v = i < 3 ? 10.0 : 30.0;
    const double expected_t = i < 3 ? 28.0 : 10.0;
    auto same = [](double a, double b) { return std::isfinite(a) && std::abs(a-b) < 1e-4; };
    if (!same(gr, i < 3 ? 40.0 : 10.0) || !same(pmax, 12.5) ||
        !same(vmax, expected_v) || !same(tmax, expected_t) || mode != 1 || timeout == 0) {
      throw std::runtime_error("Motor " + std::to_string(id) + " register mismatch");
    }
    if (timeout > 1000) {
      RCLCPP_WARN(logger(), "%s motor %u timeout %u ms: NOT an emergency stop",
        channel_.c_str(), id, timeout);
    }
  }
}

void YAMLiveHardware::read_encoder_snapshot(bool initial) {
  const auto now = std::chrono::steady_clock::now();
  const double dt = initial ? 0.0 :
    std::chrono::duration<double>(now - last_raw_read_).count();
  std::array<double,N> q{};
  for (std::size_t i=0; i<N; ++i) {
    q[i] = static_cast<double>(registers_->read_float_register(
      static_cast<uint8_t>(i + 1), 81)); // xout: output-shaft position
  }
  for (std::size_t i=0; i<N; ++i) {
    if (initial || !unwrapped_ready_[i]) {
      raw_unwrapped_[i] = q[i];
      unwrapped_ready_[i] = true;
      velocities_[i] = 0.0;
    } else {
      const double delta = nearest_wrap_delta(q[i], raw_last_[i]);
      if (!std::isfinite(delta) || std::abs(delta) > MAX_ENCODER_STEP) {
        throw std::runtime_error("Encoder jump on joint " + std::to_string(i+1));
      }
      raw_unwrapped_[i] += delta;
      velocities_[i] = dt > 0.0 ? delta/dt : 0.0;
    }
    raw_last_[i] = q[i];
    positions_[i] = raw_unwrapped_[i] - offsets_[i];
  }
  last_raw_read_ = now;
}

bool YAMLiveHardware::within_limits(const std::array<double,N> &q) const {
  for (std::size_t i=0; i<N; ++i) {
    if (!std::isfinite(q[i]) || q[i] < lower_[i] - JOINT_MARGIN ||
        q[i] > upper_[i] + JOINT_MARGIN) return false;
  }
  return true;
}

void YAMLiveHardware::put_feedback(std::size_t i, const MotorState &s) {
  if (s.error_code != 1 || !std::isfinite(s.position) ||
      !std::isfinite(s.velocity) || !std::isfinite(s.torque)) {
    throw std::runtime_error("Motor feedback invalid/fault on joint " + std::to_string(i+1));
  }
  if (!unwrapped_ready_[i]) {
    throw std::runtime_error("Cannot use MIT feedback without encoder initialization");
  }
  const double delta = nearest_wrap_delta(s.position, raw_last_[i]);
  if (!std::isfinite(delta) || std::abs(delta) > MAX_ENCODER_STEP) {
    throw std::runtime_error("MIT feedback/encoder discontinuity on joint " +
      std::to_string(i+1) + "; check motor zero and register 81 units");
  }
  raw_unwrapped_[i] += delta;
  raw_last_[i] = s.position;
  positions_[i] = raw_unwrapped_[i] - offsets_[i];
  velocities_[i] = s.velocity;
}

// The potential includes the mass and center of gravity of each segment in
// the six-joint base -> gripper chain. The fixed root base has no joint
// contribution and must not produce a torque about an actuated axis.
// U(q) = sum_i m_i * g * z_COM_i(q), with gravity along world -Z.
double YAMLiveHardware::gravitational_potential(const KDL::JntArray &q) const {
  if (!gravity_fk_ || q.rows() != gravity_chain_.getNrOfJoints()) {
    throw std::runtime_error("Gravity FK model absent or wrong joint count");
  }
  constexpr double G = 9.81;
  double potential = 0.0;
  for (unsigned int s = 0; s < gravity_chain_.getNrOfSegments(); ++s) {
    const auto &inertia = gravity_chain_.getSegment(s).getInertia();
    const double mass = inertia.getMass();
    if (!std::isfinite(mass) || mass < 0.0) {
      throw std::runtime_error("Invalid URDF link mass in gravity model");
    }
    if (mass == 0.0) continue;
    KDL::Frame link_pose;
    const int rc = gravity_fk_->JntToCart(q, link_pose, static_cast<int>(s + 1));
    if (rc < 0) {
      throw std::runtime_error("Gravity FK failed at segment " +
        std::to_string(s+1) + " (KDL code " + std::to_string(rc) + ")");
    }
    const double z = (link_pose * inertia.getCOG()).z();
    if (!std::isfinite(z)) {
      throw std::runtime_error("Non-finite URDF center-of-mass height");
    }
    potential += mass * G * z;
  }
  if (!std::isfinite(potential)) {
    throw std::runtime_error("Non-finite gravitational potential");
  }
  return potential;
}

std::array<double,YAMLiveHardware::N> YAMLiveHardware::gravity_torques() const {
  if (!gravity_fk_) throw std::runtime_error("Gravity model unavailable");
  KDL::JntArray q(N);
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(positions_[i])) {
      throw std::runtime_error("Non-finite measured joint position " + std::to_string(i+1));
    }
    q(i) = positions_[i];
  }

  // Static feedforward torque is dU/dq, canceling gravity (-dU/dq).
  // Centered finite differences avoid the ChainDynParam failure seen on
  // both CAN buses, while preserving the URDF's actual masses and geometry.
  constexpr double H = 1.0e-5; // rad
  std::array<double,N> output{};
  for (std::size_t i = 0; i < N; ++i) {
    q(i) = positions_[i] + H;
    const double plus = gravitational_potential(q);
    q(i) = positions_[i] - H;
    const double minus = gravitational_potential(q);
    q(i) = positions_[i];
    output[i] = (plus - minus) / (2.0 * H) * gravity_factor_[i];
    if (!std::isfinite(output[i]) || std::abs(output[i]) > torque_caps_[i]) {
      throw std::runtime_error("Gravity torque invalid/out of bounds on joint " +
        std::to_string(i + 1) + ": " + std::to_string(output[i]) + " Nm");
    }
  }
  return output;
}

MotorCommand YAMLiveHardware::startup_command(
    std::size_t i, double current_motor_position,
    const std::array<double,N> &gravity) const {
  MotorCommand cmd;
  cmd.position = current_motor_position;
  cmd.velocity = 0.0;
  cmd.kp = role_ == "follower" ? kp_[i] : 0.0;
  cmd.kd = role_ == "follower" ? kd_[i] : gravity_kd_[i];
  cmd.torque = gravity[i];
  return cmd;
}

void YAMLiveHardware::turn_off_best_effort() noexcept {
  if (!control_bus_) return;
  // DM disable eliminates commanded torque, but does NOT mechanically arrest the arm.
  // The cell must provide physical support and an independent stop for all commissioning.
  for (std::size_t i=0; i<N; ++i) {
    if (!motor_enabled_[i]) continue;
    try {
      control_bus_->disable_motor(static_cast<uint32_t>(i+1));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger(), "%s motor-off joint %zu failed: %s",
        role_.c_str(), i+1, e.what());
    }
    motor_enabled_[i] = false;
  }
}

hardware_interface::CallbackReturn YAMLiveHardware::on_activate(
    const rclcpp_lifecycle::State &) {
  if (!initialized_ || faulted_) return hardware_interface::CallbackReturn::ERROR;
  try {
    verify_registers();
    read_encoder_snapshot(true);
    if (!within_limits(positions_)) {
      throw std::runtime_error("Joint angles out of URDF bounds; verify absolute zero offsets");
    }
    commands_ = positions_; // hold ACTUAL measured position, not arbitrary zero
    startup_hold_ = positions_;
    follower_setpoint_ = positions_;
    follower_command_seen_ = false;
    if (!allow_actuation_) {
      active_ = true;
      RCLCPP_WARN(logger(), "%s READ-ONLY on %s. No MIT motor commands sent",
        role_.c_str(), channel_.c_str());
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    // Compute every feedforward torque BEFORE any motor enable command.
    // Both follower HOLD and leader FLOAT include gravity compensation.
    auto gravity = gravity_torques();
    RCLCPP_INFO(logger(),
      "%s gravity feedforward at measured startup pose [Nm]: "
      "j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
      role_.c_str(), gravity[0], gravity[1], gravity[2],
      gravity[3], gravity[4], gravity[5]);
    control_bus_ = std::make_unique<YAMCanDriver>(channel_);
    for (std::size_t i=0; i<N; ++i) {
      const auto id = static_cast<uint32_t>(i + 1);
      const auto response_id = static_cast<uint32_t>(0x11 + i);
      // Set cleanup flag BEFORE transmission: a CAN reply timeout does not
      // prove that the motor ignored the enable frame.
      motor_enabled_[i] = true;
      const auto enabled = control_bus_->enable_motor(id, response_id, types_[i], 20);
      // This reconciles the output-shaft register and MIT's wrapping register.
      if (std::abs(nearest_wrap_delta(enabled.position, raw_last_[i])) >
          MAX_STARTUP_SENSOR_DISAGREEMENT) {
        throw std::runtime_error("Register/MIT startup pose mismatch at joint " +
          std::to_string(i+1));
      }
      put_feedback(i, enabled);
      gravity = gravity_torques();
      auto cmd = startup_command(i, enabled.position, gravity);
      auto feedback = control_bus_->exchange(id, response_id, types_[i], cmd, 10);
      put_feedback(i, feedback);
    }
    if (!within_limits(positions_)) {
      throw std::runtime_error("Post-enable measured joint pose outside bounds");
    }
    commands_ = positions_;
    startup_hold_ = positions_;
    follower_setpoint_ = positions_;
    follower_command_seen_ = false;
    last_cycle_ = std::chrono::steady_clock::now();
    active_ = true;
    RCLCPP_WARN(logger(), "%s LIVE on %s. Physical support and independent E-stop mandatory",
      role_.c_str(), channel_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger(), "%s ACTUATION FAILURE: %s", role_.c_str(), e.what());
    faulted_ = true;
    active_ = false;
    turn_off_best_effort();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn YAMLiveHardware::on_deactivate(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  turn_off_best_effort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

YAMLiveHardware::~YAMLiveHardware() { turn_off_best_effort(); }

hardware_interface::return_type YAMLiveHardware::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  if (role_ != "follower") return hardware_interface::return_type::OK;
  const std::string first_joint = "joint_1/position";
  if (std::find(stop_interfaces.begin(), stop_interfaces.end(), first_joint) !=
      stop_interfaces.end()) {
    follower_controller_claimed_ = false;
    follower_command_seen_ = false;
    if (active_) {
      startup_hold_ = positions_;
      follower_setpoint_ = positions_;
    }
  }
  if (std::find(start_interfaces.begin(), start_interfaces.end(), first_joint) !=
      start_interfaces.end()) {
    follower_controller_claimed_ = true;
    follower_command_seen_ = false;
    follower_command_ready_at_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(300);
    RCLCPP_INFO(logger(), "Follower controller claimed position interfaces; startup hold continues for 300 ms");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YAMLiveHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_ || faulted_) return hardware_interface::return_type::ERROR;
  if (allow_actuation_) {
    // In live mode, the read() state is the fresh MIT feedback received in write().
    return hardware_interface::return_type::OK;
  }
  try {
    read_encoder_snapshot(false);
    return hardware_interface::return_type::OK;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger(), "Read-only CAN feedback fault: %s", e.what());
    faulted_ = true;
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type YAMLiveHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &period) {
  if (!active_ || faulted_) return hardware_interface::return_type::ERROR;
  if (!allow_actuation_) return hardware_interface::return_type::OK;
  try {
    auto now = std::chrono::steady_clock::now();
    if (now - last_cycle_ > MAX_CYCLE_GAP)
      throw std::runtime_error("Servo scheduler stalled >150 ms");
    last_cycle_ = now;
    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.15)
      throw std::runtime_error("Invalid servo cycle duration");
    if (!within_limits(positions_))
      throw std::runtime_error("Measured joint outside URDF safety limits");

    auto gravity = gravity_torques();
    std::array<MotorCommand,N> outputs{};
    if (role_ == "leader") {
      for (std::size_t i=0; i<N; ++i) {
        outputs[i] = startup_command(i, raw_last_[i], gravity);
      }
    } else {
      // JointGroupPositionController may not have received any ROS command
      // immediately after it activates. Reject its uninitialized/default
      // values until a complete command matches the captured startup pose.
      // This prevents a controller startup from commanding an all-zero pose.
      if (!follower_command_seen_) {
        bool aligned = follower_controller_claimed_ &&
          std::chrono::steady_clock::now() >= follower_command_ready_at_ &&
          within_limits(commands_);
        if (aligned) {
          for (std::size_t i=0; i<N; ++i) {
            if (std::abs(commands_[i] - startup_hold_[i]) > 0.10) {
              aligned = false;
            }
          }
        }
        if (aligned) {
          follower_setpoint_ = positions_;  // begin commanded trajectory at physical pose
          follower_command_seen_ = true;
          RCLCPP_INFO(logger(), "Follower accepted aligned ROS position-controller commands");
        }
      }
      if (follower_command_seen_ && !within_limits(commands_))
        throw std::runtime_error("Follower commanded target outside joint limits");
      const auto &requested = follower_command_seen_ ? commands_ : startup_hold_;
      for (std::size_t i=0; i<N; ++i) {
        // TWO different quantities must not be confused:
        //   requested - measured: whole-loop tracking error (fault guard)
        //   requested - setpoint: remaining distance in the commanded trajectory
        // The previous implementation reset the target to measured + 0.005 rad
        // EVERY 20-ms cycle; that capped the motor's proportional correction to
        // ~0.4 Nm (large joints) or ~0.05 Nm (small joints) and the follower
        // could drift instead of moving toward the leader. Here, the target
        // progresses cumulatively at a bounded joint speed.
        const double error = requested[i] - positions_[i];
        if (!std::isfinite(error) || std::abs(error) > MAX_FOLLOW_ERROR) {
          throw std::runtime_error("Follower position error on joint " +
            std::to_string(i+1) + " commanded=" + std::to_string(requested[i]) +
            " measured=" + std::to_string(positions_[i]) +
            " error=" + std::to_string(error) +
            " (limit=" + std::to_string(MAX_FOLLOW_ERROR) + " rad)");
        }
        const double advance = std::clamp(
          requested[i] - follower_setpoint_[i],
          -MAX_FOLLOW_VELOCITY * dt, MAX_FOLLOW_VELOCITY * dt);
        follower_setpoint_[i] += advance;
        const double servo_error = follower_setpoint_[i] - positions_[i];
        if (!std::isfinite(servo_error) || std::abs(servo_error) > MAX_FOLLOW_ERROR) {
          throw std::runtime_error("Follower servo lag exceeds limit on joint " +
            std::to_string(i+1) + ": " + std::to_string(servo_error) + " rad");
        }
        const double absolute_motor_target = follower_setpoint_[i] + offsets_[i];
        outputs[i] = {
          std::remainder(absolute_motor_target, MOTOR_RANGE), 0.0,
          kp_[i], kd_[i], gravity[i]};
      }
    }
    // Encode all six before sending ANY command. A bad command cannot
    // cause half a servo update to go out before throwing.
    for (std::size_t i=0; i<N; ++i) {
      (void)YAMCanDriver::encode_mit(outputs[i], types_[i]);
    }
    for (std::size_t i=0; i<N; ++i) {
      const auto state = control_bus_->exchange(
        static_cast<uint32_t>(i+1), static_cast<uint32_t>(0x11+i),
        types_[i], outputs[i], 8);
      put_feedback(i, state);
    }
    if (!within_limits(positions_))
      throw std::runtime_error("Feedback exceeded URDF joint limits");
    return hardware_interface::return_type::OK;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger(), "LIVE %s CONTROL FAULT: %s", role_.c_str(), e.what());
    faulted_ = true;
    active_ = false;
    // Disabled motors do not retain holding torque. External support is mandatory.
    turn_off_best_effort();
    return hardware_interface::return_type::ERROR;
  }
}
}  // namespace yam_ros2_control

PLUGINLIB_EXPORT_CLASS(yam_ros2_control::YAMLiveHardware, hardware_interface::SystemInterface)
