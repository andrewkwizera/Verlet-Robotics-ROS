#include "yam_ros2_control/yam_live_hardware.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace yam_ros2_control {
namespace {
auto log() { return rclcpp::get_logger("YAMLiveHardware"); }
bool yes(const std::string &value) { return value == "true" || value == "1"; }
constexpr double PI = 3.14159265358979323846;
constexpr double MOTOR_RANGE = 25.0;
constexpr double MAX_SAMPLE_DELTA = 0.35; // Fail if reported movement >0.35 rad per 50-Hz cycle.
constexpr double MAX_CMD_RATE = 0.25;      // Max 0.25 rad/s requested follower motion.
constexpr double MAX_FOLLOW_ERROR = 0.20;  // Reject large errors rather than jumping to target.
constexpr double JOINT_SOFT_MARGIN = 0.08;
constexpr double TORQUE_CAP = 5.0;         // Conservative initial gravity feedforward cap.
constexpr auto MAX_CYCLE_GAP = std::chrono::milliseconds(150);
}

hardware_interface::CallbackReturn YAMLiveHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  try {
    if (info_.joints.size() != N) throw std::runtime_error("Expected six joints");
    role_ = info_.hardware_parameters.at("role");
    channel_ = info_.hardware_parameters.at("can_interface");
    if (!((role_ == "leader" && channel_ == "can0") ||
          (role_ == "follower" && channel_ == "can1")))
      throw std::runtime_error("Unexpected role/CAN mapping");
    allow_actuation_ = yes(info_.hardware_parameters.at("allow_actuation"));
    calibrated_ = yes(info_.hardware_parameters.at("calibration_verified"));
    estop_confirmed_ = yes(info_.hardware_parameters.at("emergency_stop_verified"));
    for (std::size_t i=0; i<N; ++i) {
      const auto &j = info_.joints[i];
      if (j.name != "joint_" + std::to_string(i+1) ||
          j.command_interfaces.size()!=1 || j.command_interfaces[0].name!="position" ||
          j.state_interfaces.size()!=2)
        throw std::runtime_error("Incorrect joint interface " + j.name);
      bool q=false,dq=false;
      for (const auto &s: j.state_interfaces) {
        q |= s.name=="position"; dq |= s.name=="velocity";
      }
      if (!q || !dq) throw std::runtime_error("Missing state interfaces");
      offsets_[i] = std::stod(info_.hardware_parameters.at("offset_"+std::to_string(i+1)));
      if (!std::isfinite(offsets_[i])) throw std::runtime_error("Non-finite joint offset");
    }
    registers_ = std::make_unique<YAMCanRegisterReader>(channel_);
    positions_.fill(std::numeric_limits<double>::quiet_NaN());
    velocities_.fill(0);
    commands_.fill(std::numeric_limits<double>::quiet_NaN());
    unwrapped_ready_.fill(false);
    if (role_ == "leader" && allow_actuation_) {
      const std::string model_file = info_.hardware_parameters.at("model_file");
      KDL::Tree tree;
      if (!kdl_parser::treeFromFile(model_file, tree))
        throw std::runtime_error("Cannot parse dynamics URDF: " + model_file);
      KDL::Chain chain;
      if (!tree.getChain("base", "gripper", chain) || chain.getNrOfJoints()!=N)
        throw std::runtime_error("Dynamics chain base -> gripper must have 6 joints");
      gravity_solver_ = std::make_unique<KDL::ChainDynParam>(chain,KDL::Vector(0,0,-9.81));
    }
    initialized_=true;
    RCLCPP_WARN(log(), "%s %s initialized; allow_actuation=%s; NO motors enabled yet",
      role_.c_str(),channel_.c_str(),allow_actuation_?"true":"false");
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(log(), "init failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> YAMLiveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for(std::size_t i=0;i<N;++i) {
    v.emplace_back(info_.joints[i].name,"position",&positions_[i]);
    v.emplace_back(info_.joints[i].name,"velocity",&velocities_[i]);
  }
  return v;
}
std::vector<hardware_interface::CommandInterface> YAMLiveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for(std::size_t i=0;i<N;++i) v.emplace_back(info_.joints[i].name,"position",&commands_[i]);
  return v;
}

void YAMLiveHardware::verify_registers()
{
  for(std::size_t i=0;i<N;++i) {
    const uint8_t id=static_cast<uint8_t>(i+1);
    const double gear = registers_->read_float_register(id,20);
    const double pmax = registers_->read_float_register(id,21);
    const double vmax = registers_->read_float_register(id,22);
    const double tmax = registers_->read_float_register(id,23);
    const uint32_t mode=registers_->read_uint_register(id,10);
    const uint32_t timeout=registers_->read_uint_register(id,9);
    const double expect_gear=i<3?40.0:10.0;
    const double expect_v=i<3?10.0:30.0;
    const double expect_t=i<3?28.0:10.0;
    const auto eq=[](double a,double b){return std::isfinite(a)&&std::abs(a-b)<1e-5;};
    if (!eq(gear,expect_gear) || !eq(pmax,12.5) || !eq(vmax,expect_v) ||
        !eq(tmax,expect_t) || mode!=1 || timeout==0)
      throw std::runtime_error("Motor " + std::to_string(id) + " config mismatch or no watchdog");
    if (timeout > 1000)
      RCLCPP_WARN(log(), "%s motor %u firmware timeout %u ms is not a fast E-stop",
        channel_.c_str(),id,timeout);
  }
}

bool YAMLiveHardware::read_raw_positions()
{
  std::array<double,N> q{};
  for(std::size_t i=0;i<N;++i) {
    q[i] = static_cast<double>(registers_->read_float_register(static_cast<uint8_t>(i+1),81));
    if (!std::isfinite(q[i])) return false;
  }
  for(std::size_t i=0;i<N;++i) {
    raw_last_[i]=q[i]; raw_unwrapped_[i]=q[i]; unwrapped_ready_[i]=true;
    positions_[i]=q[i]-offsets_[i]; velocities_[i]=0;
  }
  return true;
}

bool YAMLiveHardware::within_limits(const std::array<double,N> &q) const
{
  for(std::size_t i=0;i<N;++i) {
    if (!std::isfinite(q[i]) || q[i]<lower_[i]-JOINT_SOFT_MARGIN ||
       q[i]>upper_[i]+JOINT_SOFT_MARGIN) return false;
  }
  return true;
}

void YAMLiveHardware::put_feedback(std::size_t i, const MotorState &s)
{
  if (s.error_code!=1 || !std::isfinite(s.position)||!std::isfinite(s.velocity))
    throw std::runtime_error("Motor feedback invalid or fault on joint " + std::to_string(i+1));
  if (!unwrapped_ready_[i]) {
    raw_unwrapped_[i]=s.position;
    unwrapped_ready_[i]=true;
  } else {
    double delta=s.position-raw_last_[i];
    if(delta> MOTOR_RANGE/2) delta-=MOTOR_RANGE;
    if(delta<-MOTOR_RANGE/2) delta+=MOTOR_RANGE;
    if(std::abs(delta)>MAX_SAMPLE_DELTA)
      throw std::runtime_error("Implausible joint position change " + std::to_string(i+1));
    raw_unwrapped_[i]+=delta;
  }
  raw_last_[i]=s.position;
  positions_[i]=raw_unwrapped_[i]-offsets_[i];
  velocities_[i]=s.velocity;
}

std::array<double,YAMLiveHardware::N> YAMLiveHardware::gravity_torques() const
{
  if (!gravity_solver_) throw std::runtime_error("Missing KDL gravity model");
  KDL::JntArray q(N), t(N);
  for(std::size_t i=0;i<N;++i) q(i)=positions_[i];
  if(gravity_solver_->JntToGravity(q,t)<0)
    throw std::runtime_error("KDL gravity solver failed");
  std::array<double,N> out{};
  for(std::size_t i=0;i<N;++i) {
    double value=t(i)*gravity_factor_[i];
    if(!std::isfinite(value)||std::abs(value)>TORQUE_CAP)
      throw std::runtime_error("Gravity torque outside commissioned cap at joint " + std::to_string(i+1));
    out[i]=value;
  }
  return out;
}

void YAMLiveHardware::turn_off_best_effort() noexcept
{
  if (!motors_enabled_ || !control_bus_) return;
  RCLCPP_ERROR(log(), "%s motor disable requested; robot MUST be mechanically supported", role_.c_str());
  for(std::size_t i=0;i<N;++i) {
    try { control_bus_->disable_motor(static_cast<uint32_t>(i+1)); }
    catch(const std::exception &e) { RCLCPP_ERROR(log(),"Motor disable failed: %s",e.what()); }
  }
  motors_enabled_=false;
}

hardware_interface::CallbackReturn YAMLiveHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!initialized_||faulted_) return hardware_interface::CallbackReturn::ERROR;
  try {
    verify_registers();
    if (!read_raw_positions() || !within_limits(positions_))
      throw std::runtime_error("Raw joint poses outside commissioned limits; verify offsets");
    commands_=positions_;
    if (!allow_actuation_) {
      active_=true;
      RCLCPP_WARN(log(), "%s active in READ-ONLY mode (enable_actuation=false)",role_.c_str());
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    if (!calibrated_ || !estop_confirmed_)
      throw std::runtime_error("Actuation refused: calibration_verified and emergency_stop_verified must both be true");
    if (role_=="leader") (void)gravity_torques();
    control_bus_=std::make_unique<YAMCanDriver>(channel_);
    motors_enabled_=true; // Ensure best-effort disarm even if enabling motor 2-6 fails.
    unwrapped_ready_.fill(false); // Start unwrap from MIT enable replies, not xout register.
    for(std::size_t i=0;i<N;++i) {
      const auto state=control_bus_->enable_motor(i+1,0x11+i,types_[i],20);
      put_feedback(i,state);
      // The leader is unsupported in torque control until gravity updates; support both arms.
      MotorCommand startup{};
      startup.position=state.position;
      startup.velocity=0;
      startup.kp=role_=="follower"?kp_[i]:0.0;
      startup.kd=role_=="follower"?kd_[i]:leader_kd_[i];
      startup.torque=0;
      (void)control_bus_->exchange(i+1,0x11+i,types_[i],startup,10);
    }
    if (!within_limits(positions_)) throw std::runtime_error("Measured enabled pose violates joint limits");
    commands_=positions_;
    last_cycle_=std::chrono::steady_clock::now();
    active_=true;
    RCLCPP_WARN(log(), "LIVE %s ACTIVE on %s. Manual support and independent E-stop REQUIRED",
      role_.c_str(),channel_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch(const std::exception &e) {
    RCLCPP_ERROR(log(),"LIVE activation failed: %s",e.what());
    faulted_=true;active_=false;
    turn_off_best_effort();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn YAMLiveHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_=false;
  turn_off_best_effort();
  return hardware_interface::CallbackReturn::SUCCESS;
}
YAMLiveHardware::~YAMLiveHardware() { turn_off_best_effort(); }

hardware_interface::return_type YAMLiveHardware::read(const rclcpp::Time &,const rclcpp::Duration &)
{
  if(!active_||faulted_) return hardware_interface::return_type::ERROR;
  if(!allow_actuation_) {
    try {
      if(!read_raw_positions()) throw std::runtime_error("No motor register feedback");
      return hardware_interface::return_type::OK;
    } catch(const std::exception &e) {
      RCLCPP_ERROR(log(),"Read-only feedback failed: %s",e.what());
      faulted_=true;
      return hardware_interface::return_type::ERROR;
    }
  }
  // MIT mode feedback is decoded in write() immediately after each command.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YAMLiveHardware::write(const rclcpp::Time &, const rclcpp::Duration &period)
{
  if(!active_||faulted_) return hardware_interface::return_type::ERROR;
  if(!allow_actuation_) return hardware_interface::return_type::OK;
  try {
    const auto now=std::chrono::steady_clock::now();
    if(now-last_cycle_>MAX_CYCLE_GAP) throw std::runtime_error("Controller manager scheduling watchdog expired");
    last_cycle_=now;
    const double dt=period.seconds();
    if (!std::isfinite(dt)||dt<=0||dt>0.15) throw std::runtime_error("Invalid control period");
    if(!within_limits(positions_)) throw std::runtime_error("Measured position outside joint limits");
    std::array<MotorCommand,N> outputs{};
    if(role_=="leader") {
      const auto torques=gravity_torques();
      for(std::size_t i=0;i<N;++i) {
        outputs[i]={raw_last_[i],0,0,leader_kd_[i],torques[i]};
      }
    } else {
      if(!within_limits(commands_)) throw std::runtime_error("Follower target outside joint limits");
      for(std::size_t i=0;i<N;++i) {
        const double error=commands_[i]-positions_[i];
        if(std::abs(error)>MAX_FOLLOW_ERROR)
          throw std::runtime_error("Follower tracking error exceeded limit at joint " + std::to_string(i+1));
        const double target=positions_[i]+std::clamp(error,-MAX_CMD_RATE*dt,MAX_CMD_RATE*dt);
        // MIT position must remain in [-12.5,12.5] (only the arm-space angle is unwrapped).
        double motor_target = target + offsets_[i];
        motor_target = std::remainder(motor_target, MOTOR_RANGE);
        outputs[i]={motor_target,0,kp_[i],kd_[i],0};
      }
    }
    for(std::size_t i=0;i<N;++i) (void)YAMCanDriver::encode_mit(outputs[i],types_[i]);
    for(std::size_t i=0;i<N;++i) {
      const MotorState feedback=control_bus_->exchange(i+1,0x11+i,types_[i],outputs[i],8);
      put_feedback(i,feedback);
    }
    if(!within_limits(positions_)) throw std::runtime_error("Post-command joint limit violation");
    return hardware_interface::return_type::OK;
  } catch(const std::exception &e) {
    RCLCPP_ERROR(log(),"LIVE CONTROL FAULT (%s): %s",role_.c_str(),e.what());
    faulted_=true;active_=false;
    turn_off_best_effort(); // Unsupported arms may fall; mechanical support is mandatory.
    return hardware_interface::return_type::ERROR;
  }
}
} // namespace yam_ros2_control
PLUGINLIB_EXPORT_CLASS(yam_ros2_control::YAMLiveHardware,hardware_interface::SystemInterface)
