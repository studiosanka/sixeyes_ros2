#include "sixeyes_hardware/sixeyes_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "sixeyes/serial.hpp"

namespace sixeyes_hardware
{
namespace
{
// The arm's joints in protocol order (see sixeyes_protocol.h SIXEYES_JOINT_*).
const char * const kJointNames[SIXEYES_JOINT_COUNT] = {
  "base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"};

constexpr int kGripper = SIXEYES_JOINT_GRIPPER;

// If no state arrives for this long, the USB link is treated as lost.
constexpr double kMaxStateAgeS = 0.5;
// A refusal is reported at most once per this many write cycles, so a stuck command does not flood the log.
constexpr unsigned kWarnEveryCycles = 100;

rclcpp::Logger logger() {return rclcpp::get_logger("SixEyesSystem");}

std::string parameterOr(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key,
  const std::string & fallback)
{
  const auto it = parameters.find(key);
  return it == parameters.end() ? fallback : it->second;
}
}  // namespace

hardware_interface::CallbackReturn SixEyesSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;
  const auto port = p.find("port");
  if (port == p.end() || port->second.empty()) {
    RCLCPP_FATAL(logger(), "The hardware parameter 'port' is missing. Set it to the arm's USB port, for example /dev/ttyACM0.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  port_ = port->second;
  try {
    gripper_stroke_m_ = std::stod(parameterOr(p, "gripper_stroke_m", "0.04"));
    max_step_rad_ = std::stod(parameterOr(p, "max_step_rad", "0.35"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "A hardware parameter is not a number: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  home_on_activate_ = parameterOr(p, "home_on_activate", "true") != "false";

  // Find each of the arm's joints by name, whatever order the URDF lists them in.
  state_names_.assign(SIXEYES_JOINT_COUNT, "");
  command_names_.assign(SIXEYES_JOINT_COUNT, "");
  for (int i = 0; i < SIXEYES_JOINT_COUNT; i++) {
    const auto joint = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [&](const hardware_interface::ComponentInfo & c) {return c.name == kJointNames[i];});
    if (joint == info_.joints.end()) {
      RCLCPP_FATAL(logger(), "The URDF has no joint called '%s'. The arm needs: base, shoulder, elbow, wrist_pitch, wrist_roll, gripper.", kJointNames[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint->command_interfaces.size() != 1 ||
      joint->command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger(), "Joint '%s' must have exactly one command interface: position.", kJointNames[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool has_position_state = std::any_of(
      joint->state_interfaces.begin(), joint->state_interfaces.end(),
      [](const hardware_interface::InterfaceInfo & s) {return s.name == hardware_interface::HW_IF_POSITION;});
    if (!has_position_state) {
      RCLCPP_FATAL(logger(), "Joint '%s' must have a position state interface.", kJointNames[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }
    state_names_[i] = joint->name + "/" + hardware_interface::HW_IF_POSITION;
    command_names_[i] = joint->name + "/" + hardware_interface::HW_IF_POSITION;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SixEyesSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  auto serial = sixeyes::SerialPort::open(port_, error);
  if (!serial) {
    RCLCPP_ERROR(logger(), "Could not open %s: %s", port_.c_str(), error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  session_ = std::make_unique<sixeyes::Session>(std::move(serial));

  error = session_->connect();
  if (!error.empty()) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    session_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  const sixeyes::ArmInfo info = session_->info();
  if ((info.capabilities & SIXEYES_CAP_HOMING) == 0) {
    RCLCPP_ERROR(logger(), "This arm's firmware cannot home yet, so it cannot be driven from a computer. Update the firmware.");
    session_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  error = session_->setControlUsb();
  if (!error.empty()) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    session_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger(), "Connected to the arm on %s (firmware %u.%u.%u, protocol v%u).", port_.c_str(),
    static_cast<unsigned>(info.fw_major), static_cast<unsigned>(info.fw_minor),
    static_cast<unsigned>(info.fw_patch), static_cast<unsigned>(info.protocol_version));

  sixeyes::ArmState state;
  if (session_->state(state)) {
    publishMeasured(state, true);
  }
  fault_reported_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SixEyesSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!session_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (home_on_activate_) {
    RCLCPP_WARN(logger(), "Homing the arm. Keep clear of it.");
    std::string error = session_->home();
    if (error.empty()) {
      error = session_->waitReady(120000);
    }
    if (!error.empty()) {
      RCLCPP_ERROR(logger(), "%s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  sixeyes::ArmState state;
  if (!session_->state(state)) {
    RCLCPP_ERROR(logger(), "No data from the arm yet.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Start by holding where the arm is, so activating never makes it jump.
  publishMeasured(state, true);
  fault_reported_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SixEyesSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (session_) {
    session_->stop();  // hold where it is; the result does not matter, the arm's own watchdog backs this up
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SixEyesSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  session_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void SixEyesSystem::publishMeasured(const sixeyes::ArmState & state, bool also_commands)
{
  for (int i = 0; i < SIXEYES_JOINT_COUNT; i++) {
    double value = static_cast<double>(state.measured[i]);
    if (i == kGripper) {
      value = value / 100.0 * gripper_stroke_m_;  // the arm's 0..100 -> metres of finger travel
    }
    set_state(state_names_[i], value);
    if (also_commands) {
      set_command(command_names_[i], value);
    }
  }
}

hardware_interface::return_type SixEyesSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    return hardware_interface::return_type::ERROR;
  }
  if (session_->stateAgeSeconds() > kMaxStateAgeS) {
    RCLCPP_ERROR(logger(), "No recent data from the arm. Check the USB cable and that the arm is powered.");
    return hardware_interface::return_type::ERROR;
  }
  sixeyes::ArmState state;
  if (!session_->state(state)) {
    return hardware_interface::return_type::ERROR;
  }
  // An e-stop or a lost joint bus stops the arm. Report it once and let ros2_control deactivate us.
  if ((state.faults & (SIXEYES_FAULT_HW_ESTOP | SIXEYES_FAULT_BUS_LOST)) != 0) {
    if (!fault_reported_) {
      fault_reported_ = true;
      RCLCPP_ERROR(logger(), "The arm stopped: the e-stop button is pressed or a joint board stopped answering. Fix that, then restart.");
    }
    return hardware_interface::return_type::ERROR;
  }
  publishMeasured(state, false);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SixEyesSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!session_) {
    return hardware_interface::return_type::ERROR;
  }
  sixeyes::ArmState state;
  if (!session_->state(state) || state.homing_state != SIXEYES_HOMING_STATE_READY) {
    return hardware_interface::return_type::OK;  // nothing may move until the arm is homed
  }

  float goal[SIXEYES_JOINT_COUNT];
  for (int i = 0; i < SIXEYES_JOINT_COUNT; i++) {
    double wanted = get_command(command_names_[i]);
    if (!std::isfinite(wanted)) {
      goal[i] = state.measured[i];  // no command yet: hold
      continue;
    }
    if (i == kGripper) {
      wanted = wanted / gripper_stroke_m_ * 100.0;  // metres -> the arm's 0..100
      const double step = max_step_rad_ * 100.0;    // the same limit, in the gripper's units
      wanted = std::clamp(wanted, static_cast<double>(state.measured[i]) - step, static_cast<double>(state.measured[i]) + step);
    } else {
      wanted = std::clamp(wanted, static_cast<double>(state.measured[i]) - max_step_rad_, static_cast<double>(state.measured[i]) + max_step_rad_);
    }
    goal[i] = static_cast<float>(wanted);
  }
  session_->sendTargetsNoWait(goal);

  if (++cycles_since_warning_ >= kWarnEveryCycles) {
    const std::string refusal = session_->takeRefusal();
    if (!refusal.empty()) {
      RCLCPP_WARN(logger(), "The arm refused a command: %s", refusal.c_str());
      cycles_since_warning_ = 0;
    } else {
      cycles_since_warning_ = kWarnEveryCycles;  // check again next cycle
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace sixeyes_hardware

PLUGINLIB_EXPORT_CLASS(sixeyes_hardware::SixEyesSystem, hardware_interface::SystemInterface)
