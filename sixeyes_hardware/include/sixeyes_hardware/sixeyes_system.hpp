// The SixEyes arm as a ros2_control hardware component (a "system").
//
// Joints (by name, in this order): base, shoulder, elbow, wrist_pitch, wrist_roll, gripper.
// Each has one command interface and one state interface, both `position`:
//   five joints in radians; the gripper in metres of finger travel (the arm's 0..100 is scaled by gripper_stroke_m).
//
// Parameters (in the <hardware> block of the URDF):
//   port              required. The arm's USB port, e.g. /dev/ttyACM0
//   gripper_stroke_m  finger travel at 100, default 0.04
//   max_step_rad      most any angle may be asked to move in one cycle, default 0.35
//   home_on_activate  "true" (default): home the arm when the component activates. Keep clear of the arm
//
// NOT TESTED against a real arm, or against ROS 2, yet. See docs/appendix/appendix-ros2-contract.md.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sixeyes/session.hpp"

namespace sixeyes_hardware
{
class SixEyesSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SixEyesSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Copies the arm's measured joints into the state interfaces and (optionally) the command interfaces.
  void publishMeasured(const sixeyes::ArmState & state, bool also_commands);

  std::string port_;
  double gripper_stroke_m_ = 0.04;
  double max_step_rad_ = 0.35;
  bool home_on_activate_ = true;

  std::unique_ptr<sixeyes::Session> session_;

  // Interface names such as "base/position", in the arm's joint order.
  std::vector<std::string> state_names_;
  std::vector<std::string> command_names_;

  unsigned cycles_since_warning_ = 0;
  bool fault_reported_ = false;
};

}  // namespace sixeyes_hardware
