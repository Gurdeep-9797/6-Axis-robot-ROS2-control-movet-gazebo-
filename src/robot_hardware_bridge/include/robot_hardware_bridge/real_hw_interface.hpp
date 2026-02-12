#ifndef ROBOT_HARDWARE_BRIDGE__REAL_HW_INTERFACE_HPP_
#define ROBOT_HARDWARE_BRIDGE__REAL_HW_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "robot_hardware_bridge/visibility_control.h"

namespace robot_hardware_bridge
{
class RealRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealRobotHardware)

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROBOT_HARDWARE_BRIDGE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store the command position for each joint
  std::vector<double> hw_commands_;
  // Store the state position, velocity of the robot
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // TCP Socket / Serial Handle would go here
  int socket_fd_;
};

}  // namespace robot_hardware_bridge

#endif  // ROBOT_HARDWARE_BRIDGE__REAL_HW_INTERFACE_HPP_
