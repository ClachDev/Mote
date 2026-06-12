#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// SCServo SDK — path set by CMake include_directories
#include "SMS_STS.h"

#include "mote_hardware/encoder.hpp"

namespace mote_hardware
{

class MoteHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MoteHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  SMS_STS servo_driver_;

  std::string serial_port_;
  int baud_rate_;
  int left_id_;
  int right_id_;

  // Scales rad/s to servo speed units — tune on real hardware
  double velocity_scale_;

  // Acceleration for wheel mode (0 = max, higher = slower ramp)
  int acceleration_;

  // State
  std::array<double, 2> wheel_positions_{0.0, 0.0};
  std::array<double, 2> wheel_velocities_{0.0, 0.0};

  // Commands
  std::array<double, 2> wheel_velocity_commands_{0.0, 0.0};

  // Previous raw position for rollover tracking (12-bit, 0–4095)
  std::array<int16_t, 2> last_raw_positions_{0, 0};
  std::array<bool, 2> positions_initialised_{false, false};
};

}  // namespace mote_hardware
