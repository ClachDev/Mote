#include "auldbot_hardware/auldbot_hardware.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auldbot_hardware
{

hardware_interface::CallbackReturn AuldbotHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_    = info_.hardware_parameters.at("serial_port");
  baud_rate_      = std::stoi(info_.hardware_parameters.at("baud_rate"));
  left_id_        = std::stoi(info_.hardware_parameters.at("left_wheel_id"));
  right_id_       = std::stoi(info_.hardware_parameters.at("right_wheel_id"));
  velocity_scale_ = std::stod(info_.hardware_parameters.at("velocity_scale"));
  acceleration_   = std::stoi(info_.hardware_parameters.at("acceleration"));

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("AuldbotHardware"),
      "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AuldbotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(info_.joints[LEFT].name,
    hardware_interface::HW_IF_POSITION, &wheel_positions_[LEFT]);
  interfaces.emplace_back(info_.joints[LEFT].name,
    hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[LEFT]);
  interfaces.emplace_back(info_.joints[RIGHT].name,
    hardware_interface::HW_IF_POSITION, &wheel_positions_[RIGHT]);
  interfaces.emplace_back(info_.joints[RIGHT].name,
    hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[RIGHT]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AuldbotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(info_.joints[LEFT].name,
    hardware_interface::HW_IF_VELOCITY, &wheel_velocity_commands_[LEFT]);
  interfaces.emplace_back(info_.joints[RIGHT].name,
    hardware_interface::HW_IF_VELOCITY, &wheel_velocity_commands_[RIGHT]);
  return interfaces;
}

hardware_interface::CallbackReturn AuldbotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!servo_driver_.begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(rclcpp::get_logger("AuldbotHardware"),
      "Failed to open serial port %s: %s — "
      "check the port exists and the user is in the 'dialout' group "
      "('sudo usermod -a -G dialout $USER', then re-login)",
      serial_port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set both servos to wheel (continuous rotation) mode
  servo_driver_.WheelMode(left_id_);
  servo_driver_.WheelMode(right_id_);

  wheel_velocity_commands_.fill(0.0);
  positions_initialised_.fill(false);

  RCLCPP_INFO(rclcpp::get_logger("AuldbotHardware"),
    "Activated on %s at %d baud (left ID=%d, right ID=%d)",
    serial_port_.c_str(), baud_rate_, left_id_, right_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuldbotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  servo_driver_.WriteSpe(left_id_,  0, acceleration_);
  servo_driver_.WriteSpe(right_id_, 0, acceleration_);
  servo_driver_.end();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AuldbotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const std::array<int, 2> ids = {left_id_, right_id_};

  for (int i = 0; i < 2; ++i) {
    int16_t raw_pos   = servo_driver_.ReadPos(ids[i]);
    int16_t raw_speed = servo_driver_.ReadSpeed(ids[i]);

    if (raw_pos == -1) {
      RCLCPP_WARN(rclcpp::get_logger("AuldbotHardware"),
        "Failed to read position from servo %d", ids[i]);
      continue;
    }

    // Track cumulative position across 12-bit rollover (0–4095)
    if (!positions_initialised_[i]) {
      last_raw_positions_[i] = raw_pos;
      positions_initialised_[i] = true;
    }

    int16_t delta = raw_pos - last_raw_positions_[i];
    if (delta >  ROLLOVER_THRESHOLD) delta -= static_cast<int16_t>(TICKS_PER_REV);
    if (delta < -ROLLOVER_THRESHOLD) delta += static_cast<int16_t>(TICKS_PER_REV);

    // Left wheel is mounted facing the opposite direction
    const double sign = (i == LEFT) ? -1.0 : 1.0;
    wheel_positions_[i] += sign * (delta / TICKS_PER_REV) * TWO_PI;
    last_raw_positions_[i] = raw_pos;
    wheel_velocities_[i] = sign * (raw_speed / velocity_scale_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuldbotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Left wheel is mounted facing the opposite direction
  const auto left_speed  = static_cast<int16_t>(
    -wheel_velocity_commands_[LEFT] * velocity_scale_);
  const auto right_speed = static_cast<int16_t>(
    wheel_velocity_commands_[RIGHT] * velocity_scale_);

  servo_driver_.WriteSpe(left_id_,  left_speed,  acceleration_);
  servo_driver_.WriteSpe(right_id_, right_speed, acceleration_);

  return hardware_interface::return_type::OK;
}

}  // namespace auldbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  auldbot_hardware::AuldbotHardware,
  hardware_interface::SystemInterface)
