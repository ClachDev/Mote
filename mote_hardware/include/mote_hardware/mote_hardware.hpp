#pragma once

#include <array>
#include <cstddef>
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

#include "mote_hardware/arm_joint.hpp"
#include "mote_hardware/encoder.hpp"

namespace mote_hardware
{

// The one owner of /dev/mote_servos.
//
// The SO-101 arm and the drive wheels sit on the same Feetech bus (arm IDs 1-6,
// wheels 7/9). A serial port has no kernel-level exclusion, so two openers
// interleave packets on the bus that *moves the robot*. Rather than keep the arm
// in a second process and forbid the combination, this component drives both:
// velocity command interfaces for the wheels, position command interfaces for
// the arm joints. One process, one open(), and the arm can move during a mission
// with Nav2 live.
//
// Bus budget is why the arm is not simply read and written like the wheels. The
// controller_manager runs at 50 Hz (20 ms) and every servo *read* is a round
// trip through a USB serial adapter, so:
//   * arm states are read one joint per cycle, round-robin — six joints refresh
//     at ~8 Hz, ample for TF and joint_states, and costs one extra transaction
//     per cycle rather than six;
//   * arm goals go out as a single SyncWritePosEx packet, and only when a goal
//     actually changed, so an idle arm costs no bus traffic at all.
//
// Torque follows the command interfaces, which is what makes the arm's existing
// torque policy fall out of ros2_control instead of being re-implemented: the
// arm is limp until a controller claims its position interfaces
// (perform_command_mode_switch), and goes limp again the moment one releases
// them or the component deactivates.
class MoteHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MoteHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Enable torque on one arm joint without moving it, by seeding its goal
  // register with the joint's *present* position first. Order matters: a servo
  // drives to whatever GOAL_POSITION holds the instant torque is enabled, and
  // that register may hold a stale value from a previous session.
  bool engage_arm_joint(std::size_t index);
  void disengage_arm();
  void read_arm_joint(std::size_t index);

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

  // --- Arm -----------------------------------------------------------------
  // Empty when the URDF declares no arm joints (the sim, or a base built
  // without the arm), in which case none of the arm paths below ever run.
  std::vector<ArmJoint> arm_joints_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_position_commands_;
  // Last count actually written per joint, so an unchanged goal costs no bus
  // traffic. -1 means "nothing written yet this session".
  std::vector<int> arm_written_counts_;
  // Joints confirmed present and in position mode at activation; only these are
  // ever commanded. A servo whose mode could not be verified might be in wheel
  // mode, where a position goal spins it continuously.
  std::vector<bool> arm_controllable_;
  std::vector<bool> arm_engaged_;

  int arm_moving_speed_ = 500;
  int arm_moving_acc_ = 20;

  // Round-robin cursor for the one arm state read per cycle.
  std::size_t arm_read_cursor_ = 0;

  // Joints queued by perform_command_mode_switch to take hold of their current
  // pose. write() drains one per cycle, so no single realtime cycle pays for
  // six read+write pairs, and a joint that fails to engage is dropped rather
  // than retried every cycle against a servo that is not answering.
  std::vector<std::size_t> arm_engage_queue_;

  // Sync-write scratch, sized once at init: building these in write() would
  // allocate inside the realtime loop.
  std::vector<u8> sync_ids_;
  std::vector<s16> sync_goals_;
  std::vector<u16> sync_speeds_;
  std::vector<u8> sync_accels_;

  // Indices into arm_joints_ for the wheel joints' positions in info_.joints,
  // so the wheel paths keep addressing info_.joints correctly whatever order
  // the URDF lists arm and wheel joints in.
  std::array<std::size_t, 2> wheel_joint_index_{0, 1};
};

}  // namespace mote_hardware
