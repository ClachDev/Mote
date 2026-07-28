#include "mote_hardware/mote_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mote_hardware/port_guard.hpp"

namespace mote_hardware
{

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("MoteHardware");
}

bool has_command_interface(
  const hardware_interface::ComponentInfo & joint, const std::string & type)
{
  return std::any_of(
    joint.command_interfaces.begin(), joint.command_interfaces.end(),
    [&type](const auto & iface) {return iface.name == type;});
}

std::string joint_param(
  const hardware_interface::ComponentInfo & joint, const std::string & key)
{
  const auto it = joint.parameters.find(key);
  if (it == joint.parameters.end()) {
    throw std::runtime_error(
            "arm joint '" + joint.name + "' is missing the <param name=\"" + key +
            "\"> tag — it should come from robot.yaml via mote.urdf.xacro");
  }
  return it->second;
}

// EEPROM writes need a moment to settle; the wheel path relies on the SDK's own
// pacing, but the arm re-reads to verify, and an immediate read-back races the
// relock (see mote_arm/README.md — a single read can return a garbled byte and
// make a successful write look failed).
void settle()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
}

}  // namespace

hardware_interface::CallbackReturn MoteHardware::on_init(
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

  // Joints are classified by what they are commanded with, not by position in
  // the list: the wheels take velocity, the arm takes position. That keeps the
  // URDF free to list them in any order and makes a mis-declared joint an
  // error here rather than a servo driven in the wrong mode.
  std::vector<std::size_t> wheels;
  std::vector<std::size_t> arm;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    if (has_command_interface(joint, hardware_interface::HW_IF_VELOCITY)) {
      wheels.push_back(i);
    } else if (has_command_interface(joint, hardware_interface::HW_IF_POSITION)) {
      arm.push_back(i);
    } else {
      RCLCPP_ERROR(
        logger(), "Joint '%s' declares no velocity or position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (wheels.size() != 2) {
    RCLCPP_ERROR(
      logger(), "Expected 2 velocity-commanded wheel joints, got %zu", wheels.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  wheel_joint_index_ = {wheels[0], wheels[1]};

  if (
    info_.joints[wheel_joint_index_[LEFT]].name != "left_wheel_joint" ||
    info_.joints[wheel_joint_index_[RIGHT]].name != "right_wheel_joint")
  {
    RCLCPP_WARN(
      logger(),
      "Unexpected wheel joint order: '%s', '%s' "
      "(expected left_wheel_joint, right_wheel_joint)",
      info_.joints[wheel_joint_index_[LEFT]].name.c_str(),
      info_.joints[wheel_joint_index_[RIGHT]].name.c_str());
  }

  if (!arm.empty()) {
    const auto speed = info_.hardware_parameters.find("arm_moving_speed");
    const auto acc = info_.hardware_parameters.find("arm_moving_acc");
    if (speed != info_.hardware_parameters.end()) {
      arm_moving_speed_ = std::stoi(speed->second);
    }
    if (acc != info_.hardware_parameters.end()) {
      arm_moving_acc_ = std::stoi(acc->second);
    }
  }

  for (const std::size_t i : arm) {
    const auto & joint = info_.joints[i];
    try {
      ArmJoint spec;
      spec.name = joint.name;
      spec.id = std::stoi(joint_param(joint, "id"));
      spec.min_rad = std::stod(joint_param(joint, "min"));
      spec.max_rad = std::stod(joint_param(joint, "max"));
      spec.zero_counts = std::stoi(joint_param(joint, "zero"));
      spec.sign = joint_param(joint, "invert") == "true" ? -1 : 1;

      if (spec.min_rad > spec.max_rad) {
        RCLCPP_ERROR(
          logger(), "Arm joint '%s': min %.3f > max %.3f",
          spec.name.c_str(), spec.min_rad, spec.max_rad);
        return hardware_interface::CallbackReturn::ERROR;
      }
      // The arm shares the bus with the drive wheels, so a colliding ID would
      // send arm goals to a wheel. mote_arm.config rejects this too; refuse it
      // here as well, because this is the component that would act on it.
      if (spec.id == left_id_ || spec.id == right_id_) {
        RCLCPP_ERROR(
          logger(),
          "Arm joint '%s' uses servo ID %d, which is a drive wheel ID on the "
          "shared bus %s — reassign the arm servo (see mote_hardware setup_ids)",
          spec.name.c_str(), spec.id, serial_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      for (const auto & existing : arm_joints_) {
        if (existing.id == spec.id) {
          RCLCPP_ERROR(
            logger(), "Arm joints '%s' and '%s' share servo ID %d",
            existing.name.c_str(), spec.name.c_str(), spec.id);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      arm_joints_.push_back(spec);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(logger(), "%s", exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  arm_positions_.assign(arm_joints_.size(), 0.0);
  arm_position_commands_.assign(arm_joints_.size(), 0.0);
  arm_written_counts_.assign(arm_joints_.size(), -1);
  arm_present_.assign(arm_joints_.size(), false);
  arm_controllable_.assign(arm_joints_.size(), false);
  arm_engaged_.assign(arm_joints_.size(), false);

  arm_engage_queue_.reserve(arm_joints_.size());
  sync_ids_.reserve(arm_joints_.size());
  sync_goals_.reserve(arm_joints_.size());
  sync_speeds_.reserve(arm_joints_.size());
  sync_accels_.reserve(arm_joints_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MoteHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (const int side : {LEFT, RIGHT}) {
    const auto & name = info_.joints[wheel_joint_index_[side]].name;
    interfaces.emplace_back(
      name, hardware_interface::HW_IF_POSITION, &wheel_positions_[side]);
    interfaces.emplace_back(
      name, hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[side]);
  }
  for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
    interfaces.emplace_back(
      arm_joints_[i].name, hardware_interface::HW_IF_POSITION, &arm_positions_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MoteHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (const int side : {LEFT, RIGHT}) {
    interfaces.emplace_back(
      info_.joints[wheel_joint_index_[side]].name,
      hardware_interface::HW_IF_VELOCITY, &wheel_velocity_commands_[side]);
  }
  for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
    interfaces.emplace_back(
      arm_joints_[i].name, hardware_interface::HW_IF_POSITION,
      &arm_position_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type MoteHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  // Every combination this component exports is valid: the wheels and the arm
  // are independent, and a joint that turned out to be uncontrollable at
  // activation is skipped in write() rather than refused here — one dead servo
  // must not stop the other five from being commanded.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
    const std::string key =
      arm_joints_[i].name + "/" + hardware_interface::HW_IF_POSITION;

    // Releasing is the safe direction, so it happens here and now: one torque
    // write per joint, and the arm is back-drivable before the controller that
    // was holding it finishes deactivating.
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), key) !=
      stop_interfaces.end())
    {
      if (arm_engaged_[i]) {
        servo_driver_.EnableTorque(static_cast<u8>(arm_joints_[i].id), 0);
        arm_engaged_[i] = false;
        arm_written_counts_[i] = -1;
      }
      arm_engage_queue_.erase(
        std::remove(arm_engage_queue_.begin(), arm_engage_queue_.end(), i),
        arm_engage_queue_.end());
      arm_position_commands_[i] = arm_positions_[i];
    }

    // Taking hold is deferred to write(): seeding a goal and enabling torque is
    // a read plus two writes per joint, and this callback runs inside the
    // realtime update loop.
    if (std::find(start_interfaces.begin(), start_interfaces.end(), key) !=
      start_interfaces.end())
    {
      arm_position_commands_[i] = arm_positions_[i];
      const bool queued =
        std::find(arm_engage_queue_.begin(), arm_engage_queue_.end(), i) !=
        arm_engage_queue_.end();
      if (arm_controllable_[i] && !arm_engaged_[i] && !queued) {
        arm_engage_queue_.push_back(i);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MoteHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // open() cannot tell us the bus is already busy — a second opener is accepted
  // and silently interleaves packets. Check before opening so a stale arm tool
  // or a second bringup is reported as what it is, rather than as servos that
  // stopped answering.
  const auto holders = port_holders(serial_port_);
  if (!holders.empty()) {
    RCLCPP_ERROR(
      logger(),
      "%s is already open by another process (%s). The arm and the drive wheels "
      "share this bus, so a second opener corrupts wheel traffic. Stop the other "
      "process first (e.g. `pixi run kill`).",
      serial_port_.c_str(), describe_port_holders(holders).c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!servo_driver_.begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(logger(),
      "Failed to open serial port %s: %s — "
      "check the port exists and the user is in the 'dialout' group "
      "('sudo usermod -a -G dialout $USER', then re-login)",
      serial_port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set wheel (continuous rotation) mode — only writes to EEPROM if not already set
  for (int id : {left_id_, right_id_}) {
    const auto uid = static_cast<u8>(id);
    if (servo_driver_.readByte(uid, SMS_STS_MODE) != SMS_STS_MODE_WHEEL_CLOSED) {
      servo_driver_.unLockEeprom(uid);
      servo_driver_.Mode(uid, SMS_STS_MODE_WHEEL_CLOSED);
      servo_driver_.LockEeprom(uid);
    }
  }

  wheel_velocity_commands_.fill(0.0);
  positions_initialised_.fill(false);

  // The arm comes up limp and stays limp: torque is only enabled once a
  // controller claims the position interfaces. Enumerate it here so a servo that
  // is absent or stuck in wheel mode is reported at bringup, not at the first
  // trajectory.
  arm_engage_queue_.clear();
  for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
    const auto & joint = arm_joints_[i];
    const auto uid = static_cast<u8>(joint.id);

    arm_engaged_[i] = false;
    arm_written_counts_[i] = -1;
    arm_present_[i] = false;
    arm_controllable_[i] = false;

    if (servo_driver_.Ping(uid) == -1) {
      RCLCPP_WARN(
        logger(), "Arm servo for joint '%s' (id %d) did not respond",
        joint.name.c_str(), joint.id);
      continue;
    }
    arm_present_[i] = true;

    // Torque off before anything else, so a servo that was left holding from a
    // previous session is released rather than driven to a stale goal.
    servo_driver_.EnableTorque(uid, 0);
    read_arm_joint(i);
    arm_position_commands_[i] = arm_positions_[i];

    int mode = servo_driver_.readByte(uid, SMS_STS_MODE);
    if (mode != SMS_STS_MODE_SERVO && mode != -1) {
      servo_driver_.unLockEeprom(uid);
      servo_driver_.Mode(uid, SMS_STS_MODE_SERVO);
      servo_driver_.LockEeprom(uid);
      settle();
      mode = servo_driver_.readByte(uid, SMS_STS_MODE);
    }
    if (mode != SMS_STS_MODE_SERVO) {
      // Left uncommandable rather than assumed good: wheel mode obeys
      // GOAL_SPEED, so a position goal sent to an unverified servo spins it.
      RCLCPP_WARN(
        logger(),
        "Arm joint '%s' (id %d) not confirmed in position mode — excluded from "
        "control (state reads only)", joint.name.c_str(), joint.id);
      continue;
    }
    arm_controllable_[i] = true;
  }

  if (arm_joints_.empty()) {
    RCLCPP_INFO(logger(),
      "Activated on %s at %d baud (left ID=%d, right ID=%d)",
      serial_port_.c_str(), baud_rate_, left_id_, right_id_);
  } else {
    const auto ready =
      static_cast<std::size_t>(std::count(
        arm_controllable_.begin(), arm_controllable_.end(), true));
    RCLCPP_INFO(logger(),
      "Activated on %s at %d baud (left ID=%d, right ID=%d), arm: %zu/%zu joints "
      "controllable (torque OFF — limp until a controller claims them)",
      serial_port_.c_str(), baud_rate_, left_id_, right_id_, ready,
      arm_joints_.size());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoteHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  servo_driver_.WriteSpe(left_id_,  0, acceleration_);
  servo_driver_.WriteSpe(right_id_, 0, acceleration_);
  disengage_arm();
  servo_driver_.end();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MoteHardware::disengage_arm()
{
  // Best-effort and unconditional: a failure to drop torque on one joint must
  // not stop us trying the rest.
  for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
    servo_driver_.EnableTorque(static_cast<u8>(arm_joints_[i].id), 0);
    arm_engaged_[i] = false;
    arm_written_counts_[i] = -1;
  }
  arm_engage_queue_.clear();
}

void MoteHardware::read_arm_joint(std::size_t index)
{
  const int counts = servo_driver_.ReadPos(arm_joints_[index].id);
  if (counts < 0) {
    return;  // keep the last good value; a dropped reply is not a moved joint
  }
  arm_positions_[index] = arm_joints_[index].counts_to_rad(counts);
}

void MoteHardware::read_next_arm_joint()
{
  // One arm joint per cycle, round-robin: six joints refresh at ~8 Hz for one
  // extra bus transaction, instead of six transactions for 50 Hz nobody needs.
  //
  // Joints whose servo never answered are skipped rather than retried, because
  // a read to a servo that is not there does not fail fast — it costs a full
  // serial timeout, inside the loop that also drives the wheels. On a Mote
  // built without an arm that would be paid on every single cycle, so the
  // round-robin walks past the whole arm and does nothing.
  for (std::size_t tried = 0; tried < arm_joints_.size(); ++tried) {
    const std::size_t index = arm_read_cursor_;
    arm_read_cursor_ = (arm_read_cursor_ + 1) % arm_joints_.size();
    if (arm_present_[index]) {
      read_arm_joint(index);
      return;
    }
  }
}

bool MoteHardware::engage_arm_joint(std::size_t index)
{
  const auto & joint = arm_joints_[index];
  const int counts = servo_driver_.ReadPos(joint.id);
  if (counts < 0) {
    RCLCPP_WARN(
      logger(),
      "Arm joint '%s': cannot read position, leaving it limp rather than "
      "enabling torque against an unknown goal", joint.name.c_str());
    return false;
  }
  // Seed the goal with where the joint actually is, *then* enable torque —
  // enabling first is what makes an arm snap to a pose nobody asked for.
  servo_driver_.WritePosEx(
    static_cast<u8>(joint.id), static_cast<s16>(counts),
    static_cast<u16>(arm_moving_speed_), static_cast<u8>(arm_moving_acc_));
  servo_driver_.EnableTorque(static_cast<u8>(joint.id), 1);

  arm_positions_[index] = joint.counts_to_rad(counts);
  arm_position_commands_[index] = arm_positions_[index];
  arm_written_counts_[index] = counts;
  arm_engaged_[index] = true;
  return true;
}

hardware_interface::return_type MoteHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const std::array<int, 2> ids = {left_id_, right_id_};

  for (int i = 0; i < 2; ++i) {
    int16_t raw_pos   = servo_driver_.ReadPos(ids[i]);
    int16_t raw_speed = servo_driver_.ReadSpeed(ids[i]);

    if (raw_pos == -1) {
      RCLCPP_WARN(logger(), "Failed to read position from servo %d", ids[i]);
      continue;
    }

    // Track cumulative position across 12-bit rollover (0–4095)
    if (!positions_initialised_[i]) {
      last_raw_positions_[i] = raw_pos;
      positions_initialised_[i] = true;
    }

    const int16_t delta = encoder_delta(raw_pos, last_raw_positions_[i]);
    wheel_positions_[i] += WHEEL_SIGN[i] * ticks_to_radians(delta);
    last_raw_positions_[i] = raw_pos;
    wheel_velocities_[i] = WHEEL_SIGN[i] * (raw_speed / velocity_scale_);
  }

  read_next_arm_joint();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoteHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto left_speed  = static_cast<int16_t>(
    WHEEL_SIGN[LEFT] * wheel_velocity_commands_[LEFT] * velocity_scale_);
  const auto right_speed = static_cast<int16_t>(
    WHEEL_SIGN[RIGHT] * wheel_velocity_commands_[RIGHT] * velocity_scale_);

  servo_driver_.WriteSpe(left_id_,  left_speed,  acceleration_);
  servo_driver_.WriteSpe(right_id_, right_speed, acceleration_);

  if (arm_joints_.empty()) {
    return hardware_interface::return_type::OK;
  }

  // At most one engage per cycle, so taking hold of six joints costs six
  // ordinary cycles (~120 ms) instead of one long one. A joint that fails to
  // engage leaves the queue anyway — it stays limp, and is retried the next
  // time a controller claims it, not on every cycle from here on.
  if (!arm_engage_queue_.empty()) {
    const std::size_t index = arm_engage_queue_.front();
    arm_engage_queue_.erase(arm_engage_queue_.begin());
    engage_arm_joint(index);
  }

  sync_ids_.clear();
  sync_goals_.clear();
  sync_speeds_.clear();
  sync_accels_.clear();
  for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
    if (!arm_engaged_[i]) {
      continue;
    }
    const double command = arm_position_commands_[i];
    if (!std::isfinite(command)) {
      continue;  // controller has not written a setpoint yet
    }
    const auto & joint = arm_joints_[i];
    // The soft limits are the authoritative gate and they live here, on the far
    // side of every client: a controller, a jog CLI or a task-layer trajectory
    // all pass through this clamp.
    const int counts = joint.rad_to_counts(joint.clamp_rad(command));
    if (counts == arm_written_counts_[i]) {
      continue;  // unchanged goal: no packet, no bus time
    }
    sync_ids_.push_back(static_cast<u8>(joint.id));
    sync_goals_.push_back(static_cast<s16>(counts));
    sync_speeds_.push_back(static_cast<u16>(arm_moving_speed_));
    sync_accels_.push_back(static_cast<u8>(arm_moving_acc_));
    arm_written_counts_[i] = counts;
  }
  if (!sync_ids_.empty()) {
    // One packet for however many joints moved, rather than one per joint.
    servo_driver_.SyncWritePosEx(
      sync_ids_.data(), static_cast<u8>(sync_ids_.size()), sync_goals_.data(),
      sync_speeds_.data(), sync_accels_.data());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mote_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mote_hardware::MoteHardware,
  hardware_interface::SystemInterface)
