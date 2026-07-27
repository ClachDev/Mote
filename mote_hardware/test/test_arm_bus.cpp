#include <gtest/gtest.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mote_hardware/mote_hardware.hpp"
#include "sts_bus_sim.hpp"

// MoteHardware driven against a simulated STS bus on a pty, through the real
// SCServo SDK. This is where the claims that matter get proved: that the arm
// comes up limp, that taking hold seeds a goal *before* enabling torque, that
// the soft limits are enforced in the hardware rather than in whichever client
// happens to be commanding, that an unchanged goal puts nothing on the bus, and
// that releasing the interfaces drops torque. All of those are statements about
// bus traffic, so a bus is what they are tested against.

namespace mote_hardware
{

namespace
{

constexpr int LEFT_ID = 7;
constexpr int RIGHT_ID = 9;
constexpr int PAN_ID = 1;
constexpr int ELBOW_ID = 3;

// Matches the shipped robot.yaml shape: a tight band on one joint, a wide one
// on the other, both with a home well away from the encoder mid-point.
constexpr int PAN_HOME = 3013;
constexpr int ELBOW_HOME = 2931;
constexpr double PAN_MIN = 0.010;
constexpr double PAN_MAX = 0.229;
constexpr double ELBOW_MIN = -3.291;
constexpr double ELBOW_MAX = 0.103;

std::string urdf_for(const std::string & port, bool with_arm = true)
{
  std::string arm_joints;
  if (with_arm) {
    arm_joints = R"(
    <joint name="shoulder_pan">
      <param name="id">1</param>
      <param name="min">0.010</param>
      <param name="max">0.229</param>
      <param name="home">3013</param>
      <param name="invert">false</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="elbow_flex">
      <param name="id">3</param>
      <param name="min">-3.291</param>
      <param name="max">0.103</param>
      <param name="home">2931</param>
      <param name="invert">false</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>)";
  }

  // The component parser cross-checks every <ros2_control> joint against a real
  // URDF joint, so the kinematics have to be here even though only the
  // <ros2_control> block is under test.
  std::string arm_links;
  if (with_arm) {
    arm_links = R"(
  <link name="shoulder_pan_link"/>
  <joint name="shoulder_pan" type="revolute">
    <parent link="base_link"/><child link="shoulder_pan_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="3.0"/>
  </joint>
  <link name="elbow_flex_link"/>
  <joint name="elbow_flex" type="revolute">
    <parent link="shoulder_pan_link"/><child link="elbow_flex_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="3.0"/>
  </joint>)";
  }

  return R"(<?xml version="1.0"?>
<robot name="mote_test">
  <link name="base_link"/>
  <link name="left_wheel_link"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="right_wheel_link"/>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="right_wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>)" + arm_links + R"(
  <ros2_control name="mote_hardware" type="system">
    <hardware>
      <plugin>mote_hardware/MoteHardware</plugin>
      <param name="serial_port">)" + port + R"(</param>
      <param name="baud_rate">1000000</param>
      <param name="left_wheel_id">7</param>
      <param name="right_wheel_id">9</param>
      <param name="velocity_scale">674.1</param>
      <param name="acceleration">0</param>
      <param name="arm_moving_speed">500</param>
      <param name="arm_moving_acc">20</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>)" + arm_joints + R"(
  </ros2_control>
</robot>
)";
}

hardware_interface::HardwareComponentInterfaceParams params_for(
  const std::string & urdf)
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info =
    hardware_interface::parse_control_resources_from_urdf(urdf).at(0);
  return params;
}

const rclcpp::Time TIME{0};
const rclcpp::Duration PERIOD = rclcpp::Duration::from_seconds(0.02);

// Index of the first event matching `needle`, or -1.
long index_of(const std::vector<std::string> & events, const std::string & needle)
{
  for (std::size_t i = 0; i < events.size(); ++i) {
    if (events[i].find(needle) != std::string::npos) {
      return static_cast<long>(i);
    }
  }
  return -1;
}

long count_matching(const std::vector<std::string> & events, const std::string & needle)
{
  return std::count_if(
    events.begin(), events.end(), [&needle](const std::string & event) {
      return event.find(needle) != std::string::npos;
    });
}

std::string arm_key(const std::string & joint)
{
  return joint + "/position";
}

// A fixture that gets the component all the way to "activated, arm limp".
class ArmBus : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bus = std::make_unique<test::StsBusSim>(
      std::vector<int>{LEFT_ID, RIGHT_ID, PAN_ID, ELBOW_ID});
  }

  void activate(bool with_arm = true)
  {
    ASSERT_EQ(
      hw.on_init(params_for(urdf_for(bus->port(), with_arm))),
      hardware_interface::CallbackReturn::SUCCESS);
    state_interfaces = hw.export_state_interfaces();
    command_interfaces = hw.export_command_interfaces();
    ASSERT_EQ(
      hw.on_activate(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }

  hardware_interface::CommandInterface & command(const std::string & name)
  {
    for (auto & iface : command_interfaces) {
      if (iface.get_name() == name + "/position" ||
        iface.get_name() == name + "/velocity")
      {
        return iface;
      }
    }
    throw std::runtime_error("no command interface for " + name);
  }

  double state_of(const std::string & name)
  {
    for (auto & iface : state_interfaces) {
      if (iface.get_name() == name + "/position") {
        return iface.get_optional().value();
      }
    }
    throw std::runtime_error("no position state for " + name);
  }

  // Claim the arm's position interfaces, then run enough write cycles for the
  // deferred engage (one joint per cycle) to reach every joint.
  void claim_arm()
  {
    const std::vector<std::string> keys{
      arm_key("shoulder_pan"), arm_key("elbow_flex")};
    ASSERT_EQ(
      hw.prepare_command_mode_switch(keys, {}),
      hardware_interface::return_type::OK);
    ASSERT_EQ(
      hw.perform_command_mode_switch(keys, {}),
      hardware_interface::return_type::OK);
    for (int i = 0; i < 4; ++i) {
      hw.write(TIME, PERIOD);
    }
  }

  std::unique_ptr<test::StsBusSim> bus;
  MoteHardware hw;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<hardware_interface::CommandInterface> command_interfaces;
};

}  // namespace

TEST_F(ArmBus, ExportsWheelVelocityAndArmPositionInterfaces)
{
  activate();

  std::vector<std::string> commands;
  for (const auto & iface : command_interfaces) {
    commands.push_back(iface.get_name());
  }
  EXPECT_NE(index_of(commands, "left_wheel_joint/velocity"), -1);
  EXPECT_NE(index_of(commands, "right_wheel_joint/velocity"), -1);
  EXPECT_NE(index_of(commands, "shoulder_pan/position"), -1);
  EXPECT_NE(index_of(commands, "elbow_flex/position"), -1);
  // The arm is position-controlled; it must not expose a velocity command.
  EXPECT_EQ(index_of(commands, "shoulder_pan/velocity"), -1);
}

TEST_F(ArmBus, ActivatesWithTheArmLimp)
{
  activate();
  // The whole torque policy rests on this: bringup must never leave the arm
  // holding, whatever state a previous session left the servos in.
  EXPECT_FALSE(bus->torque_enabled(PAN_ID));
  EXPECT_FALSE(bus->torque_enabled(ELBOW_ID));
  EXPECT_NE(index_of(bus->events(), "torque id=1 off"), -1);
  EXPECT_NE(index_of(bus->events(), "torque id=3 off"), -1);
}

TEST_F(ArmBus, SeedsTheArmStateFromTheServosAtActivation)
{
  bus->set_present_position(PAN_ID, PAN_HOME + 100);
  bus->set_present_position(ELBOW_ID, ELBOW_HOME - 200);
  activate();

  const ArmJoint pan{"shoulder_pan", PAN_ID, PAN_MIN, PAN_MAX, PAN_HOME, 1};
  const ArmJoint elbow{"elbow_flex", ELBOW_ID, ELBOW_MIN, ELBOW_MAX, ELBOW_HOME, 1};
  EXPECT_NEAR(state_of("shoulder_pan"), pan.counts_to_rad(PAN_HOME + 100), 1e-9);
  EXPECT_NEAR(state_of("elbow_flex"), elbow.counts_to_rad(ELBOW_HOME - 200), 1e-9);
}

TEST_F(ArmBus, PutsArmServosIntoPositionModeWhenTheyAreInWheelMode)
{
  // A servo left in wheel mode obeys GOAL_SPEED, so a position goal would spin
  // it continuously — bringup has to notice and fix it.
  bus->set_mode(PAN_ID, 1);
  activate();
  EXPECT_NE(index_of(bus->events(), "mode id=1 value=0"), -1);
  // The wheels are the mirror image: they must end up in wheel mode.
  EXPECT_NE(index_of(bus->events(), "mode id=7 value=1"), -1);
}

TEST_F(ArmBus, ReadsOneArmJointPerCycle)
{
  activate();
  bus->clear_events();

  hw.read(TIME, PERIOD);
  const auto after_one = bus->events();
  // Two wheel reads (position and speed) plus exactly one arm joint.
  EXPECT_EQ(count_matching(after_one, "read id=1 addr=56"), 1);
  EXPECT_EQ(count_matching(after_one, "read id=3 addr=56"), 0);

  hw.read(TIME, PERIOD);
  const auto after_two = bus->events();
  EXPECT_EQ(count_matching(after_two, "read id=1 addr=56"), 1);
  EXPECT_EQ(count_matching(after_two, "read id=3 addr=56"), 1);
}

TEST_F(ArmBus, WritesNoArmGoalsWhileNoControllerHoldsTheArm)
{
  activate();
  command("shoulder_pan").set_value(PAN_MAX);
  bus->clear_events();

  hw.write(TIME, PERIOD);

  // The wheels are still commanded every cycle; the arm is not touched at all.
  EXPECT_EQ(count_matching(bus->events(), "syncwrite"), 0);
  EXPECT_EQ(count_matching(bus->events(), "goal id=1"), 0);
  EXPECT_FALSE(bus->torque_enabled(PAN_ID));
}

TEST_F(ArmBus, TakingHoldSeedsTheGoalBeforeEnablingTorque)
{
  bus->set_present_position(PAN_ID, PAN_HOME + 40);
  activate();
  bus->clear_events();

  claim_arm();

  const auto events = bus->events();
  const long goal = index_of(events, "goal id=1 counts=3053");
  const long torque = index_of(events, "torque id=1 on");
  ASSERT_NE(goal, -1) << "no goal was seeded for the pan joint";
  ASSERT_NE(torque, -1) << "the pan joint never took hold";
  // Order is the whole point: enabling torque first makes the arm snap to
  // whatever stale value GOAL_POSITION happened to hold.
  EXPECT_LT(goal, torque);
  EXPECT_TRUE(bus->torque_enabled(PAN_ID));
  EXPECT_TRUE(bus->torque_enabled(ELBOW_ID));
}

TEST_F(ArmBus, EngagesOneJointPerCycle)
{
  activate();
  bus->clear_events();

  const std::vector<std::string> keys{
    arm_key("shoulder_pan"), arm_key("elbow_flex")};
  hw.perform_command_mode_switch(keys, {});

  hw.write(TIME, PERIOD);
  EXPECT_EQ(count_matching(bus->events(), "torque id=1 on"), 1);
  EXPECT_EQ(count_matching(bus->events(), "torque id=3 on"), 0)
    << "both joints engaged in one cycle — that is the long cycle this avoids";

  hw.write(TIME, PERIOD);
  EXPECT_EQ(count_matching(bus->events(), "torque id=3 on"), 1);
}

TEST_F(ArmBus, ClampsCommandsToTheSoftLimits)
{
  activate();
  claim_arm();
  bus->clear_events();

  // Far outside the band in both directions, on both joints.
  command("shoulder_pan").set_value(10.0);
  command("elbow_flex").set_value(-10.0);
  hw.write(TIME, PERIOD);

  const ArmJoint pan{"shoulder_pan", PAN_ID, PAN_MIN, PAN_MAX, PAN_HOME, 1};
  const ArmJoint elbow{"elbow_flex", ELBOW_ID, ELBOW_MIN, ELBOW_MAX, ELBOW_HOME, 1};
  EXPECT_EQ(bus->present_position(PAN_ID), pan.rad_to_counts(PAN_MAX));
  EXPECT_EQ(bus->present_position(ELBOW_ID), elbow.rad_to_counts(ELBOW_MIN));
}

TEST_F(ArmBus, SendsNothingWhenTheGoalHasNotChanged)
{
  activate();
  claim_arm();
  command("elbow_flex").set_value(-1.0);
  hw.write(TIME, PERIOD);
  bus->clear_events();

  // The controller keeps writing the same setpoint, as a holding JTC does.
  for (int i = 0; i < 5; ++i) {
    hw.write(TIME, PERIOD);
  }
  EXPECT_EQ(count_matching(bus->events(), "syncwrite"), 0)
    << "an idle arm must cost no bus time — the wheels share this bus";
}

TEST_F(ArmBus, SendsOnePacketForSeveralMovedJoints)
{
  activate();
  claim_arm();
  bus->clear_events();

  command("shoulder_pan").set_value(0.05);
  command("elbow_flex").set_value(-1.0);
  hw.write(TIME, PERIOD);

  // One sync-write instruction carried both joints, rather than a packet each.
  EXPECT_EQ(count_matching(bus->events(), "syncwrite servos=2"), 1);
  EXPECT_EQ(count_matching(bus->events(), "syncwrite"), 1);
  EXPECT_EQ(count_matching(bus->events(), "goal id="), 2);
}

TEST_F(ArmBus, ReleasingTheInterfacesDropsTorqueImmediately)
{
  activate();
  claim_arm();
  ASSERT_TRUE(bus->torque_enabled(PAN_ID));
  bus->clear_events();

  const std::vector<std::string> keys{
    arm_key("shoulder_pan"), arm_key("elbow_flex")};
  hw.perform_command_mode_switch({}, keys);

  // Not deferred to write(): letting go is the safe direction, and a component
  // being torn down may never call write() again.
  EXPECT_FALSE(bus->torque_enabled(PAN_ID));
  EXPECT_FALSE(bus->torque_enabled(ELBOW_ID));
}

TEST_F(ArmBus, DeactivatingStopsTheWheelsAndLimpsTheArm)
{
  activate();
  claim_arm();
  command("left_wheel_joint").set_value(1.0);
  hw.write(TIME, PERIOD);
  bus->clear_events();

  ASSERT_EQ(
    hw.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_FALSE(bus->torque_enabled(PAN_ID));
  EXPECT_FALSE(bus->torque_enabled(ELBOW_ID));
}

TEST_F(ArmBus, AnAbsentArmServoIsExcludedFromControl)
{
  // One dead servo must not stop the other five being commanded.
  bus->set_absent(PAN_ID, true);
  activate();
  bus->clear_events();

  claim_arm();
  command("elbow_flex").set_value(-1.0);
  hw.write(TIME, PERIOD);

  EXPECT_TRUE(bus->torque_enabled(ELBOW_ID));
  EXPECT_NE(count_matching(bus->events(), "goal id=3"), 0);
}

TEST_F(ArmBus, RefusesToActivateWhenAnotherProcessHoldsTheBus)
{
  // The guard that makes "one owner" true rather than merely intended.
  ASSERT_EQ(
    hw.on_init(params_for(urdf_for(bus->port()))),
    hardware_interface::CallbackReturn::SUCCESS);

  int ready[2];
  ASSERT_EQ(::pipe(ready), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(ready[0]);
    const int fd = ::open(bus->port().c_str(), O_RDWR | O_NOCTTY);
    const char token = fd >= 0 ? 'y' : 'n';
    ssize_t ignored = ::write(ready[1], &token, 1);
    (void)ignored;
    ::pause();
    _exit(0);
  }
  ::close(ready[1]);
  char token = 0;
  ASSERT_EQ(::read(ready[0], &token, 1), 1);
  ASSERT_EQ(token, 'y');

  EXPECT_EQ(
    hw.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);

  ::kill(child, SIGKILL);
  int status = 0;
  ::waitpid(child, &status, 0);
  ::close(ready[0]);
}

TEST_F(ArmBus, WorksWithNoArmAtAll)
{
  // The sim and any armless build go down this path: nothing arm-related may
  // run, and the wheels must be unaffected.
  activate(/*with_arm=*/false);
  bus->clear_events();

  hw.read(TIME, PERIOD);
  hw.write(TIME, PERIOD);

  EXPECT_EQ(count_matching(bus->events(), "id=1"), 0);
  EXPECT_EQ(count_matching(bus->events(), "id=3"), 0);
  EXPECT_NE(count_matching(bus->events(), "read id=7"), 0);
}

}  // namespace mote_hardware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
