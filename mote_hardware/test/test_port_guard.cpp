#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "mote_hardware/port_guard.hpp"

// The guard is what stops a second opener interleaving packets on the bus that
// moves the robot, so it is worth proving it actually sees an open fd rather
// than trusting the /proc walk by inspection. A regular temp file stands in for
// the serial device: the scan compares canonical paths and fd symlinks, neither
// of which cares that the target is a character device.

namespace mote_hardware
{

namespace
{

std::filesystem::path scratch_file(const std::string & name)
{
  auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream(path) << "x";
  return path;
}

}  // namespace

TEST(PortHolders, NoHoldersWhenNobodyHasItOpen)
{
  const auto path = scratch_file("mote_port_guard_idle");
  EXPECT_TRUE(port_holders(path.string()).empty());
  std::filesystem::remove(path);
}

TEST(PortHolders, IgnoresOurOwnOpenFd)
{
  // The component about to open the port is not a conflict with itself,
  // otherwise every activation would refuse.
  const auto path = scratch_file("mote_port_guard_self");
  const int fd = ::open(path.c_str(), O_RDONLY);
  ASSERT_GE(fd, 0);
  EXPECT_TRUE(port_holders(path.string()).empty());
  ::close(fd);
  std::filesystem::remove(path);
}

TEST(PortHolders, FindsAnotherProcessHoldingThePort)
{
  const auto path = scratch_file("mote_port_guard_held");

  int ready[2];
  ASSERT_EQ(::pipe(ready), 0);

  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(ready[0]);
    const int fd = ::open(path.c_str(), O_RDONLY);
    const char token = fd >= 0 ? 'y' : 'n';
    ssize_t ignored = ::write(ready[1], &token, 1);
    (void)ignored;
    ::pause();  // hold the fd open until the parent kills us
    _exit(0);
  }

  ::close(ready[1]);
  char token = 0;
  ASSERT_EQ(::read(ready[0], &token, 1), 1);
  ASSERT_EQ(token, 'y');

  const auto holders = port_holders(path.string());
  ASSERT_EQ(holders.size(), 1u);
  EXPECT_EQ(holders[0].first, child);
  // The cmdline is what an operator is shown; it must not be empty.
  EXPECT_FALSE(holders[0].second.empty());
  EXPECT_NE(describe_port_holders(holders).find("pid "), std::string::npos);

  ::kill(child, SIGKILL);
  int status = 0;
  ::waitpid(child, &status, 0);
  ::close(ready[0]);
  std::filesystem::remove(path);
}

TEST(PortHolders, MissingDeviceIsNotAHolder)
{
  // A port that does not exist yet must not look busy — the open() that follows
  // gives a far clearer error than "someone else has it".
  EXPECT_TRUE(port_holders("/dev/definitely_not_a_mote_bus").empty());
}

TEST(DescribeHolders, EmptyForNoHolders)
{
  EXPECT_EQ(describe_port_holders({}), "");
}

TEST(DescribeHolders, SeparatesMultipleHolders)
{
  const std::string listed = describe_port_holders({{7, "a"}, {9, "b"}});
  EXPECT_EQ(listed, "pid 7: a; pid 9: b");
}

}  // namespace mote_hardware
