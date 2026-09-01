// The systemd notification path.
//
// mote-health.service is Type=notify with WatchdogSec=15: a READY=1 that never
// arrives hangs the boot, and a WATCHDOG=1 that never arrives has systemd kill
// the monitor every 15 seconds. Both fail silently from inside the process, so
// these bind a real socket and read what actually landed on it.

#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "mote_health/sd_notify.hpp"

using mote_health::SdNotifier;

namespace
{

class Listener
{
public:
  explicit Listener(const std::string & path)
  : path_(path)
  {
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    timeval tv{1, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  ~Listener()
  {
    close(fd_);
    unlink(path_.c_str());
  }

  std::string receive()
  {
    char buf[512];
    const ssize_t n = recv(fd_, buf, sizeof(buf), 0);
    return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string();
  }

private:
  std::string path_;
  int fd_{-1};
};

std::string socket_path()
{
  return (std::filesystem::temp_directory_path() /
         ("mote_sd_notify_" + std::to_string(::getpid()))).string();
}

}  // namespace

TEST(SdNotify, NoopWithoutSocket)
{
  unsetenv("NOTIFY_SOCKET");
  SdNotifier sd;
  EXPECT_FALSE(sd.enabled());
  // All calls must be safe no-ops when not under systemd.
  sd.ready();
  sd.watchdog();
  sd.status("x");
  EXPECT_FALSE(SdNotifier::watchdog_period_s().has_value());
}

TEST(SdNotify, ReadyAndWatchdogReachTheSocket)
{
  const std::string path = socket_path();
  Listener listener(path);
  setenv("NOTIFY_SOCKET", path.c_str(), 1);

  SdNotifier sd;
  ASSERT_TRUE(sd.enabled());

  sd.ready("health monitor up");
  EXPECT_EQ(listener.receive(), "READY=1\nSTATUS=health monitor up");

  sd.watchdog();
  EXPECT_EQ(listener.receive(), "WATCHDOG=1");

  sd.status("degraded");
  EXPECT_EQ(listener.receive(), "STATUS=degraded");

  unsetenv("NOTIFY_SOCKET");
}

TEST(SdNotify, WatchdogPeriodIsHalfTheTimeout)
{
  setenv("WATCHDOG_USEC", "15000000", 1);  // 15 s
  ASSERT_TRUE(SdNotifier::watchdog_period_s().has_value());
  EXPECT_DOUBLE_EQ(*SdNotifier::watchdog_period_s(), 7.5);

  // A malformed value yields no period rather than half of a partial parse.
  setenv("WATCHDOG_USEC", "15s", 1);
  EXPECT_FALSE(SdNotifier::watchdog_period_s().has_value());
  unsetenv("WATCHDOG_USEC");
}
