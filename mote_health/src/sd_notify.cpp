#include "mote_health/sd_notify.hpp"

#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string>

#include <unistd.h>

namespace mote_health
{

SdNotifier::SdNotifier()
{
  const char * env = std::getenv("NOTIFY_SOCKET");
  if (env == nullptr || *env == '\0') {
    return;
  }
  std::string addr = env;
  // systemd names an abstract-namespace socket with a leading '@', which on the
  // wire is a leading NUL. The address is not NUL-terminated in that case, so
  // the length passed to sendto is what bounds it.
  const bool abstract = addr[0] == '@';
  if (abstract) {
    addr[0] = '\0';
  }
  if (addr.size() >= sizeof(addr_.sun_path)) {
    return;
  }

  const int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return;
  }
  addr_.sun_family = AF_UNIX;
  std::memcpy(addr_.sun_path, addr.data(), addr.size());
  addr_len_ = static_cast<socklen_t>(
    offsetof(sockaddr_un, sun_path) + addr.size() + (abstract ? 0 : 1));
  fd_ = fd;
}

SdNotifier::~SdNotifier()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

void SdNotifier::send(const std::string & message)
{
  if (fd_ < 0) {
    return;
  }
  // A notification that cannot be delivered is dropped, exactly as in the
  // Python client: the monitor's job is to watch the robot, and it must not die
  // because systemd's socket went away.
  (void)sendto(
    fd_, message.data(), message.size(), MSG_NOSIGNAL,
    reinterpret_cast<const sockaddr *>(&addr_), addr_len_);
}

void SdNotifier::ready(const std::string & status)
{
  std::string msg = "READY=1";
  if (!status.empty()) {
    msg += "\nSTATUS=" + status;
  }
  send(msg);
}

void SdNotifier::status(const std::string & text)
{
  send("STATUS=" + text);
}

void SdNotifier::watchdog()
{
  send("WATCHDOG=1");
}

std::optional<double> SdNotifier::watchdog_period_s()
{
  const char * usec = std::getenv("WATCHDOG_USEC");
  if (usec == nullptr || *usec == '\0') {
    return std::nullopt;
  }
  // Parsed as strictly as Python's int(): a trailing "5s" or "15.5" is a
  // malformed value, and reading half of one would pet the watchdog at a
  // period systemd never set.
  std::string text = usec;
  const auto first = text.find_first_not_of(" \t\n\r");
  const auto last = text.find_last_not_of(" \t\n\r");
  text = (first == std::string::npos) ? "" : text.substr(first, last - first + 1);
  size_t i = (!text.empty() && (text[0] == '+' || text[0] == '-')) ? 1 : 0;
  if (i >= text.size()) {
    return std::nullopt;
  }
  for (size_t j = i; j < text.size(); ++j) {
    if (std::isdigit(static_cast<unsigned char>(text[j])) == 0) {
      return std::nullopt;
    }
  }
  try {
    return std::stoll(text) / 1e6 / 2.0;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace mote_health
