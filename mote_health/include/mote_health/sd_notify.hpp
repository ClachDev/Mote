// Minimal sd_notify client for systemd service integration.
//
// The C++ counterpart of mote_bringup/mote_bringup/sd_notify.py, for the same
// reason and with the same contract: readiness, status and watchdog keep-alive
// datagrams to the socket named by $NOTIFY_SOCKET, with no dependency on
// libsystemd — it is just an AF_UNIX datagram.
//
// When $NOTIFY_SOCKET is unset (running outside systemd, e.g. `pixi run health`
// on a workstation) every call is a silent no-op, so the same node runs
// identically under systemd and by hand.

#ifndef MOTE_HEALTH__SD_NOTIFY_HPP_
#define MOTE_HEALTH__SD_NOTIFY_HPP_

#include <optional>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>

namespace mote_health
{

class SdNotifier
{
public:
  SdNotifier();
  ~SdNotifier();

  SdNotifier(const SdNotifier &) = delete;
  SdNotifier & operator=(const SdNotifier &) = delete;

  bool enabled() const {return fd_ >= 0;}

  void ready(const std::string & status = "");
  void status(const std::string & text);
  void watchdog();

  /// Recommended keep-alive period: half of WatchdogSec.
  ///
  /// systemd exports the timeout as WATCHDOG_USEC when a watchdog is
  /// configured; petting at half that interval leaves margin for jitter.
  /// Returns nullopt when no watchdog is set.
  static std::optional<double> watchdog_period_s();

private:
  void send(const std::string & message);

  int fd_{-1};
  sockaddr_un addr_{};
  socklen_t addr_len_{0};
};

}  // namespace mote_health

#endif  // MOTE_HEALTH__SD_NOTIFY_HPP_
