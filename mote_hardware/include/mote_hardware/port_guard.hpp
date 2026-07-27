#pragma once

#include <string>
#include <utility>
#include <vector>

// "Is anyone else already talking to this serial port?"
//
// Serial ports carry no kernel-level exclusion: a second open() is not refused,
// it just interleaves packets on a half-duplex bus, and then both openers see
// corrupt or missing replies. open() therefore cannot detect contention — the
// only way is to scan /proc for the real device behind the symlink.
//
// This is the C++ twin of mote_bringup/serial_bus.py and mote_arm.bus's
// port_holders. The realtime side has to be C++ and the bench/self-check side
// has to be importable Python, so the scan exists once per language rather than
// once per component (see the servo-bus consolidation task for the wider
// question of one implementation across both).

namespace mote_hardware
{

// (pid, cmdline) for every *other* process holding `path` open. Processes that
// cannot be inspected (other users) are skipped: this is a footgun guard, not a
// security boundary.
std::vector<std::pair<int, std::string>> port_holders(const std::string & path);

// The holders rendered as one line, for a log message.
std::string describe_port_holders(
  const std::vector<std::pair<int, std::string>> & holders);

}  // namespace mote_hardware
