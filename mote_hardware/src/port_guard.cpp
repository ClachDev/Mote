#include "mote_hardware/port_guard.hpp"

#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

namespace mote_hardware
{

namespace fs = std::filesystem;

namespace
{

bool all_digits(const std::string & s)
{
  return !s.empty() &&
         std::all_of(s.begin(), s.end(), [](unsigned char c) {return std::isdigit(c);});
}

// A cmdline is NUL-separated and can itself contain newlines (any `python3 -c`
// with a multi-line script does); callers put this straight into single-line
// diagnostics, so collapse all whitespace.
std::string read_cmdline(const fs::path & proc_entry)
{
  std::ifstream file(proc_entry / "cmdline", std::ios::binary);
  if (!file) {
    return "?";
  }
  std::string raw((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::replace(raw.begin(), raw.end(), '\0', ' ');

  std::istringstream words(raw);
  std::ostringstream out;
  std::string word;
  bool first = true;
  while (words >> word) {
    if (!first) {
      out << ' ';
    }
    out << word;
    first = false;
  }
  const std::string collapsed = out.str();
  return collapsed.empty() ? "?" : collapsed;
}

}  // namespace

std::vector<std::pair<int, std::string>> port_holders(const std::string & path)
{
  std::vector<std::pair<int, std::string>> holders;

  std::error_code ec;
  const fs::path real = fs::canonical(path, ec);
  if (ec) {
    return holders;  // the device is gone; opening it will fail with a clearer error
  }

  const auto self_pid = getpid();
  fs::directory_iterator proc(fs::path("/proc"), ec);
  if (ec) {
    return holders;
  }

  for (const auto & entry : proc) {
    const std::string name = entry.path().filename().string();
    if (!all_digits(name)) {
      continue;
    }
    const int pid = std::stoi(name);
    if (pid == self_pid) {
      continue;
    }

    fs::directory_iterator fds(entry.path() / "fd", ec);
    if (ec) {
      continue;  // not ours to inspect
    }
    for (const auto & fd : fds) {
      std::error_code link_ec;
      if (fs::read_symlink(fd.path(), link_ec) != real || link_ec) {
        continue;
      }
      holders.emplace_back(pid, read_cmdline(entry.path()));
      break;
    }
  }
  return holders;
}

std::string describe_port_holders(
  const std::vector<std::pair<int, std::string>> & holders)
{
  std::ostringstream out;
  bool first = true;
  for (const auto & [pid, cmd] : holders) {
    if (!first) {
      out << "; ";
    }
    out << "pid " << pid << ": " << cmd;
    first = false;
  }
  return out.str();
}

}  // namespace mote_hardware
