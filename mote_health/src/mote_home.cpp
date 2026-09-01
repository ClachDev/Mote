#include "mote_health/mote_home.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>

#include <pwd.h>
#include <unistd.h>

namespace mote_health::mote_home
{
namespace
{

std::string home_directory()
{
  const char * home = std::getenv("HOME");
  if (home != nullptr && *home != '\0') {
    return home;
  }
  // Python's expanduser falls back to the password database when $HOME is
  // unset, which is what a systemd unit without Environment=HOME would hit.
  const passwd * pw = getpwuid(getuid());
  if (pw != nullptr && pw->pw_dir != nullptr) {
    return pw->pw_dir;
  }
  return "~";
}

std::string expand_user(const std::string & value)
{
  if (value.empty() || value[0] != '~') {
    return value;
  }
  if (value.size() > 1 && value[1] != '/') {
    // `~user` is left alone: nothing sets MOTE_HOME that way, and guessing at
    // another account's home would be worse than not expanding.
    return value;
  }
  return home_directory() + value.substr(1);
}

}  // namespace

std::string dir()
{
  const char * env = std::getenv("MOTE_HOME");
  return expand_user(env != nullptr ? env : "~/.mote");
}

std::string path(const std::string & name)
{
  return (std::filesystem::path(dir()) / name).string();
}

std::string override_path(const std::string & name, const std::string & fallback)
{
  const std::string user = path(name);
  std::error_code ec;
  return std::filesystem::exists(user, ec) ? user : fallback;
}

}  // namespace mote_health::mote_home
