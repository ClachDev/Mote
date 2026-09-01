// health.yaml: what the monitor watches, and how loudly each absence reports.
//
// The file is the same one the Python monitor read, with the same per-robot
// override rule ($MOTE_HOME/health.yaml beats the packaged default), so a robot
// carrying a local override keeps it across the port.

#ifndef MOTE_HEALTH__CONFIG_HPP_
#define MOTE_HEALTH__CONFIG_HPP_

#include <memory>
#include <string>
#include <vector>

#include "mote_health/health_rollup.hpp"

namespace mote_health
{

/// Statuses lifted from the shared /diagnostics into the roll-up, matched by
/// exact name: system_monitor's host status, and slip_monitor's
/// odometry-residual verdict. Both are first-party monitors publishing one
/// named status. Overridable via health.yaml's `diagnostic_statuses`.
///
/// A function rather than a namespace-scope vector, because `Config`'s default
/// member initialiser reads it: across translation units that is static
/// initialisation order, and a `Config` built too early would forward nothing.
const std::vector<std::string> & default_diagnostic_statuses();

/// One watched topic: the type string the generic subscription needs, and the
/// watch that scores its arrivals.
struct TopicEntry
{
  std::string type;
  std::shared_ptr<TopicWatch> watch;
};

struct Config
{
  double period{1.0};
  std::vector<TopicEntry> topics;
  std::vector<TfWatch> tf;
  bool subscribe_diagnostics{true};
  std::vector<std::string> diagnostic_statuses{default_diagnostic_statuses()};
};

/// The config file this robot uses: $MOTE_HOME/health.yaml, else the packaged
/// default from this package's share directory.
std::string config_path();

/// Parse a health.yaml. Throws std::invalid_argument on a spec with no reading
/// — an unknown severity, a missing topic name — because a monitor that guesses
/// at what it is watching is worse than one that refuses to start.
Config load_config(const std::string & path);

/// Parse from YAML text. The file-reading half of `load_config`, split out so
/// the tests need no temporary files.
Config parse_config(const std::string & yaml_text);

}  // namespace mote_health

#endif  // MOTE_HEALTH__CONFIG_HPP_
