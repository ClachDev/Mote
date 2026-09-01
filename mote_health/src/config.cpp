#include "mote_health/config.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mote_health/mote_home.hpp"

namespace mote_health
{

const std::vector<std::string> & default_diagnostic_statuses()
{
  static const std::vector<std::string> names{"system", "slip"};
  return names;
}

namespace
{

/// A key that is absent, and one written with no value, mean the same thing.
///
/// yaml-cpp hands back a *truthy* Null node for `severity:` with nothing after
/// it, whose `as<std::string>()` is the literal "null" — which would be refused
/// as an unknown severity. PyYAML gave None, which the Python monitor read as
/// absent, so absent is what it means here too.
bool present(const YAML::Node & node)
{
  return node && !node.IsNull();
}

std::optional<std::string> optional_string(const YAML::Node & node)
{
  return present(node) ? std::optional<std::string>(node.as<std::string>()) : std::nullopt;
}

std::optional<bool> optional_bool(const YAML::Node & node)
{
  return present(node) ? std::optional<bool>(node.as<bool>()) : std::nullopt;
}

std::optional<double> optional_double(const YAML::Node & node)
{
  return present(node) ? std::optional<double>(node.as<double>()) : std::nullopt;
}

std::string required_string(const YAML::Node & spec, const char * key, const char * what)
{
  if (!spec[key]) {
    throw std::invalid_argument(std::string(what) + " entry has no '" + key + "'");
  }
  return spec[key].as<std::string>();
}

uint8_t fault_level_of(const YAML::Node & spec, const std::string & name)
{
  return severity_level(
    optional_string(spec["severity"]), optional_bool(spec["critical"]), name);
}

}  // namespace

std::string config_path()
{
  const std::string packaged =
    (std::filesystem::path(ament_index_cpp::get_package_share_directory("mote_health")) /
    "config" / "health.yaml").string();
  return mote_home::override_path("health.yaml", packaged);
}

Config parse_config(const std::string & yaml_text)
{
  const YAML::Node root = YAML::Load(yaml_text);
  // A file that exists and says nothing is a truncated write or an override
  // somebody created before editing it, and `override_path` only tests that it
  // exists. Accepting it would give a monitor that watches nothing and reports
  // `OK` forever — the exact failure this node exists to prevent, and worse
  // than the crash the Python monitor gave here, which at least restart-looped
  // in the journal. An explicit `topics: []` is a different thing and is
  // allowed, as it was before.
  if (!root || !root.IsMap()) {
    throw std::invalid_argument("health.yaml is empty or is not a mapping");
  }
  Config cfg;

  if (root["period"]) {
    cfg.period = root["period"].as<double>();
  }

  const YAML::Node topics = root["topics"];
  for (size_t i = 0; topics && i < topics.size(); ++i) {
    const YAML::Node spec = topics[i];
    const std::string name = required_string(spec, "name", "topics");
    TopicEntry entry;
    entry.type = required_string(spec, "type", "topics");
    entry.watch = std::make_shared<TopicWatch>(
      name,
      required_string(spec, "topic", "topics"),
      optional_double(spec["min_rate"]),
      optional_double(spec["timeout"]).value_or(2.0),
      fault_level_of(spec, name));
    cfg.topics.push_back(std::move(entry));
  }

  const YAML::Node tf = root["tf"];
  for (size_t i = 0; tf && i < tf.size(); ++i) {
    const YAML::Node spec = tf[i];
    const std::string name = required_string(spec, "name", "tf");
    cfg.tf.emplace_back(
      name,
      required_string(spec, "parent", "tf"),
      required_string(spec, "child", "tf"),
      optional_double(spec["timeout"]).value_or(2.0),
      fault_level_of(spec, name));
  }

  if (root["subscribe_diagnostics"]) {
    cfg.subscribe_diagnostics = root["subscribe_diagnostics"].as<bool>();
  }
  const YAML::Node wanted = root["diagnostic_statuses"];
  if (wanted) {
    cfg.diagnostic_statuses.clear();
    for (size_t i = 0; i < wanted.size(); ++i) {
      cfg.diagnostic_statuses.push_back(wanted[i].as<std::string>());
    }
  }
  return cfg;
}

Config load_config(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::invalid_argument("could not open " + path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  try {
    return parse_config(buffer.str());
  } catch (const std::invalid_argument & exc) {
    // Which file is the first thing an operator needs, and there are two it
    // could be: the packaged default or a per-robot override.
    throw std::invalid_argument(path + ": " + exc.what());
  }
}

}  // namespace mote_health
