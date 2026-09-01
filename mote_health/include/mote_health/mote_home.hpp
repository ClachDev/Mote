// Per-robot state (MOTE_HOME) versus shared package config, for C++ nodes.
//
// The rule and its rationale live in mote_bringup/mote_bringup/mote_home.py:
// shared config ships inside the package and an update replaces it wholesale;
// per-robot state lives under MOTE_HOME (~/.mote by default), outside the
// package, so an update can never clobber identity, calibration or maps. This
// is that rule for the health monitor, which is C++ and so cannot import it.
// Keep the two in step: `test_mote_home.cpp` pins the cases the Python tests
// pin, and health.yaml is resolved by both during a paired CPU measurement.

#ifndef MOTE_HEALTH__MOTE_HOME_HPP_
#define MOTE_HEALTH__MOTE_HOME_HPP_

#include <string>

namespace mote_health::mote_home
{

/// The per-robot state root: $MOTE_HOME, else ~/.mote.
///
/// A leading `~` is expanded against $HOME, as Python's `Path.expanduser()`
/// does, so `MOTE_HOME=~/somewhere` resolves the same either side.
std::string dir();

/// A path inside the per-robot state root.
std::string path(const std::string & name);

/// `$MOTE_HOME/<name>` if it exists, else the packaged `fallback`.
///
/// Named `override_path` rather than `override` only because the latter reads
/// as the contextual keyword at every call site.
std::string override_path(const std::string & name, const std::string & fallback);

}  // namespace mote_health::mote_home

#endif  // MOTE_HEALTH__MOTE_HOME_HPP_
