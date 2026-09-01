"""The roll-up's status names are the ones the monitors here actually publish.

`mote_health` decides what to do with a status; the nodes that publish those
statuses live in this package, and nothing else ties the two together. The
roll-up matches by exact name — `/diagnostics` is shared, so it cannot take
whatever it finds there — and a status that stops being folded in degrades
nothing while the robot still looks healthy. That is the failure this holds off,
and it is the reason the assertion stayed in Python when the monitor did not:
`slip_monitor.STATUS_NAME` cannot be read from a gtest.

The other end — that `mote_health`'s own default list is the same two names —
is `mote_health/test/test_config.cpp`.
"""

import os

import yaml

HEALTH_CONFIG = os.path.join(
    os.path.dirname(__file__), "..", "..", "mote_health", "config", "health.yaml"
)


def _configured():
    with open(HEALTH_CONFIG) as f:
        return yaml.safe_load(f)["diagnostic_statuses"]


def test_forwarded_status_names_match_the_publishers():
    from mote_bringup.slip_monitor import STATUS_NAME as SLIP_STATUS

    configured = _configured()
    assert SLIP_STATUS in configured
    # system_monitor's status name is a literal in that node.
    assert "system" in configured
    assert set(configured) == {"system", "slip"}


def test_every_watched_subsystem_names_a_severity_the_monitor_knows():
    """An unknown severity is refused at load, i.e. the monitor will not start.

    The C++ refuses it and says which entry; this catches an edit to the
    packaged file without needing a build.
    """
    with open(HEALTH_CONFIG) as f:
        cfg = yaml.safe_load(f)
    for spec in cfg.get("topics", []) + cfg.get("tf", []):
        assert spec.get("severity") in {"critical", "degraded", "info"}, spec["name"]
