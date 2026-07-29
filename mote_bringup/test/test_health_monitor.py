"""health_monitor freshness/rate logic drives the OK/DEGRADED/FAULT roll-up."""

import os
import time

import pytest
import yaml
from diagnostic_msgs.msg import DiagnosticStatus

from mote_bringup.health_monitor import (
    DIAGNOSTIC_STATUS_NAMES,
    _one_line,
    _severity_level,
    _TfWatch,
    _TopicWatch,
)
from mote_bringup.sd_notify import SdNotifier

HEALTH_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "health.yaml")


def _watch(severity="critical", min_rate=5.0, timeout=2.0):
    return _TopicWatch(
        {
            "name": "scan",
            "topic": "/scan",
            "min_rate": min_rate,
            "timeout": timeout,
            "severity": severity,
        }
    )


def test_never_received_critical_is_fault():
    level, msg, _ = _watch("critical").evaluate(window=1.0)
    assert level == DiagnosticStatus.ERROR
    assert "no messages" in msg


def test_never_received_degraded_is_warn():
    level, _, _ = _watch("degraded").evaluate(window=1.0)
    assert level == DiagnosticStatus.WARN


def test_fresh_and_fast_is_ok():
    w = _watch(min_rate=5.0)
    for _ in range(10):
        w.on_msg(None)
    level, msg, values = w.evaluate(window=1.0)  # 10 msgs / 1s = 10 Hz
    assert level == DiagnosticStatus.OK
    assert msg == "ok"
    assert float(values["rate_hz"]) >= 5.0


def test_fresh_but_slow_is_degraded():
    w = _watch(min_rate=5.0)
    w.on_msg(None)  # a single message this window -> ~1 Hz
    level, msg, _ = w.evaluate(window=1.0)
    assert level == DiagnosticStatus.WARN
    assert "slow" in msg


def test_stale_critical_is_fault():
    w = _watch("critical", timeout=2.0)
    w.on_msg(None)
    w.last_stamp = time.monotonic() - 10.0  # last seen 10s ago
    level, msg, _ = w.evaluate(window=1.0)
    assert level == DiagnosticStatus.ERROR
    assert "stale" in msg


def test_stale_degraded_is_warn():
    w = _watch("degraded", timeout=2.0)
    w.on_msg(None)
    w.last_stamp = time.monotonic() - 10.0
    assert w.evaluate(window=1.0)[0] == DiagnosticStatus.WARN


def test_recovery_back_to_ok():
    w = _watch(min_rate=5.0, timeout=2.0)
    w.last_stamp = time.monotonic() - 10.0
    assert w.evaluate(window=1.0)[0] == DiagnosticStatus.ERROR
    for _ in range(10):
        w.on_msg(None)
    assert w.evaluate(window=1.0)[0] == DiagnosticStatus.OK


def test_info_severity_never_degrades():
    """An `info` subsystem is reported but must not degrade the robot summary.

    map->odom is legitimately absent when only the hardware base runs, so
    scoring it would leave a healthy idle robot permanently DEGRADED.
    """
    w = _watch("info", min_rate=5.0, timeout=2.0)
    level, msg, _ = w.evaluate(window=1.0)  # never received
    assert level == DiagnosticStatus.OK
    assert "no messages" in msg
    w.on_msg(None)  # fresh but slow
    assert w.evaluate(window=1.0)[0] == DiagnosticStatus.OK


def test_tf_info_severity_unavailable_is_ok():
    tf = _TfWatch(
        {"name": "localization", "parent": "map", "child": "odom", "severity": "info"}
    )
    assert tf.fault_level == DiagnosticStatus.OK


def test_boolean_critical_still_supported():
    assert _severity_level({"critical": True}) == DiagnosticStatus.ERROR
    assert _severity_level({"critical": False}) == DiagnosticStatus.WARN


def test_unknown_severity_rejected():
    with pytest.raises(ValueError):
        _severity_level({"name": "x", "severity": "catastrophic"})


def test_forwarded_status_names_match_the_publishers():
    """The roll-up matches by exact name, so a renamed status silently vanishes.

    /diagnostics is shared, so health_monitor cannot take whatever it finds
    there — it lifts named statuses. Nothing else ties those names to the nodes
    that publish them, and a status that stops being folded in degrades nothing
    while still looking healthy.
    """
    from mote_bringup.slip_monitor import STATUS_NAME as SLIP_STATUS

    with open(HEALTH_CONFIG) as f:
        configured = yaml.safe_load(f)["diagnostic_statuses"]
    assert SLIP_STATUS in configured
    # system_monitor's status name is a literal in that node.
    assert "system" in configured
    assert set(configured) == set(DIAGNOSTIC_STATUS_NAMES)


def test_one_line_collapses_embedded_newlines():
    """A third-party diagnostic message must not break the /health summary.

    controller_manager publishes a multi-line "High execution jitter" status on
    the shared /diagnostics topic, and embedding such a message verbatim would
    split the single-line summary across several messages.
    """
    messy = "High execution jitter or mean error :\n[ mote_hardware  mote_hardware ]\n"
    assert _one_line(messy) == (
        "High execution jitter or mean error : [ mote_hardware mote_hardware ]"
    )
    assert "\n" not in _one_line(messy)


def test_sd_notify_noop_without_socket(monkeypatch):
    monkeypatch.delenv("NOTIFY_SOCKET", raising=False)
    sd = SdNotifier()
    assert not sd.enabled
    # All calls must be safe no-ops when not under systemd.
    sd.ready()
    sd.watchdog()
    sd.status("x")
    assert SdNotifier.watchdog_period_s() is None


def test_sd_notify_watchdog_period(monkeypatch):
    monkeypatch.setenv("WATCHDOG_USEC", "15000000")  # 15 s
    assert SdNotifier.watchdog_period_s() == 7.5


def test_health_config_override_honours_mote_home(tmp_path, monkeypatch):
    """The per-robot override must resolve through MOTE_HOME, not a literal ~/.mote.

    mote_home is the one place that rule lives, so MOTE_HOME is honoured
    everywhere. A hardcoded ~/.mote looks identical on a robot, where the two are
    the same path, and is wrong for the sim and for tests.
    """
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    override = tmp_path / "health.yaml"
    override.write_text("period: 9.5\ntopics: []\ntf: []\n")

    from mote_bringup import health_monitor

    cfg = health_monitor._load_config()
    assert cfg["period"] == 9.5, "override under MOTE_HOME was not picked up"


def test_health_config_falls_back_to_the_packaged_default(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))  # empty: no override present

    from mote_bringup import health_monitor

    cfg = health_monitor._load_config()
    # The packaged default defines the real subsystems.
    assert {t["name"] for t in cfg["topics"]} >= {"scan", "joint_states"}
