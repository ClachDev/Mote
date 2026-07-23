"""health_monitor freshness/rate logic drives the OK/DEGRADED/FAULT roll-up."""

import time

from diagnostic_msgs.msg import DiagnosticStatus

from mote_bringup.health_monitor import _TopicWatch
from mote_bringup.sd_notify import SdNotifier


def _watch(critical=True, min_rate=5.0, timeout=2.0):
    return _TopicWatch(
        {
            "name": "scan",
            "topic": "/scan",
            "min_rate": min_rate,
            "timeout": timeout,
            "critical": critical,
        }
    )


def test_never_received_critical_is_fault():
    w = _watch(critical=True)
    level, msg, _ = w.evaluate(window=1.0)
    assert level == DiagnosticStatus.ERROR
    assert "no messages" in msg


def test_never_received_noncritical_is_degraded():
    w = _watch(critical=False)
    level, _, _ = w.evaluate(window=1.0)
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
    w = _watch(critical=True, timeout=2.0)
    w.on_msg(None)
    w.last_stamp = time.monotonic() - 10.0  # last seen 10s ago
    level, msg, _ = w.evaluate(window=1.0)
    assert level == DiagnosticStatus.ERROR
    assert "stale" in msg


def test_stale_noncritical_is_degraded():
    w = _watch(critical=False, timeout=2.0)
    w.on_msg(None)
    w.last_stamp = time.monotonic() - 10.0
    level, _, _ = w.evaluate(window=1.0)
    assert level == DiagnosticStatus.WARN


def test_recovery_back_to_ok():
    w = _watch(min_rate=5.0, timeout=2.0)
    w.last_stamp = time.monotonic() - 10.0
    assert w.evaluate(window=1.0)[0] == DiagnosticStatus.ERROR
    for _ in range(10):
        w.on_msg(None)
    assert w.evaluate(window=1.0)[0] == DiagnosticStatus.OK


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
