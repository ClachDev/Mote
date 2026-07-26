"""Tests for the shared-bus port guard.

The arm shares its serial bus with the drive wheels, so a second opener would
interleave packets with wheel traffic. These tests use a temp file as the
"port" — `port_holders` only cares that some process holds that path open.
"""

import subprocess
import sys
import time

import pytest

from mote_arm.bus import BusError, FeetechBus, port_holders


def test_no_holders_when_unopened(tmp_path):
    port = tmp_path / "fake_tty"
    port.write_text("")
    assert port_holders(str(port)) == []


def test_detects_another_holder(tmp_path):
    """A second process holding the port is reported with its pid."""
    port = tmp_path / "fake_tty"
    port.write_text("")
    # A separate process holding the fd open is exactly the situation we guard
    # against (the ros2_control node owning the wheel bus).
    child = subprocess.Popen(
        [sys.executable, "-c", f"f = open({str(port)!r}); input()"],
        stdin=subprocess.PIPE,
    )
    try:
        holders = []
        for _ in range(200):
            holders = port_holders(str(port))
            if holders:
                break
            time.sleep(0.01)
        assert [h[0] for h in holders] == [child.pid]
    finally:
        child.stdin.close()
        child.wait(timeout=10)


def test_open_refuses_when_port_held(tmp_path, monkeypatch):
    port = tmp_path / "fake_tty"
    port.write_text("")
    monkeypatch.setattr(
        "mote_arm.bus.port_holders", lambda _p: [(4242, "ros2_control_node")]
    )
    bus = FeetechBus(str(port), 1000000)
    with pytest.raises(BusError, match="already open"):
        bus.open()


def test_open_allows_shared_when_overridden(tmp_path, monkeypatch):
    """allow_shared bypasses the guard (and then fails later, on the SDK)."""
    port = tmp_path / "fake_tty"
    port.write_text("")
    monkeypatch.setattr(
        "mote_arm.bus.port_holders", lambda _p: [(4242, "ros2_control_node")]
    )
    bus = FeetechBus(str(port), 1000000)
    # It still fails (a temp file is not a serial port), but on the SDK rather
    # than the guard — which is the point: the guard was bypassed.
    with pytest.raises(Exception) as exc:
        bus.open(allow_shared=True)
    assert "already open" not in str(exc.value)
