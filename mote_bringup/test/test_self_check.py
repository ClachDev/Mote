"""self_check gates mission readiness on static pre-flight hardware checks."""

import time

from mote_bringup import self_check
from mote_bringup.self_check import ADVISORY, CRITICAL, Result


def test_result_critical_failure_blocks():
    r = Result()
    r.add("servo_bus", True, CRITICAL, "ok")
    assert r.ok
    r.add("lidar", False, CRITICAL, "unplugged")
    assert not r.ok


def test_result_advisory_failure_does_not_block():
    r = Result()
    r.add("camera", False, ADVISORY, "unplugged")
    assert r.ok


def test_check_device_missing_symlink(tmp_path):
    r = Result()
    ok = self_check._check_device(r, "lidar", str(tmp_path / "nope"), CRITICAL)
    assert not ok and not r.ok


def test_check_device_regular_file_is_not_a_device(tmp_path):
    # A dangling-then-recreated symlink can resolve to a plain file; that must
    # not read as a healthy device node.
    f = tmp_path / "fake"
    f.write_text("")
    r = Result()
    ok = self_check._check_device(r, "lidar", str(f), CRITICAL)
    assert not ok and not r.ok


def test_check_device_real_char_device():
    # /dev/null is always a char device and openable — proves the happy path.
    r = Result()
    ok = self_check._check_device(r, "servo_bus", "/dev/null", CRITICAL)
    assert ok and r.ok


def test_clock_pre_2024_is_flagged(monkeypatch):
    r = Result()
    monkeypatch.setattr(time, "time", lambda: 1000.0)  # 1970
    self_check._check_clock(r)
    check = r.checks[-1]
    assert not check["passed"] and check["severity"] == ADVISORY
    assert r.ok  # advisory, so it does not block


def test_clock_now_passes():
    r = Result()
    self_check._check_clock(r)
    assert r.checks[-1]["passed"]


def test_disk_check_reports_free_space():
    r = Result()
    self_check._check_disk(r)
    check = r.checks[-1]
    assert check["name"] == "disk_space"
    assert "free" in check["detail"]


def test_write_status_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    r = Result()
    r.add("servo_bus", True, CRITICAL, "ok")
    r.add("lidar", False, CRITICAL, "unplugged")
    self_check._write_status(r)
    import yaml

    data = yaml.safe_load((tmp_path / "self_check_status.yaml").read_text())
    assert data["ok"] is False
    names = {c["name"] for c in data["checks"]}
    assert names == {"servo_bus", "lidar"}


def test_held_servo_bus_blocks_bringup_and_names_the_holder(monkeypatch):
    """A bus held by the arm must fail the gate with an actionable reason.

    Pinging a contended bus reports "servos did not respond", which is true but
    points at the wrong problem and hides the fix (stop the arm driver).
    """
    monkeypatch.setattr(
        self_check.serial_bus,
        "port_holders",
        lambda _p: [(4321, "ros2 run mote_arm jog")],
    )
    r = Result()
    cfg = {
        "servos": {
            "port": "/dev/null",
            "baud_rate": 1000000,
            "left_id": 7,
            "right_id": 9,
        }
    }
    self_check._check_servos(r, cfg, do_ping=True)

    assert not r.ok
    held = [c for c in r.checks if c["name"] == "servo_bus_free"]
    assert held and not held[0]["passed"]
    assert "4321" in held[0]["detail"] and "mote_arm jog" in held[0]["detail"]
    # The ping must not have run at all — that is the point.
    assert not any(c["name"] == "servo_ping" for c in r.checks)


def test_free_servo_bus_lets_the_ping_proceed(monkeypatch):
    monkeypatch.setattr(self_check.serial_bus, "port_holders", lambda _p: [])
    r = Result()
    cfg = {
        "servos": {
            "port": "/dev/null",
            "baud_rate": 1000000,
            "left_id": 7,
            "right_id": 9,
        }
    }
    self_check._check_servos(r, cfg, do_ping=False)

    free = [c for c in r.checks if c["name"] == "servo_bus_free"]
    assert free and free[0]["passed"]
    assert any(c["name"] == "servo_ping" for c in r.checks)
    assert r.ok
