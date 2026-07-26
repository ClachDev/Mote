"""port_holders detects a second opener of a shared serial bus.

The arm and the drive wheels share /dev/mote_servos, and serial ports carry no
kernel-level exclusion — a second open() succeeds and simply interleaves packets
on a half-duplex bus. So contention cannot be detected by opening the port; it
has to be found by scanning /proc.
"""

import os

from mote_bringup import serial_bus


def test_excludes_the_calling_process():
    fd = os.open("/dev/null", os.O_RDWR)
    try:
        assert all(
            pid != os.getpid() for pid, _ in serial_bus.port_holders("/dev/null")
        )
    finally:
        os.close(fd)


def test_finds_another_process_holding_the_port(tmp_path):
    """A child holding the device open must be reported with its cmdline."""
    import subprocess
    import time

    # Hold /dev/null open in a child; it stands in for the servo bus, since the
    # scan only cares that a /proc/<pid>/fd entry resolves to the same device.
    child = subprocess.Popen(
        ["python3", "-c", "open('/dev/null'); import time; time.sleep(30)"]
    )
    try:
        for _ in range(50):  # give it a moment to actually open the fd
            holders = serial_bus.port_holders("/dev/null")
            if any(pid == child.pid for pid, _ in holders):
                break
            time.sleep(0.1)
        holders = serial_bus.port_holders("/dev/null")
        found = [(pid, cmd) for pid, cmd in holders if pid == child.pid]
        assert found, f"child {child.pid} not among holders {holders}"
        assert "time.sleep" in found[0][1]
    finally:
        child.kill()
        child.wait()


def test_follows_symlinks_to_the_real_device(tmp_path):
    """/dev/mote_servos is a symlink; /proc/<pid>/fd points at the real node."""
    link = tmp_path / "mote_servos_link"
    link.symlink_to("/dev/null")
    fd = os.open("/dev/null", os.O_RDWR)
    try:
        # Resolving the symlink is what makes the two sets comparable at all.
        assert os.path.realpath(str(link)) == "/dev/null"
        # Same answer through the link as through the real path.
        assert serial_bus.port_holders(str(link)) == serial_bus.port_holders(
            "/dev/null"
        )
    finally:
        os.close(fd)


def test_missing_path_yields_no_holders():
    assert serial_bus.port_holders("/dev/definitely_not_here") == []


def test_holder_cmdline_is_single_line():
    """A holder's cmdline must be one line.

    Callers put it straight into single-line diagnostics, and a cmdline can
    legitimately contain newlines — any `python3 -c` with a multi-line script —
    which would truncate the message and lose the command name.
    """
    import subprocess
    import time

    child = subprocess.Popen(
        ["python3", "-c", "open('/dev/null')\nimport time\ntime.sleep(30)\n"]
    )
    try:
        found = []
        for _ in range(50):
            found = [
                (pid, cmd)
                for pid, cmd in serial_bus.port_holders("/dev/null")
                if pid == child.pid
            ]
            if found:
                break
            time.sleep(0.1)
        assert found, f"child {child.pid} not found among holders"
        cmd = found[0][1]
        assert "\n" not in cmd and "\r" not in cmd
        # The whole command survives, not just the part before the first newline.
        assert "time.sleep" in cmd
    finally:
        child.kill()
        child.wait()
