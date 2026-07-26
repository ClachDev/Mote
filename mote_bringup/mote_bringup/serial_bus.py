"""Shared facts about the robot's serial buses.

The drive wheels and the SO-101 arm sit on the *same* Feetech bus
(``robot.yaml``: ``arm.port`` == ``servos.port``, wheel IDs 7/9, arm IDs 1-6), so
"is anyone else already talking to this port" is a question several components
need to ask. It lives here, in the base package, rather than in whichever
component asked first.

``mote_arm.bus.port_holders`` implements the same scan for the arm's side.
"""

import os


def port_holders(path):
    """``(pid, cmdline)`` for every *other* process holding ``path`` open.

    Serial ports carry no kernel-level exclusion: a second opener is not
    refused, it just interleaves packets on a half-duplex bus, and then both
    openers see corrupt or missing replies. ``open()`` therefore cannot detect
    contention — the only way is to scan ``/proc`` for the real device behind the
    symlink. Processes that cannot be inspected (other users) are skipped: this
    is a footgun guard, not a security boundary.
    """
    real = os.path.realpath(path)
    self_pid = os.getpid()
    holders = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == self_pid:
            continue
        fd_dir = f"/proc/{entry}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(f"{fd_dir}/{fd}") != real:
                    continue
            except OSError:
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as f:
                    raw = f.read().replace(b"\0", b" ").decode(errors="replace")
                # A cmdline can contain newlines (any `python3 -c` with a
                # multi-line script does); callers put this straight into
                # single-line diagnostics, so collapse all whitespace.
                cmd = " ".join(raw.split())
            except OSError:
                cmd = "?"
            holders.append((pid, cmd or "?"))
            break
    return holders
