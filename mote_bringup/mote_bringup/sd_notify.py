"""Minimal sd_notify client for systemd service integration.

Sends readiness, status, and watchdog keep-alive datagrams to the socket named
by ``$NOTIFY_SOCKET`` (set by systemd for ``Type=notify`` services with
``NotifyAccess`` allowing this process). No dependency on the ``systemd`` python
package — it is just an ``AF_UNIX`` datagram, so this works in the plain robot
pixi env.

When ``$NOTIFY_SOCKET`` is unset (running outside systemd, e.g. ``pixi run
health`` on a workstation) every call is a silent no-op, so the same node runs
identically under systemd and by hand.
"""

import os
import socket


class SdNotifier:
    def __init__(self):
        self._sock = None
        addr = os.environ.get("NOTIFY_SOCKET")
        if not addr:
            return
        # Abstract namespace sockets are named with a leading '@' by systemd.
        if addr[0] == "@":
            addr = "\0" + addr[1:]
        try:
            self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._addr = addr
        except OSError:
            self._sock = None

    @property
    def enabled(self):
        return self._sock is not None

    def _send(self, message):
        if self._sock is None:
            return
        try:
            self._sock.sendto(message.encode("utf-8"), self._addr)
        except OSError:
            pass

    def ready(self, status=None):
        msg = "READY=1"
        if status:
            msg += f"\nSTATUS={status}"
        self._send(msg)

    def status(self, text):
        self._send(f"STATUS={text}")

    def watchdog(self):
        self._send("WATCHDOG=1")

    @staticmethod
    def watchdog_period_s():
        """Recommended keep-alive period: half of ``WatchdogSec``.

        systemd exports the timeout as ``WATCHDOG_USEC`` (microseconds) when a
        watchdog is configured; petting at half that interval leaves margin for
        jitter. Returns ``None`` when no watchdog is set.
        """
        usec = os.environ.get("WATCHDOG_USEC")
        if not usec:
            return None
        try:
            return int(usec) / 1e6 / 2.0
        except ValueError:
            return None
