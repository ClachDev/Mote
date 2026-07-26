"""Wire protocol and client for the off-board depth server.

The server runs torch in its own pixi environment (no ROS); the node runs rclpy on
the robot (no torch). The only things they share are this module and a TCP socket,
so the protocol is deliberately primitive: length-prefixed binary frames, no
dependency beyond numpy, debuggable with a hexdump. Alternatives considered and
rejected for this link (one client, one LAN hop, ~1 MB replies at ~2 Hz):

- a ROS topic/service would pull rclpy + DDS into the torch environment — the
  exact coupling the two-process split exists to avoid;
- gRPC/protobuf buys schema evolution and cross-language types, at the cost of a
  heavy dependency and codegen for what is two fixed messages;
- HTTP adds per-request headers and a client library without removing the need
  to define the binary payload layout.

Protocol (all integers big-endian):

    request : uint32 n, then n bytes of one compressed image (JPEG/PNG)
    reply   : uint32 H, uint32 W, then H*W float32 depth (row-major, metres)
              H == W == 0 means the frame was rejected; no payload follows.

    health  : uint32 n == HEALTH_MAGIC (0xFFFFFFFF, never a real image length),
              no payload; the server replies uint32 L, then L bytes of UTF-8
              JSON describing the running service (model, device, versions).

A connection carries any number of request/reply cycles; either end closing the
socket ends the session. The health request shares this framing so the same
persistent socket and reconnect logic cover it — the robot can ask "who am I
talking to?" over the existing link (see WireClient.health).
"""

import json
import os
import socket
import struct
import subprocess
from pathlib import Path

import numpy as np

DEFAULT_PORT = 5601

# Leading uint32 sentinel marking a health request instead of an image length.
# 0xFFFFFFFF (4 GiB) can never be a real compressed-image length, so servers can
# branch on it before reading any payload.
HEALTH_MAGIC = 0xFFFFFFFF


def recvall(sock, n):
    """Read exactly n bytes, or None if the peer closed first."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def recv_image(conn):
    """Server side: receive one image blob, or None when the client is done."""
    hdr = recvall(conn, 4)
    if hdr is None:
        return None
    (n,) = struct.unpack(">I", hdr)
    return recvall(conn, n)


def send_depth(conn, depth):
    """Server side: reply with a float32 depth map."""
    h, w = depth.shape
    conn.sendall(
        struct.pack(">II", h, w) + np.ascontiguousarray(depth, np.float32).tobytes()
    )


def send_rejection(conn):
    """Server side: reply that the frame could not be processed."""
    conn.sendall(struct.pack(">II", 0, 0))


def send_health(conn, info):
    """Server side: reply to a health request with a JSON status blob.

    `info` is any JSON-serialisable dict (service name, model, device, versions).
    Shared by every server so the client's health probe is service-agnostic.
    """
    body = json.dumps(info).encode("utf-8")
    conn.sendall(struct.pack(">I", len(body)) + body)


def repo_revision():
    """The revision this side is running, for the health blob, or None.

    Robot and server share this protocol module, so they must be updated
    together; reporting each side's revision makes a stale inference server
    visible from the robot (`pixi run inference-health`) instead of surfacing
    later as a confusing protocol error.

    MOTE_VERSION wins when set: the inference server ships as a container with no
    git and no checkout, so its version is baked in at image build time. Falling
    back to `git describe` covers the robot and dev machines, which do run from a
    checkout. Best-effort — neither being available is not an error.
    """
    baked = os.environ.get("MOTE_VERSION")
    if baked:
        return baked
    try:
        out = subprocess.run(
            [
                "git",
                "-C",
                str(Path(__file__).resolve().parent),
                "describe",
                "--always",
                "--dirty",
                "--tags",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return out.stdout.strip() or None if out.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


class WireClient:
    """Persistent connection to an inference server, reconnecting on demand.

    Shared plumbing for the depth and detect clients: subclasses set ``NAME``
    (for log lines) and implement ``infer``, returning None when the frame
    could not be served (server unreachable, connection lost, or frame
    rejected) — callers skip the frame either way. A failed call tears the
    socket down so the next call reconnects. `warn` receives one line per
    failure (a logger or print).
    """

    NAME = "inference"

    def __init__(self, host, port, timeout=2.0, warn=print):
        self.host, self.port, self.timeout, self.warn = host, port, timeout, warn
        self.sock = None

    def connect(self):
        """The live socket, connecting first if needed; None if unreachable."""
        if self.sock is not None:
            return self.sock
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.connect((self.host, self.port))
            self.sock = s
        except OSError as e:
            self.warn(f"{self.NAME} server unavailable ({e}); skipping frame")
            self.sock = None
        return self.sock

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def health(self):
        """The server's status dict, or None if unreachable or the check failed.

        Sends the health sentinel over the persistent socket and reads back the
        JSON status blob. Works against any server (depth, detect, future
        tenants) since the framing is shared. A failure tears the socket down so
        the next call — health or infer — reconnects, exactly like `infer`.
        """
        s = self.connect()
        if s is None:
            return None
        try:
            s.sendall(struct.pack(">I", HEALTH_MAGIC))
            hdr = recvall(s, 4)
            if hdr is None:
                raise ConnectionError("server closed")
            (n,) = struct.unpack(">I", hdr)
            body = recvall(s, n)
            if body is None:
                raise ConnectionError("server closed mid-health")
            return json.loads(body.decode("utf-8"))
        except (OSError, ConnectionError, ValueError) as e:
            self.warn(f"{self.NAME} health check failed ({e}); will reconnect")
            self.close()
            return None


class DepthClient(WireClient):
    NAME = "depth"

    def __init__(self, host, port=DEFAULT_PORT, timeout=2.0, warn=print):
        super().__init__(host, port, timeout, warn)

    def infer(self, blob):
        """One compressed image in, one float32 depth map (or None) out."""
        s = self.connect()
        if s is None:
            return None
        try:
            s.sendall(struct.pack(">I", len(blob)) + blob)
            hdr = recvall(s, 8)
            if hdr is None:
                raise ConnectionError("server closed")
            h, w = struct.unpack(">II", hdr)
            if h == 0 or w == 0:
                self.warn("depth server rejected frame; skipping")
                return None
            body = recvall(s, h * w * 4)
            if body is None:
                raise ConnectionError("server closed mid-depth")
            return np.frombuffer(body, np.float32).reshape(h, w)
        except (OSError, ConnectionError) as e:
            self.warn(f"inference failed ({e}); will reconnect")
            self.close()
            return None
