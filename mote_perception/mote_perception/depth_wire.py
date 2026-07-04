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

A connection carries any number of request/reply cycles; either end closing the
socket ends the session.
"""

import socket
import struct

import numpy as np

DEFAULT_PORT = 5601


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


class DepthClient:
    """Persistent connection to the depth server, reconnecting on demand.

    `infer` returns the depth map, or None when the frame could not be served
    (server unreachable, connection lost, or frame rejected) — callers skip the
    frame either way. A failed call tears the socket down so the next call
    reconnects. `warn` receives one line per failure (a logger or print).
    """

    def __init__(self, host, port=DEFAULT_PORT, timeout=2.0, warn=print):
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
            self.warn(f"depth server unavailable ({e}); skipping frame")
            self.sock = None
        return self.sock

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

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
