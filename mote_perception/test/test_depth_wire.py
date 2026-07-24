"""Unit tests for the depth-server wire protocol (depth_wire.py).

Exercises the length-prefixed framing (recvall / recv_image / send_depth /
send_rejection) over a real socket pair, and the DepthClient request/reply
cycle, rejection handling, and reconnect-on-failure against a tiny in-process
TCP server.
"""

import socket
import struct
import threading

import numpy as np

from mote_perception.depth_wire import (
    HEALTH_MAGIC,
    DepthClient,
    recv_image,
    recvall,
    send_depth,
    send_health,
    send_rejection,
)


def test_recvall_reads_exact_length():
    a, b = socket.socketpair()
    try:
        a.sendall(b"hello world")
        assert recvall(b, 5) == b"hello"
        assert recvall(b, 6) == b" world"
    finally:
        a.close()
        b.close()


def test_recvall_returns_none_when_peer_closes():
    a, b = socket.socketpair()
    a.sendall(b"ab")
    a.close()
    # Two bytes are available, but three are requested and the peer is gone.
    assert recvall(b, 3) is None
    b.close()


def test_recv_image_round_trip():
    a, b = socket.socketpair()
    try:
        blob = b"\xff\xd8jpeg-bytes\xff\xd9"
        a.sendall(struct.pack(">I", len(blob)) + blob)
        assert recv_image(b) == blob
    finally:
        a.close()
        b.close()


def test_recv_image_returns_none_on_clean_close():
    a, b = socket.socketpair()
    a.close()  # client done, nothing sent
    assert recv_image(b) is None
    b.close()


def test_send_depth_frames_shape_and_payload():
    a, b = socket.socketpair()
    try:
        depth = np.arange(12, dtype=np.float32).reshape(3, 4) * 0.5
        send_depth(a, depth)
        hdr = recvall(b, 8)
        h, w = struct.unpack(">II", hdr)
        assert (h, w) == (3, 4)
        body = recvall(b, h * w * 4)
        got = np.frombuffer(body, np.float32).reshape(h, w)
        np.testing.assert_array_equal(got, depth)
    finally:
        a.close()
        b.close()


def test_send_depth_casts_non_float32_and_non_contiguous():
    a, b = socket.socketpair()
    try:
        # float64 and a non-contiguous view (transpose) must still frame correctly.
        depth = (np.arange(6, dtype=np.float64).reshape(2, 3)).T
        send_depth(a, depth)
        h, w = struct.unpack(">II", recvall(b, 8))
        assert (h, w) == depth.shape
        got = np.frombuffer(recvall(b, h * w * 4), np.float32).reshape(h, w)
        np.testing.assert_allclose(got, depth)
    finally:
        a.close()
        b.close()


def test_send_rejection_is_zero_header():
    a, b = socket.socketpair()
    try:
        send_rejection(a)
        assert struct.unpack(">II", recvall(b, 8)) == (0, 0)
    finally:
        a.close()
        b.close()


class _Server:
    """Minimal in-process depth server driven by a per-request handler."""

    def __init__(self, handler):
        self.handler = handler
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        conn, _ = self.sock.accept()
        with conn:
            while True:
                blob = recv_image(conn)
                if blob is None:
                    return
                if not self.handler(conn, blob):
                    return

    def close(self):
        self.sock.close()


def test_depth_client_infer_round_trip():
    depth = np.arange(20, dtype=np.float32).reshape(4, 5) + 0.25
    seen = []

    def handler(conn, blob):
        seen.append(blob)
        send_depth(conn, depth)
        return True

    server = _Server(handler)
    try:
        client = DepthClient("127.0.0.1", port=server.port, warn=lambda m: None)
        out = client.infer(b"an-image")
        np.testing.assert_array_equal(out, depth)
        assert seen == [b"an-image"]
        # A second inference reuses the same connection.
        out2 = client.infer(b"another")
        np.testing.assert_array_equal(out2, depth)
        client.close()
    finally:
        server.close()


def test_depth_client_handles_rejection():
    warnings = []

    def handler(conn, blob):
        send_rejection(conn)
        return True

    server = _Server(handler)
    try:
        client = DepthClient("127.0.0.1", port=server.port, warn=warnings.append)
        assert client.infer(b"img") is None
        assert any("rejected" in w for w in warnings)
        client.close()
    finally:
        server.close()


def test_depth_client_unreachable_returns_none_and_warns():
    warnings = []
    # Port 1 is unbound for this test process -> connect refused.
    client = DepthClient("127.0.0.1", port=1, timeout=0.5, warn=warnings.append)
    assert client.infer(b"img") is None
    assert any("unavailable" in w for w in warnings)


def test_depth_client_reconnects_after_server_drop():
    warnings = []

    def handler(conn, blob):
        conn.close()  # drop the connection mid-reply
        return False

    server = _Server(handler)
    try:
        client = DepthClient("127.0.0.1", port=server.port, warn=warnings.append)
        assert client.infer(b"img") is None
        assert client.sock is None  # socket torn down so the next call reconnects
        assert any("reconnect" in w for w in warnings)
    finally:
        server.close()


class _HealthServer:
    """Server that mirrors the real loop: branch on the health sentinel, else echo."""

    def __init__(self, info):
        self.info = info
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        conn, _ = self.sock.accept()
        with conn:
            while True:
                hdr = recvall(conn, 4)
                if hdr is None:
                    return
                (n,) = struct.unpack(">I", hdr)
                if n == HEALTH_MAGIC:
                    send_health(conn, self.info)
                    continue
                blob = recvall(conn, n)
                if blob is None:
                    return
                send_depth(conn, np.ones((2, 2), np.float32))

    def close(self):
        self.sock.close()


def test_health_round_trip_and_interleaves_with_infer():
    info = {"service": "depth", "model": "m", "device": "cuda", "torch": "2.11"}
    server = _HealthServer(info)
    try:
        client = DepthClient("127.0.0.1", port=server.port, warn=lambda m: None)
        assert client.health() == info
        # health and infer share the one persistent socket, in any order.
        np.testing.assert_array_equal(client.infer(b"img"), np.ones((2, 2), np.float32))
        assert client.health() == info
        client.close()
    finally:
        server.close()


def test_health_returns_none_when_unreachable():
    client = DepthClient("127.0.0.1", port=1, timeout=0.5, warn=lambda m: None)
    assert client.health() is None
