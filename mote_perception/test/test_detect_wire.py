"""Round-trip the detection wire protocol over a real socket, no torch/ROS."""

import socket
import struct
import threading

import pytest

from mote_perception.depth_wire import recvall
from mote_perception.detect_wire import (
    HEALTH_MAGIC,
    DetectClient,
    recv_request,
    send_detections,
    send_health,
    send_rejection,
)


def serve_once(handler):
    """One-shot server on an ephemeral port; returns (port, thread)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def run():
        conn, _ = srv.accept()
        try:
            while True:
                req = recv_request(conn)
                if req is None:
                    break
                handler(conn, *req)
        finally:
            conn.close()
            srv.close()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return port, t


def test_round_trip():
    seen = {}

    def handler(conn, blob, labels):
        seen["blob"], seen["labels"] = blob, labels
        send_detections(conn, [(0, 0.9, (1.0, 2.0, 3.0, 4.0)), (1, 0.5, (5, 6, 7, 8))])

    port, _ = serve_once(handler)
    client = DetectClient("127.0.0.1", port)
    dets = client.infer(b"jpegbytes", ["red box", "shoe"])
    client.close()

    assert seen["blob"] == b"jpegbytes"
    assert seen["labels"] == ["red box", "shoe"]
    assert len(dets) == 2
    label, score, box = dets[0]
    assert label == "red box"
    assert score == pytest.approx(0.9)
    assert box == (1.0, 2.0, 3.0, 4.0)
    assert dets[1][0] == "shoe"


def test_empty_and_rejection():
    replies = iter(["empty", "reject"])

    def handler(conn, blob, labels):
        if next(replies) == "empty":
            send_detections(conn, [])
        else:
            send_rejection(conn)

    port, _ = serve_once(handler)
    client = DetectClient("127.0.0.1", port, warn=lambda m: None)
    assert client.infer(b"x", ["thing"]) == []
    assert client.infer(b"x", ["thing"]) is None
    client.close()


def test_unreachable_server():
    client = DetectClient("127.0.0.1", 1, timeout=0.2, warn=lambda m: None)
    assert client.infer(b"x", ["thing"]) is None


def test_health_round_trip():
    info = {"service": "detect", "model": "owlv2", "device": "cuda"}

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def run():
        conn, _ = srv.accept()
        with conn:
            hdr = recvall(conn, 4)
            (n,) = struct.unpack(">I", hdr)
            if n == HEALTH_MAGIC:
                send_health(conn, info)
        srv.close()

    threading.Thread(target=run, daemon=True).start()
    client = DetectClient("127.0.0.1", port, warn=lambda m: None)
    assert client.health() == info
    client.close()
