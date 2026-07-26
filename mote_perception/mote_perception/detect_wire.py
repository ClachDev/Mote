"""Wire protocol and client for the off-board open-vocabulary detection server.

Same two-process split and framing style as depth_wire.py (which holds the
rationale): the server runs torch in the pixi inference environment, the node runs
rclpy without torch, and the only shared pieces are this module and a TCP
socket. Detection adds one twist over depth — the query is part of the request,
because open-vocabulary labels come from the live task command, not from
configuration.

Protocol (all integers big-endian):

    request : uint32 n, then n bytes of one compressed image (JPEG/PNG),
              uint32 m, then m bytes of UTF-8 label text, one label per line
    reply   : uint32 k, then k detections of
              uint32 label_index, float32 score,
              float32 x0, y0, x1, y1 (pixel corners in the request image)
              k == 0xFFFFFFFF means the frame was rejected; no payload follows.

    health  : uint32 n == HEALTH_MAGIC, no payload; server replies with the
              shared JSON status blob (see depth_wire). Same framing as depth so
              WireClient.health works unchanged against this service too.

A connection carries any number of request/reply cycles; either end closing the
socket ends the session.
"""

import struct

from mote_perception.depth_wire import (
    HEALTH_MAGIC,
    WireClient,
    recvall,
    repo_revision,
    send_health,
)

__all__ = [
    "HEALTH_MAGIC",
    "send_health",
    "repo_revision",
    "recv_request",
    "send_detections",
    "send_rejection",
    "DetectClient",
    "DEFAULT_PORT",
    "REJECTED",
]

DEFAULT_PORT = 5602
REJECTED = 0xFFFFFFFF
_RECORD = struct.Struct(">If4f")


def recv_request(conn):
    """Server side: receive one (image blob, labels) request, or None when done."""
    hdr = recvall(conn, 4)
    if hdr is None:
        return None
    blob = recvall(conn, struct.unpack(">I", hdr)[0])
    if blob is None:
        return None
    hdr = recvall(conn, 4)
    if hdr is None:
        return None
    text = recvall(conn, struct.unpack(">I", hdr)[0])
    if text is None:
        return None
    return blob, text.decode("utf-8").splitlines()


def send_detections(conn, detections):
    """Server side: reply with [(label_index, score, (x0, y0, x1, y1)), ...]."""
    out = [struct.pack(">I", len(detections))]
    for idx, score, box in detections:
        out.append(_RECORD.pack(int(idx), float(score), *map(float, box)))
    conn.sendall(b"".join(out))


def send_rejection(conn):
    """Server side: reply that the frame could not be processed."""
    conn.sendall(struct.pack(">I", REJECTED))


class DetectClient(WireClient):
    """Persistent connection to the detection server (plumbing in WireClient).

    `infer` returns a list of (label, score, (x0, y0, x1, y1)) with `label`
    resolved back to the query string, or None when the frame could not be
    served — an empty list means the frame was processed and nothing matched.
    """

    NAME = "detect"

    def __init__(self, host, port=DEFAULT_PORT, timeout=10.0, warn=print):
        super().__init__(host, port, timeout, warn)

    def infer(self, blob, labels):
        """One compressed image + label list in, detections (or None) out."""
        s = self.connect()
        if s is None:
            return None
        text = "\n".join(labels).encode("utf-8")
        try:
            s.sendall(
                struct.pack(">I", len(blob))
                + blob
                + struct.pack(">I", len(text))
                + text
            )
            hdr = recvall(s, 4)
            if hdr is None:
                raise ConnectionError("server closed")
            (k,) = struct.unpack(">I", hdr)
            if k == REJECTED:
                self.warn("detect server rejected frame; skipping")
                return None
            body = recvall(s, k * _RECORD.size)
            if body is None:
                raise ConnectionError("server closed mid-reply")
            out = []
            for i in range(k):
                idx, score, x0, y0, x1, y1 = _RECORD.unpack_from(body, i * _RECORD.size)
                if idx < len(labels):
                    out.append((labels[idx], score, (x0, y0, x1, y1)))
            return out
        except (OSError, ConnectionError) as e:
            self.warn(f"inference failed ({e}); will reconnect")
            self.close()
            return None
