"""Depth Anything 3 inference server -- same socket protocol as depth_server.py.

DA3 needs Python <= 3.13, but the pixi `depth` env is 3.14, so this runs in its own
uv venv (it is NOT a pixi task). The depth_anything_3 package's export path pulls
heavy 3D/video deps (open3d, moviepy, pycolmap, ...) that single-image depth never
touches, so we stub them at import time -- only `prediction.depth` is used. The
output is depth (near = small); SSI models (DA3-SMALL) are arbitrary-scale and the
client refits scale, metric models (DA3METRIC-*) are already in metres.

Set up the venv once:
    uv venv --python 3.13 /path/da3venv
    uv pip install --python /path/da3venv torch torchvision --index-url \
        https://download.pytorch.org/whl/cpu
    uv pip install --python /path/da3venv "numpy<2" einops huggingface_hub \
        safetensors omegaconf opencv-python-headless imageio pillow addict \
        matplotlib scipy
    uv pip install --python /path/da3venv --no-deps -e /path/Depth-Anything-3
Run:
    /path/da3venv/bin/python mote_perception/tools/depth_server_da3.py \
        --model depth-anything/DA3-SMALL [--port 5601]

Protocol (see depth_server.py):
  request : uint32 nbytes, then nbytes of JPEG
  reply   : uint32 H, uint32 W, then H*W float32 depth (resized to the input size)
"""

import argparse
import importlib.abc
import importlib.machinery
import io
import logging
import socket
import struct
import sys
import time
import types
from unittest.mock import MagicMock

# Stub the export/3D/video-only deps before importing depth_anything_3.
_STUB_PREFIXES = (
    "moviepy",
    "open3d",
    "trimesh",
    "plyfile",
    "pycolmap",
    "e3nn",
    "evo",
    "pillow_heif",
    "gsplat",
)


class _Stub(types.ModuleType):
    __path__ = []

    def __getattr__(self, k):
        return MagicMock()


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in _STUB_PREFIXES:
            return importlib.machinery.ModuleSpec(name, self)
        return None

    def create_module(self, spec):
        m = _Stub(spec.name)
        m.__path__ = []
        return m

    def exec_module(self, module):
        pass


sys.meta_path.insert(0, _StubFinder())

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from depth_anything_3.api import DepthAnything3  # noqa: E402

logging.getLogger("depth_anything_3").setLevel(logging.WARNING)


def recvall(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--model", default="depth-anything/DA3-SMALL")
    # metric models use a canonical-focal transform, so pass the camera intrinsics
    # "fx,fy,cx,cy" for correct metric scale (ignored by SSI models)
    ap.add_argument("--intrinsics", default=None)
    args = ap.parse_args()

    K = None
    if args.intrinsics:
        fx, fy, cx, cy = (float(v) for v in args.intrinsics.split(","))
        K = np.array([[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]], np.float32)

    print("loading", args.model)
    model = DepthAnything3.from_pretrained(args.model).to("cpu").eval()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"DA3 server listening on {args.host}:{args.port}", flush=True)

    while True:
        conn, addr = srv.accept()
        try:
            while True:
                hdr = recvall(conn, 4)
                if hdr is None:
                    break
                (n,) = struct.unpack(">I", hdr)
                blob = recvall(conn, n)
                if blob is None:
                    break
                img = np.asarray(Image.open(io.BytesIO(blob)).convert("RGB"))
                H, W = img.shape[:2]
                t0 = time.perf_counter()
                pred = model.inference([img], intrinsics=K)
                depth = np.asarray(pred.depth)[0].astype(np.float32)
                depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_LINEAR)
                dt = (time.perf_counter() - t0) * 1000
                conn.sendall(struct.pack(">II", H, W) + depth.tobytes())
                print(f"served {W}x{H} in {dt:.0f} ms", flush=True)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()
