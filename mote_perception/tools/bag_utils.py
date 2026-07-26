"""Shared bag-loading helpers for the offline perception tools.

Dev-env only (needs rosbag2_py). The tools import this as a sibling module — the
tools directory is sys.path[0] when a tool runs as a script.
"""

import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from tf2_msgs.msg import TFMessage

from mote_perception.ground_projection import (
    chain_static_transforms,
    transform_to_matrix,
)


def _reader(bag, topics=None):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    if topics is not None:
        r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return r


def load_perception_bag(bag):
    """All frames + scans and the static context from a `pixi run record` bag.

    Returns (imgs, scans, tf_static, caminfo): imgs as (recv_stamp_ns, jpeg bytes),
    scans as (recv_stamp_ns, LaserScan). Exits with a message if a topic is missing.
    """
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    imgs, scans, tf_static, caminfo = [], [], None, None
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/image_raw/compressed":
            imgs.append((t, bytes(deserialize_message(data, CompressedImage).data)))
        elif topic == "/scan_filtered":
            scans.append((t, deserialize_message(data, LaserScan)))
        elif topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, TFMessage)
        elif topic == "/camera_info" and caminfo is None:
            caminfo = deserialize_message(data, CameraInfo)
    if not imgs or not scans or tf_static is None or caminfo is None:
        sys.exit(
            "bag is missing one of /image_raw/compressed /scan_filtered "
            "/tf_static /camera_info -- record the `perception` stream"
        )
    return imgs, scans, tf_static, caminfo


def load_tf_poses(bag, parent, child):
    """Every dynamic ``parent<-child`` transform in a bag as pose samples.

    Returns (stamps_ns, T) where stamps_ns is an ascending int64 array of the
    transform *header* stamps and T is a matching (N, 4, 4) stack, each mapping a
    point in ``child`` into ``parent``. The header stamp (not the bag receipt time)
    is used, since that is the instant the pose is valid for and what a consumer
    interpolates against. Exits if the pair never appears.
    """
    r = _reader(bag, ["/tf"])
    stamps, mats = [], []
    while r.has_next():
        _, data, _ = r.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                stamps.append(tr.header.stamp.sec * 10**9 + tr.header.stamp.nanosec)
                mats.append(
                    transform_to_matrix(tr.transform.translation, tr.transform.rotation)
                )
    if not stamps:
        sys.exit(f"bag has no dynamic TF {parent!r}<-{child!r}")
    order = np.argsort(stamps)
    return np.asarray(stamps, dtype=np.int64)[order], np.asarray(mats)[order]


def load_image_stamps(bag):
    """Per-frame (header_ns, recv_ns) for every compressed image, ascending.

    header_ns is the v4l2 capture stamp carried in the message; recv_ns is the bag
    receipt time. Their difference is the driver+transport latency whose *jitter*
    (not its mean) perturbs pose-at-stamp interpolation. Bytes are not retained --
    use `load_images_at` to fetch pixels for a chosen few.
    """
    r = _reader(bag, ["/image_raw/compressed"])
    hdr, recv = [], []
    while r.has_next():
        _, data, t = r.read_next()
        m = deserialize_message(data, CompressedImage)
        hdr.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
        recv.append(t)
    hdr, recv = np.asarray(hdr, dtype=np.int64), np.asarray(recv, dtype=np.int64)
    order = np.argsort(hdr)
    return hdr[order], recv[order]


def load_images_at(bag, header_stamps):
    """Decoded BGR frames for a set of image *header* stamps, in the given order.

    Re-reads the image topic and keeps only the frames whose header stamp is in
    `header_stamps` (exact match against `load_image_stamps` values), so the caller
    can select keyframes from stamps first and pull pixels for just those.
    """
    want = set(int(s) for s in header_stamps)
    frames = {}
    r = _reader(bag, ["/image_raw/compressed"])
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, CompressedImage)
        s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if s in want and s not in frames:
            frames[s] = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    return [frames[int(s)] for s in header_stamps if int(s) in frames]


def load_static_context(bag):
    """(tf_static, caminfo) only -- the first of each, without decoding images.

    For harnesses that need the camera intrinsics and static frame tree but pull
    their own image/pose streams (e.g. the Stage 0 geometry harness).
    """
    r = _reader(bag, ["/tf_static", "/camera_info"])
    tf_static, caminfo = None, None
    while r.has_next() and (tf_static is None or caminfo is None):
        topic, data, _ = r.read_next()
        if topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, TFMessage)
        elif topic == "/camera_info" and caminfo is None:
            caminfo = deserialize_message(data, CameraInfo)
    if tf_static is None or caminfo is None:
        sys.exit("bag is missing /tf_static or /camera_info")
    return tf_static, caminfo


def base_transforms(tf_static, scans):
    """(T_base_optical, T_base_scan) from a bag's static transforms.

    The scan frame is taken from the scans themselves — on this robot it is yawed
    90 degrees from base, so plotting raw scan coordinates without T_base_scan is
    not an approximation, it is wrong.
    """
    T_bo = chain_static_transforms(
        tf_static.transforms, "camera_optical_link", "base_footprint"
    )
    T_bs = chain_static_transforms(
        tf_static.transforms, scans[0][1].header.frame_id, "base_footprint"
    )
    return T_bo, T_bs


def nearest_scan(scans, stamp):
    """The buffered (stamp, LaserScan) pair nearest a capture stamp."""
    return min(scans, key=lambda s: abs(s[0] - stamp))


def colorize(depth, vmax):
    """Colorized (TURBO) uint8 render of a depth map, clipped to [0, vmax] m."""
    d = np.clip(np.nan_to_num(depth), 0, vmax)
    return cv2.applyColorMap((255 * d / vmax).astype(np.uint8), cv2.COLORMAP_TURBO)
