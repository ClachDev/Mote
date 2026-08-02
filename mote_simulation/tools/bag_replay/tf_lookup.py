"""A minimal offline tf2: resolve a frame chain over a bag's recorded transforms.

ROS-free on purpose. The acceptance simulator (``acceptance.py``) has to know the
*same* ``odom -> base_frame`` pose slam_toolbox's ``GetPoseHelper`` will read out
of its own tf2 buffer for a given scan stamp — if the two disagree the prediction
is wrong and the lockstep leg aborts. So the parts of tf2 that decide that number
are reimplemented here rather than approximated: a per-edge time cache, exact-stamp
hits, linear translation interpolation with quaternion slerp between the two
bracketing samples, and refusal (``None``) rather than extrapolation outside the
cache — which is what makes a scan arriving before the first transform, or after
the last, unprocessable for the real node too.

Transforms are ``((tx, ty, tz), (qx, qy, qz, qw))``; a frame has exactly one
parent, as in tf2.
"""

from __future__ import annotations

import math
from bisect import bisect_left

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w*t + q_vec x t
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def q_slerp(a, b, s):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    dot = ax * bx + ay * by + az * bz + aw * bw
    if dot < 0.0:
        bx, by, bz, bw, dot = -bx, -by, -bz, -bw, -dot
    if dot > 0.9995:
        x, y, z, w = (
            ax + s * (bx - ax),
            ay + s * (by - ay),
            az + s * (bz - az),
            aw + s * (bw - aw),
        )
        n = math.sqrt(x * x + y * y + z * z + w * w)
        return (x / n, y / n, z / n, w / n)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    k0 = math.sin((1.0 - s) * theta) / sin_theta
    k1 = math.sin(s * theta) / sin_theta
    return (
        ax * k0 + bx * k1,
        ay * k0 + by * k1,
        az * k0 + bz * k1,
        aw * k0 + bw * k1,
    )


def t_mul(a, b):
    """Compose: the transform ``a`` applied to ``b`` (``a`` parent of ``b``)."""
    (atx, aty, atz), aq = a
    (btx, bty, btz), bq = b
    rx, ry, rz = q_rotate(aq, (btx, bty, btz))
    return ((atx + rx, aty + ry, atz + rz), q_mul(aq, bq))


def t_inv(a):
    t, q = a
    qi = q_conj(q)
    x, y, z = q_rotate(qi, t)
    return ((-x, -y, -z), qi)


def t_interp(a, b, s):
    (ax, ay, az), aq = a
    (bx, by, bz), bq = b
    return (
        (ax + s * (bx - ax), ay + s * (by - ay), az + s * (bz - az)),
        q_slerp(aq, bq, s),
    )


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def se2_of(transform):
    (x, y, _z), q = transform
    return (x, y, yaw_of(q))


def se2_compose(pose, offset):
    """Karto's ``Transform(pose).TransformPose(offset)`` — plain SE(2) composition."""
    px, py, pth = pose
    ox, oy, oth = offset
    c, s = math.cos(pth), math.sin(pth)
    return (px + c * ox - s * oy, py + s * ox + c * oy, normalize_angle(pth + oth))


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class _Edge:
    __slots__ = ("parent", "stamps", "transforms", "static")

    def __init__(self, parent, static):
        self.parent = parent
        self.static = static
        self.stamps = []
        self.transforms = []

    def at(self, stamp_ns):
        if self.static:
            return self.transforms[0]
        stamps = self.stamps
        i = bisect_left(stamps, stamp_ns)
        if i == len(stamps):
            return None  # extrapolation into the future
        if stamps[i] == stamp_ns:
            return self.transforms[i]
        if i == 0:
            return None  # extrapolation into the past
        t0, t1 = stamps[i - 1], stamps[i]
        s = (stamp_ns - t0) / (t1 - t0)
        return t_interp(self.transforms[i - 1], self.transforms[i], s)


class TfTree:
    """Frames keyed by child, each with one parent — a bag's transform tree."""

    def __init__(self):
        self._edges = {}

    def add_static(self, parent, child, transform):
        if child not in self._edges:
            e = _Edge(parent, static=True)
            e.transforms.append(transform)
            self._edges[child] = e

    def add_dynamic(self, parent, child, stamp_ns, transform):
        e = self._edges.get(child)
        if e is None:
            e = self._edges[child] = _Edge(parent, static=False)
        if e.static:
            return
        e.stamps.append(stamp_ns)
        e.transforms.append(transform)

    def finalize(self):
        """Sort each dynamic edge by stamp, dropping duplicates (last wins)."""
        for e in self._edges.values():
            if e.static or not e.stamps:
                continue
            order = sorted(range(len(e.stamps)), key=lambda i: (e.stamps[i], i))
            stamps, transforms = [], []
            for i in order:
                if stamps and stamps[-1] == e.stamps[i]:
                    transforms[-1] = e.transforms[i]
                    continue
                stamps.append(e.stamps[i])
                transforms.append(e.transforms[i])
            e.stamps, e.transforms = stamps, transforms

    def has_frame(self, frame):
        return frame in self._edges or any(
            e.parent == frame for e in self._edges.values()
        )

    def _path(self, frame):
        """``frame`` and each ancestor in turn, ending at the chain's root."""
        path = [frame]
        seen = {frame}
        cur = frame
        while cur in self._edges:
            cur = self._edges[cur].parent
            if cur in seen:  # a cycle; not a tree
                break
            seen.add(cur)
            path.append(cur)
        return path

    def _split(self, target, source):
        """Both paths, cut at their deepest shared ancestor.

        Only the edges *below* that ancestor take part in the lookup — as in
        tf2, where a ``base_link -> lidar`` query is answered without touching
        the odometry above them.
        """
        p_src, p_tgt = self._path(source), self._path(target)
        shared = set(p_tgt)
        for anc in p_src:
            if anc in shared:
                return p_src[: p_src.index(anc)], p_tgt[: p_tgt.index(anc)]
        return None, None

    def _down(self, path, stamp_ns):
        """Compose the edges of a cut path, ancestor-first."""
        out = IDENTITY
        for frame in reversed(path):
            t = self._edges[frame].at(stamp_ns)
            if t is None:
                return None
            out = t_mul(out, t)
        return out

    def dynamic_chain(self, target, source):
        """The time-varying edges a ``lookup(target, source, t)`` depends on.

        A caller feeding these transforms live needs to know when a stamp has
        become resolvable, and it has become resolvable exactly once every one of
        these edges holds a sample at or past it.
        """
        below_src, below_tgt = self._split(target, source)
        if below_src is None:
            return set()
        return {
            (self._edges[f].parent, f)
            for f in below_src + below_tgt
            if not self._edges[f].static
        }

    def lookup(self, target, source, stamp_ns):
        """``source``'s pose expressed in ``target`` — tf2's lookupTransform."""
        below_src, below_tgt = self._split(target, source)
        if below_src is None:
            return None
        t_src = self._down(below_src, stamp_ns)
        t_tgt = self._down(below_tgt, stamp_ns)
        if t_src is None or t_tgt is None:
            return None
        return t_mul(t_inv(t_tgt), t_src)
