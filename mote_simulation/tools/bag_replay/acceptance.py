"""Predict, exactly, which scans slam_toolbox will insert into its pose graph.

A paced replay spends its wall clock waiting: of the 12,811 scans in the
21-minute flat mapping bag of 2026-07-29, slam_toolbox inserts 186 into its pose
graph, and the rest cost nothing but the real-time gap between them. Lockstep
replay (``replayer.py --lockstep``) skips that gap by feeding only the scans the
node would keep — which is only sound if "would keep" is *predicted*, not
approximated. An approximation was tried and failed: gating on travel with a
safety margin chained the feed anchor on fed scans while slam chains its own on
accepted ones, so feed spacing quantised the node spacing and the graph came out
silently different (63% cell agreement).

So this module is a transcription of the real chain, in two stages, both of which
run on **odometry alone** — no scan match, no correction, nothing the replay can
only learn by running SLAM:

1. ``SlamToolbox::shouldProcessScan`` (``slam_toolbox_common.cpp``) — a
   free first measurement, then throttle, ``minimum_time_interval``, a
   fixed 5-scan warm-up, and squared travel of the **base** frame against
   ``0.8 * minimum_travel_distance^2``. Note there is no heading test here
   unless ``check_min_dist_and_heading_precisely`` is set.
2. ``karto::Mapper::HasMovedEnough`` (``lib/karto_sdk/src/Mapper.cpp``) — reached
   only if (1) passed, and testing the **sensor** frame (base composed with the
   laser's mounting offset) against the *full* ``minimum_travel_distance`` and
   ``minimum_travel_heading``.

The two stages keep **separate anchors**, and stage 1's advances even when stage 2
rejects — that gap (travel between 0.8x and 1.0x the distance gate) is why a scan
stage 2 will discard still has to be fed: withholding it would leave stage 1's
anchor where the real node's would not be. So ``simulate`` distinguishes the two
rejections: ``MAPPER_REJECT`` is published like an insertion but acknowledged by
nothing, while ``NODE_REJECT`` is never published at all.

Karto's own ``MinimumTimeInterval`` accept is in the chain for completeness but
never fires in practice: slam_toolbox never sets it, so it keeps Karto's 3600 s
default while the *node* parameter of the same name is a separate, unrelated
reject test.

Everything here is a pure function over ``(stamp, pose)`` pairs, so the chain is
unit-tested without ROS, a bag, or a running node — and verified against the real
node at replay time, where every predicted acceptance must be acknowledged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from tf_lookup import normalize_angle, se2_compose

#: ``shouldProcessScan``'s "for initial stabilization" test: ``scan_ctr < 5``.
WARMUP_MIN_CTR = 5
#: karto ``Math.h``.
KT_TOLERANCE = 1e-06
#: karto's ``MinimumTimeInterval`` default. slam_toolbox never overrides it — its
#: own ``minimum_time_interval`` parameter goes to the node gate instead.
KARTO_MINIMUM_TIME_INTERVAL = 3600.0

#: Withheld before the node ever sees it — no pose at the scan stamp, so
#: ``getOdomPose`` would fail (or tf2's message filter would never dispatch it).
NO_TF = "no_tf"
#: Rejected by ``shouldProcessScan``: not fed, and none of the node's state moves.
NODE_REJECT = "node_reject"
#: Accepted by ``shouldProcessScan``, rejected by ``HasMovedEnough``: fed, because
#: the node gate's anchor advances on it, but no pose graph node and so no ack.
MAPPER_REJECT = "mapper_reject"
#: Inserted into the pose graph — fed, and must be acknowledged on ``/pose``.
ACCEPT = "accept"


@dataclass(frozen=True)
class Gates:
    """The acceptance chain's parameters, as the node resolves them."""

    throttle_scans: int = 1
    minimum_time_interval_ns: int = 500_000_000
    minimum_travel_distance: float = 0.5
    minimum_travel_heading: float = 0.5
    check_precisely: bool = False

    @staticmethod
    def from_params(params):
        """Read the gates out of a slam_toolbox params file's parsed contents.

        Accepts either the whole file (``slam_toolbox: ros__parameters: ...``) or
        the inner mapping. Defaults match the node's own, including the one
        upstream quirk worth preserving: ``minimum_time_interval`` defaults to
        whatever ``transform_timeout`` resolved to, because ``setParams`` reuses
        the same scratch variable for both.
        """
        p = params or {}
        p = p.get("slam_toolbox", p)
        p = p.get("ros__parameters", p)
        transform_timeout = float(p.get("transform_timeout", 0.5))
        interval = float(p.get("minimum_time_interval", transform_timeout))
        return Gates(
            throttle_scans=int(p.get("throttle_scans", 1)),
            minimum_time_interval_ns=int(interval * 1e9),
            minimum_travel_distance=float(p.get("minimum_travel_distance", 0.5)),
            minimum_travel_heading=float(p.get("minimum_travel_heading", 0.5)),
            check_precisely=bool(p.get("check_min_dist_and_heading_precisely", False)),
        )


def simulate(scans, gates, laser_offset=(0.0, 0.0, 0.0)):
    """Decide each scan of a stream, in order.

    ``scans`` is an iterable of ``(stamp_ns, pose)`` where ``pose`` is the
    ``(x, y, yaw)`` of the node's ``base_frame`` in the odom frame at that stamp,
    or ``None`` when no transform covers it. ``laser_offset`` is the laser's
    ``(x, y, yaw)`` in ``base_frame``. Returns a list of decision constants, one
    per input scan.
    """
    min_dist2 = gates.minimum_travel_distance * gates.minimum_travel_distance
    karto_dist2 = min_dist2 - KT_TOLERANCE

    first_measurement = True
    scan_ctr = 0
    last_pose = None
    last_stamp_ns = 0
    mapper_pose = None  # odometric base pose of the last graph-inserted scan
    mapper_time = 0.0

    out = []
    for stamp_ns, pose in scans:
        if pose is None:
            out.append(NO_TF)
            continue

        scan_ctr += 1
        if first_measurement:
            first_measurement = False
        elif scan_ctr % gates.throttle_scans != 0:
            out.append(NODE_REJECT)
            continue
        elif stamp_ns - last_stamp_ns < gates.minimum_time_interval_ns:
            out.append(NODE_REJECT)
            continue
        elif scan_ctr < WARMUP_MIN_CTR:
            out.append(NODE_REJECT)
            continue
        else:
            dist2 = (pose[0] - last_pose[0]) ** 2 + (pose[1] - last_pose[1]) ** 2
            if gates.check_precisely:
                heading = abs(normalize_angle(pose[2] - last_pose[2]))
                if dist2 < min_dist2 and heading < gates.minimum_travel_heading:
                    out.append(NODE_REJECT)
                    continue
            elif dist2 < 0.8 * min_dist2:
                out.append(NODE_REJECT)
                continue
        last_pose = pose
        last_stamp_ns = stamp_ns

        time_s = stamp_ns / 1e9
        if mapper_pose is None:
            inserted = True
        elif time_s - mapper_time >= KARTO_MINIMUM_TIME_INTERVAL:
            inserted = True
        else:
            sensor = se2_compose(pose, laser_offset)
            prev = se2_compose(mapper_pose, laser_offset)
            heading = abs(normalize_angle(sensor[2] - prev[2]))
            if heading >= gates.minimum_travel_heading:
                inserted = True
            else:
                travel2 = (sensor[0] - prev[0]) ** 2 + (sensor[1] - prev[1]) ** 2
                inserted = travel2 >= karto_dist2
        if inserted:
            mapper_pose = pose
            mapper_time = time_s
        out.append(ACCEPT if inserted else MAPPER_REJECT)
    return out


#: Published only to move the node's scan counter along; see ``pad_before``.
FILLER = "filler"


def pad_before(sent, throttle_scans, first):
    """Filler scans needed so the next real scan lands on a usable scan counter.

    The node counts *every* scan it receives, and lockstep does not send it every
    scan, so two counter-dependent tests would otherwise fire on the wrong scans:
    the ``throttle_scans`` modulo, and the 5-scan warm-up — which alone silently
    swallows the second, third and fourth insertion of every lockstep leg.

    The counter is realigned by feeding extra scans that the chain will reject,
    and the arithmetic here is what guarantees they are rejected *without any
    assumption about their content*: the padded counters run from where we are up
    to just below the next multiple of ``throttle_scans``, so a filler either sits
    below the warm-up threshold or on a non-multiple, and in both cases the chain
    returns before it looks at travel or time.

    ``sent`` is how many scan messages the node has already received; ``first``
    marks the scan that takes the free first-measurement pass.
    """
    if first:
        return 0
    ctr = sent + 1
    target = max(ctr, WARMUP_MIN_CTR)
    if throttle_scans > 1:
        target = int(math.ceil(target / throttle_scans)) * throttle_scans
    return target - ctr


@dataclass(frozen=True)
class FeedStep:
    """One scan to publish: its index in the bag's scan stream, and its role."""

    index: int
    role: str

    @property
    def expect_ack(self):
        return self.role == ACCEPT


def feed_plan(decisions, gates):
    """Turn per-scan decisions into the exact sequence lockstep publishes.

    Padding is drawn from the bag's *own* nearby rejected scans rather than
    invented, which is what keeps it honest twice over: a filler is a scan the
    node can place (a re-sent old scan cannot be — slam's transform cache is 30 s
    deep and a lockstep leg outruns that in a moment, so the re-send would be
    parked in the message filter and never reach the counter at all), and it is a
    scan whose rejection needs no argument beyond the counter arithmetic above.
    """
    plan = []
    sent = 0
    first = True
    previous = -1
    for i, decision in enumerate(decisions):
        if decision not in (ACCEPT, MAPPER_REJECT):
            continue
        pad = pad_before(sent, gates.throttle_scans, first)
        if pad:
            # Only NODE_REJECT scans qualify: a scan with no transform never
            # reaches the counter, so it would pad nothing.
            fillers = [
                j for j in range(previous + 1, i) if decisions[j] == NODE_REJECT
            ][:pad]
            if len(fillers) < pad:
                raise ValueError(
                    f"cannot realign the scan counter before scan {i}: {pad} "
                    f"filler scans needed, {len(fillers)} available"
                )
            plan += [FeedStep(index=j, role=FILLER) for j in fillers]
            sent += pad
        plan.append(FeedStep(index=i, role=decision))
        sent += 1
        first = False
        previous = i
    return plan
