"""Stage 0 multi-view depth feasibility harness (research issue #21).

Before building any multi-view depth candidate, this measures -- offline, from an
existing `perception` bag -- the three quantities the whole triage in
`design/research/sfm_multiview_depth.md` hinges on. It needs no depth server and no
lidar: it is pure geometry over `/tf` (odom->base), `/image_raw/compressed`, and the
static camera mount, plus `/camera_info` for the intrinsics.

1. Parallax distribution across the image. Sparse LK tracks between odometry-selected
   keyframe pairs (camera baseline ~B_target, low inter-view rotation), disparity
   normalised to a common baseline, mapped to a grid heatmap and -- the part that
   decides forward-motion feasibility -- binned by radial distance from the focus of
   expansion (FOE) and inside the near/far halves of the usable floor band. An upright
   obstacle at the far band edge sits closest to the FOE, i.e. where parallax is
   weakest exactly where marks matter for planning distance.

2. Pose-at-stamp accuracy. (a) TF interpolation self-consistency: leave-one-out
   interpolation of each odom->base sample from its neighbours bounds the error of
   sampling TF at an image stamp between publications. (b) Stamp jitter: the header
   interval and header-vs-receipt jitter of the v4l2 stamps. Both are folded into the
   triangulation error budget below.

3. Usable-baseline duty cycle. Fraction of mission *time* spent translating (vs
   stopped or turning in place -- this robot turns in place, which carries zero useful
   parallax), and the fraction of image frames that can reach a usable baseline
   (camera translation >= B_min with inter-view rotation below a cap) within a
   staleness window.

Kill criteria (from the plan): if usable-baseline time is low, or stamp/pose error
dominates the triangulation error budget (paper target dz ~ 7-15 mm in the 0.25-1.2 m
band, i.e. matching error ~0.5-1 px at B~0.1-0.3 m), the metric multi-view candidates
(research stages B/C) die here and only the flicker-only video-depth swap (stage A)
survives. Either way the output is a verdict + diagnostic images, not a pipeline change.

Run in the dev/default env (no server needed):
    pixi run python mote_perception/tools/sfm_stage0_geometry.py <bag> [--out DIR] [...]
"""

import argparse
import os
import tempfile

import cv2
import numpy as np

import bag_utils
from mote_perception.ground_projection import (
    GroundProjector,
    chain_static_transforms,
    quat_to_matrix,
)

NS = 1e9


# --- pose interpolation -----------------------------------------------------


def mat_to_quat(m):
    """(x, y, z, w) quaternion from a 3x3 rotation matrix (Shepperd's method)."""
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def _slerp(q0, q1, a):
    d = float(np.dot(q0, q1))
    if d < 0:  # take the short way round
        q1, d = -q1, -d
    if d > 0.9995:  # nearly identical -- lerp and renormalise
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - a) * th) * q0 + np.sin(a * th) * q1) / np.sin(th)


class PoseTrack:
    """Interpolatable odom<-base pose stream (linear translation, slerp rotation)."""

    def __init__(self, stamps_ns, mats):
        self.stamps = stamps_ns
        self.trans = mats[:, :3, 3]
        self.quats = np.array([mat_to_quat(m[:3, :3]) for m in mats])

    def at(self, t_ns, skip=None):
        """4x4 odom<-base pose at a stamp, or None outside the sampled span.

        `skip` excludes one sample index from the neighbour search -- used by the
        leave-one-out interpolation self-consistency check.
        """
        s = self.stamps
        if t_ns < s[0] or t_ns > s[-1]:
            return None
        j = int(np.searchsorted(s, t_ns))
        i = j - 1
        if skip is not None:  # step the bracket past the excluded sample
            if i == skip:
                i -= 1
            if j == skip:
                j += 1
        i = max(i, 0)
        j = min(j, len(s) - 1)
        if i == j:
            q, tr = self.quats[i], self.trans[i]
        else:
            a = (t_ns - s[i]) / (s[j] - s[i])
            q = _slerp(self.quats[i], self.quats[j], a)
            tr = self.trans[i] + a * (self.trans[j] - self.trans[i])
        m = np.eye(4)
        m[:3, :3] = quat_to_matrix(*q)
        m[:3, 3] = tr
        return m


def rot_angle_deg(R):
    """Geodesic magnitude of a 3x3 rotation, in degrees."""
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


# --- 3. usable-baseline duty cycle -----------------------------------------


def duty_cycle(track, v_move, w_turn, dt_cap=0.5):
    """Fraction of mission *time* stopped / turning-in-place / translating.

    Classifies each native odom interval by base-frame linear and yaw speed. Intervals
    longer than dt_cap (a recording gap) are dropped so a pause in TF is not counted as
    a long stop. Returns (frac_stopped, frac_turning, frac_translating, moving_speeds).
    """
    s, tr, q = track.stamps, track.trans, track.quats
    t_stop = t_turn = t_move = 0.0
    speeds = []
    for i in range(len(s) - 1):
        dt = (s[i + 1] - s[i]) / NS
        if dt <= 0 or dt > dt_cap:
            continue
        v = np.linalg.norm(tr[i + 1, :2] - tr[i, :2]) / dt
        dR = quat_to_matrix(*q[i]).T @ quat_to_matrix(*q[i + 1])
        w = np.radians(rot_angle_deg(dR)) / dt
        if v < v_move and w < w_turn:
            t_stop += dt
        elif w >= w_turn and v < v_move:
            t_turn += dt
        else:
            t_move += dt
            speeds.append(v)
    total = t_stop + t_turn + t_move or 1.0
    return t_stop / total, t_turn / total, t_move / total, np.array(speeds)


def cam_centers(track, stamps, T_base_optical):
    """Camera optical-frame centre in odom for each image stamp (NaN if unposed)."""
    C = np.full((len(stamps), 3), np.nan)
    for k, t in enumerate(stamps):
        Tob = track.at(int(t))
        if Tob is not None:
            C[k] = (Tob @ T_base_optical)[:3, 3]
    return C


def cam_pose(track, t_ns, T_base_optical):
    """4x4 odom<-optical camera pose at a stamp, or None."""
    Tob = track.at(int(t_ns))
    return None if Tob is None else Tob @ T_base_optical


def usable_baseline(track, stamps, T_base_optical, B_min, rot_max, stale_max):
    """Per image frame: can it reach a usable baseline looking back in time?

    Usable = a past frame exists whose camera translation from this one is >= B_min
    with inter-view rotation < rot_max, reached within stale_max seconds. In-place
    turns swing the camera on a short arc (it is offset from the wheel axis) but the
    rotation gate rejects those, isolating genuine translation parallax. Returns
    (usable_fraction, baselines, stalenesses) over frames that are both posed and have
    enough history.
    """
    poses = [cam_pose(track, t, T_base_optical) for t in stamps]
    usable, baselines, stales = 0, [], []
    considered = 0
    for k in range(len(stamps)):
        Pk = poses[k]
        if Pk is None:
            continue
        considered += 1
        Ck = Pk[:3, 3]
        for j in range(k - 1, -1, -1):
            if (stamps[k] - stamps[j]) / NS > stale_max:
                break
            Pj = poses[j]
            if Pj is None:
                continue
            b = float(np.linalg.norm(Ck - Pj[:3, 3]))
            if b >= B_min:
                if rot_angle_deg(Pj[:3, :3].T @ Pk[:3, :3]) < rot_max:
                    usable += 1
                    baselines.append(b)
                    stales.append((stamps[k] - stamps[j]) / NS)
                break
    frac = usable / considered if considered else 0.0
    return frac, np.array(baselines), np.array(stales)


# --- 2. pose-at-stamp accuracy ---------------------------------------------


def interp_consistency(track, T_base_optical, w_turn):
    """Leave-one-out interpolation residuals at the native pose rate.

    For each interior odom sample, interpolate the pose at its own stamp from its
    neighbours (excluding it) and compare -- a conservative (2x-gap) bound on the error
    of sampling TF at an image stamp between publications. Split by whether the sample
    is translating (yaw rate < w_turn): triangulation only ever happens on those, and
    the interp error there is what enters the error budget; the turning samples are
    reported separately to show where the large residuals live. Returns a dict of
    (pos_mm, rot_mdeg) arrays for 'move' and 'all'.
    """
    s, q = track.stamps, track.quats
    move = dict(pos=[], rot=[])
    alls = dict(pos=[], rot=[])
    for i in range(1, len(s) - 1):
        if (s[i + 1] - s[i - 1]) / NS > 0.5:
            continue
        est = track.at(int(s[i]), skip=i)
        if est is None:
            continue
        true = np.eye(4)
        true[:3, :3] = quat_to_matrix(*q[i])
        true[:3, 3] = track.trans[i]
        pe = np.linalg.norm(
            ((est @ T_base_optical)[:3, 3]) - (true @ T_base_optical)[:3, 3]
        )
        re = rot_angle_deg(est[:3, :3].T @ true[:3, :3])
        dt = (s[i + 1] - s[i - 1]) / NS
        yaw = np.radians(
            rot_angle_deg(quat_to_matrix(*q[i - 1]).T @ quat_to_matrix(*q[i + 1]))
        )
        alls["pos"].append(pe * 1000)
        alls["rot"].append(re * 1000)
        if yaw / dt < w_turn:
            move["pos"].append(pe * 1000)
            move["rot"].append(re * 1000)
    return {
        k: {kk: np.array(vv) for kk, vv in d.items()}
        for k, d in (("move", move), ("all", alls))
    }


# --- 1. parallax across the image ------------------------------------------


def floor_band_rows(proj, x_near=0.25, x_far=1.2):
    """(row_near, row_far, horizon_row) image rows for the usable floor band.

    row_near is the bottom of the band (x_near ahead), row_far the top (x_far ahead,
    closest to the horizon and FOE), horizon_row where a floor ray goes parallel.
    """
    rn = float(proj.ground_to_pixels([[x_near, 0.0]])[0, 1])
    rf = float(proj.ground_to_pixels([[x_far, 0.0]])[0, 1])
    xs = np.linspace(x_far, 60.0, 200)
    rows = proj.ground_to_pixels(np.column_stack([xs, np.zeros_like(xs)]))[:, 1]
    return rn, rf, float(rows.min())


def track_pair(imgA, imgB, max_corners=1200, quality=0.01, min_dist=6, fb_thresh=1.0):
    """Forward-backward-checked LK tracks A->B. Returns (uvA, uvB) float arrays."""
    gA = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
    gB = cv2.cvtColor(imgB, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(gA, max_corners, quality, min_dist)
    if p0 is None:
        return np.empty((0, 2)), np.empty((0, 2))
    lk = dict(winSize=(21, 21), maxLevel=3)
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(gA, gB, p0, None, **lk)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(gB, gA, p1, None, **lk)
    ok = (st1.ravel() == 1) & (st2.ravel() == 1)
    fb = np.linalg.norm((p0 - p0b).reshape(-1, 2), axis=1)
    ok &= fb < fb_thresh
    return p0.reshape(-1, 2)[ok], p1.reshape(-1, 2)[ok]


def epipole(Pa, Pb, K, D):
    """FOE pixel in frame A (projection of B's centre in A's optical frame), or None."""
    t_ab = (np.linalg.inv(Pa) @ Pb)[:3, 3]
    if t_ab[2] <= 1e-3:
        return None
    e = cv2.projectPoints(t_ab.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, D)[0]
    return e.reshape(2)


def select_pairs(stamps, C, B_lo, B_hi, min_gap_frames, max_pairs):
    """Keyframe index pairs (a, b) with camera baseline in [B_lo, B_hi], spread out."""
    pairs, last_b = [], -(10**9)
    n = len(stamps)
    for a in range(n):
        if not np.isfinite(C[a]).all():
            continue
        for b in range(a + min_gap_frames, n):
            if not np.isfinite(C[b]).all():
                continue
            base = np.linalg.norm(C[b] - C[a])
            if base >= B_lo:
                if base <= B_hi and a - last_b >= min_gap_frames:
                    pairs.append((a, b))
                    last_b = a
                break
        if len(pairs) >= max_pairs:
            break
    return pairs


def parallax(bag, track, stamps, T_base_optical, proj, K, D, args, out):
    """Measure disparity across the image over odometry-selected keyframe pairs."""
    C = cam_centers(track, stamps, T_base_optical)
    pairs = select_pairs(
        stamps, C, args.baseline_lo, args.baseline_hi, args.pair_gap, args.pairs
    )
    if not pairs:
        print("  no keyframe pairs in the target baseline band -- skipping parallax")
        return None
    need = sorted({stamps[i] for p in pairs for i in p})
    frames = dict(zip(need, bag_utils.load_images_at(bag, need)))

    H, W = proj.height, proj.width
    gh, gw = 12, 16  # heatmap grid
    grid_sum = np.zeros((gh, gw))
    grid_cnt = np.zeros((gh, gw))
    rad_bins = np.linspace(0, np.hypot(H, W) / 2, 9)
    rad_sum = np.zeros(len(rad_bins) - 1)
    rad_cnt = np.zeros(len(rad_bins) - 1)
    rn, rf, horizon = floor_band_rows(proj)
    band_mid = (rn + rf) / 2
    band_near, band_far = [], []  # normalised disparities in each band half
    example = None

    used = 0
    for a, b in pairs:
        fa, fb = frames.get(stamps[a]), frames.get(stamps[b])
        if fa is None or fb is None:
            continue
        Pa = cam_pose(track, stamps[a], T_base_optical)
        Pb = cam_pose(track, stamps[b], T_base_optical)
        base = float(np.linalg.norm(C[b] - C[a]))
        rot = rot_angle_deg(Pa[:3, :3].T @ Pb[:3, :3])
        if rot > args.rot_max:
            continue
        uvA, uvB = track_pair(fa, fb)
        if len(uvA) < 20:
            continue
        disp = np.linalg.norm(uvB - uvA, axis=1) * (args.baseline_norm / base)
        used += 1
        e = epipole(Pa, Pb, K, D)

        for (u, v), d in zip(uvA, disp):
            gi = min(int(v / H * gh), gh - 1)
            gj = min(int(u / W * gw), gw - 1)
            grid_sum[gi, gj] += d
            grid_cnt[gi, gj] += 1
            if e is not None:
                r = np.hypot(u - e[0], v - e[1])
                rb = np.searchsorted(rad_bins, r) - 1
                if 0 <= rb < len(rad_sum):
                    rad_sum[rb] += d
                    rad_cnt[rb] += 1
            if rf <= v <= rn:  # inside the floor band (rf is the far/top row)
                (band_far if v < band_mid else band_near).append(d)

        if example is None and e is not None:
            example = _draw_pair(fa, uvA, uvB, e, (rn, rf), args.baseline_norm)

    if used == 0:
        print("  keyframe pairs found but none tracked enough features")
        return None

    grid = np.where(grid_cnt > 0, grid_sum / np.maximum(grid_cnt, 1), np.nan)
    _save_heatmap(grid, proj, (rn, rf, horizon), f"{out}/parallax_heatmap.png")
    if example is not None:
        cv2.imwrite(f"{out}/parallax_example.png", example)
    rad_mean = np.where(rad_cnt > 0, rad_sum / np.maximum(rad_cnt, 1), np.nan)

    near_foe = float(rad_mean[0]) if np.isfinite(rad_mean[0]) else float("nan")
    return dict(
        pairs=used,
        baseline_norm=args.baseline_norm,
        grid=grid,
        grid_cnt=grid_cnt,
        rad_bins=rad_bins,
        rad_mean=rad_mean,
        near_foe=near_foe,
        band_near=np.array(band_near),
        band_far=np.array(band_far),
        rows=(rn, rf, horizon),
    )


def _draw_pair(img, uvA, uvB, e, band, bnorm):
    vis = img.copy()
    for (ua, va), (ub, vb) in zip(uvA, uvB):
        cv2.arrowedLine(
            vis, (int(ua), int(va)), (int(ub), int(vb)), (0, 255, 0), 1, tipLength=0.3
        )
    rn, rf = band
    cv2.line(vis, (0, int(rn)), (vis.shape[1], int(rn)), (255, 180, 0), 1)
    cv2.line(vis, (0, int(rf)), (vis.shape[1], int(rf)), (255, 180, 0), 1)
    if 0 <= e[0] < vis.shape[1] and 0 <= e[1] < vis.shape[0]:
        cv2.drawMarker(
            vis, (int(e[0]), int(e[1])), (0, 0, 255), cv2.MARKER_CROSS, 20, 2
        )
    cv2.putText(
        vis,
        f"FOE (red), floor band (orange); arrows scaled -- disparity norm to B={bnorm}m",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    return vis


def _save_heatmap(grid, proj, rows, path):
    H, W = proj.height, proj.width
    gh, gw = grid.shape
    vmax = np.nanmax(grid) if np.isfinite(grid).any() else 1.0
    cells = np.clip(np.nan_to_num(grid) / max(vmax, 1e-6) * 255, 0, 255).astype(
        np.uint8
    )
    big = cv2.resize(cells, (W, H), interpolation=cv2.INTER_NEAREST)
    heat = cv2.applyColorMap(big, cv2.COLORMAP_TURBO)
    heat[cv2.resize((grid != grid).astype(np.uint8), (W, H)) > 0] = (40, 40, 40)
    rn, rf, horizon = rows
    for r in (rn, rf):
        cv2.line(heat, (0, int(r)), (W, int(r)), (255, 255, 255), 1)
    cv2.line(heat, (0, int(horizon)), (W, int(horizon)), (0, 0, 0), 1)
    for i in range(gh):
        for j in range(gw):
            if np.isfinite(grid[i, j]):
                cv2.putText(
                    heat,
                    f"{grid[i, j]:.1f}",
                    (int((j + 0.15) * W / gw), int((i + 0.6) * H / gh)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (255, 255, 255),
                    1,
                )
    cv2.putText(
        heat,
        "disparity px (norm baseline); white=floor band, black=horizon",
        (8, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.imwrite(path, heat)


# --- report ----------------------------------------------------------------


def _pct(a, ps=(50, 90)):
    return "  ".join(f"p{p}={np.percentile(a, p):.3f}" for p in ps) if len(a) else "n/a"


def budget(par, jit, interp, v_med, f, B, z_list):
    """Fold the measured errors into a triangulation error budget and a verdict.

    Only the *translating* interp residuals enter the budget -- triangulation runs on
    translating pairs, and the large turning residuals ride along with the zero-baseline
    gate. Two pose-error channels: baseline error (stamp jitter x speed + interp
    translation error) shrinks depth accuracy directly; residual-rotation error (interp
    rotation + jitter x any residual yaw) reprojects into a disparity error competing
    with the matcher.
    """
    lines = ["", "=== TRIANGULATION ERROR BUDGET (kill criterion) ==="]
    sigma_t = jit["recv_jit_ms"] / 1000.0  # stamp jitter, s (1 sigma)
    mv = interp["move"]
    interp_pos_mm = np.percentile(mv["pos"], 90) if len(mv["pos"]) else 0.0
    interp_rot_rad = (
        np.radians(np.percentile(mv["rot"], 90) / 1000) if len(mv["rot"]) else 0.0
    )
    dB = v_med * sigma_t * np.sqrt(2)  # baseline error from stamp jitter
    dB_total = dB + interp_pos_mm / 1000.0
    dd_pose = f * interp_rot_rad  # rotation interp reprojects to this many px
    lines.append(
        f"stamp jitter sigma_t={sigma_t * 1000:.1f} ms x moving speed {v_med:.2f} m/s"
        f" -> dB~{dB * 1000:.1f} mm; + interp pos p90 {interp_pos_mm:.1f} mm"
        f" -> total dB~{dB_total * 1000:.1f} mm"
    )
    lines.append(
        f"residual-rotation reprojection (interp rot p90 {np.degrees(interp_rot_rad):.3f} deg"
        f" x f={f:.0f}) <= {dd_pose:.2f} px  [conservative 2x-gap+ICP-noise bound;"
        " vs ~0.7 px matcher noise -- only bites near the FOE where parallax is weakest]"
    )
    if par is not None:
        lines.append(
            f"parallax within {par['rad_bins'][1]:.0f} px of the FOE (norm to B={B} m)"
            f" = {par['near_foe']:.2f} px  [matching needs ~0.5-1 px to triangulate]"
        )
    lines.append(f"  {'z':>5} {'dz|match(0.7px)':>16} {'dz|pose-baseline':>17}")
    for z in z_list:
        dz_match = z * z * 0.7 / (f * B) * 1000
        dz_base = z * dB_total / B * 1000
        lines.append(f"  {z:>5.2f} {dz_match:>13.1f} mm {dz_base:>14.1f} mm")
    lines.append(
        "  (dz|match is matcher-limited depth error at 0.7 px; dz|pose-baseline is the"
        " pose/stamp contribution -- multi-view dies if the latter dominates)"
    )
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag")
    ap.add_argument("--out", default=None)
    ap.add_argument("--v-move", type=float, default=0.03, help="translating speed thr")
    ap.add_argument("--w-turn", type=float, default=0.15, help="in-place turn yaw thr")
    ap.add_argument("--baseline-min", type=float, default=0.10, help="usable baseline")
    ap.add_argument("--rot-max", type=float, default=8.0, help="max inter-view rot deg")
    ap.add_argument("--stale-max", type=float, default=3.0, help="max look-back s")
    ap.add_argument("--baseline-lo", type=float, default=0.10)
    ap.add_argument("--baseline-hi", type=float, default=0.30)
    ap.add_argument("--baseline-norm", type=float, default=0.10, help="disp norm base")
    ap.add_argument("--pairs", type=int, default=40, help="max keyframe pairs")
    ap.add_argument("--pair-gap", type=int, default=15, help="min frames between pairs")
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix="sfm_stage0_")
    os.makedirs(out, exist_ok=True)

    tf_static, caminfo = bag_utils.load_static_context(args.bag)
    T_base_optical = chain_static_transforms(
        tf_static.transforms, "camera_optical_link", "base_footprint"
    )
    proj = GroundProjector.from_camera_info(caminfo, T_base_optical)
    K = np.asarray(caminfo.k, np.float64).reshape(3, 3)
    D = np.asarray(caminfo.d, np.float64)
    f = float(K[0, 0])

    stamps_ns, mats = bag_utils.load_tf_poses(args.bag, "odom", "base_footprint")
    track = PoseTrack(stamps_ns, mats)
    hdr, recv = bag_utils.load_image_stamps(args.bag)

    dur = (stamps_ns[-1] - stamps_ns[0]) / NS
    path_len = float(np.linalg.norm(np.diff(mats[:, :2, 3], axis=0), axis=1).sum())
    print(
        f"bag: {len(hdr)} frames, {len(stamps_ns)} odom poses, {dur:.0f}s, "
        f"{path_len:.1f}m driven ({path_len / dur:.2f} m/s avg) -> {out}\n"
    )

    # 3. duty cycle
    f_stop, f_turn, f_move, mspeeds = duty_cycle(track, args.v_move, args.w_turn)
    frac_use, bl, stale = usable_baseline(
        track, hdr, T_base_optical, args.baseline_min, args.rot_max, args.stale_max
    )
    v_med = float(np.median(mspeeds)) if len(mspeeds) else 0.0
    print("=== 3. USABLE-BASELINE DUTY CYCLE ===")
    print(
        f"time: stopped {f_stop * 100:.0f}%  turning-in-place {f_turn * 100:.0f}%  "
        f"translating {f_move * 100:.0f}%   (median moving speed {v_med:.3f} m/s)"
    )
    print(
        f"frames reaching usable baseline (>= {args.baseline_min} m, rot < "
        f"{args.rot_max} deg, within {args.stale_max}s): {frac_use * 100:.0f}%"
    )
    if len(bl):
        print(f"  achieved baseline m: {_pct(bl)}   look-back s: {_pct(stale)}")

    # 2. pose-at-stamp accuracy
    hdr_dt = np.diff(hdr) / 1e6  # ms
    offset = (recv - hdr) / 1e6  # ms
    recv_jit = float(np.std(offset))
    interp = interp_consistency(track, T_base_optical, args.w_turn)
    print("\n=== 2. POSE-AT-STAMP ACCURACY ===")
    print(
        f"image header interval: median {np.median(hdr_dt):.1f} ms  "
        f"std {np.std(hdr_dt):.1f}  (dropped-frame max {hdr_dt.max():.0f})"
    )
    print(
        f"header->receipt offset: mean {np.mean(offset):.1f} ms  "
        f"jitter(std) {recv_jit:.1f} ms   (this jitter x speed = baseline error)"
    )
    print(
        f"TF interp self-consistency (leave-one-out, ~2x the {np.median(np.diff(stamps_ns)) / 1e6:.0f}ms "
        "pose gap):"
    )
    print(
        f"  translating: cam-pos mm {_pct(interp['move']['pos'])}  "
        f"rot mdeg {_pct(interp['move']['rot'])}   <- enters budget"
    )
    print(
        f"  all (incl. turns): cam-pos mm {_pct(interp['all']['pos'])}  "
        f"rot mdeg {_pct(interp['all']['rot'])}"
    )

    # 1. parallax
    print("\n=== 1. PARALLAX ACROSS THE IMAGE ===")
    par = parallax(args.bag, track, hdr, T_base_optical, proj, K, D, args, out)
    if par is not None:
        rn, rf, horizon = par["rows"]
        print(
            f"tracked {par['pairs']} keyframe pairs; disparity normalised to "
            f"B={par['baseline_norm']} m"
        )
        off = " (near edge below frame)" if rn > proj.height else ""
        print(
            f"floor band rows: near(bottom)={rn:.0f} far(top)={rf:.0f} "
            f"horizon={horizon:.0f} [image H={proj.height}]{off}"
        )
        bn, bf = par["band_near"], par["band_far"]
        print(
            f"  floor-band-row features (norm px): near/bottom half {_pct(bn)} (n={len(bn)})"
        )
        print(
            f"                                     far/top  half {_pct(bf)} (n={len(bf)})"
        )
        print(
            "  (few near/bottom tracks = textureless close floor -- kills dense floor match)"
        )
        print(
            "  disparity vs radius from FOE (px, inner->outer; the honest near-FOE test):"
        )
        for lo, hi, m in zip(par["rad_bins"], par["rad_bins"][1:], par["rad_mean"]):
            bar = "#" * int(np.nan_to_num(m) * 3)
            print(f"    {lo:4.0f}-{hi:4.0f}px: {m:5.2f} {bar}")

    # budget + verdict
    jit = dict(recv_jit_ms=recv_jit)
    for ln in budget(
        par, jit, interp, max(v_med, 0.01), f, args.baseline_norm, [0.5, 1.0, 1.2]
    ):
        print(ln)

    # Two kinds of kill with different consequences: a GEOMETRY kill (no parallax /
    # stamp error dominates) sinks any triangulation, so only stage A survives. A
    # DUTY-CYCLE kill (robot rarely translates) means multi-view can't be the *primary*
    # depth path but still works as a supplement that holds while stopped -- which
    # favours stage B (anchors persist when parked) over a full stage-C replacement.
    print("\n=== VERDICT ===")
    geom_kill, duty_kill = [], []
    near_foe = par["near_foe"] if par is not None else float("nan")
    if np.isfinite(near_foe) and near_foe < 0.5:
        geom_kill.append(
            f"near-FOE parallax {near_foe:.2f} px < 0.5 px (norm baseline)"
        )
    if par is None:
        geom_kill.append("no keyframe pair tracked enough features (matching too weak)")
    if f_move < 0.35:
        duty_kill.append(
            f"translating only {f_move * 100:.0f}% of mission time (< 35%)"
        )
    if frac_use < 0.5:
        duty_kill.append(
            f"only {frac_use * 100:.0f}% of frames reach a usable baseline (< 50%)"
        )

    if geom_kill:
        print("GEOMETRY KILL for metric multi-view (stages B and C):")
        for k in geom_kill:
            print(f"  - {k}")
        print("  => triangulation is unreliable here; keep the single-image path and")
        print("     pursue only stage A (flicker-only video-depth swap).")
    elif duty_kill:
        print("Geometry is adequate when moving, but a DUTY-CYCLE limit is decisive:")
        for k in duty_kill:
            print(f"  - {k}")
        print("  => metric multi-view cannot be the PRIMARY depth path (it would fall")
        print("     back to the single image most of the time). Recommendation:")
        print(
            "     * stage A (video-depth flicker swap) -- robust, motion-independent: DO."
        )
        print(
            "     * stage B (sparse triangulated anchors) -- viable SUPPLEMENT; holds"
        )
        print(
            "       anchors while stopped, so low duty cycle hurts it least: OPTIONAL."
        )
        print(
            "     * stage C (learned-MVS replacement) -- high cost for <1/3 duty: DEFER."
        )
    else:
        print("No kill signal: geometry sound AND enough usable-baseline time.")
        print(
            "  => a stage-B prototype (triangulated anchors vs the lidar) is warranted."
        )
    print(f"\nimages + heatmap written to {out}")


if __name__ == "__main__":
    main()
