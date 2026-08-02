#!/usr/bin/env python3
"""Angular-structure scoring, on synthetic wall masks only (no data files).

The fixtures encode the design's central claim: a building with an angled
hallway has three genuine wall directions and must not be scored as broken for
it, while a drift-rotated *section* of a rectilinear building — which duplicates
that section's whole orthogonal frame — must be separable from it.

Masks are drawn analytically at each angle rather than by rotating a raster, so
a rotation test measures the metric and not the resampler. Residual movement
under rotation is the staircase of a rasterised diagonal line; the pinned
tolerances are set from the measured values with margin, and are stated in the
assertions rather than hidden in a helper.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mote_bringup.map_cleanup.angular_stats import (  # noqa: E402
    SpectrumParams,
    angular_stats,
    fold_90,
    refine_peak,
    wall_rotation,
)

N = 600

# Axis-aligned rooms: (centre_y, centre_x, height, width), in a 600 px canvas.
ROOMS = [
    (130, 130, 155, 185),
    (130, 355, 155, 215),
    (340, 130, 170, 185),
    (340, 370, 170, 240),
    (500, 240, 115, 285),
]


def _line(mask, p0, p1):
    n = int(max(abs(p1[0] - p0[0]), abs(p1[1] - p0[1]))) * 3 + 2
    iy = np.rint(np.linspace(p0[0], p1[0], n)).astype(int)
    ix = np.rint(np.linspace(p0[1], p1[1], n)).astype(int)
    ok = (iy >= 0) & (iy < mask.shape[0]) & (ix >= 0) & (ix < mask.shape[1])
    mask[iy[ok], ix[ok]] = True


def _rect(mask, cy, cx, h, w, deg=0.0):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    pts = [
        (cy + dy * c - dx * s, cx + dy * s + dx * c)
        for dy, dx in (
            (-h / 2, -w / 2),
            (-h / 2, w / 2),
            (h / 2, w / 2),
            (h / 2, -w / 2),
        )
    ]
    for i in range(4):
        _line(mask, pts[i], pts[(i + 1) % 4])


def pure(deg=0.0):
    """A rectilinear room set, optionally rotated as a whole."""
    m = np.zeros((N, N), bool)
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    for cy, cx, h, w in ROOMS:
        dy, dx = cy - N / 2, cx - N / 2
        _rect(m, N / 2 + dy * c - dx * s, N / 2 + dy * s + dx * c, h, w, deg)
    return m


def angled_corridor(deg=20.0):
    """The operator's case: the same rooms plus a hallway at an angle.

    Drawn as two parallel walls with no end caps, which is what a hallway is —
    it opens at both ends — so it contributes exactly *one* extra direction.
    """
    m = pure()
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    cy, cx, length = 305, 300, 430
    for off in (-19.0, 19.0):
        _line(
            m,
            (cy + off * c + length / 2 * s, cx + off * s - length / 2 * c),
            (cy + off * c - length / 2 * s, cx + off * s + length / 2 * c),
        )
    return m


def rotated_rooms(deg=13.0):
    """A drift-rotated section: a subset of rooms carrying its own frame."""
    m = np.zeros((N, N), bool)
    for i, (cy, cx, h, w) in enumerate(ROOMS):
        _rect(m, cy, cx, h, w, deg if i >= 3 else 0.0)
    return m


def _dirs(stats):
    return [d["angle_deg"] for d in stats["directions"]]


def _shift(a, b):
    """Signed-free angular movement between two orientations, on [0, 90]."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


# --------------------------------------------------------------------------
# The ranked scalars


def test_rectilinear_map_is_angularly_tight():
    """A pure rectilinear map uses few directions and smears little."""
    s = angular_stats(pure())
    assert s["angular_support_deg"] < 25.0, s["angular_support_deg"]
    assert s["angular_entropy_norm"] < 0.65, s["angular_entropy_norm"]
    assert s["unassigned_energy_frac"] < 0.10, s["unassigned_energy_frac"]
    # Two wall families, one orthogonal frame holding both.
    assert s["n_peaks"] == 2, _dirs(s)
    assert len(s["frames"]) == 1
    assert s["frames"][0]["n_directions"] == 2
    assert s["dominant_frame_share"] > 0.9


def test_angled_corridor_is_not_scored_as_a_defect():
    """The regression that keeps the metric from calling the operator's flat broken.

    A third genuine wall direction must cost something (three families is more
    angular support than two, by construction) but must not read as damage.
    """
    base = angular_stats(pure())
    corr = angular_stats(angled_corridor())

    # Support may rise, but nowhere near a defect's rise (see the next test,
    # where a rotated section is pinned well above this bound).
    assert corr["angular_support_deg"] < 1.30 * base["angular_support_deg"], (
        corr["angular_support_deg"],
        base["angular_support_deg"],
    )
    assert corr["angular_entropy_norm"] < 1.10 * base["angular_entropy_norm"]
    # Smear must not rise at all: a coherent extra family is not smear.
    assert corr["unassigned_energy_frac"] <= base["unassigned_energy_frac"] + 0.02

    # It is one extra direction, in a frame of its own holding only itself.
    assert corr["n_peaks"] == 3, _dirs(corr)
    secondary = corr["frames"][1]
    assert secondary["n_directions"] == 1, corr["frames"]


def test_rotated_section_separates_from_an_angled_corridor():
    """A duplicated frame is what a drift-rotated section looks like."""
    corr = angular_stats(angled_corridor())
    rot = angular_stats(rotated_rooms(13.0))

    # The frame table is the discriminator: two directions ~90 deg apart in the
    # secondary frame, against the corridor's one.
    assert len(rot["frames"]) >= 2, rot["frames"]
    assert rot["frames"][1]["n_directions"] >= 2, rot["frames"]
    assert corr["frames"][1]["n_directions"] == 1, corr["frames"]

    # And it is angularly looser than the corridor on the ranked scalars.
    assert rot["angular_support_deg"] > corr["angular_support_deg"]


@pytest.mark.parametrize("skew", [20.0, 25.0, 30.0, 40.0])
def test_frame_table_is_trustworthy_for_the_tears_it_is_relied_on_for(skew):
    """The tear alarm's working band.

    Where the trajectory does not close there is no loop-drift number, and this
    table is the only automated tear signal. It is relied on for tears of about
    20 deg and up (run 3's real pair were 22.5 and 41 deg apart), so that band
    is pinned rather than left to the 13 deg case alone.
    """
    s = angular_stats(rotated_rooms(skew))
    assert len(s["frames"]) >= 2, s["frames"]
    secondary = s["frames"][1]
    assert secondary["n_directions"] >= 2, s["frames"]
    assert secondary["energy_frac"] > 0.15, s["frames"]
    # The reported offset is the tear angle, folded onto [0, 45].
    expected = skew % 90.0
    expected = min(expected, 90.0 - expected)
    assert abs(secondary["offset_from_dominant_deg"] - expected) < 5.0, s["frames"]


def test_frame_table_is_blind_below_its_merge_tolerance():
    """The limit, pinned as a fact rather than left as a caveat in prose.

    Directions closer than the merge tolerance are one frame by construction --
    they have to be, or the shear a genuine frame carries (7.5 deg measured on a
    real leg) would be reported as a tear. So a small rotation is invisible here
    and something with a prior has to catch it.
    """
    s = angular_stats(rotated_rooms(5.0))
    assert len(s["frames"]) == 1, s["frames"]
    # ...and the scalars do still notice something changed, they just cannot say
    # whether it is drift or architecture.
    assert s["angular_support_deg"] > angular_stats(pure())["angular_support_deg"]


def test_reference_directions_convict_the_rotation_and_absolve_the_hallway():
    """The defect verdict needs a prior; with one, the two cases part decisively."""
    base = angular_stats(pure())
    ref = _dirs(base)  # the site's known rectilinear directions

    clean = angular_stats(pure(), reference_directions=ref)
    rot = angular_stats(rotated_rooms(13.0), reference_directions=ref)
    # Declared without the hallway, the hallway itself reads as off-reference —
    # which is precisely why the set must include it.
    corr_undeclared = angular_stats(angled_corridor(), reference_directions=ref)
    corr_declared = angular_stats(
        angled_corridor(),
        reference_directions=ref + [_dirs(angular_stats(angled_corridor()))[1]],
    )

    assert clean["off_reference_energy_frac"] < 0.10
    assert rot["off_reference_energy_frac"] > 0.4
    assert (
        rot["off_reference_energy_frac"]
        > 2 * corr_undeclared["off_reference_energy_frac"]
    )
    # Declaring the hallway is what makes the good map score as a good map.
    assert corr_declared["off_reference_energy_frac"] < 0.10, corr_declared

    # A map matching its reference sits on it; a torn one does not.
    assert clean["reference_dispersion_deg"] < rot["reference_dispersion_deg"]


def test_reference_set_is_free_to_rotate_as_a_whole():
    """A map frame's absolute rotation is arbitrary, so the fit absorbs it."""
    ref = _dirs(angular_stats(pure()))
    turned = angular_stats(pure(23.0), reference_directions=ref)
    assert turned["off_reference_energy_frac"] < 0.10, turned


# --------------------------------------------------------------------------
# Rotation invariance


@pytest.mark.parametrize("deg", [17.0, -31.0])
def test_ranked_scalars_survive_a_global_rotation(deg):
    """The same building, turned: the ranked scalars must not move materially.

    Rotation is a circular shift of the angular spectrum, and none of the three
    depend on where the shift starts. The residual is the staircase of drawing
    diagonal lines on a pixel grid (measured within 6%; pinned at 10%).
    """
    base = angular_stats(pure())
    turned = angular_stats(pure(deg))

    for key, tol in (
        ("angular_support_deg", 0.10),
        ("angular_entropy_norm", 0.10),
    ):
        rel = abs(turned[key] - base[key]) / base[key]
        assert rel < tol, (key, base[key], turned[key], rel)
    assert abs(turned["unassigned_energy_frac"] - base["unassigned_energy_frac"]) < 0.05


@pytest.mark.parametrize("deg", [17.0, -31.0])
def test_direction_table_tracks_a_global_rotation(deg):
    """The families are the same families, moved by the rotation."""
    base = _dirs(angular_stats(pure()))
    turned = _dirs(angular_stats(pure(deg)))
    assert len(turned) == len(base)

    expected = abs(deg) % 90.0
    expected = min(expected, 90.0 - expected)
    for a in turned:
        moved = min(_shift(a, b) for b in base)
        assert abs(moved - expected) < 4.0, (deg, base, turned, a, moved)


# --------------------------------------------------------------------------
# The alignment primitive: windowed, folded 0/90, sub-bin interpolated


def _room_outline(deg, n=400, half_y=120, half_x=150, thick=3):
    """A single rotated rectangular room outline - a clean rotation target."""
    yy, xx = np.mgrid[0:n, 0:n]
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    y = (yy - n / 2) * c + (xx - n / 2) * s
    x = -(yy - n / 2) * s + (xx - n / 2) * c
    outer = (np.abs(y) < half_y) & (np.abs(x) < half_x)
    inner = (np.abs(y) < half_y - thick) & (np.abs(x) < half_x - thick)
    return outer & ~inner


@pytest.mark.parametrize("deg", [0.0, 1.5, 7.25, 17.0, 23.5, 31.0, 44.0])
def test_wall_rotation_recovers_a_known_rotation(deg):
    r = wall_rotation(_room_outline(deg))
    truth = (-deg) % 90.0
    err = abs(r["angle_deg"] - truth) % 90.0
    err = min(err, 90.0 - err)
    assert err < 1.5, (deg, truth, r)
    assert r["energy_frac"] > 0.3, r


def test_sub_bin_refinement_beats_the_bin_grid():
    """Why the interpolation is there: 0.5 deg bins alone are not enough."""
    raw, refined = [], []
    for deg in (1.5, 7.25, 12.0, 23.5, 31.0, 44.0):
        r = wall_rotation(_room_outline(deg))
        truth = (-deg) % 90.0
        for value, into in ((r["bin_angle_deg"], raw), (r["angle_deg"], refined)):
            e = abs(value - truth) % 90.0
            into.append(min(e, 90.0 - e))
    assert np.mean(refined) < np.mean(raw), (np.mean(raw), np.mean(refined))
    # Good enough to measure a wall grid, not to certify a 1-2 deg shear absent.
    # Measured 0.14 deg mean / 0.26 deg worst; pinned with margin.
    assert np.mean(refined) < 0.35, np.mean(refined)
    assert max(refined) < 0.8, max(refined)


def test_windowing_does_not_pin_the_fold_to_zero():
    """The failure this consolidation exists to prevent, pinned as a regression.

    A hand-rolled fold once read 0 deg on a rotated map because the rectangular
    aperture's axis-aligned leakage dominated. This implementation does not do
    that with *or* without the taper -- ``_angular_energy`` drops the DC
    neighbourhood and works on magnitude, not power -- and this test is what
    would notice if that ever stopped being true.
    """
    for deg in (17.0, 31.0):
        truth = (-deg) % 90.0
        for window in (False, True):
            r = wall_rotation(_room_outline(deg), window=window)
            err = abs(r["angle_deg"] - truth) % 90.0
            assert min(err, 90.0 - err) < 1.5, (deg, window, r)
            assert r["windowed"] is window


def test_fold_90_and_refine_peak_are_usable_standalone():
    """The alignment step imports the pieces, not just the wrapper."""
    angles = np.arange(360) * 0.5 + 0.25
    values = np.zeros(360)
    values[40] = values[40 + 180] = 1.0  # one frame: two families 90 deg apart
    fa, fv = fold_90(angles, values)
    assert len(fa) == 180
    assert fv[40] == 2.0
    values[39], values[41] = 0.5, 0.9
    fa, fv = fold_90(angles, values)
    refined = refine_peak(fa, fv, int(np.argmax(fv)))
    assert fa[40] < refined < fa[41], (fa[40], refined, fa[41])


def test_wall_rotation_is_invariant_to_incidental_map_extent():
    """Padding the canvas must not move the measured rotation."""
    tight = _room_outline(17.0)
    big = np.zeros((700, 820), bool)
    big[100 : 100 + tight.shape[0], 220 : 220 + tight.shape[1]] = tight
    a, b = wall_rotation(tight), wall_rotation(big)
    assert abs(a["angle_deg"] - b["angle_deg"]) < 1e-6, (a, b)


def test_tapering_a_tight_crop_would_be_worse_than_not_tapering():
    """Why wall_rotation pads before it tapers.

    Pinned because it is counter-intuitive: adding a window to a tight crop
    makes the measurement *worse* than leaving it off, so a future simplification
    that drops the padding would quietly halve the accuracy.
    """
    from mote_bringup.map_cleanup import angular_stats as mod

    angles = (1.5, 7.25, 12.0, 23.5, 31.0, 44.0)

    def mean_err(pad):
        saved, mod.ROTATION_PAD_FRAC = mod.ROTATION_PAD_FRAC, pad
        try:
            errs = []
            for d in angles:
                truth = (-d) % 90.0
                e = abs(wall_rotation(_room_outline(d))["angle_deg"] - truth) % 90.0
                errs.append(min(e, 90.0 - e))
            return float(np.mean(errs))
        finally:
            mod.ROTATION_PAD_FRAC = saved

    assert mean_err(0.0) > 2 * mean_err(mod.ROTATION_PAD_FRAC), (
        mean_err(0.0),
        mean_err(mod.ROTATION_PAD_FRAC),
    )


def test_wall_rotation_degrades_on_a_structureless_mask():
    r = wall_rotation(np.zeros((40, 40), bool))
    assert r["angle_deg"] is None and "note" in r


# --------------------------------------------------------------------------
# Contract and edges


def test_angles_reported_are_pixel_frame_and_bounded():
    s = angular_stats(pure(23.0))
    for d in s["directions"]:
        assert 0.0 <= d["angle_deg"] < 180.0
        assert d["width_deg"] >= 0.0
    for f in s["frames"]:
        assert 0.0 <= f["angle_deg"] < 90.0
        assert 0.0 <= f["offset_from_dominant_deg"] <= 45.0
    assert abs(sum(d["energy_frac"] for d in s["directions"]) - 1.0) < 1.0


def test_score_is_invariant_to_incidental_map_extent():
    """Padding the grid with never-observed space must not change the score."""
    m = pure()
    padded = np.zeros((N + 260, N + 190), bool)
    padded[130 : 130 + N, 90 : 90 + N] = m
    a, b = angular_stats(m), angular_stats(padded)
    assert abs(a["angular_support_deg"] - b["angular_support_deg"]) < 1e-6
    assert abs(a["unassigned_energy_frac"] - b["unassigned_energy_frac"]) < 1e-6


def test_declutter_params_duck_type_in():
    """The declutter pass passes its own Params straight through."""
    from mote_bringup.map_cleanup.structure_extraction import Params

    s = angular_stats(pure(), Params())
    assert s["n_peaks"] == 2


def test_empty_and_degenerate_masks_do_not_raise():
    for m in (
        np.zeros((40, 40), bool),
        np.zeros((0, 0), bool),
        np.ones((2, 2), bool),
    ):
        s = angular_stats(m)
        assert s["n_peaks"] == 0
        assert "note" in s


def test_defaults_match_the_declutter_params():
    """SpectrumParams carries its own copy of the shared defaults; they must agree."""
    from mote_bringup.map_cleanup.structure_extraction import Params

    full, spec = Params(), SpectrumParams()
    for f in vars(spec):
        assert getattr(full, f) == getattr(spec, f), f


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
