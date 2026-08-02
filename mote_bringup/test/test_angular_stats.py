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
