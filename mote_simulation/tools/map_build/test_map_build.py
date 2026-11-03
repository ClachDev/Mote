#!/usr/bin/env python3
"""Unit tests for the map-build orchestrator's ROS-free half.

    python mote_simulation/tools/map_build/test_map_build.py

What is covered is what a wrong answer would be *silent* about: the pixel
convention a revision's map is written in (get it inverted and the map still
renders, mirrored, with every wall where free space was), the origin that lands
in ``map.yaml``, the baseline reader that has to invert the same convention,
and the metric diff's direction — where "lower is better" is data and reading
it the wrong way round would print `better` over a regression.

The solve itself needs slam_toolbox and a real bag, and is exercised by
``pixi run map-build``, not from here.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "mote_bringup"))
sys.path.insert(0, str(HERE))

import build_report  # noqa: E402
import revision as rev  # noqa: E402


def sample_grid() -> np.ndarray:
    """A 4x6 grid with one of each cell class, and an asymmetric top and bottom
    so a vertical flip cannot pass unnoticed."""
    grid = np.full((4, 6), -1, dtype=np.int16)
    grid[0, :] = 0  # free along the bottom row of the world
    grid[3, :] = 100  # occupied along the top
    grid[1, 2] = 100
    return grid


class MapPixels(unittest.TestCase):
    def test_grid_to_png_uses_map_saver_values(self):
        image = rev.grid_to_png_array(sample_grid())
        # Row 0 of the image is the *top* of the world, i.e. grid row 3.
        self.assertTrue((image[0] == rev.OCCUPIED_PX).all())
        self.assertTrue((image[-1] == rev.FREE_PX).all())
        self.assertEqual(image[2, 2], rev.OCCUPIED_PX)
        self.assertEqual(image[1, 0], rev.UNKNOWN_PX)

    def test_png_to_grid_inverts_it(self):
        grid = sample_grid()
        back = rev.png_to_grid(rev.grid_to_png_array(grid))
        # The trinary round trip is exact on class, not on value: a cell at 100
        # comes back as 100, one at 0 as 0, unknown as -1.
        self.assertTrue(((back >= 0) == (grid >= 0)).all())
        self.assertTrue(((back == 100) == (grid == 100)).all())
        self.assertTrue(((back == 0) == (grid == 0)).all())

    def test_the_declared_thresholds_read_the_written_shades_back(self):
        """The bug this exists to stop: unknown space read back as free, i.e. as
        somewhere the planner may drive straight through."""
        for shade, expected in (
            (rev.FREE_PX, 0),
            (rev.UNKNOWN_PX, -1),
            (rev.OCCUPIED_PX, 100),
        ):
            back = rev.png_to_grid(np.full((1, 1), shade, dtype=np.uint8))
            self.assertEqual(int(back[0, 0]), expected, f"shade {shade}")

    def test_grid_classes_split_where_every_other_reader_splits(self):
        grid = np.array([[rev.GRID_FREE_MAX, rev.GRID_OCC_MIN, 50]])
        image = rev.grid_to_png_array(grid)
        self.assertEqual(list(image[0]), [rev.FREE_PX, rev.OCCUPIED_PX, rev.UNKNOWN_PX])


class MapPair(unittest.TestCase):
    def write(self, tmp: Path, **extra):
        npz = tmp / "map.npz"
        np.savez_compressed(
            npz,
            grid=sample_grid(),
            resolution=np.float64(0.05),
            origin=np.array([-1.25, -2.5]),
            **extra,
        )
        return rev.write_map_pair(npz, tmp / "revision")

    def test_map_yaml_carries_the_frame(self):
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            frame = self.write(tmp, origin_yaw=np.float64(0.25))
            text = (tmp / "revision" / "map.yaml").read_text()
            self.assertIn("image: map.png", text)
            self.assertIn("resolution: 0.050", text)
            self.assertIn("origin: [-1.250, -2.500, 0.250000]", text)
            self.assertEqual(frame["width"], 6)
            self.assertEqual(frame["height"], 4)
            self.assertTrue(frame["origin_yaw_recorded"])

    def test_a_missing_origin_yaw_is_zero_and_says_so(self):
        """Harness output from before the yaw was recorded must not read as a
        measured zero: the report says which it was."""
        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            frame = self.write(tmp)
            self.assertIn(
                "origin: [-1.250, -2.500, 0.000000]",
                (tmp / "revision" / "map.yaml").read_text(),
            )
            self.assertFalse(frame["origin_yaw_recorded"])

    def test_the_pair_passes_the_bundle_validator(self):
        """The whole point of writing this layout: the registry accepts it."""
        from mote_bringup import bundle

        with tempfile.TemporaryDirectory() as raw:
            tmp = Path(raw)
            self.write(tmp, origin_yaw=np.float64(0.0))
            rev_dir = tmp / "revision"
            for name in bundle.CONTINUABLE:
                (rev_dir / name).write_bytes(b"posegraph bytes")
            rev.write_meta(rev_dir, {"schema": 1, "saved": "2026-09-01T00:00:00"})
            report = bundle.validate(rev_dir)
            self.assertEqual(report.errors, [])


class ServedMap(unittest.TestCase):
    """The candidate is scored from the map it will publish, not from the solve.

    A revision keeps two images — the raw solve and the decluttered one it
    serves — and only the second is ever published. Reading the wrong one puts
    the raw map's speckle beside a baseline's cleaned figure and prints a
    regression that is not there.
    """

    def test_revision_metrics_reads_the_image_map_yaml_names(self):
        import cv2

        sys.path.insert(0, str(REPO / "mote_simulation" / "tools" / "map_build"))
        sys.path.insert(0, str(REPO / "mote_simulation" / "tools" / "benchmark"))
        import map_build

        with tempfile.TemporaryDirectory() as raw:
            rev_dir = Path(raw) / "20260901T120000"
            npz = Path(raw) / "map.npz"
            np.savez_compressed(
                npz,
                grid=sample_grid(),
                resolution=np.float64(0.05),
                origin=np.array([0.0, 0.0]),
                origin_yaw=np.float64(0.0),
            )
            rev.write_map_pair(npz, rev_dir)
            served = rev_dir / "map.png"
            # The raw image is deliberately the opposite of the served one: a
            # reader that took map_raw.png would report every cell inverted.
            cv2.imwrite(
                str(rev_dir / "map_raw.png"),
                255 - cv2.imread(str(served), cv2.IMREAD_GRAYSCALE),
            )
            scored = map_build.revision_metrics(rev_dir)
            self.assertAlmostEqual(scored["occ_frac"], 7 / 24)


class BagIdentity(unittest.TestCase):
    def test_digest_covers_content_and_names(self):
        with tempfile.TemporaryDirectory() as raw:
            bag = Path(raw) / "20260802_142539"
            bag.mkdir()
            (bag / "a_0.mcap").write_bytes(b"one")
            (bag / "metadata.yaml").write_text("version: 9\n")
            first = rev.digest_bag(bag)
            self.assertEqual(first["name"], "20260802_142539")
            self.assertEqual(
                [f["name"] for f in first["files"]], ["a_0.mcap", "metadata.yaml"]
            )

            # Same bytes, different file name: a different bag.
            (bag / "a_0.mcap").rename(bag / "b_0.mcap")
            self.assertNotEqual(rev.digest_bag(bag)["sha256"], first["sha256"])


class Diff(unittest.TestCase):
    def test_direction_decides_better_from_worse(self):
        candidate = {"map": {"speckle_frac": 0.10, "explored_area_m2": 50.0}}
        baseline = {"map": {"speckle_frac": 0.20, "explored_area_m2": 80.0}}
        rows = {row["metric"]: row for row in build_report.compare(candidate, baseline)}
        # Less speckle is better; less explored area is worse.
        self.assertEqual(rows["map.speckle_frac"]["verdict"], "better")
        self.assertEqual(rows["map.explored_area_m2"]["verdict"], "worse")

    def test_a_change_inside_the_deadband_is_the_same(self):
        candidate = {"map": {"speckle_frac": 0.2 * (1 + build_report.DEADBAND / 2)}}
        rows = {
            row["metric"]: row
            for row in build_report.compare(candidate, {"map": {"speckle_frac": 0.2}})
        }
        self.assertEqual(rows["map.speckle_frac"]["verdict"], "same")

    def test_no_baseline_reports_the_candidate_and_no_verdict(self):
        rows = build_report.compare({"map": {"speckle_frac": 0.1}}, None)
        self.assertEqual(len(rows), 1)
        self.assertNotIn("verdict", rows[0])
        self.assertNotIn("baseline", rows[0])

    def test_canvas_dependent_metrics_are_not_diffed(self):
        """Two metrics ``map_quality`` reports must not appear as ranked rows.

        ``angular_support_deg`` is confounded by coverage; ``unknown_frac`` is a
        fraction of the grid, so a candidate whose bounding box is a few pixels
        wider reads worse on it while covering more floor. Either one printed
        beside a `worse` invites a reviewer to reject a map for a reason that is
        not about the map.
        """
        diffed = [metric for metric, _, _ in build_report.DIFFED]
        self.assertNotIn("map.angular_support_deg", diffed)
        self.assertNotIn("map.unknown_frac", diffed)


class Report(unittest.TestCase):
    def minimal(self) -> dict:
        return {
            "revision": "20260901T120000",
            "built": "20260901T120000Z",
            "verdict": "candidate emitted",
            "verdict_detail": "somewhere",
            "inputs": {
                "bag": {"path": "/bags/x", "sha256": "abc", "files": [{"bytes": 3}]},
                "params": {"path": "/p.yaml", "sha256": "def"},
                "frame": None,
                "feed": "lockstep",
                "harness_commit": "cafe",
            },
            "stages": [{"name": "solve", "outcome": "ok", "detail": "186 nodes"}],
            "validation": {"summary": "valid", "errors": [], "warnings": []},
            "diff": build_report.compare({"map": {"speckle_frac": 0.1}}, None),
            "angular": {"frames": [], "n_peaks": 0},
            "zones": {"added": ["room_01"], "carry_forward": "nothing to carry"},
            "images": [],
            "next": "upload it",
        }

    def test_markdown_renders_without_a_baseline(self):
        text = build_report.build_markdown(self.minimal())
        self.assertIn("# Map build 20260901T120000", text)
        self.assertIn("No baseline given", text)
        self.assertIn("room_01", text)

    def test_markdown_names_the_alignment_gap(self):
        """A reviewer must not read 'no alignment step' as 'the map is square'."""
        text = build_report.build_markdown(self.minimal())
        self.assertIn("does **not** align the map frame", text)
        self.assertIn("615", text)

    def test_a_failed_build_still_renders(self):
        build = self.minimal()
        build["verdict"] = "build failed"
        build["validation"] = {"summary": "not reached", "errors": [], "warnings": []}
        build["diff"] = []
        self.assertIn("build failed", build_report.build_markdown(build))


if __name__ == "__main__":
    unittest.main(verbosity=2)
