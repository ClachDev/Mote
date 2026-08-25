"""The offline build's SLAM params differ from the robot's only where declared.

`slam_toolbox_build_params.yaml` is a whole copy of `slam_toolbox_params.yaml`
with a handful of values changed, because slam_toolbox loads one file and there
is no overlay seam to load a patch through. A copy drifts: a value tuned on the
robot lands in one file and is never carried to the other, and the build goes on
solving under a setting nobody chose. Nothing fails when that happens — the
build still produces a map, just not the map the tuning was meant to produce.

So the copy is held here. Every key must hold the same value in both files
unless the build file marks it `# DIVERGENCE:` and says why, and the check runs
in both directions: an undeclared divergence fails, and so does a declared one
that no longer diverges, which is what turns the live file catching up (task
335, `loop_match_minimum_chain_size`) into a failing test rather than a stale
comment. Comments are deliberately not compared — the two files explain the same
value to different readers.
"""

import pathlib
import re

import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]
LIVE = REPO / "mote_bringup" / "config" / "slam_toolbox_params.yaml"
BUILD = REPO / "mote_bringup" / "config" / "slam_toolbox_build_params.yaml"

KEY_RE = re.compile(r"^ {4}(\w+):")
COMMENT_RE = re.compile(r"^ {4}# ?(.*)$")
MARKER = "DIVERGENCE:"


def _params(path):
    doc = yaml.safe_load(path.read_text())
    return doc["slam_toolbox"]["ros__parameters"]


def _declared_divergences(path):
    """{key: why} for every key marked `# DIVERGENCE:` in its own comment block.

    A marker applies to the next parameter key below it, and only while the
    comment block runs unbroken down to that key — a blank line or a value in
    between ends the block, so a note can never drift onto a key it was not
    written for.
    """
    found, note = {}, []
    for line in path.read_text().splitlines():
        comment = COMMENT_RE.match(line)
        if comment:
            text = comment.group(1).strip()
            if text.startswith(MARKER):
                note = [text[len(MARKER) :].strip()]
            elif note:
                note.append(text)
            continue
        key = KEY_RE.match(line)
        if key and note:
            found[key.group(1)] = " ".join(p for p in note if p).strip()
        note = []
    return found


def _orphan_markers(path):
    """Marker lines that no key ever consumed — a note doing nothing."""
    orphans, pending = [], None
    for n, line in enumerate(path.read_text().splitlines(), 1):
        comment = COMMENT_RE.match(line)
        if comment:
            text = comment.group(1).strip()
            if text.startswith(MARKER):
                if pending:
                    orphans.append(pending)
                pending = n
            continue
        if KEY_RE.match(line):
            pending = None
        elif pending:
            orphans.append(pending)
            pending = None
    return orphans + ([pending] if pending else [])


def test_both_files_declare_the_same_keys():
    live, build = _params(LIVE), _params(BUILD)
    assert set(build) == set(live), (
        "the build params are a copy of the live ones, so the key sets must "
        f"match. Only in live: {sorted(set(live) - set(build))}. Only in "
        f"build: {sorted(set(build) - set(live))}. Add the key to both files, "
        "and mark it `# DIVERGENCE:` in the build file if the values differ."
    )


def test_undeclared_keys_hold_the_live_value():
    live, build = _params(LIVE), _params(BUILD)
    declared = _declared_divergences(BUILD)
    drifted = {
        key: (live[key], build[key])
        for key in sorted(set(live) & set(build))
        if key not in declared and live[key] != build[key]
    }
    assert not drifted, (
        "these keys differ between the live and build SLAM params with no "
        f"declared reason: {drifted}. Either carry the live value across, or "
        "add a `# DIVERGENCE: <why>` comment directly above the key in "
        f"{BUILD.name} saying what the build buys by differing."
    )


def test_declared_divergences_actually_diverge():
    live, build = _params(LIVE), _params(BUILD)
    declared = _declared_divergences(BUILD)
    converged = {
        key: live[key]
        for key in declared
        if key in live and live[key] == build.get(key)
    }
    assert not converged, (
        f"{BUILD.name} marks these keys as divergent, but the live file now "
        f"holds the same value: {converged}. The note is stale — delete the "
        "`# DIVERGENCE:` comment (the key itself stays)."
    )


@pytest.mark.parametrize("key", sorted(_declared_divergences(BUILD)))
def test_every_divergence_says_why(key):
    why = _declared_divergences(BUILD)[key]
    assert len(why) > 20, (
        f"the `# DIVERGENCE:` note on {key} is empty or near-empty ({why!r}). "
        "A divergence with no stated reason cannot be reviewed or retired."
    )


def test_no_marker_applies_to_nothing():
    orphans = _orphan_markers(BUILD)
    assert not orphans, (
        f"{BUILD.name} has `# DIVERGENCE:` markers at line(s) {orphans} that "
        "no parameter key follows, so they declare nothing and the key they "
        "were meant for is being checked against the live value. A marker must "
        "sit in the comment block immediately above its key."
    )
