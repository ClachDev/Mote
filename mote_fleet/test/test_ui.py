"""Run the dashboard's JavaScript tests from the same `pytest` invocation.

The UI's MQTT codec and its world→pixel transform are the two pieces where a
mistake is silent — a dropped packet type looks like "the robot is quiet", a
sign error looks like "the robot is over there" — so they are tested, in node,
against the very files the browser loads (``ui_test.mjs``).

Skipped where there is no node, exactly as the broker end-to-end test skips
where there is no mosquitto: the Pi and CI both have one of the two.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

TEST_DIR = Path(__file__).resolve().parent
UI_TEST = TEST_DIR / "ui_test.mjs"


@pytest.mark.skipif(shutil.which("node") is None, reason="no node to run the UI tests")
def test_the_dashboard_javascript_passes():
    result = subprocess.run(
        ["node", "--test", str(UI_TEST)],
        capture_output=True,
        text=True,
        timeout=120,
        cwd=TEST_DIR,
    )
    assert result.returncode == 0, result.stdout + result.stderr
