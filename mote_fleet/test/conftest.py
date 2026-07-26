"""Make both halves of the package importable from the test session.

``mote_fleet/`` holds the robot-side python package; ``mote_fleet/server/``
holds the off-board server, which is a set of scripts rather than a package
because the fleet box runs it without installing anything (the same shape as
``mote_perception/tools/``). Tests exercise both, so both are on the path.
"""

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

for entry in (PACKAGE_ROOT, PACKAGE_ROOT / "server"):
    if str(entry) not in sys.path:
        sys.path.insert(0, str(entry))
