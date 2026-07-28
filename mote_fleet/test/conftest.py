"""Make both halves of the package importable, and share the API fixtures.

``mote_fleet/`` holds the robot-side python package; ``mote_fleet/server/``
holds the off-board server, which is a set of scripts rather than a package
because the fleet box runs it without installing anything (the same shape as
``mote_perception/tools/``). Tests exercise both, so both are on the path.

The fixtures below are here rather than in one test file because two of them
run against the same live server: the API tests and the map-registry tests.
The scaffolding itself is ``api_harness.py``.
"""

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

for entry in (PACKAGE_ROOT, PACKAGE_ROOT / "server"):
    if str(entry) not in sys.path:
        sys.path.insert(0, str(entry))

import pytest  # noqa: E402

import api_harness  # noqa: E402


@pytest.fixture
def server(tmp_path):
    """A live fleet server with one published floor and a stub broker."""
    api_harness.write_bundle(tmp_path / "sites", "home", "ground")
    httpd = api_harness.start_server(tmp_path)
    yield httpd
    httpd.shutdown()
    httpd.server_close()


@pytest.fixture
def operator(server):
    return server.registry.new_operator(name="michael")


@pytest.fixture
def robot(server):
    api_harness.enroll(server, "serial:aaa", name="Scout")
    return "mote-01"
