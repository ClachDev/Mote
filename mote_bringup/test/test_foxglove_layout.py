"""The shipped Foxglove layout is well-formed and agrees with the robot's config.

Foxglove is not importable here, so these tests cannot prove the layout renders.
What they do prove is the half that drifts silently: that the layout is
internally consistent, and that the teleop numbers still match the controller
they drive. Change `cmd_vel_timeout` or a velocity limit in controllers.yaml and
the shipped layout stops being safe to hand an operator — that is the failure
these tests are here to catch.
"""

import json
import pathlib

import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]
LAYOUT = REPO / "mote_bringup" / "foxglove" / "mote.json"
CONTROLLERS = REPO / "mote_bringup" / "config" / "controllers.yaml"


@pytest.fixture(scope="module")
def layout():
    return json.loads(LAYOUT.read_text())


@pytest.fixture(scope="module")
def diff_drive():
    cfg = yaml.safe_load(CONTROLLERS.read_text())
    return cfg["diff_drive_controller"]["ros__parameters"]


def _panel_ids(node):
    """Every panel id in the mosaic tree, which is a string or a split dict."""
    if isinstance(node, str):
        return {node}
    return _panel_ids(node["first"]) | _panel_ids(node["second"])


def test_every_placed_panel_is_configured(layout):
    assert _panel_ids(layout["layout"]) == set(layout["configById"])


def test_panel_ids_are_type_bang_suffix(layout):
    for panel_id in layout["configById"]:
        kind, sep, suffix = panel_id.partition("!")
        assert sep and kind and suffix, panel_id


def test_the_layout_has_the_panels_the_milestone_promises(layout):
    kinds = {p.split("!")[0] for p in layout["configById"]}
    assert {"3D", "Image", "Teleop"} <= kinds


def test_teleop_outruns_the_controller_deadman(layout, diff_drive):
    """Below 1/cmd_vel_timeout the robot halts between commands and stutters."""
    rate = layout["configById"]["Teleop!drive"]["publishRate"]
    assert rate > 1.0 / diff_drive["cmd_vel_timeout"]


def test_teleop_stays_inside_the_controller_limits(layout, diff_drive):
    teleop = layout["configById"]["Teleop!drive"]
    limits = {
        "linear-x": diff_drive["linear"]["x"]["max_velocity"],
        "angular-z": diff_drive["angular"]["z"]["max_velocity"],
    }
    for button in ("upButton", "downButton", "leftButton", "rightButton"):
        field, value = teleop[button]["field"], teleop[button]["value"]
        assert abs(value) <= limits[field], f"{button} exceeds {field} limit"


def test_teleop_publishes_to_the_relay_not_the_controller(layout):
    """The panel emits Twist; the controller needs TwistStamped via twist_relay."""
    assert layout["configById"]["Teleop!drive"]["topic"] == "/cmd_vel_teleop"


def test_the_camera_panel_uses_the_compressed_stream(layout):
    image = layout["configById"]["Image!camera"]["imageMode"]["imageTopic"]
    assert image.endswith("/compressed")


def test_the_3d_panel_follows_the_base_frame(layout, diff_drive):
    assert layout["configById"]["3D!main"]["followTf"] == diff_drive["base_frame_id"]
