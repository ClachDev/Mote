"""The costmap layers Nav2 is configured with, checked against the real index.

Two failure modes here are silent. A `plugin:` string naming a class no
installed package registers leaves one line in the container log while the rest
of the stack comes up around it and navigates with a layer missing — the same
family as `test_nav2_composition.py`'s. And the local `camera_layer` is a
`spatio_temporal_voxel_layer`, chosen because it expires marks on a clock; two
of its settings quietly switch that off, and one of them turns the layer back
into the permanent-mark bug it was brought in to fix. Both are asserted against
the shipped `nav2_params.yaml`.

`mote_bringup/tools/camera_layer_decay.py` is the live counterpart: it runs a
real costmap and times an actual mark. This file is the part that is cheap
enough to run on every build.
"""

import pathlib
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, get_resources

REPO = pathlib.Path(__file__).resolve().parents[2]
NAV2_PARAMS = REPO / "mote_bringup" / "config" / "nav2_params.yaml"

LAYER_PLUGIN_RESOURCE = "nav2_costmap_2d__pluginlib__plugin"
COSTMAPS = ("local_costmap", "global_costmap")


@pytest.fixture(scope="module")
def params():
    return yaml.safe_load(NAV2_PARAMS.read_text())


def costmap_layers(params, costmap):
    """The `{layer name: config}` of one costmap, in its configured order."""
    section = params[costmap][costmap]["ros__parameters"]
    return {name: section[name] for name in section["plugins"]}


def registered_layer_classes():
    """Every `nav2_costmap_2d::Layer` lookup name any installed package exports."""
    classes = set()
    for package in get_resources(LAYER_PLUGIN_RESOURCE):
        relative, _ = get_resource(LAYER_PLUGIN_RESOURCE, package)
        share = pathlib.Path(get_package_share_directory(package)).parent.parent
        for description in relative.split():
            root = ET.parse(share / description).getroot()
            # the file is a bare <library> or a <class_libraries> wrapping them
            libraries = [root] if root.tag == "library" else root.findall("library")
            for entry in (c for lib in libraries for c in lib.findall("class")):
                if entry.get("base_class_type") != "nav2_costmap_2d::Layer":
                    continue
                # pluginlib falls back to the C++ type when no lookup name is
                # given, which is how every nav2_costmap_2d layer is declared
                classes.add(entry.get("name") or entry.get("type"))
    return classes


@pytest.mark.parametrize("costmap", COSTMAPS)
def test_every_layer_plugin_is_installed(params, costmap):
    """A layer whose class no package exports is skipped, not fatal.

    `spatio_temporal_voxel_layer` in particular is not part of the nav2
    metapackage set, so it only reaches the robot if `pixi.toml` asks for it by
    name — and forgetting that costs the camera layer with nav2 still healthy.
    """
    registered = registered_layer_classes()
    for name, layer in costmap_layers(params, costmap).items():
        assert layer["plugin"] in registered, (
            f"{costmap}/{name} is configured as {layer['plugin']}, "
            "which no installed package registers as a costmap layer"
        )


@pytest.fixture(scope="module")
def camera_layer(params):
    return costmap_layers(params, "local_costmap")["camera_layer"]


def test_camera_layer_marks_expire(camera_layer):
    """The whole reason this layer is not a plain VoxelLayer.

    The cloud carries only above-floor points, so a departed obstacle is never
    raytraced away — measured as a permanent mark. A decay model of -1 is
    `PERSISTENT`, which would reinstate exactly that.
    """
    assert camera_layer["decay_model"] in (0, 1)
    assert camera_layer["voxel_decay"] > 0


def test_camera_layer_source_is_emptied_after_reading(camera_layer):
    """`clear_after_reading` is load-bearing, not tidiness.

    The measurement buffer holds its newest cloud until something empties it,
    and every costmap update re-marks whatever it reads — marking stamps each
    voxel with the current time, restarting its decay. Without this the cloud
    of a departed obstacle is refreshed at the costmap rate forever and
    `voxel_decay` never fires: measured as indistinguishable from the
    VoxelLayer this replaced.
    """
    assert camera_layer["camera"]["clear_after_reading"] is True


def test_camera_layer_go_under_gate_is_applied(camera_layer):
    """STVL runs the height bounds as its filter's z limits.

    So `filter: "none"` does not merely skip downsampling — it drops the
    go-under gate, and the robot starts marking tabletops it drives under.
    """
    source = camera_layer["camera"]
    assert source.get("filter", "passthrough") != "none"
    assert source["min_obstacle_height"] < source["max_obstacle_height"]


def test_camera_layer_cannot_erase_a_lidar_mark(params):
    """The camera is additive over the lidar, never authoritative against it.

    `combination_method` 1 is Maximum: free space in this layer cannot lower a
    cell the lidar's `obstacle_layer` marked. 0 (Overwrite) would let a
    monocular-depth miss clear a real lidar return.
    """
    layers = costmap_layers(params, "local_costmap")
    assert layers["camera_layer"]["combination_method"] == 1
    assert list(layers).index("obstacle_layer") < list(layers).index("camera_layer")
