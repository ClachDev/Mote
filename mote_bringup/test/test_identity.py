"""Identity + per-robot-state tests (no ROS graph needed)."""

import pytest
import yaml

from mote_bringup import identity, mote_home


@pytest.fixture(autouse=True)
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    return tmp_path


def test_mote_home_follows_the_env(home):
    assert mote_home.mote_dir() == home
    assert mote_home.path("robot.yaml") == home / "robot.yaml"


def test_override_prefers_the_per_robot_file(home):
    packaged = "/opt/share/perception.yaml"
    assert mote_home.override("perception.yaml", packaged) == packaged
    (home / "perception.yaml").write_text("inference_host: gpu-box\n")
    assert mote_home.override("perception.yaml", packaged) == str(
        home / "perception.yaml"
    )


def test_unset_identity_reads_as_none():
    assert identity.load() is None
    assert identity.robot_id() is None


def test_set_then_read_back(home):
    identity.set_identity(id="mote-01", name="Scout", site="hq")
    assert identity.robot_id() == "mote-01"
    record = yaml.safe_load((home / "robot.yaml").read_text())
    assert record == {
        "schema": 1,
        "id": "mote-01",
        "name": "Scout",
        "site": "hq",
    }


def test_identity_survives_a_reboot(home):
    """The record is a plain file under MOTE_HOME, so a fresh process — a
    reboot, or the stack restarting — reads back the same id."""
    identity.set_identity(id="mote-01", name="Scout")
    assert identity.load() == identity.load()
    assert identity.robot_id() == "mote-01"


def test_partial_update_keeps_other_fields():
    identity.set_identity(id="mote-01", name="Scout", site="hq")
    identity.set_identity(name="Rover")
    assert identity.load() == {
        "schema": 1,
        "id": "mote-01",
        "name": "Rover",
        "site": "hq",
    }


def test_name_defaults_to_the_id():
    assert identity.set_identity(id="mote-01")["name"] == "mote-01"


@pytest.mark.parametrize(
    "bad",
    [
        "Mote-01",
        "mote_01",
        "mote 01",
        "mote/01",
        "mote+01",
        "-mote",
        "mote-",
        "",
        "x" * 33,
    ],
)
def test_ids_that_would_break_dns_mqtt_or_paths_are_rejected(bad):
    with pytest.raises(ValueError):
        identity.set_identity(id=bad)


@pytest.mark.parametrize("good", ["mote-01", "m", "robot2", "a" * 32])
def test_valid_ids(good):
    assert identity.set_identity(id=good)["id"] == good


def test_an_id_is_required():
    with pytest.raises(ValueError):
        identity.set_identity(name="nameless")
