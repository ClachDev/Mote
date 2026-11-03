"""The systemd units in `mote_bringup/systemd/` agree with each other.

Every rule here fails silently on the robot: a pull edge back to bringup
bypasses its restart backoff, a target member without `PartOf=` survives
`pixi run sleep`, and a shared member `PartOf=` a mission target is stopped by
the `Conflicts=` between the targets on every switch, leaving an active target
with nothing running. `mote_bringup/README.md` "systemd services" has the
layout and the measurements behind it.
"""

import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
SYSTEMD = REPO / "mote_bringup" / "systemd"
MISSION_TARGETS = ("mote-robot.target", "mote-mapping.target")
STANDBY = ("mote-agent.service", "mote-foxglove.service")


def parse(path):
    """Unit file → {section: {key: [values]}}; repeated keys accumulate."""
    sections, current = {}, None
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith(("#", ";")):
            continue
        if line.startswith("[") and line.endswith("]"):
            current = sections.setdefault(line[1:-1], {})
            continue
        key, _, value = line.partition("=")
        current.setdefault(key.strip(), []).append(value.strip())
    return sections


def unit_names(section, key):
    return [name for value in section.get(key, []) for name in value.split()]


def load(name):
    """Parse a unit, resolving `name@instance` to its template with %i filled in."""
    template = re.sub(r"@[^.]+\.", "@.", name)
    path = SYSTEMD / (name if (SYSTEMD / name).exists() else template)
    sections = parse(path)
    instance = name.split("@", 1)[1].rsplit(".", 1)[0] if "@" in name else None
    if instance is not None:
        for section in sections.values():
            for key, values in section.items():
                section[key] = [v.replace("%i", instance) for v in values]
    return sections


UNIT_FILES = sorted(p.name for p in SYSTEMD.glob("mote-*.service"))
TARGET_FILES = sorted(p.name for p in SYSTEMD.glob("mote-*.target"))


def members(target):
    """Units a target starts, expanding mote-base.target."""
    out = []
    for name in unit_names(load(target)["Unit"], "Wants"):
        out += members(name) if name.endswith(".target") else [name]
    return out


def test_targets_exist():
    assert set(TARGET_FILES) == {"mote-base.target", *MISSION_TARGETS}


def test_mission_targets_conflict():
    robot, mapping = (load(t)["Unit"] for t in MISSION_TARGETS)
    assert "mote-mapping.target" in unit_names(robot, "Conflicts")
    assert "mote-robot.target" in unit_names(mapping, "Conflicts")


@pytest.mark.parametrize("target", MISSION_TARGETS)
def test_every_member_stops_with_its_target(target):
    """`systemctl stop <target>` must reach every unit it started."""
    direct = unit_names(load(target)["Unit"], "Wants")
    assert "mote-base.target" in direct
    for name in direct:
        part_of = unit_names(load(name)["Unit"], "PartOf")
        if name == "mote-base.target":
            assert load(name)["Unit"]["StopWhenUnneeded"] == ["yes"]
            continue
        assert target in part_of, f"{name} is started by {target} but not PartOf= it"
    for name in members("mote-base.target"):
        assert "mote-base.target" in unit_names(load(name)["Unit"], "PartOf"), name


def test_shared_members_are_not_part_of_a_mission_target():
    """A PartOf= on a mission target stops the member on every target switch."""
    for name in members("mote-base.target"):
        part_of = unit_names(load(name)["Unit"], "PartOf")
        assert not set(part_of) & set(MISSION_TARGETS), name


def test_target_membership():
    assert sorted(members("mote-robot.target")) == sorted(
        [
            "mote-bringup.service",
            "mote-health.service",
            "mote-nav@robot.service",
            "mote-tasks.service",
        ]
    )
    assert sorted(members("mote-mapping.target")) == sorted(
        [
            "mote-bringup.service",
            "mote-health.service",
            "mote-slam.service",
            "mote-nav@mapping.service",
            "mote-record.service",
        ]
    )


@pytest.mark.parametrize(
    "instance,localisation", [("robot", "true"), ("mapping", "false")]
)
def test_nav_instance_argument(instance, localisation):
    unit = load(f"mote-nav@{instance}.service")
    assert unit["Service"]["ExecStart"][0].endswith(
        "pixi run nav localisation:=${NAV_LOCALISATION}"
    )
    env_file = unit["Service"]["EnvironmentFile"][0].replace("@REPO@", str(REPO))
    assert (
        pathlib.Path(env_file).read_text().splitlines()[-1]
        == f"NAV_LOCALISATION={localisation}"
    )


def test_tasks_bound_to_robot_nav():
    unit = load("mote-tasks.service")["Unit"]
    for key in ("After", "BindsTo", "PartOf"):
        assert "mote-nav@robot.service" in unit_names(unit, key), key


@pytest.mark.parametrize("name", UNIT_FILES)
def test_nothing_pulls_bringup(name):
    """A pull edge from a restart-looping dependent bypasses bringup's backoff."""
    unit = load(name)["Unit"]
    for key in ("Wants", "Requires", "BindsTo", "Requisite", "Upholds"):
        assert "mote-bringup.service" not in unit_names(unit, key), f"{name} {key}="


@pytest.mark.parametrize("name", UNIT_FILES)
def test_only_standby_units_start_at_boot(name):
    assert ("Install" in load(name)) == (name in STANDBY), name


@pytest.mark.parametrize("name", UNIT_FILES)
def test_hardening_and_dds_block(name):
    unit = load(name)
    service = unit["Service"]
    assert unit["Unit"]["StartLimitIntervalSec"] == ["0"]
    assert service["Restart"] == ["always"]
    assert service["RestartSec"] == ["2"]
    assert service["RestartSteps"] == ["5"]
    assert service["RestartMaxDelaySec"] == ["30"]
    env = service["Environment"]
    assert "CYCLONEDDS_URI=file://@REPO@/mote_bringup/config/cyclonedds.xml" in env
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in env


def test_bringup_stops_with_sigint_to_pixi():
    """SIGTERM to the cgroup aborts sllidar_node before its stop-motor path and
    the C1 scans on with no service running; SIGINT to the whole group reaches
    launch twice and can hang it after its children exit."""
    service = load("mote-bringup.service")["Service"]
    assert service["KillMode"] == ["mixed"]
    assert service["KillSignal"] == ["SIGINT"]


def test_health_keeps_its_watchdog():
    service = load("mote-health.service")["Service"]
    assert service["Type"] == ["notify"]
    assert service["WatchdogSec"]


def test_install_sh_installs_targets_and_polkit_rule():
    script = (SYSTEMD / "install.sh").read_text()
    assert '"$SRC_DIR"/*.target' in script
    assert "/etc/polkit-1/rules.d/50-mote.rules" in script
    assert "enable --now mote-foxglove.service" in script
    assert "enable --now mote-agent.service" in script
    assert not re.search(
        r"^\s*sudo systemctl enable\b.*mote-(robot|mapping)\.target", script, re.M
    )
