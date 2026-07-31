"""The sweep decides what to kill, so what it *declines* to kill is the test.

The rule it replaces is ``pkill -9 -f <names>``, whose failure modes are all of
the shape "matched something that was never ours": the sweeper's own shell,
another checkout's live run, a file manager with the pattern in its argv. Most
of these cases are a process table away from being unreproducible, so the
selection rule is a pure function of one and the fabricated tables below are the
real subject. One end-to-end case spawns an actual orphan and reaps it, to keep
the /proc reading honest.
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from mote_bringup import sweep_orphans as sweep

JOBS = Path("/home/u/.claude/jobs")
WORKTREES = Path("/home/u/checkout/.claude/worktrees")
ROOTS = [JOBS, WORKTREES]

ROS_ENV = {"AMENT_PREFIX_PATH": "/home/u/checkout/install/mote_bringup"}
OLD = sweep.DEFAULT_MIN_AGE * 2


def proc(pid, *, ppid=1, argv=None, env=None, exe=None, cwd=None, age=OLD, **kw):
    return sweep.Proc(
        pid=pid,
        ppid=ppid,
        starttime=pid * 100,
        age=age,
        argv=argv or ["/usr/bin/thing"],
        env=dict(ROS_ENV if env is None else env),
        exe=exe,
        cwd=cwd,
        **kw,
    )


def table(*procs):
    return {p.pid: p for p in procs}


# A pid that is not in any fabricated table, so the sweeper's own ancestry never
# collides with the processes under test.
SELF = 99999


def select(*procs, min_age=sweep.DEFAULT_MIN_AGE, self_pid=SELF):
    return sweep.select(table(*procs), ROOTS, min_age=min_age, self_pid=self_pid)


# --- what gets swept -------------------------------------------------------


def test_orphaned_node_from_a_dead_job_is_swept():
    node = proc(10, exe=str(JOBS / "abc/wt-x/install/mote_nav/lib/icp_odom_gate"))
    assert [p.pid for p in select(node)] == [10]


def test_a_chain_of_orphans_is_swept_whole():
    """The launch died with the job; its children are orphans of an orphan."""
    launch = proc(10, exe=str(JOBS / "abc/wt-x/.pixi/bin/ros2"))
    child = proc(11, ppid=10, exe=str(JOBS / "abc/wt-x/lib/foxglove_bridge"))
    assert [p.pid for p in select(launch, child)] == [10, 11]


def test_provenance_can_come_from_the_environment_alone():
    """A worktree run against the main checkout's env has a main-checkout exe.

    ``pixi run`` from a worktree uses the primary checkout's environment, so the
    interpreter and the binaries live outside the job directory entirely and
    only the overlay prefixes implicate it. Matching on the executable alone —
    as the obvious version of this sweep would — misses these.
    """
    node = proc(
        10,
        exe="/home/u/checkout/.pixi/envs/default/bin/python3.12",
        cwd="/home/u/checkout",
        argv=["python3.12", "-u", "-c", "pass"],
        env={"AMENT_PREFIX_PATH": str(JOBS / "abc/tmp/overlay/mote_bringup")},
    )
    assert [p.pid for p in select(node)] == [10]


def test_a_deleted_executable_still_implicates_its_job():
    """An unlinked exe reads back with a ' (deleted)' suffix glued on."""
    node = proc(10, exe=str(JOBS / "abc/wt-x/lib/mosquitto") + " (deleted)")
    assert [p.pid for p in select(node)] == [10]
    assert node.name == "mosquitto"


def test_worktrees_under_the_checkout_are_swept_too():
    node = proc(10, cwd=str(WORKTREES / "sim-sites"))
    assert [p.pid for p in select(node)] == [10]


# --- what is left alone ----------------------------------------------------


def test_a_run_still_owned_by_a_shell_is_left_alone():
    """Another agent's live run: its ancestry reaches a shell, not init."""
    shell = proc(10, argv=["/usr/bin/zsh"], exe="/usr/bin/zsh", env={})
    launch = proc(11, ppid=10, exe=str(JOBS / "abc/wt-x/.pixi/bin/ros2"))
    node = proc(12, ppid=11, exe=str(JOBS / "abc/wt-x/lib/twist_mux"))
    assert select(shell, launch, node) == []


def test_the_sweeper_never_matches_its_own_ancestry():
    """The ``pkill -f`` foot-gun: the sweep must not be able to kill itself.

    Both the sweeping process and everything that spawned it are excluded, so a
    sweep run from inside a worktree — the normal case — cannot reap the shell
    it was typed into, nor the job it belongs to.
    """
    shell = proc(10, exe=str(JOBS / "abc/wt-x/.pixi/bin/python3.12"))
    me = proc(11, ppid=10, exe=str(JOBS / "abc/wt-x/.pixi/bin/python3.12"))
    stranger = proc(12, exe=str(JOBS / "def/wt-y/lib/twist_mux"))
    assert [p.pid for p in select(shell, me, stranger, self_pid=11)] == [12]


def test_a_non_ros_process_holding_a_job_path_is_left_alone():
    """A file manager opened on a job directory has the path but not the graph."""
    files = proc(
        10,
        argv=["/usr/bin/cosmic-files", str(JOBS / "abc/tmp/")],
        exe="/usr/bin/cosmic-files",
        env={},
    )
    assert select(files) == []


def test_never_swept_programs_are_left_alone():
    """A `pixi shell` in a worktree gives a shell every marker a node has."""
    for name in ("zsh", "claude", "nvim", "git"):
        shell = proc(
            10,
            argv=[f"/usr/bin/{name}"],
            exe=f"/usr/bin/{name}",
            cwd=str(JOBS / "abc/wt-x"),
        )
        assert select(shell) == [], name


def test_a_young_process_is_left_alone():
    """A session-detached live run looks orphaned; only its age says otherwise.

    ``run_sim_smoke.sh`` ``setsid``s its launch, so a perfectly healthy smoke
    test has ppid 1 and no live non-candidate ancestor. Age is the only thing
    separating it from a leak, which is why the sweep has a floor.
    """
    node = proc(10, exe=str(JOBS / "abc/wt-x/lib/twist_mux"), age=20.0)
    assert select(node) == []
    assert [p.pid for p in select(node, min_age=0)] == [10]


def test_a_process_outside_every_agent_root_is_left_alone():
    """The user's own checkout is not agent leakage, however orphaned it is."""
    node = proc(
        10,
        exe="/home/u/checkout/install/mote_nav/lib/icp_odom_gate",
        cwd="/home/u/checkout",
    )
    assert select(node) == []


# --- identity and grouping -------------------------------------------------


def test_name_resolves_through_the_interpreter():
    """`python3.12 /.../lib/mote_bringup/twist_relay` is a twist_relay."""
    node = proc(
        10,
        exe="/home/u/checkout/.pixi/envs/default/bin/python3.12",
        argv=[
            "/home/u/.../python3.12",
            str(JOBS / "a/install/mote_bringup/lib/twist_relay"),
            "--ros-args",
        ],
    )
    assert node.name == "twist_relay"


def test_name_skips_interpreter_flags():
    node = proc(
        10, exe="/usr/bin/python3", argv=["python3", "-u", "/opt/thing/server.py"]
    )
    assert node.name == "server.py"


def test_name_falls_back_to_the_executable():
    node = proc(
        10, exe="/opt/ros/lib/twist_mux/twist_mux", argv=["twist_mux", "--ros-args"]
    )
    assert node.name == "twist_mux"


@pytest.mark.parametrize(
    "path",
    [
        JOBS / "abc/wt-x/install/lib/node",
        JOBS / "abc/tmp/wt-x/install/lib/node",
        JOBS / "abc/tmp/overlay/lib/node",
    ],
)
def test_owning_job_groups_by_job_whatever_the_layout(path):
    """Jobs disagree about where the worktree goes; the job is what died."""
    assert sweep.owning_job(proc(10, exe=str(path)), ROOTS) == str(JOBS / "abc")


def test_owning_job_names_the_worktree_for_checkout_worktrees():
    node = proc(10, cwd=str(WORKTREES / "sim-sites/mote_nav"))
    assert sweep.owning_job(node, ROOTS) == str(WORKTREES / "sim-sites")


def test_owning_job_is_none_outside_the_roots():
    assert sweep.owning_job(proc(10, exe="/usr/bin/thing"), ROOTS) is None


# --- the scoped mode behind `pixi run kill` --------------------------------

CHECKOUT = Path("/home/u/checkout")


def test_scope_takes_a_live_stack_not_just_orphans():
    """A stuck stack has its launch right there; the orphan rule would spare it."""
    shell = proc(10, argv=["/usr/bin/zsh"], exe="/usr/bin/zsh", env={})
    launch = proc(11, ppid=10, exe=str(CHECKOUT / ".pixi/envs/default/bin/ros2"))
    node = proc(12, ppid=11, exe=str(CHECKOUT / "install/mote_nav/lib/icp_odom_gate"))
    found = sweep.owned_by(table(shell, launch, node), CHECKOUT, self_pid=SELF)
    assert [p.pid for p in found] == [11, 12]


def test_scope_is_ageless():
    """A stack started a second ago is exactly what a reset is aimed at."""
    node = proc(10, exe=str(CHECKOUT / "install/lib/node"), age=0.0)
    assert [p.pid for p in sweep.owned_by(table(node), CHECKOUT, self_pid=SELF)] == [10]


def test_scope_spares_the_shell_running_it():
    """The regression that made `pixi run kill` truncate itself.

    ``pkill -9 -f '<driver names>'`` matched the task's own shell — those names
    are in its command line — killed it, and so never ran the daemon reset that
    followed. Nothing in the sweeper's own ancestry may be selected, and a shell
    is never selected regardless.
    """
    shell = proc(
        10,
        argv=["sh", "-c", "sweep --scope . --kill; ros2 daemon stop"],
        exe="/usr/bin/sh",
    )
    me = proc(
        11,
        ppid=10,
        exe=str(CHECKOUT / ".pixi/envs/default/bin/python3.12"),
        argv=["python", "-m", "mote_bringup.sweep_orphans"],
    )
    node = proc(12, exe=str(CHECKOUT / "install/mote_nav/lib/icp_odom_gate"))
    found = sweep.owned_by(table(shell, me, node), CHECKOUT, self_pid=11)
    assert [p.pid for p in found] == [12]


def test_scope_ignores_another_checkout():
    node = proc(
        10,
        exe="/home/u/other-checkout/install/lib/node",
        env={"AMENT_PREFIX_PATH": "/home/u/other-checkout/install/mote_nav"},
    )
    assert sweep.owned_by(table(node), CHECKOUT, self_pid=SELF) == []


# --- reaping ---------------------------------------------------------------


def test_pid_reuse_is_not_killed():
    """A pid recycled between the scan and the signal must be skipped.

    ``_still_is`` compares the start time recorded at scan against the live one,
    so a stale record — here, our own pid with somebody else's start time —
    fails to match and is never signalled.
    """
    live = sweep.read_proc(os.getpid(), sweep._uptime())
    assert sweep._still_is(live)

    stale = proc(os.getpid())
    stale.starttime = live.starttime + 1
    assert not sweep._still_is(stale)


def test_reap_ignores_a_process_that_has_already_gone():
    gone = proc(2**22 - 1)  # above /proc/sys/kernel/pid_max on any sane host
    assert sweep.reap([gone], grace=0.1) == ([], [])


def _alive(pid):
    return Path(f"/proc/{pid}").exists()


def _ppid(pid):
    proc = sweep.read_proc(pid, sweep._uptime())
    return proc.ppid if proc else None


@pytest.fixture
def orphan(tmp_path):
    """A real orphaned process wearing a ROS environment, under a fake job root.

    The leak this sweep exists for is a *grandchild*: a fixture terminated the
    `ros2 run` wrapper it held, and the node the wrapper had spawned reparented
    to init. So the process here is spawned through a launcher that exits
    immediately, which is the only way to get a genuine ppid of 1 —
    ``start_new_session`` detaches the session but leaves the parent in place.
    """
    root = tmp_path / "jobs"
    workdir = root / "job1" / "wt-x"
    workdir.mkdir(parents=True)
    pidfile = tmp_path / "pid"

    launcher = (
        "import os, subprocess, sys\n"
        f"p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(300)'],\n"
        f"    cwd={str(workdir)!r},\n"
        f"    env={{**os.environ, 'AMENT_PREFIX_PATH': {str(workdir / 'install/mote_bringup')!r}}},\n"
        "    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)\n"
        f"open({str(pidfile)!r}, 'w').write(str(p.pid))\n"
    )
    subprocess.run([sys.executable, "-c", launcher], check=True, timeout=60)
    pid = int(pidfile.read_text())

    deadline = time.monotonic() + 10
    while _ppid(pid) not in (1, None) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _ppid(pid) != 1:
        pytest.skip("orphan was not reparented to init (a subreaper claimed it)")

    try:
        yield pid, root
    finally:
        if _alive(pid):
            os.kill(pid, 9)


def test_finds_and_reaps_a_real_orphan(orphan):
    """End to end against a real process, without needing ROS to be running.

    An orphan with a ROS-looking environment and a working directory under an
    agent root is exactly the shape of the leak, so this exercises the /proc
    reading, the ancestry test and the kill together.
    """
    pid, root = orphan

    found = sweep.select(sweep.read_all(), [root], min_age=0)
    assert pid in [p.pid for p in found], "the orphan was not found"

    target = next(p for p in found if p.pid == pid)
    assert sweep.owning_job(target, [root]) == str(root / "job1")

    terminated, _ = sweep.reap([target], grace=5.0)
    assert terminated == [pid]

    deadline = time.monotonic() + 10
    while _alive(pid) and time.monotonic() < deadline:
        time.sleep(0.1)
    assert not _alive(pid), "the orphan survived the sweep"


def test_a_real_orphan_is_spared_while_it_is_too_young(orphan):
    """The age floor is what keeps a live session-detached run out of the sweep."""
    pid, root = orphan
    found = sweep.select(sweep.read_all(), [root])  # default min age
    assert pid not in [p.pid for p in found]
    assert _alive(pid)


def test_the_sweep_ignores_a_real_orphan_outside_its_roots(orphan, tmp_path):
    """Same process, a root that does not contain it: not the sweep's business."""
    pid, _ = orphan
    elsewhere = tmp_path / "not-jobs"
    elsewhere.mkdir()
    found = sweep.select(sweep.read_all(), [elsewhere], min_age=0)
    assert pid not in [p.pid for p in found]
