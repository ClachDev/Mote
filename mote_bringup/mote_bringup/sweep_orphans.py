"""Find and kill ROS processes left behind by agent worktrees that are gone.

Every agent job runs in its own worktree under ``~/.claude/jobs/<id>/`` or
``<checkout>/.claude/worktrees/<name>/``. When a job ends, its worktree goes
away — but a process it started does not, if nothing reaped it. Those survivors
reparent to init and run until the box is rebooted: two whole Nav2 stacks, a
dozen ``twist_mux`` and a handful of brokers were found here 2-4 days after the
jobs that spawned them had finished.

They matter beyond the wasted core and RAM. They are the exact process names a
benchmark measures, and while ``overhead.py`` scopes its own match to the
current checkout, the system-wide counters a benchmark sits in — context
switches, interrupts, memory pressure, CPU contention — cannot be scoped. Every
measurement on the box is taken against a drifting background until they go.

    ros2 run mote_bringup sweep_orphans           # pixi run sweep: report only
    ros2 run mote_bringup sweep_orphans --kill    # actually reap them
    ros2 run mote_bringup sweep_orphans --json

**Matching is on identity, never on the command line.** ``pkill -f`` matches the
sweeper's own shell (whose command line contains the pattern it was handed) and
every other checkout's live run alike; it is how a cleanup turns into an outage.
So a process is only a candidate when four independent things hold:

* it carries a **ROS environment** (``AMENT_PREFIX_PATH`` and friends), which is
  what separates a node from the file manager that happens to have a job path in
  its argv;
* something about it — its executable, its working directory, its command line
  or its ament/colcon/pixi prefixes — **lives under an agent root**, which is
  what scopes the sweep to agent leakage rather than to the user's own work;
* it is **orphaned**: walking its ancestry reaches init without passing a live
  process that is not itself a candidate, so anything still owned by a shell, a
  pytest run or an agent session is left alone;
* it is **older than ``--min-age``** (default 30 min). A deliberately
  session-detached run — the sim smoke test ``setsid``s its launch — is
  indistinguishable from an orphan by ancestry alone, and age is what tells them
  apart. The real leaks were days old; nothing legitimate is that old and idle.

A small never-sweep set (shells, editors, ``claude`` itself) is the belt to that
brace: those inherit a ROS environment from a ``pixi shell`` like anything else,
and killing the operator's terminal is not a cleanup.

Signalling re-checks each pid's start time against the value read during the
scan, so a pid recycled between scanning and killing is skipped rather than
shot — the window is small but the consequence is somebody else's process.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")

#: Environment variables that mark a process as belonging to a ROS graph.
ROS_ENV_MARKERS = ("AMENT_PREFIX_PATH", "ROS_DISTRO", "RMW_IMPLEMENTATION")

#: Environment variables naming a prefix that implicates a process in a worktree.
PREFIX_ENV_VARS = ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "PIXI_PROJECT_ROOT")

#: Never swept, whatever else matches: a `pixi shell` in a worktree gives these
#: a ROS environment and agent provenance, and they are the operator's, not ours.
NEVER_SWEEP = frozenset(
    {
        "bash",
        "sh",
        "zsh",
        "fish",
        "dash",
        "tcsh",
        "csh",
        "claude",
        "code",
        "cursor",
        "nvim",
        "vim",
        "vi",
        "emacs",
        "nano",
        "tmux",
        "screen",
        "sshd",
        "ssh",
        "mosh-server",
        "systemd",
        "init",
        "dbus-daemon",
        "gnome-terminal-server",
        "git",
        "pixi",
        "docker",
        "containerd",
        "dockerd",
    }
)

DEFAULT_MIN_AGE = 1800.0  # seconds


def agent_roots(checkout: Path | None = None) -> list[Path]:
    """Directories under which an agent job's worktree lives."""
    roots = [Path.home() / ".claude" / "jobs"]
    if checkout is not None:
        roots.append(checkout / ".claude" / "worktrees")
    return roots


def _read(path: str) -> bytes | None:
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except OSError:
        return None


def _readlink(path: str) -> str | None:
    try:
        return os.readlink(path)
    except OSError:
        return None


def _uptime() -> float:
    with open("/proc/uptime") as handle:
        return float(handle.read().split()[0])


@dataclass
class Proc:
    """One process as the sweep sees it.

    Everything the sweep decides on is captured at scan time, including
    ``starttime`` — the value that makes a later signal safe against pid reuse.
    Reading it out of /proc is ``read_proc``'s job, so every rule below is a
    plain function of plain data and can be tested without a process to match.
    """

    pid: int
    ppid: int
    starttime: int
    age: float
    argv: list[str]
    env: dict[str, str]
    exe: str | None = None
    cwd: str | None = None
    rss: int = 0

    @property
    def cmdline(self) -> str:
        return " ".join(self.argv)

    @property
    def name(self) -> str:
        """The program this process *is*, as a bare name.

        A node run through a console script is ``python3.12 /.../lib/pkg/node``,
        so the interpreter's own name says nothing; the script path does.
        """
        exe = Path(self.exe.removesuffix(" (deleted)")).name if self.exe else ""
        if exe.startswith("python") and len(self.argv) > 1:
            for arg in self.argv[1:]:
                if arg.startswith("-"):
                    continue
                return Path(arg).name
        return exe or Path(self.argv[0]).name

    def provenance(self) -> list[str]:
        """Every path that ties this process to a place on disk."""
        paths = []
        for value in (self.exe, self.cwd):
            if value:
                paths.append(value.removesuffix(" (deleted)"))
        paths.extend(arg for arg in self.argv if arg.startswith("/"))
        for var in PREFIX_ENV_VARS:
            paths.extend(p for p in self.env.get(var, "").split(":") if p)
        return paths

    def is_ros(self) -> bool:
        return any(marker in self.env for marker in ROS_ENV_MARKERS)


def read_proc(pid: int, uptime: float) -> Proc | None:
    """Read one process out of /proc, or None if it is not one we can judge.

    Kernel threads have an empty command line and a process that exits mid-read
    leaves partial files; neither is something to sweep, so both come back None
    rather than raising into the scan loop.
    """
    raw_cmdline = _read(f"/proc/{pid}/cmdline") or b""
    argv = [a for a in raw_cmdline.decode("utf8", "replace").split("\0") if a]
    stat = _read(f"/proc/{pid}/stat")
    if stat is None or not argv:
        return None
    try:
        # The comm field is parenthesised and may itself contain spaces, so the
        # numeric fields start after the *last* ')'.
        fields = stat.decode("utf8", "replace").rsplit(")", 1)[1].split()
        ppid = int(fields[1])
        starttime = int(fields[19])  # field 22, 1-indexed, of proc(5)
    except (IndexError, ValueError):
        return None

    environ = _read(f"/proc/{pid}/environ") or b""
    statm = _read(f"/proc/{pid}/statm")
    return Proc(
        pid=pid,
        ppid=ppid,
        starttime=starttime,
        age=max(0.0, uptime - starttime / CLK_TCK),
        argv=argv,
        env=dict(
            entry.split("=", 1)
            for entry in environ.decode("utf8", "replace").split("\0")
            if "=" in entry
        ),
        exe=_readlink(f"/proc/{pid}/exe"),
        cwd=_readlink(f"/proc/{pid}/cwd"),
        rss=int(statm.split()[1]) * PAGE_SIZE if statm else 0,
    )


def read_all() -> dict[int, Proc]:
    """Every process on the host that the sweep can read, keyed by pid."""
    uptime = _uptime()
    procs = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        proc = read_proc(int(entry), uptime)
        if proc is not None:
            procs[proc.pid] = proc
    return procs


def _under(path: str, root: Path) -> bool:
    try:
        Path(path).relative_to(root)
    except ValueError:
        return False
    return True


def owning_job(proc: Proc, roots: list[Path]) -> str | None:
    """The agent job this process came out of, or None.

    Answered as the first path component below an agent root — a job id under
    ``~/.claude/jobs``, a worktree name under ``.claude/worktrees`` — so every
    process from one job groups under one heading, whichever of its paths
    happened to implicate it. Jobs vary in where they put their worktree
    (``<id>/wt-foo`` and ``<id>/tmp/wt-foo`` both occur), and the job is the unit
    that died in either case.
    """
    for path in proc.provenance():
        for root in roots:
            if not _under(path, root):
                continue
            parts = Path(path).relative_to(root).parts
            if not parts:
                continue
            return str(root / parts[0])
    return None


def _orphaned(proc: Proc, by_pid: dict[int, Proc], candidates: set[int]) -> bool:
    """True when nothing outside the candidate set still owns this process.

    Walks up the ancestry. Reaching init having only passed candidates means the
    whole tree was reparented — the job that owned it is gone. Meeting any live
    process that is not a candidate (a shell, a pytest run, an agent session)
    means the run is still owned, and is left alone.
    """
    seen = set()
    pid = proc.ppid
    while pid > 1 and pid not in seen:
        seen.add(pid)
        parent = by_pid.get(pid)
        if parent is None:  # exited between reads; treat the chain as broken
            return True
        if pid not in candidates:
            return False
        pid = parent.ppid
    return True


def select(
    by_pid: dict[int, Proc],
    roots: list[Path],
    min_age: float = DEFAULT_MIN_AGE,
    self_pid: int | None = None,
) -> list[Proc]:
    """Every orphaned ROS process in ``by_pid`` traceable to a job under ``roots``.

    A pure function of the process table, so the whole rule — including the
    self-match guard that ``pkill -f`` lacks — is testable without a process to
    match against.
    """
    ours = _ancestry(os.getpid() if self_pid is None else self_pid, by_pid)
    candidates = {
        pid
        for pid, proc in by_pid.items()
        if proc.is_ros()
        and proc.name not in NEVER_SWEEP
        and owning_job(proc, roots) is not None
        and pid not in ours
    }
    return sorted(
        (
            by_pid[pid]
            for pid in candidates
            if by_pid[pid].age >= min_age and _orphaned(by_pid[pid], by_pid, candidates)
        ),
        key=lambda p: p.pid,
    )


def scan(roots: list[Path], min_age: float = DEFAULT_MIN_AGE) -> list[Proc]:
    """Every orphaned ROS process on this host traceable to a job under ``roots``."""
    return select(read_all(), roots, min_age=min_age)


def owned_by(
    by_pid: dict[int, Proc], scope: Path, self_pid: int | None = None
) -> list[Proc]:
    """Every ROS process running out of ``scope``, orphaned or not.

    What ``pixi run kill`` wants: a stuck stack in this checkout is not orphaned
    — its launch is right there — so the sweep's ancestry and age rules would
    spare exactly the processes it exists to clear. The self-ancestry and
    never-sweep guards still apply, because the shell running the reset is
    itself a process of this checkout.
    """
    scope = scope.resolve()
    ours = _ancestry(os.getpid() if self_pid is None else self_pid, by_pid)
    return sorted(
        (
            proc
            for pid, proc in by_pid.items()
            if proc.is_ros()
            and proc.name not in NEVER_SWEEP
            and pid not in ours
            and any(_under(path, scope) for path in proc.provenance())
        ),
        key=lambda p: p.pid,
    )


def _ancestry(pid: int, by_pid: dict[int, Proc]) -> set[int]:
    """A pid and all of its ancestors — the sweeper never signals its own line."""
    chain = {pid}
    while (proc := by_pid.get(pid)) is not None and proc.ppid > 1:
        pid = proc.ppid
        if pid in chain:
            break
        chain.add(pid)
    return chain


def _still_is(proc: Proc) -> bool:
    """True when this pid is still the process that was scanned.

    A pid freed and reissued between the scan and the signal would otherwise be
    killed in place of the orphan.
    """
    stat = _read(f"/proc/{proc.pid}/stat")
    if stat is None:
        return False
    try:
        return (
            int(stat.decode("utf8", "replace").rsplit(")", 1)[1].split()[19])
            == proc.starttime
        )
    except (IndexError, ValueError):
        return False


def reap(procs: list[Proc], grace: float = 5.0) -> tuple[list[int], list[int]]:
    """SIGTERM, then SIGKILL whatever is left. Returns (terminated, killed)."""
    terminated, killed = [], []
    for proc in procs:
        if not _still_is(proc):
            continue
        try:
            os.kill(proc.pid, signal.SIGTERM)
            terminated.append(proc.pid)
        except (ProcessLookupError, PermissionError):
            continue

    deadline = time.monotonic() + grace
    remaining = [p for p in procs if p.pid in terminated]
    while remaining and time.monotonic() < deadline:
        time.sleep(0.2)
        remaining = [p for p in remaining if _still_is(p)]

    for proc in remaining:
        if not _still_is(proc):
            continue
        try:
            os.kill(proc.pid, signal.SIGKILL)
            killed.append(proc.pid)
        except (ProcessLookupError, PermissionError):
            continue
    return terminated, killed


# --- not leaking in the first place ----------------------------------------


def spawn_reapable(command: list[str], **kwargs) -> subprocess.Popen:
    """Start a process that can actually be stopped again.

    ``ros2 run`` is a Python wrapper that ``Popen``s the real executable and
    installs no SIGTERM handler — it only tolerates ``KeyboardInterrupt``, on the
    assumption (true only of a Ctrl-C at a terminal) that the signal reached the
    whole process group. So ``proc.terminate()`` on the wrapper kills the wrapper
    alone and hands the node to init. That is where most of this repo's leaked
    nodes came from: one per run, on the path where the run *succeeded*.

    Putting the child in its own session makes the wrapper and the node
    addressable together, which is what ``reap_group`` then signals.
    """
    return subprocess.Popen(command, start_new_session=True, **kwargs)


def reap_group(proc: subprocess.Popen | None, grace: float = 10.0) -> None:
    """SIGTERM a ``spawn_reapable`` process group, then SIGKILL what remains."""
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            continue


def _age(seconds: float) -> str:
    days, rest = divmod(int(seconds), 86400)
    hours, rest = divmod(rest, 3600)
    minutes = rest // 60
    if days:
        return f"{days}d{hours:02d}h"
    if hours:
        return f"{hours}h{minutes:02d}m"
    return f"{minutes}m"


def report(
    procs: list[Proc], roots: list[Path], scope: Path | None = None, stream=sys.stdout
) -> None:
    if not procs:
        where = f"running out of {scope}" if scope else "from dead agent worktrees"
        print(f"no ROS processes {where}", file=stream)
        return

    groups: dict[str, list[Proc]] = {}
    for proc in procs:
        key = str(scope) if scope else (owning_job(proc, roots) or "?")
        groups.setdefault(key, []).append(proc)

    total_rss = sum(p.rss for p in procs)
    what = (
        f"{len(procs)} ROS process(es) in this checkout"
        if scope
        else f"{len(procs)} orphaned ROS process(es) from {len(groups)} dead job(s)"
    )
    print(f"{what}, {total_rss / 1e6:.0f} MB resident", file=stream)
    home = str(Path.home())
    for owner, members in sorted(groups.items()):
        print(f"\n  {owner.replace(home, '~')}", file=stream)
        for proc in members:
            # An executable whose file is unlinked is the clearest sign the
            # worktree it came from has already been removed underneath it.
            gone = "  (exe deleted)" if (proc.exe or "").endswith(" (deleted)") else ""
            print(
                f"    {proc.pid:>8}  {_age(proc.age):>6}  {proc.rss / 1e6:>6.1f} MB  "
                f"{proc.name}{gone}",
                file=stream,
            )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--kill",
        action="store_true",
        help="actually reap them; without this the sweep only reports",
    )
    parser.add_argument(
        "--min-age",
        type=float,
        default=DEFAULT_MIN_AGE,
        metavar="SECONDS",
        help="ignore processes younger than this (default: %(default)s). A "
        "session-detached run that is still live looks orphaned by ancestry; "
        "age is what separates it from a leak.",
    )
    parser.add_argument(
        "--root",
        action="append",
        type=Path,
        metavar="DIR",
        help="additional directory under which agent worktrees live "
        "(repeatable); defaults to ~/.claude/jobs and <checkout>/.claude/worktrees",
    )
    parser.add_argument(
        "--checkout",
        type=Path,
        default=Path.cwd(),
        help="checkout whose .claude/worktrees to include (default: cwd)",
    )
    parser.add_argument(
        "--scope",
        type=Path,
        metavar="DIR",
        help="instead of sweeping dead jobs, take every ROS process running out "
        "of DIR whether orphaned or not — what `pixi run kill` needs to clear a "
        "stuck stack, which is live by definition",
    )
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)

    roots = agent_roots(args.checkout) + list(args.root or [])
    if args.scope is not None:
        procs = owned_by(read_all(), args.scope)
    else:
        procs = scan(roots, min_age=args.min_age)

    if args.kill:
        terminated, killed = reap(procs)
    else:
        terminated, killed = [], []

    if args.json:
        json.dump(
            {
                "processes": [
                    {
                        "pid": p.pid,
                        "ppid": p.ppid,
                        "name": p.name,
                        "age_s": round(p.age, 1),
                        "rss_bytes": p.rss,
                        "worktree": owning_job(p, roots),
                        "cmdline": p.cmdline,
                    }
                    for p in procs
                ],
                "terminated": terminated,
                "killed": killed,
                "swept": args.kill,
            },
            sys.stdout,
            indent=2,
        )
        print()
    else:
        report(procs, roots, scope=args.scope)
        if args.kill and procs:
            print(
                f"\nreaped {len(terminated)} (SIGKILL needed for {len(killed)})",
                file=sys.stdout,
            )
        elif procs:
            print("\nnothing killed — re-run with --kill to reap them", file=sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
