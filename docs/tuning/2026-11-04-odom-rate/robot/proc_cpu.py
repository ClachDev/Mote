"""CPU of named ROS processes over an interval, from /proc, stdlib only.

    python3 proc_cpu.py <seconds> <node name>...

Matches a process by `__node:=<name>` on its command line, which is how launch
names a container; `node_cpu.py` only matches executables installed by this
checkout, and the containers are installed by the environment.
"""

import os
import sys
import time

TICK = os.sysconf("SC_CLK_TCK")


def find(name):
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            argv = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        if f"__node:={name}".encode() in argv and b"launch" not in argv[0]:
            return int(pid)
    return None


def ticks(pid):
    fields = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12])


seconds = float(sys.argv[1])
pids = {n: find(n) for n in sys.argv[2:]}
start = {n: ticks(p) for n, p in pids.items() if p}
t0 = time.monotonic()
time.sleep(seconds)
dt = time.monotonic() - t0
for n, p in pids.items():
    if not p:
        print(f"{n}: not running")
        continue
    pct = 100.0 * (ticks(p) - start[n]) / TICK / dt
    print(f"{n}: {pct:.2f}% of a core over {dt:.0f} s (pid {p})")
print("loadavg", open("/proc/loadavg").read().strip())
