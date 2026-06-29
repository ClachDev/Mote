"""Enforce a rolling disk budget on a rosbag2 output tree.

Periodically sums the size of every .mcap segment under a directory and deletes
the oldest segments once the total exceeds a cap, so a robot that records
continuously never fills its disk. The newest segment (the one currently being
written) is never deleted. Run directories left empty by pruning are removed
along with their now-stale metadata.

Each .mcap segment is self-contained, so pruning older segments out of a still
active run does not affect playback of the segments that remain.
"""

import argparse
import os
import shutil
import time


EMPTY_RUN_MIN_AGE_SECONDS = 120.0


def _segments(root):
    files = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            if not name.endswith(".mcap"):
                continue
            path = os.path.join(dirpath, name)
            try:
                st = os.stat(path)
            except FileNotFoundError:
                continue
            files.append((path, st.st_mtime, st.st_size))
    files.sort(key=lambda f: f[1])  # oldest first
    return files


def _remove_empty_runs(root):
    now = time.time()
    for entry in os.scandir(root):
        if not entry.is_dir():
            continue
        try:
            age = now - entry.stat().st_mtime
        except FileNotFoundError:
            continue
        if age < EMPTY_RUN_MIN_AGE_SECONDS:
            continue
        has_mcap = any(
            name.endswith(".mcap")
            for _d, _sub, names in os.walk(entry.path)
            for name in names
        )
        if not has_mcap:
            shutil.rmtree(entry.path, ignore_errors=True)
            print(f"bag_pruner: removed empty run {entry.path}", flush=True)


def prune(root, cap_bytes):
    segs = _segments(root)
    total = sum(s[2] for s in segs)
    # Keep the newest segment regardless of the cap — it is still being written.
    for path, _mtime, size in segs[:-1]:
        if total <= cap_bytes:
            break
        try:
            os.remove(path)
            print(f"bag_pruner: removed {path} ({size / 1e6:.0f} MB)", flush=True)
        except FileNotFoundError:
            pass
        total -= size
    _remove_empty_runs(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", required=True, help="rosbag2 output tree to prune")
    parser.add_argument(
        "--max-gb", type=float, required=True, help="disk cap for the tree, in GB"
    )
    parser.add_argument(
        "--interval", type=float, default=30.0, help="seconds between sweeps"
    )
    args = parser.parse_args()

    cap_bytes = int(args.max_gb * 1e9)
    os.makedirs(args.dir, exist_ok=True)
    try:
        while True:
            prune(args.dir, cap_bytes)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
