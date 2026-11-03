"""List every key in a params file that the running node did not declare.

A key a node does not declare is silently ignored, which is how a renamed or
removed Nav2 parameter shows up. Run beside a live stack:
    python param_audit.py nav2_params.yaml [more.yaml ...]
"""

import subprocess
import sys

import yaml


def flatten(d, prefix=""):
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            yield from flatten(v, key + ".")
        else:
            yield key


def nodes_in(doc, prefix=""):
    for k, v in doc.items():
        if not isinstance(v, dict):
            continue
        if "ros__parameters" in v:
            yield prefix + k, v["ros__parameters"]
        else:
            yield from nodes_in(v, prefix + k + "/")


def declared(node):
    out = subprocess.run(
        ["ros2", "param", "list", "/" + node],
        capture_output=True,
        text=True,
        timeout=30,
    )
    if out.returncode != 0 or not out.stdout.strip():
        return None
    return {line.strip() for line in out.stdout.splitlines() if line.startswith("  ")}


status = 0
for path in sys.argv[1:]:
    doc = yaml.safe_load(open(path))
    for node, params in nodes_in(doc):
        names = declared(node)
        if names is None:
            print(f"{node}: NOT RUNNING")
            continue
        missing = sorted(k for k in flatten(params) if k not in names)
        print(f"{node}: {len(missing)} undeclared")
        for k in missing:
            print(f"    {k}")
            status = 1
sys.exit(status)
