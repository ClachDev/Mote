"""Sweep spec: parse the YAML, validate it, expand the parameter grid.

A sweep spec names the parameters to vary, the config file and key path each
lives in, the values to try, and how the benchmark should score each set. It is
deliberately ROS-free and side-effect-free (pure parse + expand) so it can be
unit-tested without a sim.

Format (see ``sweep/README.md`` for the full reference)::

    name: office_nav          # optional label for the report
    benchmark:                # how bench.py scores each set
      worlds: [office_world.sdf]
      trials: 2
      goal_timeout: 180
      order: pickup,dropoff,home
      settle: 8
    parameters:
      - name: amcl_max_particles          # optional; derived from path if absent
        file: nav2                        # target: nav2 | slam | controllers
        path: amcl.ros__parameters.max_particles   # dotted, or a YAML list
        values: [1000, 2000, 4000]
      - name: inflation_radius
        file: nav2
        paths:                            # one value applied to several key paths
          - [local_costmap, local_costmap, ros__parameters, inflation_layer, inflation_radius]
          - [global_costmap, global_costmap, ros__parameters, inflation_layer, inflation_radius]
        range: {start: 0.25, stop: 0.55, step: 0.15}   # inclusive; or `values:`
    scoring:                  # optional; score.py documents the defaults
      weights: {success: 3.0, localization: 1.0, time: 1.0, smoothness: 0.5}
      world_weights: {office_world.sdf: 1.0}

Key paths may be given as a dotted string (``a.b.c``) or a list of keys. Use the
list form when a key itself contains a dot (Nav2 writes some plugin params as
literal dotted keys, e.g. ``FollowPath.max_vel_x`` is *nested* here but
``WheelSpeedLimit.scale`` is a literal key).
"""

from __future__ import annotations

import itertools
from pathlib import Path

import yaml

VALID_TARGETS = ("nav2", "slam", "controllers")


class SpecError(ValueError):
    """Raised for a malformed sweep spec, with a human-readable reason."""


def _as_key_paths(param, idx):
    """Return a list of key-path lists for a parameter entry (supports ``path``
    for one and ``paths`` for several sharing the same value)."""
    if ("path" in param) == ("paths" in param):
        raise SpecError(f"parameter #{idx}: give exactly one of `path` or `paths`")
    raw = param["paths"] if "paths" in param else [param["path"]]
    if "paths" in param and not isinstance(raw, list):
        raise SpecError(f"parameter #{idx}: `paths` must be a list of key paths")
    out = []
    for p in raw:
        if isinstance(p, str):
            keys = p.split(".")
        elif isinstance(p, list):
            keys = [str(k) for k in p]
        else:
            raise SpecError(f"parameter #{idx}: each path must be a string or list")
        if not keys or any(k == "" for k in keys):
            raise SpecError(f"parameter #{idx}: empty key in path {p!r}")
        out.append(keys)
    return out


def _values(param, idx):
    """Explicit ``values`` list, or an inclusive numeric ``range``."""
    if "values" in param:
        vals = param["values"]
        if not isinstance(vals, list) or not vals:
            raise SpecError(f"parameter #{idx}: `values` must be a non-empty list")
        return list(vals)
    if "range" in param:
        r = param["range"]
        try:
            start, stop, step = float(r["start"]), float(r["stop"]), float(r["step"])
        except (KeyError, TypeError, ValueError):
            raise SpecError(
                f"parameter #{idx}: `range` needs numeric start/stop/step"
            ) from None
        if step <= 0:
            raise SpecError(f"parameter #{idx}: `range.step` must be > 0")
        out, v, n = [], start, 0
        # Walk by index to avoid float drift, include stop within half a step.
        while v <= stop + step / 2:
            out.append(round(v, 10))
            n += 1
            v = start + n * step
        return out
    raise SpecError(f"parameter #{idx}: give `values` or `range`")


class Parameter:
    """One swept parameter: a target file, one or more key paths, and its values."""

    def __init__(self, name, target, key_paths, values):
        self.name = name
        self.target = target
        self.key_paths = key_paths
        self.values = values

    def label(self, value):
        return f"{self.name}={value}"


class Spec:
    def __init__(self, raw, source=None):
        self.source = source
        self.name = raw.get("name") or (Path(source).stem if source else "sweep")

        bench = raw.get("benchmark") or {}
        worlds = bench.get("worlds") or ["office_world.sdf"]
        if isinstance(worlds, str):
            worlds = [w.strip() for w in worlds.split(",") if w.strip()]
        self.worlds = worlds
        self.trials = int(bench.get("trials", 2))
        self.goal_timeout = float(bench.get("goal_timeout", 120.0))
        self.order = str(bench.get("order", "pickup,dropoff,home"))
        self.settle = float(bench.get("settle", 8.0))

        params = raw.get("parameters")
        if not params:
            raise SpecError("spec has no `parameters`")
        self.parameters = []
        for i, p in enumerate(params):
            target = p.get("file")
            if target not in VALID_TARGETS:
                raise SpecError(
                    f"parameter #{i}: `file` must be one of {VALID_TARGETS}, got "
                    f"{target!r}"
                )
            key_paths = _as_key_paths(p, i)
            values = _values(p, i)
            name = p.get("name") or key_paths[0][-1]
            self.parameters.append(Parameter(name, target, key_paths, values))

        scoring = raw.get("scoring") or {}
        self.weights = scoring.get("weights")
        self.world_weights = scoring.get("world_weights")

    def grid(self):
        """Every parameter set as a list of ``(Parameter, value)`` assignments,
        with the all-defaults **baseline** (empty assignments) first."""
        sets = [[]]  # baseline: no overrides -> committed defaults
        axes = [[(p, v) for v in p.values] for p in self.parameters]
        for combo in itertools.product(*axes):
            sets.append(list(combo))
        return sets

    def grid_size(self):
        n = 1
        for p in self.parameters:
            n *= len(p.values)
        return n


def load(path):
    """Load and validate a sweep spec from a YAML file."""
    text = Path(path).read_text()
    raw = yaml.safe_load(text)
    if not isinstance(raw, dict):
        raise SpecError(f"{path}: top level must be a mapping")
    return Spec(raw, source=str(path))
