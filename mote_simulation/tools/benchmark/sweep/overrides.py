"""Build merged config files for one parameter set, without touching committed
configs.

For each target that a parameter set overrides, the base committed YAML is
loaded, the swept key paths are set on an in-memory copy, and the result is
written to a fresh file. The sweep runner exports the matching
``MOTE_*_PARAMS_FILE`` variable (see ``mote_bringup.param_overrides``) so the
launch reads the merged file instead of the committed one. The committed files
are never modified.
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

# target -> committed config file, relative to the repo root.
BASE_FILES = {
    "nav2": "mote_bringup/config/nav2_params.yaml",
    "slam": "mote_bringup/config/slam_toolbox_params.yaml",
    "controllers": "mote_bringup/config/controllers.yaml",
}

# target -> env var the launch consults (kept in sync with param_overrides).
ENV_VARS = {
    "nav2": "MOTE_NAV2_PARAMS_FILE",
    "slam": "MOTE_SLAM_PARAMS_FILE",
    "controllers": "MOTE_CONTROLLERS_FILE",
}


def base_path(repo_root, target):
    return Path(repo_root) / BASE_FILES[target]


def deep_get(doc, key_path):
    """Read the leaf at ``key_path`` (list of keys); raises KeyError if absent."""
    node = doc
    for k in key_path:
        node = node[k]
    return node


def deep_set(doc, key_path, value):
    """Set the leaf at ``key_path`` (list of keys), creating intermediate maps.

    Raises KeyError if the path traverses through a value that is not a mapping,
    so a typo'd path fails loudly rather than silently growing the config.
    """
    node = doc
    for k in key_path[:-1]:
        if k not in node:
            node[k] = {}
        node = node[k]
        if not isinstance(node, dict):
            raise KeyError(f"path {key_path} traverses non-mapping at {k!r}")
    node[key_path[-1]] = value


def default_value(repo_root, target, key_path):
    """The committed value at ``key_path`` in ``target``'s base file, or None if
    the path does not exist there."""
    doc = yaml.safe_load(base_path(repo_root, target).read_text())
    try:
        return deep_get(doc, key_path)
    except (KeyError, TypeError):
        return None


def build_override_files(assignments, repo_root, out_dir):
    """Write merged config files for the targets touched by ``assignments`` (a
    list of ``(Parameter, value)``). Returns ``{ENV_VAR: file_path}`` to export;
    empty for the baseline (no assignments)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    by_target = {}
    for param, value in assignments:
        by_target.setdefault(param.target, []).append((param, value))

    env = {}
    for target, items in by_target.items():
        doc = yaml.safe_load(base_path(repo_root, target).read_text())
        doc = copy.deepcopy(doc)
        for param, value in items:
            for key_path in param.key_paths:
                deep_set(doc, key_path, value)
        dest = out_dir / f"{target}_params.yaml"
        dest.write_text(yaml.safe_dump(doc, sort_keys=False))
        env[ENV_VARS[target]] = str(dest)
    return env
