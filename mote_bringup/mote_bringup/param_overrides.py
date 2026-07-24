"""Launch-time override seam for tunable config files.

Each tunable config file has an environment variable that, when set, points the
launch at a caller-supplied file instead of the committed one. This is the seam
the parameter-sweep tool (``mote_simulation/tools/benchmark/sweep``) uses to try
parameter sets without ever mutating the committed configs: it writes a merged
temp params file per target and exports the matching variable before launching
the benchmark.

When a variable is unset — every robot run, every ordinary sim run — the
committed file is used unchanged, so this is inert outside a sweep.

Targets and their variables:

======================  ================================  =========================================
target                  variable                          committed file
======================  ================================  =========================================
``nav2``                ``MOTE_NAV2_PARAMS_FILE``         ``mote_bringup/config/nav2_params.yaml``
``slam``                ``MOTE_SLAM_PARAMS_FILE``         ``mote_bringup/config/slam_toolbox_params.yaml``
``controllers``         ``MOTE_CONTROLLERS_FILE``         ``mote_bringup/config/controllers.yaml``
======================  ================================  =========================================
"""

from __future__ import annotations

import os

ENV_VARS = {
    "nav2": "MOTE_NAV2_PARAMS_FILE",
    "slam": "MOTE_SLAM_PARAMS_FILE",
    "controllers": "MOTE_CONTROLLERS_FILE",
}


def override_path(target, default):
    """Return the override file for ``target`` if its env var names an existing
    file, else ``default`` (which may be a plain path string or a launch
    substitution — it is returned untouched)."""
    env = ENV_VARS.get(target)
    path = os.environ.get(env) if env else None
    if path and os.path.isfile(path):
        return path
    return default
