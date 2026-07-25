"""Per-robot state (``MOTE_HOME``) versus shared package config.

Two kinds of configuration live in a Mote deployment, and the fleet layer needs
the line between them to be sharp:

**Shared config** ships inside the package and is identical on every robot of a
given version — ``mote_description/config/robot.yaml`` (wheel geometry, servo
bus, device paths), the Nav2/SLAM params, ``zones.default.yaml``. An update
replaces it wholesale.

**Per-robot state** belongs to this one machine and outlives every update. It
lives under ``MOTE_HOME`` (``~/.mote`` by default), *outside* the package, so an
update can never clobber identity, site selection, calibration, maps or bags::

    $MOTE_HOME/robot.yaml               identity: id/name/site      (identity.py)
    $MOTE_HOME/active.yaml              which site + floor is loaded (sites.py)
    $MOTE_HOME/sites/<site>/...         map + zone bundles           (sites.py)
    $MOTE_HOME/bags/<kind>/<ts>/        recordings        (record_launch.py)
    $MOTE_HOME/camera_calibration.yaml  overrides mote_perception/config/camera_info.default.yaml
    $MOTE_HOME/perception.yaml          overrides mote_perception/config/perception.yaml

The two ``robot.yaml`` files are different things and never mix:
``mote_description/config/robot.yaml`` is *shared hardware description*;
``$MOTE_HOME/robot.yaml`` is *this robot's identity*.

The override rule for a config file that has both halves is one line —
``mote_home.override("perception.yaml", packaged_default)`` — the per-robot file
wins when it exists, else the packaged default is used. Launch files use it so
that ``MOTE_HOME`` is honoured everywhere (the sim points ``MOTE_HOME`` at its
in-repo bundle; tests point it at a tmpdir).
"""

import os
from pathlib import Path


def mote_dir() -> Path:
    """The per-robot state root: ``$MOTE_HOME``, else ``~/.mote``."""
    return Path(os.environ.get("MOTE_HOME", "~/.mote")).expanduser()


def path(*parts: str) -> Path:
    """A path inside the per-robot state root."""
    return mote_dir().joinpath(*parts)


def override(name: str, default: str) -> str:
    """``$MOTE_HOME/<name>`` if it exists, else the packaged ``default``."""
    user = path(name)
    return str(user) if user.exists() else default
