"""The Augere open specifications, as Mote implements them.

Three contracts, versioned independently, all at v0:

* :mod:`~mote_bringup.spec.capability` — what a platform can be asked to do.
* :mod:`~mote_bringup.spec.mission` — how one action is dispatched, and how its
  outcome comes back over a link that drops.
* :mod:`~mote_bringup.spec.zone` — how places are named once for a whole fleet
  and located separately by every robot in it.

Mote is the reference implementation, which is why these live here rather than
in :mod:`mote_fleet`: the same three ends need them and none of them may depend
on the others. The robot's task layer (:mod:`mote_tasks`) *executes* missions,
the fleet agent (:mod:`mote_fleet`) *bridges* them, and the off-board fleet
server *dispatches* them with neither ROS nor a checkout. That is the same
reasoning that put :mod:`mote_bringup.bundle` here — one module imported by
everything, rather than a second implementation on the server that agrees only
by convention.

So, like ``bundle`` and ``mote_fleet.protocol``, this package is **ROS-free**;
unlike ``bundle`` it is also **stdlib-only**, because the fleet server's
container installs it beside no framework at all and nothing here reads a file
format that a library already owns.

The prose contracts are the spec repository's ``mission/v0/README.md``,
``capability/v0/README.md`` and ``zone/v0/README.md``, and their JSON Schema
mirrors are the authority where the two disagree. Mote does **not** vendor
those schemas: a copy is a copy, and it drifts. ``test_spec_conformance.py``
instead validates real payloads against the spec repository's own schemas when
a checkout of it is present, and skips when it is not.

Where Mote and a spec disagree, the disagreement is a bug in one of them and is
recorded as such — not papered over here.
"""


class SpecError(ValueError):
    """A payload or a declaration that does not meet the contract.

    Raised at the point of *construction* rather than carried to the wire: a
    payload this package cannot build is a defect in the software that asked
    for it, and the fleet finding out by rejecting it later is strictly worse.
    """
