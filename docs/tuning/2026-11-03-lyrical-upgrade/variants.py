"""Write the variant nav2 params files of the rate experiment into argv[1].

Run from the repository root: python variants.py <outdir>
"""

import copy
import pathlib
import sys

import yaml

here = pathlib.Path(sys.argv[1])
here.mkdir(parents=True, exist_ok=True)
base = yaml.safe_load(open("mote_bringup/config/nav2_params.yaml"))


def write(name, edit):
    doc = copy.deepcopy(base)
    edit(doc)
    yaml.safe_dump(doc, open(here / f"{name}.yaml", "w"))


def cs(d):
    return d["controller_server"]["ros__parameters"]


def lc(d):
    return d["local_costmap"]["local_costmap"]["ros__parameters"]


write("A_baseline", lambda d: None)
write("B_no_costmap_wait", lambda d: cs(d).update(costmap_update_timeout=0.0))
write("C_tf_tol_005", lambda d: lc(d).update(transform_tolerance=0.05))


def no_debug(d):
    pc = cs(d)["FollowPath"]["primary_controller"]
    for k in (
        "publish_evaluation",
        "publish_local_plan",
        "publish_trajectories",
        "publish_cost_grid_pc",
    ):
        pc[k] = False


write("D_no_dwb_debug", no_debug)
write(
    "E_heading_once", lambda d: cs(d)["FollowPath"].update(rotate_to_heading_once=True)
)
write("F_path_orient", lambda d: cs(d)["FollowPath"].update(use_path_orientations=True))
