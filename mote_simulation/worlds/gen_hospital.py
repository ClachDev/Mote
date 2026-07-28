#!/usr/bin/env python3
"""Generator for hospital_world.sdf -- the "hard" Mote simulation world.

Edit the layout in build_world() and regenerate rather than hand-editing the
SDF:

    python mote_simulation/worlds/gen_hospital.py

Why a generator instead of raw SDF? In SDF every wall is an absolute box (a
centre *and* a size), duplicated across a <collision> and a <visual>, and a
doorway is a manual split of one wall into two hand-solved boxes. At this
scale (~60x40 m, ~50 rooms, doors everywhere) that is hundreds of fragile
coordinates. Here you describe the layout in room/corridor terms and the
helpers compute the wall segments, cut the door gaps, and emit the box pairs.

Difficulty design (a deliberate "hard" tier above mote_world/office_world):
  * Looping corridor grid -- a spine through the origin, parallel access
    corridors, and cross corridors -- gives several navigable loops, so the
    planner has path choices and SLAM gets *genuine* loop closures to get
    right (or wrong).
  * Long corridors past the ~12 m lidar range keep office_world's aperture
    problem: nothing to lock onto longitudinally -> localisation drift.
  * Dense clusters of near-identical wards keep office_world's false loop
    closures: the scan matcher can snap a ward onto a look-alike ward.
  * Furniture clutter adds real lidar features and tight spaces for nav.

Invariants (kept by construction, not luck):
  * Furniture must intersect the scan plane. The lidar sits at ~5 cm and the
    sim is headless, so only collision geometry at that height is ever seen.
    Every piece is a floor-standing box spanning 0..FURN_H (= 0.3 m), like
    mote_world's obstacles -- a correct floor footprint, not a raised slab on
    legs the rays pass under.
  * Deterministic output: no RNG, everything placed explicitly, so re-running
    reproduces byte-identical SDF.
  * The robot spawns at the world origin (0, 0) with no x/y override, so the
    origin is left in open corridor by construction.
  * Task-layer zones (hospital_world.zones.yaml) are emitted alongside the SDF
    and asserted to clear every wall/furniture box, so a layout edit cannot
    silently strand a zone inside geometry.
"""

import math
import os

WALL_T = 0.15  # wall thickness (m)
WALL_H = 2.5  # wall height (m) -- the ~5 cm lidar cannot see over it
DOOR_W = 0.9  # doorway gap width (m)
FURN_H = 0.3  # furniture height (m); boxes span 0..FURN_H across the scan plane
MAT = "0.8 0.8 0.82 1"  # wall material (matches office_world)

# Corridor half-width: corridors are the empty gaps left between room banks.
CORR_HALF = 1.25  # 2.5 m wide corridors

# Outer footprint (perimeter wall centrelines).
HALF_X = 29.0
HALF_Y = 19.0

# Corridor centrelines. Where an empty band crosses another they form an open
# junction, so this grid of corridors is fully connected -> many loops.
SPINE_Y = 0.0  # main E-W corridor, through the origin
ACCESS_Y = 12.0  # parallel access corridors north and south of the spine
VERT_X = 22.0  # N-S cross corridors left and right of centre
CENTRE_VERT_X = 0.0  # central N-S corridor, through the origin

ZONE_CLEAR = 0.4  # minimum distance (m) from a zone to any wall/furniture box

_walls = []  # list of (cx, cy, sx, sy)
_furniture = []  # list of (cx, cy, sx, sy)
_zones = {}  # name -> (x, y, yaw, polygon|None)
_rooms = []  # walkable rectangles (x0, y0, x1, y1) -- room-segmentation truth


def _add_wall(cx, cy, sx, sy):
    _walls.append((cx, cy, sx, sy))


def furniture(cx, cy, sx, sy):
    """A floor-standing box spanning the scan plane (0..FURN_H)."""
    _furniture.append((cx, cy, sx, sy))


def _room_rect(x0, x1, y0, y1):
    """Record one enclosed room's walkable rectangle (wall faces, not centrelines).

    Emitted as hospital_world.rooms.yaml: the ground truth a map-segmentation
    run is scored against, which only this generator knows -- the SDF that comes
    out the other end is a pile of boxes with no notion of a room.
    """
    inset = WALL_T / 2
    xa, xb = sorted((x0, x1))
    ya, yb = sorted((y0, y1))
    _rooms.append((xa + inset, ya + inset, xb - inset, yb - inset))


def zone(name, x, y, yaw=0.0, polygon=None):
    """A named place for the task layer (mote_tasks): a pose to navigate to
    (fetch waypoint or ``goto <name>``), plus an optional ``polygon`` giving it
    an area footprint (rooms carry one; bare waypoints don't)."""
    _zones[name] = (x, y, yaw, polygon)


def _check_zone_clearance():
    """Every zone pose must clear every box and lie inside its own footprint,
    so a layout edit cannot silently strand a zone inside geometry or outside
    the room it names."""
    for name, (zx, zy, _yaw, polygon) in _zones.items():
        for cx, cy, sx, sy in _walls + _furniture:
            dx = max(abs(zx - cx) - sx / 2, 0.0)
            dy = max(abs(zy - cy) - sy / 2, 0.0)
            if math.hypot(dx, dy) < ZONE_CLEAR:
                raise SystemExit(
                    f"zone '{name}' at ({zx:g}, {zy:g}) is within {ZONE_CLEAR} m "
                    f"of the box at ({cx:g}, {cy:g}) size ({sx:g}, {sy:g})"
                )
        if polygon is not None and not _in_polygon(zx, zy, polygon):
            raise SystemExit(
                f"zone '{name}' at ({zx:g}, {zy:g}) is outside its own footprint"
            )


def _in_polygon(px, py, vertices):
    """Ray-cast membership, mirroring mote_tasks.zones.Polygon.contains."""
    inside = False
    for (ax, ay), (bx, by) in zip(vertices, vertices[1:] + vertices[:1]):
        if (ay > py) != (by > py):
            if px < ax + (py - ay) * (bx - ax) / (by - ay):
                inside = not inside
    return inside


def _segments(lo, hi, doors):
    """Solid sub-spans of [lo, hi] with a DOOR_W gap centred on each door."""
    gaps = sorted((d - DOOR_W / 2, d + DOOR_W / 2) for d in doors)
    segs = []
    a = lo
    for g0, g1 in gaps:
        if g0 > a:
            segs.append((a, g0))
        a = max(a, g1)
    if a < hi:
        segs.append((a, hi))
    return segs


def wall(x0, y0, x1, y1, doors=()):
    """An axis-aligned wall centreline, split into solid segments around doors."""
    if y0 == y1:  # horizontal, varies in x
        lo, hi = sorted((x0, x1))
        for a, b in _segments(lo, hi, doors):
            _add_wall((a + b) / 2, y0, b - a, WALL_T)
    elif x0 == x1:  # vertical, varies in y
        lo, hi = sorted((y0, y1))
        for a, b in _segments(lo, hi, doors):
            _add_wall(x0, (a + b) / 2, WALL_T, b - a)
    else:
        raise ValueError("wall must be axis-aligned")


def _edges(a, b, n):
    return [a + (b - a) * i / n for i in range(n + 1)]


def room_bank_h(x0, x1, y_door, y_back, n, *, back=True, bed=True):
    """A row of n rooms side by side along x, doors on the y_door wall.

    y_door is the corridor side; y_back the outer side. Dividers (and the two
    ends) run the room depth. Set back=False when y_back coincides with a wall
    already drawn (e.g. the perimeter or a back-to-back bank's shared wall).
    """
    edges = _edges(x0, x1, n)
    centres = [(edges[i] + edges[i + 1]) / 2 for i in range(n)]
    wall(x0, y_door, x1, y_door, doors=centres)
    if back:
        wall(x0, y_back, x1, y_back)
    for x in edges:
        wall(x, y_door, x, y_back)
    for i in range(n):
        _room_rect(edges[i], edges[i + 1], y_door, y_back)
    if bed:
        by = y_door + (y_back - y_door) * 0.62
        for cx in centres:
            furniture(cx, by, 0.9, 1.9)


def room_bank_v(y0, y1, x_door, x_back, n, *, back=True, bed=True):
    """A column of n rooms stacked along y, doors on the x_door wall."""
    edges = _edges(y0, y1, n)
    centres = [(edges[i] + edges[i + 1]) / 2 for i in range(n)]
    wall(x_door, y0, x_door, y1, doors=centres)
    if back:
        wall(x_back, y0, x_back, y1)
    for y in edges:
        wall(x_door, y, x_back, y)
    for i in range(n):
        _room_rect(x_door, x_back, edges[i], edges[i + 1])
    if bed:
        bx = x_door + (x_back - x_door) * 0.62
        for cy in centres:
            furniture(bx, cy, 1.9, 0.9)


def build_world():
    # Corridor edges derived from the centrelines above.
    acc_n_in = ACCESS_Y - CORR_HALF  # 10.75: spine-side edge of north access
    acc_n_out = ACCESS_Y + CORR_HALF  # 13.25: perimeter-side edge
    acc_s_in = -ACCESS_Y + CORR_HALF  # -10.75
    acc_s_out = -ACCESS_Y - CORR_HALF  # -13.25
    spine_n = SPINE_Y + CORR_HALF  # 1.25: north wall of the spine
    spine_s = SPINE_Y - CORR_HALF  # -1.25
    vx_w_in = -VERT_X + CORR_HALF  # -20.75: inner edge of the west cross corridor
    vx_w_out = -VERT_X - CORR_HALF  # -23.25: outer (wing) edge
    vx_e_in = VERT_X - CORR_HALF  # 20.75
    vx_e_out = VERT_X + CORR_HALF  # 23.25
    cv_w = CENTRE_VERT_X - CORR_HALF  # -1.25: walls flanking the central corridor
    cv_e = CENTRE_VERT_X + CORR_HALF  # 1.25

    # --- Perimeter -----------------------------------------------------------
    wall(-HALF_X, HALF_Y, HALF_X, HALF_Y)  # north
    wall(-HALF_X, -HALF_Y, HALF_X, -HALF_Y)  # south
    wall(-HALF_X, -HALF_Y, -HALF_X, HALF_Y)  # west
    wall(HALF_X, -HALF_Y, HALF_X, HALF_Y)  # east

    # The two wide interior x-blocks, west (B) and east (C), each bounded by a
    # cross corridor on the outside and the central corridor on the inside.
    b0, b1 = vx_w_in, cv_w  # [-20.75, -1.25]
    c0, c1 = cv_e, vx_e_in  # [1.25, 20.75]

    # --- North shallow ward rows (against the north perimeter) ---------------
    room_bank_h(b0, b1, acc_n_out, HALF_Y, 4, back=False)
    room_bank_h(c0, c1, acc_n_out, HALF_Y, 4, back=False)

    # --- North deep block: waiting hall (west) + radiology wards (east) ------
    waiting_hall(b0, b1, spine_n, acc_n_in)
    mid_n = (spine_n + acc_n_in) / 2  # shared wall of the back-to-back rows
    room_bank_h(c0, c1, acc_n_in, mid_n, 4)  # rooms facing the access corridor
    room_bank_h(c0, c1, spine_n, mid_n, 4, back=False)  # facing the spine

    # --- South deep block: two dense ward clusters (false loop closures) -----
    mid_s = (spine_s + acc_s_in) / 2
    for x0, x1 in ((b0, b1), (c0, c1)):
        room_bank_h(x0, x1, acc_s_in, mid_s, 4)
        room_bank_h(x0, x1, spine_s, mid_s, 4, back=False)

    # --- South shallow ward rows (against the south perimeter) ---------------
    room_bank_h(b0, b1, acc_s_out, -HALF_Y, 4, back=False)
    room_bank_h(c0, c1, acc_s_out, -HALF_Y, 4, back=False)

    # --- West and east wings: rooms facing the cross corridors ---------------
    # Each wing is broken into the same y-blocks by the horizontal corridors.
    wing_blocks = [
        (acc_n_out, HALF_Y, 1),  # north shallow
        (spine_n, acc_n_in, 2),  # north deep
        (acc_s_in, spine_s, 2),  # south deep
        (-HALF_Y, acc_s_out, 1),  # south shallow
    ]
    for y_lo, y_hi, n in wing_blocks:
        room_bank_v(y_lo, y_hi, vx_w_out, -HALF_X, n, back=False)  # west wing
        room_bank_v(y_lo, y_hi, vx_e_out, HALF_X, n, back=False)  # east wing

    # --- Corridor clutter: a handful of carts, clear of the origin -----------
    for cx, cy in [
        (-VERT_X, 6.0),
        (VERT_X, -6.0),
        (-8.0, -ACCESS_Y),
        (8.0, ACCESS_Y),
        (VERT_X, 9.0),
        (-VERT_X, -9.0),
    ]:
        furniture(cx, cy, 0.5, 0.5)

    # --- Task-layer zones (fetch pickup/dropoff/home; see mote_tasks) --------
    # pickup: in the waiting hall, facing the reception desk. dropoff: the far
    # north-east shallow ward, so a fetch crosses the spine, a cross corridor,
    # and an access corridor. home: the spawn point in the spine corridor.
    zone("pickup", (b0 + b1) / 2, (spine_n + acc_n_in) / 2 - 1.5, math.pi / 2)
    zone("dropoff", c0 + (c1 - c0) * 7 / 8, acc_n_out + 1.55, math.pi / 2)
    zone("home", 0.0, 0.0)

    # --- Room zones (goto <name>; see mote_tasks.zones) ----------------------
    # Named rooms in the deep shallow-ward rows (5.75 m deep, so plenty of
    # clearance), placed just inside the doorway clear of the bed at 0.62 depth
    # and facing into the room. The footprint is the room's own walkable
    # rectangle, so "am I in the kitchen" follows the walls instead of a circle
    # that would spill into the neighbours. Geometry comes from the same
    # _edges() the room banks use, so both track a layout change.
    def _room(name, x0, x1, i, y_door, y_back, n=4):
        edges = _edges(x0, x1, n)
        inset = WALL_T / 2  # wall centrelines -> the faces the robot sees
        xa, xb = edges[i] + inset, edges[i + 1] - inset
        ya, yb = sorted((y_door, y_back))
        ya, yb = ya + inset, yb - inset
        into = 1 if y_back > y_door else -1
        zone(
            name,
            (xa + xb) / 2,
            y_door + 1.25 * into,
            into * math.pi / 2,
            [(xa, ya), (xb, ya), (xb, yb), (xa, yb)],
        )

    _room("kitchen", b0, b1, 0, acc_n_out, HALF_Y)
    _room("pharmacy", b0, b1, 2, acc_n_out, HALF_Y)
    _room("laboratory", b0, b1, 1, acc_s_out, -HALF_Y)
    _room("ward_east", c0, c1, 3, acc_s_out, -HALF_Y)


def waiting_hall(x0, x1, y_spine, y_access):
    """An open hall with two doors each side, columns, and a reception desk."""
    doors = [x0 + (x1 - x0) * 0.3, x0 + (x1 - x0) * 0.7]
    wall(x0, y_spine, x1, y_spine, doors=doors)
    wall(x0, y_access, x1, y_access, doors=doors)
    wall(x0, y_spine, x0, y_access)
    wall(x1, y_spine, x1, y_access)
    _room_rect(x0, x1, y_spine, y_access)
    cy = (y_spine + y_access) / 2
    for cx in (x0 + (x1 - x0) * 0.33, x0 + (x1 - x0) * 0.66):
        for dy in (-2.0, 2.0):
            furniture(cx, cy + dy, 0.4, 0.4)  # columns
    # U-shaped reception desk near the hall centre.
    dx = (x0 + x1) / 2
    furniture(dx, cy, 2.4, 0.4)
    furniture(dx - 1.2, cy + 0.7, 0.4, 1.4)
    furniture(dx + 1.2, cy + 0.7, 0.4, 1.4)


HEADER = """<?xml version="1.0"?>
<!-- GENERATED by gen_hospital.py -- edit that script and regenerate, do not
     hand-edit this file.

     The "hard" Mote world: a ~58x38 m hospital with a looping corridor grid
     (a spine through the origin, parallel access corridors, and cross
     corridors), ~50 rooms off them, and furniture clutter.

     It stacks the failure modes the easier worlds isolate:
       * aperture-problem localisation drift on corridors longer than the
         ~12 m lidar range;
       * false loop closures from dense clusters of near-identical wards;
       * and adds genuine navigable loops, so SLAM must get real loop closures
         right and Nav2 has path choices.

     Walls are full height (2.5 m) so the ~5 cm lidar cannot see over them, and
     all furniture is floor-standing (spans the scan plane) so it is actually
     visible to the lidar. The robot spawns at the origin, left clear in the
     spine corridor. -->
<sdf version="1.8">
  <world name="hospital_world">
    <physics name="default_physics" type="dartsim">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>70 50</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>70 50</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def _box_pair(idx, cx, cy, cz, sx, sy, sz, material=False):
    size = f"{sx:g} {sy:g} {sz:g}"
    pose = f"{cx:g} {cy:g} {cz:g} 0 0 0"
    mat = (
        f"          <material><ambient>{MAT}</ambient><diffuse>{MAT}</diffuse></material>\n"
        if material
        else ""
    )
    return (
        f'        <collision name="c{idx}">\n'
        f"          <pose>{pose}</pose>\n"
        f"          <geometry><box><size>{size}</size></box></geometry>\n"
        f"        </collision>\n"
        f'        <visual name="v{idx}">\n'
        f"          <pose>{pose}</pose>\n"
        f"          <geometry><box><size>{size}</size></box></geometry>\n"
        f"{mat}"
        f"        </visual>\n"
    )


def render():
    parts = [HEADER]
    parts.append(
        '    <model name="building">\n      <static>true</static>\n      <link name="link">\n'
    )
    for i, (cx, cy, sx, sy) in enumerate(_walls):
        parts.append(_box_pair(i, cx, cy, WALL_H / 2, sx, sy, WALL_H, material=True))
    parts.append("      </link>\n    </model>\n")

    parts.append(
        '    <model name="furniture">\n      <static>true</static>\n      <link name="link">\n'
    )
    for i, (cx, cy, sx, sy) in enumerate(_furniture):
        parts.append(_box_pair(i, cx, cy, FURN_H / 2, sx, sy, FURN_H))
    parts.append("      </link>\n    </model>\n")

    parts.append(FOOTER)
    return "".join(parts)


def render_zones():
    lines = [
        "# GENERATED by gen_hospital.py -- edit that script and regenerate, do\n"
        "# not hand-edit this file. Named zones for hospital_world.sdf, placed by\n"
        "# the generator (and checked for wall clearance) so they stay valid when\n"
        "# the layout changes. Zones with a polygon are room footprints (goto\n"
        "# targets, outlined by the room walls); the rest are bare fetch\n"
        "# waypoints. The pose is the doorway approach, not the room centre --\n"
        "# a bed sits in the middle of each ward.\n"
        "frame_id: map\n"
        "zones:\n"
    ]
    for name, (x, y, yaw, polygon) in _zones.items():
        pose = f"x: {round(x, 3):g}, y: {round(y, 3):g}, yaw: {round(yaw, 3):g}"
        if polygon is None:
            lines.append(f"  {name}: {{{pose}}}\n")
            continue
        verts = ", ".join(f"[{round(vx, 3):g}, {round(vy, 3):g}]" for vx, vy in polygon)
        lines.append(f"  {name}: {{{pose},\n    polygon: [{verts}]}}\n")
    return "".join(lines)


def render_rooms():
    lines = [
        "# GENERATED by gen_hospital.py -- edit that script and regenerate, do\n"
        "# not hand-edit this file. Every enclosed room of hospital_world.sdf as\n"
        "# its walkable rectangle (x0, y0, x1, y1, metres, wall faces not\n"
        "# centrelines). This is ground truth for scoring map room segmentation\n"
        "# (mote_bringup.map_cleanup.room_segmentation) against a SLAM map of\n"
        "# this world -- see mote_simulation/test/room_segmentation_eval.py.\n"
        "# Corridors are not rooms and are not listed.\n"
        "rooms:\n"
    ]
    for x0, y0, x1, y1 in _rooms:
        vals = ", ".join(f"{round(v, 3):g}" for v in (x0, y0, x1, y1))
        lines.append(f"  - [{vals}]\n")
    return "".join(lines)


def main():
    build_world()
    _check_zone_clearance()
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "hospital_world.sdf")
    with open(out, "w") as f:
        f.write(render())
    zones_out = os.path.join(here, "hospital_world.zones.yaml")
    with open(zones_out, "w") as f:
        f.write(render_zones())
    rooms_out = os.path.join(here, "hospital_world.rooms.yaml")
    with open(rooms_out, "w") as f:
        f.write(render_rooms())
    print(
        f"wrote {out}: {len(_walls)} wall segments, {len(_furniture)} furniture boxes"
    )
    rooms = sum(1 for v in _zones.values() if v[-1] is not None)
    print(f"wrote {zones_out}: {len(_zones)} zones ({rooms} with a footprint)")
    print(f"wrote {rooms_out}: {len(_rooms)} rooms")


if __name__ == "__main__":
    main()
