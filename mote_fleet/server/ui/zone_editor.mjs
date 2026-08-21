// Zone editing on a map revision: drag a vertex, drag a zone, place a pose,
// rename, name the kind, add aliases, add, delete — then save the lot as a
// *candidate* revision.
//
// The editor never writes to the revision it is looking at. Saving POSTs the
// edited set to the server, which derives a new candidate from that revision
// (same map bytes, new zones); the operator then promotes it exactly like a
// robot-published map. Stored revisions stay immutable, which the announced
// digests rely on, and promotion stays the only write that changes a floor.
//
// It lives in the review pane (`review.mjs`) because a zone is a coordinate in
// one map frame: the map under the zones has to be the map they belong to, and
// only that pane draws a *candidate's* own map. Editing a candidate is the
// point — a fresh build arrives with `zone_01`..`zone_07` from `segment-map`,
// and an editor that could only edit the published map would have required
// promoting those placeholder names in order to be allowed to fix them.
//
// Geometry and the vocabulary rules live in pure functions over zone objects in
// *world* metres, so every edit operation is testable under node with no canvas
// and no DOM.

import { pixelToWorld, worldToPixel } from './map.mjs';

// Ray-cast membership over [[x, y], ...]. Concave polygons are fine, which
// matters because the hallway is one.
export function pointInPolygon(polygon, x, y) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i, i += 1) {
    const [xi, yi] = polygon[i];
    const [xj, yj] = polygon[j];
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

export function nearestVertex(zone, x, y) {
  if (!zone.polygon) return null;
  let best = null;
  zone.polygon.forEach(([vx, vy], index) => {
    const distance = Math.hypot(vx - x, vy - y);
    if (!best || distance < best.distance) best = { index, distance };
  });
  return best;
}

function segmentDistance(x, y, [ax, ay], [bx, by]) {
  const dx = bx - ax;
  const dy = by - ay;
  const lengthSq = dx * dx + dy * dy;
  const t = lengthSq ? Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / lengthSq)) : 0;
  return Math.hypot(ax + t * dx - x, ay + t * dy - y);
}

// The edge nearest a point; `index` is the vertex the edge *starts* at, which
// is where an inserted vertex goes (after it).
export function nearestEdge(zone, x, y) {
  if (!zone.polygon || zone.polygon.length < 2) return null;
  let best = null;
  for (let i = 0; i < zone.polygon.length; i += 1) {
    const j = (i + 1) % zone.polygon.length;
    const distance = segmentDistance(x, y, zone.polygon[i], zone.polygon[j]);
    if (!best || distance < best.distance) best = { index: i, distance };
  }
  return best;
}

// What the pointer is over, in the order a drag claims it: a vertex first (it
// is the smallest target and sits on top of its own zone), then a pose cross,
// then the zone's interior. `null` means the map — which is why the same
// function draws the hover: an operator who cannot tell a vertex drag from a
// zone drag from a map pan is guessing, and this is the one answer all three
// read.
export function hitTest(zones, x, y, reach) {
  for (const zone of zones) {
    const vertex = zone.polygon ? nearestVertex(zone, x, y) : null;
    if (vertex && vertex.distance <= reach) {
      return { kind: 'vertex', zone: zone.name, index: vertex.index };
    }
  }
  for (const zone of zones) {
    if (typeof zone.x !== 'number') continue;
    // A pose is a cross rather than a corner to aim at, so it is given a
    // little more reach than a vertex.
    if (Math.hypot(zone.x - x, zone.y - y) <= reach * 1.2) {
      return { kind: 'pose', zone: zone.name };
    }
  }
  for (const zone of zones) {
    if (zone.polygon && pointInPolygon(zone.polygon, x, y)) {
      return { kind: 'zone', zone: zone.name };
    }
  }
  return null;
}

// The cursor for a target — the half of the answer that arrives before the
// pointer has touched anything.
export function cursorFor(target, placing = false) {
  if (placing) return 'crosshair';
  if (!target) return ''; // the stylesheet's `grab`: this drag pans the map
  return target.kind === 'vertex' ? 'crosshair' : 'move';
}

export function withVertex(zone, index, x, y) {
  const polygon = zone.polygon.map((point, i) =>
    i === index ? [round(x), round(y)] : point,
  );
  return { ...zone, polygon };
}

export function withInsertedVertex(zone, afterIndex, x, y) {
  const polygon = zone.polygon.slice();
  polygon.splice(afterIndex + 1, 0, [round(x), round(y)]);
  return { ...zone, polygon };
}

// A polygon needs three vertices to enclose anything; refuse rather than
// letting a delete quietly produce a line.
export function withoutVertex(zone, index) {
  if (!zone.polygon || zone.polygon.length <= 3) return null;
  return { ...zone, polygon: zone.polygon.filter((_, i) => i !== index) };
}

// Moving a zone moves its footprint and its pose together: they name the same
// place, and dragging a room away from its own goto target is never the intent.
export function translated(zone, dx, dy) {
  const moved = { ...zone };
  if (zone.polygon) {
    moved.polygon = zone.polygon.map(([x, y]) => [round(x + dx), round(y + dy)]);
  }
  if (typeof zone.x === 'number') moved.x = round(zone.x + dx);
  if (typeof zone.y === 'number') moved.y = round(zone.y + dy);
  return moved;
}

export function withPose(zone, x, y) {
  return { ...zone, x: round(x), y: round(y) };
}

// A new zone arrives as a rectangle at the view centre with the first free
// generated name; the operator renames it to what the place is called.
export function freshZone(existing, cx, cy, half = 1.0) {
  const names = new Set(existing.map((zone) => zone.name));
  let n = 1;
  while (names.has(`zone_${String(n).padStart(2, '0')}`)) n += 1;
  return {
    name: `zone_${String(n).padStart(2, '0')}`,
    kind: 'room',
    x: round(cx),
    y: round(cy),
    yaw: 0.0,
    polygon: [
      [round(cx - half), round(cy - half)],
      [round(cx + half), round(cy - half)],
      [round(cx + half), round(cy + half)],
      [round(cx - half), round(cy + half)],
    ],
  };
}

// A default outline, one metre either side of the pose, for a zone that has
// just been called a room and has no extent yet. It is a starting shape to drag
// onto the walls, not a guess at the room.
function squareAround(zone, map, half = 1.0) {
  // On the grid, like every other coordinate this editor writes. The *pose* is
  // left where it is — it was measured by driving a robot there, and moving it
  // two centimetres to tidy a number would be inventing data — but the outline
  // is this editor's own, so it starts where the next drag would put it.
  const corner = (x, y) => {
    const point = snapToPixel(map, x, y);
    return [point.x, point.y];
  };
  return [
    corner(zone.x - half, zone.y - half),
    corner(zone.x + half, zone.y - half),
    corner(zone.x + half, zone.y + half),
    corner(zone.x - half, zone.y + half),
  ];
}

// A pose for a zone that has only an outline. The robot's loader derives one
// the same way when it loads a polygon-only zone; here it is needed when an
// outline is about to be dropped, so that the zone is left with a position at
// all. Concave outlines whose centroid falls outside them get `null`, and the
// caller refuses the change rather than inventing a pose in a wall.
export function poseFor(zone) {
  if (typeof zone.x === 'number' && typeof zone.y === 'number') {
    return { x: zone.x, y: zone.y };
  }
  if (!zone.polygon || !zone.polygon.length) return null;
  const sum = zone.polygon.reduce((acc, [x, y]) => [acc[0] + x, acc[1] + y], [0, 0]);
  const centre = { x: round(sum[0] / zone.polygon.length), y: round(sum[1] / zone.polygon.length) };
  return pointInPolygon(zone.polygon, centre.x, centre.y) ? centre : null;
}

// **The kind decides whether a zone is a point or an area, and the geometry
// follows it.** A `charger` is a pose to dock at; a `room` is a place with
// walls, and "am I in it" is the question it exists to answer. Editing them as
// two independent things — a kind here, an outline toggled over there — is what
// leaves a `dropoff` carrying a seven-vertex outline nothing reads, and a room
// with no extent that `zones.containing` can never match.
//
// So: naming a bare pose an area gives it a starting outline to drag onto the
// walls, and naming an outlined zone a point drops the outline and keeps the
// pose. Returns `null` if that second move would leave the zone with no
// position at all, which is a zone the robot's loader refuses.
export function withKind(zone, kind, map = null) {
  const next = { ...zone, kind };
  if (isAreaKind(kind)) {
    if (!next.polygon && typeof next.radius !== 'number' && typeof next.x === 'number') {
      next.polygon = squareAround(next, map);
    }
    return next;
  }
  if (!next.polygon && typeof next.radius !== 'number') return next;
  const pose = poseFor(next);
  if (!pose) return null;
  delete next.polygon;
  delete next.radius;
  return { ...next, x: pose.x, y: pose.y };
}

// The map's own pixel grid, in world metres. A vertex dropped anywhere inside a
// pixel covers exactly the same cells as one at its centre, so the free-hand
// coordinate is precision the map does not have — and two zones meant to share
// a wall end up a few millimetres apart, differently each time. Snapping to
// centres (half a pixel off the origin, which is a pixel *edge*) makes "the
// same place" the same number.
export function snapToPixel(map, x, y) {
  if (!map || !map.resolution) return { x: round(x), y: round(y) };
  const axis = (value, origin) =>
    round(origin + (Math.floor((value - origin) / map.resolution) + 0.5) * map.resolution);
  return { x: axis(x, map.origin[0]), y: axis(y, map.origin[1]) };
}

// A whole number of pixels, for dragging a zone bodily: snapping each vertex
// would pull the shape about, and snapping the *movement* keeps it rigid — a
// room traced onto its walls stays traced when it is nudged.
export function snapDelta(map, delta) {
  if (!map || !map.resolution) return delta;
  return round(Math.round(delta / map.resolution) * map.resolution);
}

// A machine name a dispatcher can type — the same rule the robot's loader and
// the bundle validator enforce, applied here so a bad rename fails in the
// input rather than at save.
export const NAME_RE = /^[a-z][a-z0-9_]*$/;

// zone/v0's kinds, in the spec's order. Mirrored from `bundle.ZONE_KINDS`
// rather than fetched, because this is a `<select>`'s options and a dropdown
// that cannot be drawn until a request comes back is a worse thing than a list
// held in step by a test (`ui_test.mjs` reads bundle.py and compares).
export const ZONE_KINDS = [
  'area',
  'room',
  'corridor',
  'doorway',
  'threshold',
  'elevator',
  'stair',
  'dock',
  'charger',
  'pickup',
  'dropoff',
  'staging',
  'home',
  'keepout',
  'slow',
];

// Kinds that say where a robot may not go, and so are not destinations.
export const CONSTRAINT_KINDS = new Set(['keepout', 'slow']);

// Kinds that name a *pose* rather than a region — a charger is where the robot
// docks, not somewhere it may be anywhere inside of. Mirrors
// `bundle.POINT_KINDS`, which carries the reasoning.
export const POINT_KINDS = new Set(['dock', 'charger', 'pickup', 'dropoff', 'home']);

export const isAreaKind = (kind) => !POINT_KINDS.has(kind || 'area');

// Whether a zone of this kind may be dispatched to, absent an explicit say-so.
export const navigableByDefault = (kind) => !CONSTRAINT_KINDS.has(kind || 'area');

// A comma-separated field, which is how an operator writes a short list
// without a list widget — aliases and tags are both this. Blank entries are
// dropped and a spelling repeated in one field is kept once: a duplicate inside
// a single zone is a typo, not the collision `ambiguities` is about.
export function parseList(text) {
  const seen = new Set();
  const aliases = [];
  for (const raw of String(text || '').split(',')) {
    const alias = raw.trim();
    if (!alias) continue;
    const key = normaliseAlias(alias);
    if (seen.has(key)) continue;
    seen.add(key);
    aliases.push(alias);
  }
  return aliases;
}

export function formatList(items) {
  return (items || []).join(', ');
}

// The comparison zone/v0's resolver uses, and therefore the only one collision
// detection may use: case-insensitive and whitespace-normalised, matching
// `bundle.normalise_alias`.
export function normaliseAlias(text) {
  return String(text).split(/\s+/).filter(Boolean).join(' ').toLowerCase();
}

// Two zones answering one query, which the robot's loader *refuses* to load —
// so an editor that can produce one produces a map no robot will take. Mirrors
// `bundle.ambiguities`: names and aliases, never display names.
export function ambiguities(zones) {
  const claimed = new Map();
  const problems = [];
  for (const zone of zones) {
    for (const spelling of [zone.name, ...(zone.aliases || [])]) {
      const key = normaliseAlias(spelling);
      if (!key) continue;
      const owner = claimed.get(key);
      if (owner !== undefined && owner !== zone.name) {
        problems.push(`"${owner}" and "${zone.name}" both answer to "${key}"`);
      } else {
        claimed.set(key, zone.name);
      }
    }
  }
  return problems;
}

// The wire shape: keyed by name, no echoed `name` field, no empty extras.
//
// `navigable` is dropped when it is what the kind already implies, which is
// what makes the kind editable at all: every zone arrives from the server with
// the field filled in (`bundle.zone_term` defaults it), so writing it back
// verbatim would carry a `keepout`'s `navigable: false` onto a zone just
// changed to `room` and quietly leave a room nothing can be dispatched to. A
// value that genuinely deviates from its kind is kept, because that one was
// meant.
export function zonesPayload(zones) {
  const payload = {};
  for (const zone of zones) {
    const entry = {};
    for (const [key, value] of Object.entries(zone)) {
      if (key === 'name' || value === null || value === undefined || value === '') continue;
      if (Array.isArray(value) && value.length === 0) continue;
      if (key === 'navigable' && value === navigableByDefault(zone.kind)) continue;
      entry[key] = value;
    }
    payload[zone.name] = entry;
  }
  return payload;
}

const round = (value) => Math.round(value * 1000) / 1000;

function el(tag, attributes = {}) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attributes)) {
    if (key === 'class') node.className = value;
    else if (key === 'text') node.textContent = value;
    else node.setAttribute(key, value);
  }
  return node;
}

// One labelled control. The label is a real `<label>` so that clicking it
// reaches the input, which on a phone is most of the target.
function field(name, control) {
  const row = document.createElement('label');
  row.className = 'zone-field';
  row.append(el('span', { class: 'zone-field-name', text: name }), control);
  return row;
}

const HANDLE = 'rgba(240, 130, 34, 1)';
const EDIT_STROKE = 'rgba(240, 130, 34, 0.9)';
const EDIT_FILL = 'rgba(240, 130, 34, 0.08)';
const SELECTED_FILL = 'rgba(240, 130, 34, 0.18)';
// Hover is deliberately louder than selection: selection says which row you are
// looking at, hover says what the next press will move — and only one of those
// is about to change the map.
const HOVER_FILL = 'rgba(240, 130, 34, 0.3)';
// Ink, not white. A canvas gets no cascade, so the theme cannot supply this —
// and the surface under it is not the theme's background but the *basemap*,
// whose free space is white in both themes. A white ring was invisible on
// exactly the floor an operator is editing over (measured: it moved 1.5% of the
// pixels around the handle; this moves 12%).
const HOVER_RING = 'rgba(13, 17, 23, 0.9)';

export class ZoneEditor {
  constructor(mapView, dom, { onSave, onExit } = {}) {
    this.mapView = mapView;
    this.dom = dom; // { panel, rows, add, save, cancel, note }
    this.onSave = onSave || (() => {});
    this.onExit = onExit || (() => {});
    this.active = false;
    this.zones = [];
    this.selected = null;
    this._drag = null;
    // What a press here and now would grab. Kept because it is drawn, not
    // because it is needed to grab: `_down` re-runs the same hit test.
    this._hover = null;
    this._bind();
  }

  begin(zones) {
    this.zones = (zones || []).map((zone) => JSON.parse(JSON.stringify(zone)));
    this.selected = null;
    this.active = true;
    this._drag = null;
    this._placing = null;
    this._hover = null;
    this.dom.panel.hidden = false;
    this.dom.note.textContent = '';
    this.mapView.overlay = (ctx) => this._draw(ctx);
    this._render();
    this.mapView.draw();
  }

  end() {
    this.active = false;
    this._drag = null;
    this._placing = null;
    this._hover = null;
    this.selected = null;
    this._cursor('');
    this.dom.panel.hidden = true;
    this.mapView.overlay = null;
    this.mapView.draw();
  }

  // -- pointer editing ---------------------------------------------------

  _bind() {
    const canvas = this.mapView.canvas;
    // Capture phase, so an edit consumes the event before the map's own
    // pan/select handlers see it; a miss falls through and the map pans.
    canvas.addEventListener('pointerdown', (event) => this._down(event), true);
    canvas.addEventListener('pointermove', (event) => this._move(event), true);
    const up = (event) => this._up(event);
    canvas.addEventListener('pointerup', up, true);
    canvas.addEventListener('pointercancel', up, true);
    // Not in the capture phase and not stopped: leaving the canvas is the
    // map's business too, and all this does is drop a highlight.
    canvas.addEventListener('pointerleave', () => this._hoverAt(null));
    canvas.addEventListener('dblclick', (event) => this._dblclick(event), true);
  }

  _world(event) {
    const rect = this.mapView.canvas.getBoundingClientRect();
    const sx = event.clientX - rect.left;
    const sy = event.clientY - rect.top;
    const view = this.mapView.view;
    const px = (sx - view.tx) / view.scale;
    const py = (sy - view.ty) / view.scale;
    return pixelToWorld(this.mapView.map, px, py);
  }

  // Where a press *writes*, as against where it landed: on the pixel grid,
  // unless Shift is held. Shift rather than Alt because a desktop's window
  // manager takes Alt-drag for moving windows, and a modifier the page never
  // receives is a modifier that does not exist.
  _point(event) {
    const world = this._world(event);
    if (event.shiftKey) return world;
    return snapToPixel(this.mapView.map, world.x, world.y);
  }

  // Hit reach in metres at the current zoom: 10 screen px, whatever the scale.
  _reach() {
    const view = this.mapView.view;
    return (10 / view.scale) * this.mapView.map.resolution;
  }

  _down(event) {
    if (!this.active || !this.mapView.map) return;
    // The hit test asks where the pointer *is*; every write below asks where it
    // should land, which is `_point`.
    const point = this._world(event);
    const reach = this._reach();

    // Placing a pose: the next click *is* the pose, wherever it lands, so it
    // takes precedence over every hit test below. This is the one way to give
    // a pose to a zone that has none — a `segment-map` room is a polygon with
    // no `x`/`y`, so it draws no cross to drag and the robot derives a
    // centroid to drive to, which lands wherever the outline's middle is
    // rather than where you would send a robot in that room.
    if (this._placing) {
      const name = this._placing;
      this._placing = null;
      event.stopPropagation();
      event.preventDefault();
      const placed = this._point(event);
      this._update(name, (zone) => withPose(zone, placed.x, placed.y));
      this.selected = name;
      this.note('');
      this._cursor(cursorFor(this._hover, false));
      this._render();
      this.mapView.draw();
      return;
    }

    const target = hitTest(this.zones, point.x, point.y, reach);
    if (target) {
      this._grab(
        event,
        target.kind === 'zone'
          ? { ...target, from: point, before: this.zones.find((z) => z.name === target.zone) }
          : target,
      );
      return;
    }
    this.selected = null;
    this._render();
    this.mapView.draw();
    // Nothing hit: let the map pan. Which is what the cursor has been saying
    // since the pointer arrived here — a press that pans when the operator
    // meant to drag a zone is the surprise the hover exists to prevent.
  }

  // The hover, and the cursor that goes with it. Only a *change* redraws: this
  // runs on every pointer move over the canvas, and the map underneath is a
  // full repaint.
  _hoverAt(target) {
    const before = this._hover;
    const same =
      (!before && !target) ||
      (before &&
        target &&
        before.kind === target.kind &&
        before.zone === target.zone &&
        before.index === target.index);
    this._hover = target;
    this._cursor(cursorFor(target, Boolean(this._placing)));
    if (!same) this.mapView.draw();
  }

  _cursor(value) {
    this.mapView.canvas.style.cursor = value;
  }

  _grab(event, drag) {
    event.stopPropagation();
    event.preventDefault();
    this.mapView.canvas.setPointerCapture(event.pointerId);
    this._drag = drag;
    this.selected = drag.zone;
    this._render();
    this.mapView.draw();
  }

  _move(event) {
    if (!this.active || !this.mapView.map) return;
    if (!this._drag) {
      const point = this._world(event);
      this._hoverAt(hitTest(this.zones, point.x, point.y, this._reach()));
      return;
    }
    event.stopPropagation();
    const drag = this._drag;
    if (drag.kind === 'vertex' || drag.kind === 'pose') {
      const point = this._point(event);
      this._update(drag.zone, (zone) =>
        drag.kind === 'vertex'
          ? withVertex(zone, drag.index, point.x, point.y)
          : withPose(zone, point.x, point.y),
      );
    } else {
      // Measured from the grab, against the zone as it was then: rounding each
      // step's delta instead would leave the zone drifting behind the pointer
      // by whatever the rounding threw away.
      const point = this._world(event);
      let dx = point.x - drag.from.x;
      let dy = point.y - drag.from.y;
      if (!event.shiftKey) {
        dx = snapDelta(this.mapView.map, dx);
        dy = snapDelta(this.mapView.map, dy);
      }
      this._update(drag.zone, () => translated(drag.before, dx, dy));
    }
    this.mapView.draw();
  }

  _up(event) {
    if (!this.active || !this._drag) return;
    event.stopPropagation();
    this._drag = null;
  }

  _dblclick(event) {
    if (!this.active || !this.mapView.map) return;
    const point = this._world(event);
    const placed = this._point(event);
    const reach = this._reach();
    for (const zone of this.zones) {
      if (!zone.polygon) continue;
      const vertex = nearestVertex(zone, point.x, point.y);
      if (vertex && vertex.distance <= reach) {
        const trimmed = withoutVertex(zone, vertex.index);
        if (trimmed) {
          event.stopPropagation();
          this._update(zone.name, () => trimmed);
          this.mapView.draw();
        }
        return;
      }
      const edge = nearestEdge(zone, point.x, point.y);
      if (edge && edge.distance <= reach) {
        event.stopPropagation();
        this._update(zone.name, (z) => withInsertedVertex(z, edge.index, placed.x, placed.y));
        this.mapView.draw();
        return;
      }
    }
  }

  _update(name, transform) {
    this.zones = this.zones.map((zone) => (zone.name === name ? transform(zone) : zone));
  }

  // -- the panel ----------------------------------------------------------

  addZone() {
    const view = this.mapView.view;
    const rect = this.mapView.canvas.getBoundingClientRect();
    const middle = pixelToWorld(this.mapView.map, (rect.width / 2 - view.tx) / view.scale, (rect.height / 2 - view.ty) / view.scale);
    const centre = snapToPixel(this.mapView.map, middle.x, middle.y);
    const zone = freshZone(this.zones, centre.x, centre.y);
    this.zones = [...this.zones, zone];
    this.selected = zone.name;
    this._render();
    this.mapView.draw();
  }

  payload() {
    return zonesPayload(this.zones);
  }

  // Arm the next map click as this zone's pose. Nothing is changed until that
  // click, so arming and thinking better of it costs nothing.
  placePose(name) {
    this._placing = this._placing === name ? null : name;
    this.selected = name;
    this.note(this._placing ? `click the map to place ${name}’s pose` : '');
    this._cursor(cursorFor(this._hover, Boolean(this._placing)));
    this._render();
    this.mapView.draw();
  }

  note(text, bad = false) {
    this.dom.note.textContent = text;
    this.dom.note.className = `note ${bad ? 'error' : ''}`;
  }

  // Every rename is checked against the same rules the robot's loader enforces
  // — the name shape, and two zones answering one query — because the loader
  // *refuses* an ambiguous vocabulary rather than resolving it by dict order.
  // A set this editor is willing to save is a set a robot will load.
  problems() {
    const seen = new Set();
    for (const zone of this.zones) {
      if (!NAME_RE.test(zone.name)) return `"${zone.name}" is not a machine name (lowercase, a-z0-9_)`;
      if (seen.has(zone.name)) return `two zones named "${zone.name}"`;
      seen.add(zone.name);
    }
    const ambiguous = ambiguities(this.zones);
    if (ambiguous.length) return `${ambiguous[0]} — a query matching both cannot be answered`;
    if (!this.zones.length) return 'no zones — cancel to leave this revision’s zones as they are';
    return null;
  }

  _render() {
    this._renderRows();
    this._renderDetail();
  }

  _renderRows() {
    const rows = this.dom.rows;
    rows.replaceChildren();
    for (const zone of this.zones) {
      const row = document.createElement('div');
      const chosen = zone.name === this.selected;
      row.className = 'zone-row' + (chosen ? ' selected' : '');
      // The name is what you *pick* the zone by, so it is a button and looks
      // like one being pressed. It was a text input, which put a caret where a
      // click was meant to select — the row is a list, and renaming is a
      // deliberate act that belongs with the zone's other fields.
      const name = document.createElement('button');
      name.type = 'button';
      name.className = 'zone-name';
      name.textContent = zone.name;
      name.setAttribute('aria-pressed', String(chosen));
      name.title = `select ${zone.name}`;
      // The row carries what you scan *across* zones — which place, what sort of
      // place, what shape — and nothing else. Every other field belongs to one
      // zone at a time and edits in the panel below, which is what lets zone/v0
      // grow a field without costing every row a column (and the operator a
      // text box they will not fill).
      const kind = document.createElement('select');
      kind.title = 'what kind of place this is (zone/v0)';
      for (const option of ZONE_KINDS) {
        const node = document.createElement('option');
        node.value = option;
        node.textContent = option;
        kind.append(node);
      }
      kind.value = zone.kind || 'area';
      kind.title =
        'what kind of place this is — a point (charger, pickup…) or an area ' +
        '(room, corridor, keepout…), which is what decides whether it has an ' +
        'outline';
      kind.addEventListener('change', () => {
        const reshaped = withKind(zone, kind.value, this.mapView.map);
        if (!reshaped) {
          kind.value = zone.kind || 'area';
          this.note(
            `${zone.name} is an outline with no pose inside it — place one with ⌖ first`,
            true,
          );
          return;
        }
        const lost = Boolean((zone.polygon || zone.radius) && !reshaped.polygon && !reshaped.radius);
        this._update(zone.name, () => reshaped);
        this.selected = zone.name;
        this.note(
          lost ? `${zone.name} is a ${kind.value}: a pose, so its outline is gone` : '',
        );
        this._render();
        this.mapView.draw();
      });
      const placed = typeof zone.x === 'number';
      // Only for a zone with no pose to drag. Everything else on the map is
      // moved by dragging it, and a button that duplicates a drag is a second
      // way to do one thing; this is the case dragging cannot reach, because
      // a `segment-map` room is an outline with no `x`/`y` and so draws no
      // cross to take hold of. Once placed, the cross is the control.
      const pose = document.createElement(placed ? 'span' : 'button');
      pose.className = 'place';
      if (!placed) {
        pose.type = 'button';
        pose.textContent = '⌖';
        if (this._placing === zone.name) pose.classList.add('armed');
        pose.title = 'this zone has no pose — click here, then click the map';
        pose.addEventListener('click', (event) => {
          event.stopPropagation();
          this.placePose(zone.name);
        });
      }
      const del = document.createElement('button');
      del.type = 'button';
      del.textContent = '×';
      del.title = 'delete zone';
      del.addEventListener('click', () => {
        this.zones = this.zones.filter((z) => z.name !== zone.name);
        if (this.selected === zone.name) this.selected = null;
        if (this._placing === zone.name) this._placing = null;
        this._render();
        this.mapView.draw();
      });
      row.addEventListener('click', () => {
        if (this.selected !== zone.name) {
          this.selected = zone.name;
          this._render();
          this.mapView.draw();
        }
      });
      row.append(name, kind, pose, del);
      rows.append(row);
    }
  }

  // -- the selected zone --------------------------------------------------

  // Everything a zone carries that is not its identity or its shape: one zone
  // at a time, as a form. zone/v0 already has seven vocabulary fields and will
  // have more; a column each would make the list unreadable long before the
  // spec ran out, and would put a paragraph-wide text box on every row for a
  // field most zones leave empty.
  _renderDetail() {
    const panel = this.dom.detail;
    if (!panel) return;
    const zone = this.zones.find((entry) => entry.name === this.selected);
    panel.replaceChildren();
    if (!zone) {
      panel.append(
        field('', el('p', { class: 'dim', text: 'select a zone to name it' })),
      );
      return;
    }
    // Renaming is here rather than in the row for the same reason the row
    // stopped being a form: in the list, the name is what you select by. A
    // rename is a deliberate act on the zone you have selected, and this is
    // where that zone's fields are.
    const rename = document.createElement('input');
    rename.className = 'zone-rename';
    rename.value = zone.name;
    rename.title = 'the machine name goto takes — lowercase, a-z0-9_';
    rename.addEventListener('change', () => {
      const renamed = rename.value.trim();
      this._update(zone.name, (z) => ({ ...z, name: renamed }));
      this.selected = renamed;
      this._render();
      this.mapView.draw();
    });

    const text = (key, placeholder, title) => {
      const input = document.createElement('input');
      input.value = zone[key] || '';
      input.placeholder = placeholder;
      input.title = title;
      input.addEventListener('change', () => {
        this._update(zone.name, (z) => ({ ...z, [key]: input.value.trim() }));
      });
      return input;
    };

    const list = (key, placeholder, title) => {
      const input = document.createElement('input');
      input.value = formatList(zone[key]);
      input.placeholder = placeholder;
      input.title = title;
      input.addEventListener('change', () => {
        const parsed = parseList(input.value);
        input.value = formatList(parsed);
        this._update(zone.name, (z) => ({ ...z, [key]: parsed }));
      });
      return input;
    };

    // The box shows what the robot will do, which for a zone that has never
    // said is what its kind implies. Ticking it back to that is not a decision
    // that needs storing — `zonesPayload` drops it — so a `keepout` changed to
    // `room` becomes navigable rather than staying silently undispatchable.
    const navigable = document.createElement('input');
    navigable.type = 'checkbox';
    navigable.checked =
      typeof zone.navigable === 'boolean'
        ? zone.navigable
        : navigableByDefault(zone.kind);
    navigable.title = 'whether a robot may be dispatched here';
    navigable.addEventListener('change', () => {
      this._update(zone.name, (z) => ({ ...z, navigable: navigable.checked }));
      this._render();
    });

    // A picker rather than a text box: a parent must name a zone on this floor,
    // and typing one is the only way to name one that is not.
    const parent = document.createElement('select');
    parent.title = 'the zone this one is inside';
    parent.append(el('option', { value: '', text: '—' }));
    for (const other of this.zones) {
      if (other.name === zone.name) continue;
      parent.append(el('option', { value: other.name, text: other.name }));
    }
    parent.value = zone.parent || '';
    parent.addEventListener('change', () => {
      this._update(zone.name, (z) => ({ ...z, parent: parent.value }));
    });

    // No example placeholders. A grey "The Kitchen" in the display-name box of a
    // zone called `pickup` reads as a value that is already there, and the
    // label beside it already says what the field is; the only hint worth
    // giving is the one about punctuation.
    panel.append(
      field('name', rename),
      field('display name', text('display_name', '', 'what an operator reads')),
      field(
        'also called',
        list('aliases', 'comma separated', 'other spellings goto should accept'),
      ),
      field('navigable', navigable),
      field('inside', parent),
      field('tags', list('tags', 'comma separated', 'free labels for whatever needs them')),
      field('description', text('description', '', 'a note for whoever reads this next')),
    );
  }

  // -- drawing ------------------------------------------------------------

  _draw(ctx) {
    if (!this.active || !this.mapView.map) return;
    const view = this.mapView.view;
    const toScreen = (x, y) => {
      const pixel = worldToPixel(this.mapView.map, x, y);
      return { x: pixel.x * view.scale + view.tx, y: pixel.y * view.scale + view.ty };
    };
    const hover = this._hover;
    for (const zone of this.zones) {
      const selected = zone.name === this.selected;
      const over = hover && hover.zone === zone.name ? hover.kind : '';
      ctx.save();
      ctx.strokeStyle = EDIT_STROKE;
      ctx.fillStyle =
        over === 'zone' ? HOVER_FILL : selected ? SELECTED_FILL : EDIT_FILL;
      ctx.lineWidth = selected || over === 'zone' ? 2.5 : 1.5;
      if (zone.polygon) {
        ctx.beginPath();
        zone.polygon.forEach(([x, y], index) => {
          const point = toScreen(x, y);
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        zone.polygon.forEach(([x, y], index) => {
          const point = toScreen(x, y);
          const grabbed = over === 'vertex' && hover.index === index;
          const half = grabbed ? 6 : 4;
          ctx.fillStyle = HANDLE;
          ctx.fillRect(point.x - half, point.y - half, half * 2, half * 2);
          if (grabbed) {
            // A ring rather than only a bigger square: on a dark basemap the
            // square alone grows into the wall it is sitting on.
            ctx.strokeStyle = HOVER_RING;
            ctx.lineWidth = 2;
            ctx.strokeRect(point.x - half, point.y - half, half * 2, half * 2);
            ctx.strokeStyle = EDIT_STROKE;
          }
        });
      }
      if (typeof zone.x === 'number') {
        const point = toScreen(zone.x, zone.y);
        const grabbed = over === 'pose';
        ctx.strokeStyle = HANDLE;
        ctx.lineWidth = grabbed ? 3 : 2;
        ctx.beginPath();
        ctx.moveTo(point.x - 7, point.y);
        ctx.lineTo(point.x + 7, point.y);
        ctx.moveTo(point.x, point.y - 7);
        ctx.lineTo(point.x, point.y + 7);
        ctx.stroke();
        if (grabbed) {
          ctx.strokeStyle = HOVER_RING;
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(point.x, point.y, 9, 0, Math.PI * 2);
          ctx.stroke();
          ctx.strokeStyle = HANDLE;
        }
        ctx.font = '11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillStyle = HANDLE;
        ctx.fillText(zone.name, point.x, point.y - 10);
      }
      ctx.restore();
    }
  }
}
