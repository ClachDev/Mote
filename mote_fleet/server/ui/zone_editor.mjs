// Zone editing on a map revision: drag a vertex, drag a zone, place a pose,
// rename, write a note, add, delete — then save the lot as a *candidate*
// revision.
//
// **A zone is a place-name**: a human name bound to geometry. The record is the
// name, a free-text note for what the name cannot say, and where it is — so
// that is what the list and the details column carry, and there is nothing else
// to fill in. The semantics come from the mission layer's resolver, which
// already knows what a store room is; what it cannot know is that this
// building's store room is where the stationery lives.
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
// Geometry and the naming rules live in pure functions over zone objects in
// *world* metres, so every edit operation is testable under node with no canvas
// and no DOM.

import { pixelToWorld, worldToPixel, zoneLabel } from './map.mjs';
import { theme } from './theme.mjs';

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

// `source` records what made the zone: `save-zone` for a pose a robot was
// driven to and captured, `segment-map` for a room read off a saved map, and
// `editor` for geometry placed or dragged here. It is a note and nothing reads
// it to decide anything — a zone is a coordinate in the floor's frame however
// it got there — so what it buys is an operator being able to see which zones
// somebody drew.
export const EDITOR_SOURCE = 'editor';

// Any edit to a zone's geometry makes this editor what most recently placed it,
// so every geometry helper goes through here. A zone this edit does not touch
// keeps the source it arrived with, which is why nothing stamps the whole set
// on save.
export function sourced(zone) {
  return { ...zone, source: EDITOR_SOURCE };
}

export function withVertex(zone, index, x, y) {
  const polygon = zone.polygon.map((point, i) =>
    i === index ? [round(x), round(y)] : point,
  );
  return sourced({ ...zone, polygon });
}

export function withInsertedVertex(zone, afterIndex, x, y) {
  const polygon = zone.polygon.slice();
  polygon.splice(afterIndex + 1, 0, [round(x), round(y)]);
  return sourced({ ...zone, polygon });
}

// A polygon needs three vertices to enclose anything; refuse rather than
// letting a delete quietly produce a line.
export function withoutVertex(zone, index) {
  if (!zone.polygon || zone.polygon.length <= 3) return null;
  return sourced({ ...zone, polygon: zone.polygon.filter((_, i) => i !== index) });
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
  return sourced(moved);
}

export function withPose(zone, x, y) {
  return sourced({ ...zone, x: round(x), y: round(y) });
}

// A new zone arrives as a rectangle at the view centre with the first free
// generated name; the operator renames it to what the place is called.
export function freshZone(existing, cx, cy, half = 1.0) {
  const names = new Set(existing.map((zone) => zone.name));
  let n = 1;
  while (names.has(`zone_${String(n).padStart(2, '0')}`)) n += 1;
  return sourced({
    name: `zone_${String(n).padStart(2, '0')}`,
    x: round(cx),
    y: round(cy),
    yaw: 0.0,
    polygon: [
      [round(cx - half), round(cy - half)],
      [round(cx + half), round(cy - half)],
      [round(cx + half), round(cy + half)],
      [round(cx - half), round(cy + half)],
    ],
  });
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

// One zone's geometry in a phrase. Shown in the list whether or not the list is
// being edited — the shape is a fact about the zone, not a control.
//
// A zone is a *point* or an *area*, and that is geometry rather than a type it
// was declared to be: a zone has a pose, and it may also have an extent. The
// two words are the operator's, not the file format's — `polygon` and
// `waypoint` named the representation, which is the one thing about a place
// nobody standing in it needs to know.
export function zoneSummary(zone) {
  if (zone.polygon && zone.polygon.length >= 3) {
    return `area · ${zone.polygon.length} corners`;
  }
  if (typeof zone.radius === 'number') return `area · r ${zone.radius.toFixed(2)} m`;
  if (typeof zone.x === 'number' && typeof zone.y === 'number') {
    return `point ${zone.x.toFixed(2)}, ${zone.y.toFixed(2)}`;
  }
  return 'no position';
}

// A name anyone can type, which is the only rule a place-name has: printable
// text, and no leading or trailing space to make two names look identical and
// resolve differently. It mirrors `zone.ZONE_NAME_RE`, so a bad rename fails in
// the input rather than at save.
export const NAME_RE = /^(?!\s)[^\x00-\x1f\x7f]+(?<!\s)$/;

// The comparison zone/v0's resolver uses, and therefore the only one collision
// detection may use: case-insensitive and whitespace-normalised, matching
// `bundle.normalise_name`.
export function normaliseName(text) {
  return String(text).split(/\s+/).filter(Boolean).join(' ').toLowerCase();
}

// Two zones answering one query, which the robot's loader *refuses* to load —
// so an editor that can produce one produces a map no robot will take. Mirrors
// `bundle.ambiguities`. With one name per zone and no aliases, the only way to
// make one is to call two places the same thing.
export function ambiguities(zones) {
  const claimed = new Map();
  const problems = [];
  for (const zone of zones) {
    const key = normaliseName(zone.name);
    if (!key) continue;
    const owner = claimed.get(key);
    if (owner !== undefined && owner !== zone.name) {
      problems.push(`"${owner}" and "${zone.name}" both answer to "${key}"`);
    } else {
      claimed.set(key, zone.name);
    }
  }
  return problems;
}

// The wire shape: keyed by name, no echoed `name` field, no empty extras.
//
// `navigable` now travels verbatim. It used to be dropped when it agreed with
// the zone's kind, because a kind the operator had just changed would otherwise
// have carried the old kind's default with it; with no kind there is nothing
// for it to disagree with, and a flag that says a robot must not be sent here
// is not one to infer.
export function zonesPayload(zones) {
  const payload = {};
  for (const zone of zones) {
    const entry = {};
    for (const [key, value] of Object.entries(zone)) {
      if (key === 'name' || value === null || value === undefined || value === '') continue;
      if (Array.isArray(value) && value.length === 0) continue;
      entry[key] = value;
    }
    payload[zone.name] = entry;
  }
  return payload;
}

const round = (value) => Math.round(value * 1000) / 1000;

function el(tag, attributes = {}, children = []) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attributes)) {
    if (key === 'class') node.className = value;
    else if (key === 'text') node.textContent = value;
    else node.setAttribute(key, value);
  }
  for (const child of [].concat(children)) if (child) node.append(child);
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

// The same row with nothing to click into: a `<div>`, because a `<label>` with
// no control in it is a label for whatever the browser finds next.
function value(name, shown) {
  return el('div', { class: 'zone-field' }, [
    el('span', { class: 'zone-field-name', text: name }),
    shown,
  ]);
}

// Where the zone is, for reading. An area drawn by `segment-map` has no pose of
// its own until somebody places one, and saying so beats printing a coordinate
// derived from the outline as though a robot had been driven there.
export function poseText(zone) {
  if (typeof zone.x !== 'number' || typeof zone.y !== 'number') return 'not placed';
  return `${zone.x.toFixed(2)}, ${zone.y.toFixed(2)} m`;
}

export class ZoneEditor {
  constructor(mapView, dom, { onSave, onExit } = {}) {
    this.mapView = mapView;
    this.dom = dom; // { rows, detail, note }
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

  // The list, not being edited: the same rows the editor draws, with text where
  // its controls are. One renderer, because two of them are two layouts that
  // drift apart — and because the difference between looking and editing should
  // be the controls, not the page.
  //
  // A zone is selected here too, and its record shows beside the list. A place
  // has a note as well as a name, and a details column that appeared only once
  // an edit had begun would have hidden the one field an operator reads before
  // deciding whether to edit at all.
  show(zones) {
    if (this.active) return;
    this.zones = zones || [];
    this.selected = this.zones.length ? this.zones[0].name : null;
    this._render();
  }

  begin(zones) {
    this.zones = (zones || []).map((zone) => JSON.parse(JSON.stringify(zone)));
    // Something is always selected, so the panel always has a zone in it: an
    // empty panel is a state that has to be explained, and the explanation
    // ("select a zone to…") can only ever name one of the things it is for.
    this.selected = this.zones.length ? this.zones[0].name : null;
    this.active = true;
    this._drag = null;
    this._placing = null;
    this._hover = null;
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
    this._cursor('');
    this.mapView.overlay = null;
    this._render();
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
    // One line, so the list under it never moves — the whole message is on the
    // hover for the rare one that outruns the width.
    this.dom.note.title = text;
    this.dom.note.className = `note ${bad ? 'error' : ''}`;
  }

  // Every rename is checked against the same rules the robot's loader enforces
  // — the name shape, and two zones answering one query — because the loader
  // *refuses* two zones answering one query rather than picking by dict order.
  // A set this editor is willing to save is a set a robot will load.
  problems() {
    const seen = new Set();
    for (const zone of this.zones) {
      if (!NAME_RE.test(zone.name)) return `"${zone.name}" is not a name anyone can type`;
      if (seen.has(zone.name)) return `two zones named "${zone.name}"`;
      seen.add(zone.name);
    }
    const ambiguous = ambiguities(this.zones);
    if (ambiguous.length) return ambiguous[0];
    if (!this.zones.length) return 'no zones to save';
    return null;
  }

  _render() {
    this._renderRows();
    this._renderDetail();
  }

  // One renderer, one row shape, edited or not: the place-name and what shape
  // it is, and — while editing — the two controls that act on that one zone.
  // The cells do not move when the controls arrive in them.
  //
  // The row carries only what is compared *across* zones. The note is a
  // sentence and belongs to one zone at a time, so it edits in the details
  // column beside the list rather than as a column nobody could read.
  _renderRows() {
    const rows = this.dom.rows;
    rows.replaceChildren();
    rows.classList.toggle('editing', this.active);
    for (const zone of this.zones) {
      const row = document.createElement('div');
      const chosen = zone.name === this.selected;
      row.className = 'zone-row' + (chosen ? ' selected' : '');
      // The name is what you *pick* the zone by, so it is a button and looks
      // like one being pressed — in both states, because in both states the
      // details column is showing one zone's record and this is how you say
      // which.
      const name = document.createElement('button');
      name.type = 'button';
      name.className = 'zone-name';
      name.textContent = zone.name;
      name.setAttribute('aria-pressed', String(chosen));
      name.title = `select ${zone.name}`;
      const shape = el('span', { class: 'dim zone-shape', text: zoneSummary(zone) });
      row.addEventListener('click', () => this.select(zone.name));
      // The two control cells are rendered empty while looking rather than
      // left out, so the row is literally the same row: `edit zones` puts
      // controls into cells that were already there, and nothing to the left
      // of them moves.
      row.append(
        name,
        shape,
        this.active ? this._placeCell(zone) : el('span', { class: 'place' }),
        this.active ? this._deleteCell(zone) : el('span', { class: 'zone-del' }),
      );
      rows.append(row);
    }
    if (!this.active) return;
    // Adding a zone acts on the list, so it is the end of the list — not a
    // third button beside the two that begin and end the whole edit, which is
    // where it sat and read as one of them. Inside the scroll box, so a floor
    // with twenty zones and one with two put the same geometry on screen.
    const add = document.createElement('button');
    add.type = 'button';
    add.className = 'zone-add';
    add.textContent = 'add zone';
    add.title = 'add a zone at the middle of the view';
    add.addEventListener('click', () => this.addZone());
    rows.append(add);
  }

  select(name) {
    if (this.selected === name) return;
    this.selected = name;
    this._render();
    this.mapView.draw();
  }

  // Only for a zone with no pose to drag. Everything else on the map is moved
  // by dragging it, and a button that duplicates a drag is a second way to do
  // one thing; this is the case dragging cannot reach, because a `segment-map`
  // room is an outline with no `x`/`y` and so draws no cross to take hold of.
  // Once placed, the cross is the control. The cell is held even when empty, or
  // the delete buttons would step in and out down the list.
  _placeCell(zone) {
    if (typeof zone.x === 'number') return el('span', { class: 'place' });
    const pose = document.createElement('button');
    pose.type = 'button';
    pose.className = 'place';
    pose.textContent = '⌖';
    if (this._placing === zone.name) pose.classList.add('armed');
    pose.title = 'no pose: click here, then click the map';
    pose.addEventListener('click', (event) => {
      event.stopPropagation();
      this.placePose(zone.name);
    });
    return pose;
  }

  _deleteCell(zone) {
    const del = document.createElement('button');
    del.type = 'button';
    del.className = 'zone-del';
    del.textContent = '×';
    del.title = 'delete zone';
    del.addEventListener('click', (event) => {
      event.stopPropagation();
      const index = this.zones.findIndex((z) => z.name === zone.name);
      this.zones = this.zones.filter((z) => z.name !== zone.name);
      if (this.selected === zone.name) {
        // The neighbour, so the panel stays occupied and the operator stays
        // where they were in the list.
        const next = this.zones[Math.min(index, this.zones.length - 1)];
        this.selected = next ? next.name : null;
      }
      if (this._placing === zone.name) this._placing = null;
      this._render();
      this.mapView.draw();
    });
    return del;
  }

  // -- the selected zone --------------------------------------------------

  // The selected zone's whole record: its name, the note, and where it is.
  // Three fields, because a place-name is three facts — what it is called, what
  // the name cannot say, and where. Text when looking, inputs when editing, in
  // the same rows either way, so entering the mode does not relay the column.
  _renderDetail() {
    const panel = this.dom.detail;
    if (!panel) return;
    const zone = this.zones.find((entry) => entry.name === this.selected);
    panel.replaceChildren();
    if (!zone) {
      panel.append(el('p', { class: 'dim', text: 'no zones on this revision yet' }));
      return;
    }
    if (!this.active) {
      panel.append(
        value('name', el('span', { class: 'zone-name-value', text: zone.name })),
        value('note', el('span', { text: zone.note || '—' })),
        value('pose', el('span', { text: poseText(zone) })),
      );
      return;
    }
    panel.append(
      field('name', this._nameInput(zone)),
      field('note', this._noteInput(zone)),
      field('pose', this._poseInput(zone)),
    );
  }

  _nameInput(zone) {
    const rename = document.createElement('input');
    rename.className = 'zone-rename';
    rename.value = zone.name;
    rename.title = 'what an operator calls this place, and what goto takes';
    // Said while it is being typed, not at save. The rule is real — the robot's
    // loader refuses a name it cannot be told to drive to — but a field that
    // looks like free text until a save fails is a field that does not look
    // like it has a rule at all.
    rename.addEventListener('input', () => {
      rename.classList.toggle('bad', !NAME_RE.test(rename.value.trim()));
    });
    rename.addEventListener('change', () => {
      const renamed = rename.value.trim();
      if (!renamed || renamed === zone.name) return;
      this._update(zone.name, (z) => ({ ...z, name: renamed }));
      this.selected = renamed;
      this._render();
      this.mapView.draw();
    });
    return rename;
  }

  // A paragraph, not a line: the note is where an operator writes what the name
  // cannot say, and a single-line box says "a word or two" to everyone who
  // sees it.
  _noteInput(zone) {
    const note = document.createElement('textarea');
    note.className = 'zone-note';
    note.rows = 3;
    note.value = zone.note || '';
    note.title = 'what the name cannot say: where the stationery really lives';
    note.addEventListener('change', () => {
      this._update(zone.name, (z) => ({ ...z, note: note.value.trim() }));
    });
    return note;
  }

  // The numeric way to the same coordinate the cross on the map drags. Typed
  // values are **not** snapped to the pixel grid, where a drag is: a drag lands
  // wherever the pointer happened to be and the grid is what makes "the same
  // place" the same number, while a number somebody typed is already the number
  // they meant.
  _poseInput(zone) {
    const box = el('div', { class: 'zone-pose' });
    const axis = (key) => {
      const input = document.createElement('input');
      input.className = 'zone-coord';
      input.inputMode = 'decimal';
      input.value = typeof zone[key] === 'number' ? zone[key].toFixed(2) : '';
      input.setAttribute('aria-label', key);
      input.addEventListener('change', () => {
        const next = { ...zone, [key]: Number(input.value) };
        if (!Number.isFinite(next.x) || !Number.isFinite(next.y)) {
          input.value = typeof zone[key] === 'number' ? zone[key].toFixed(2) : '';
          return;
        }
        this._update(zone.name, (z) => withPose(z, next.x, next.y));
        this._render();
        this.mapView.draw();
      });
      return input;
    };
    box.append(axis('x'), axis('y'), el('span', { class: 'dim', text: 'm' }));
    return box;
  }

  // -- drawing ------------------------------------------------------------

  // Edit mode has to say which zone the record beside the list belongs to, so
  // **the selected zone alone is drawn in full** — filled, with its vertex
  // handles out — and the rest fall back to markers at half weight. Without
  // that the canvas looked identical whichever row was picked, and the only
  // sign of a selection was a border in a list somewhere else on the page.
  //
  // Hover overrides the dimming, because it answers a different question: not
  // "which one am I editing" but "what will this press take", and that is true
  // of a zone whether or not it is the selected one.
  _draw(ctx) {
    if (!this.active || !this.mapView.map) return;
    const view = this.mapView.view;
    const toScreen = (x, y) => {
      const pixel = worldToPixel(this.mapView.map, x, y);
      return { x: pixel.x * view.scale + view.tx, y: pixel.y * view.scale + view.ty };
    };
    const hover = this._hover;
    const palette = theme();
    for (const zone of this.zones) {
      const selected = zone.name === this.selected;
      const over = hover && hover.zone === zone.name ? hover.kind : '';
      const lit = selected || Boolean(over);
      ctx.save();
      ctx.globalAlpha = lit ? 1 : 0.45;
      ctx.strokeStyle = palette.editStroke;
      // `--edit-hover` is louder than `--edit-selected` deliberately: selection
      // says which row you are looking at, hover says what the next press will
      // move — and only one of those is about to change the map.
      ctx.fillStyle =
        over === 'zone' ? palette.editHover : selected ? palette.editSelected : palette.editFill;
      ctx.lineWidth = lit ? 2.5 : 1.5;
      if (zone.polygon) {
        ctx.beginPath();
        zone.polygon.forEach(([x, y], index) => {
          const point = toScreen(x, y);
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        if (lit) ctx.fill();
        ctx.stroke();
        // Handles are what you drag, and they belong to the zone being edited:
        // every outline wearing them made a dozen rooms one field of squares.
        if (lit) {
          zone.polygon.forEach(([x, y], index) => {
            const point = toScreen(x, y);
            const grabbed = over === 'vertex' && hover.index === index;
            const half = grabbed ? 6 : 4;
            ctx.fillStyle = palette.edit;
            ctx.fillRect(point.x - half, point.y - half, half * 2, half * 2);
            if (grabbed) {
              // A ring rather than only a bigger square: on a dark basemap the
              // square alone grows into the wall it is sitting on.
              ctx.strokeStyle = palette.editRing;
              ctx.lineWidth = 2;
              ctx.strokeRect(point.x - half, point.y - half, half * 2, half * 2);
              ctx.strokeStyle = palette.editStroke;
            }
          });
        }
      }
      if (typeof zone.x === 'number') {
        const point = toScreen(zone.x, zone.y);
        const grabbed = over === 'pose';
        const arm = lit ? 7 : 4;
        ctx.strokeStyle = palette.edit;
        ctx.lineWidth = grabbed ? 3 : lit ? 2 : 1.5;
        ctx.beginPath();
        ctx.moveTo(point.x - arm, point.y);
        ctx.lineTo(point.x + arm, point.y);
        ctx.moveTo(point.x, point.y - arm);
        ctx.lineTo(point.x, point.y + arm);
        ctx.stroke();
        if (grabbed) {
          ctx.strokeStyle = palette.editRing;
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(point.x, point.y, 9, 0, Math.PI * 2);
          ctx.stroke();
          ctx.strokeStyle = palette.edit;
        }
        ctx.font = '11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillStyle = palette.edit;
        ctx.fillText(zoneLabel(zone), point.x, point.y - arm - 3);
      }
      ctx.restore();
    }
  }
}
