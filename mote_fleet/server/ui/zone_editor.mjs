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

// Whether a zone of this kind may be dispatched to, absent an explicit say-so.
export const navigableByDefault = (kind) => !CONSTRAINT_KINDS.has(kind || 'area');

// Aliases are edited as one comma-separated field, which is how an operator
// says "it is also called this" without a list widget. Blank entries are
// dropped and a spelling repeated in one field is kept once — a duplicate
// inside a single zone is a typo, not the collision `ambiguities` is about.
export function parseAliases(text) {
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

export function formatAliases(aliases) {
  return (aliases || []).join(', ');
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

const HANDLE = 'rgba(240, 130, 34, 1)';
const EDIT_STROKE = 'rgba(240, 130, 34, 0.9)';
const EDIT_FILL = 'rgba(240, 130, 34, 0.08)';
const SELECTED_FILL = 'rgba(240, 130, 34, 0.18)';

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
    this._bind();
  }

  begin(zones) {
    this.zones = (zones || []).map((zone) => JSON.parse(JSON.stringify(zone)));
    this.selected = null;
    this.active = true;
    this._drag = null;
    this._placing = null;
    this.dom.panel.hidden = false;
    this.dom.note.textContent = '';
    this.mapView.overlay = (ctx) => this._draw(ctx);
    this._renderRows();
    this.mapView.draw();
  }

  end() {
    this.active = false;
    this._drag = null;
    this._placing = null;
    this.selected = null;
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

  // Hit reach in metres at the current zoom: 10 screen px, whatever the scale.
  _reach() {
    const view = this.mapView.view;
    return (10 / view.scale) * this.mapView.map.resolution;
  }

  _down(event) {
    if (!this.active || !this.mapView.map) return;
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
      this._update(name, (zone) => withPose(zone, point.x, point.y));
      this.selected = name;
      this.note('');
      this._renderRows();
      this.mapView.draw();
      return;
    }

    for (const zone of this.zones) {
      const vertex = zone.polygon ? nearestVertex(zone, point.x, point.y) : null;
      if (vertex && vertex.distance <= reach) {
        this._grab(event, { kind: 'vertex', zone: zone.name, index: vertex.index });
        return;
      }
    }
    for (const zone of this.zones) {
      if (typeof zone.x !== 'number') continue;
      if (Math.hypot(zone.x - point.x, zone.y - point.y) <= reach * 1.2) {
        this._grab(event, { kind: 'pose', zone: zone.name });
        return;
      }
    }
    for (const zone of this.zones) {
      if (zone.polygon && pointInPolygon(zone.polygon, point.x, point.y)) {
        this._grab(event, { kind: 'zone', zone: zone.name, last: point });
        return;
      }
    }
    this.selected = null;
    this._renderRows();
    this.mapView.draw();
    // Nothing hit: let the map pan.
  }

  _grab(event, drag) {
    event.stopPropagation();
    event.preventDefault();
    this.mapView.canvas.setPointerCapture(event.pointerId);
    this._drag = drag;
    this.selected = drag.zone;
    this._renderRows();
    this.mapView.draw();
  }

  _move(event) {
    if (!this.active || !this._drag) return;
    event.stopPropagation();
    const point = this._world(event);
    const drag = this._drag;
    this._update(drag.zone, (zone) => {
      if (drag.kind === 'vertex') return withVertex(zone, drag.index, point.x, point.y);
      if (drag.kind === 'pose') return withPose(zone, point.x, point.y);
      const moved = translated(zone, point.x - drag.last.x, point.y - drag.last.y);
      drag.last = point;
      return moved;
    });
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
        this._update(zone.name, (z) => withInsertedVertex(z, edge.index, point.x, point.y));
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
    const centre = pixelToWorld(this.mapView.map, (rect.width / 2 - view.tx) / view.scale, (rect.height / 2 - view.ty) / view.scale);
    const zone = freshZone(this.zones, centre.x, centre.y);
    this.zones = [...this.zones, zone];
    this.selected = zone.name;
    this._renderRows();
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
    this._renderRows();
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

  _renderRows() {
    const rows = this.dom.rows;
    rows.replaceChildren();
    for (const zone of this.zones) {
      const row = document.createElement('div');
      row.className = 'zone-row' + (zone.name === this.selected ? ' selected' : '');
      const name = document.createElement('input');
      name.value = zone.name;
      name.title = 'machine name (goto <name>)';
      name.addEventListener('change', () => {
        this._update(zone.name, (z) => ({ ...z, name: name.value.trim() }));
        this.selected = name.value.trim();
        this._renderRows();
        this.mapView.draw();
      });
      const display = document.createElement('input');
      display.value = zone.display_name || '';
      display.placeholder = 'display name';
      display.title = 'what an operator sees — “The Kitchen”';
      display.addEventListener('change', () => {
        this._update(zone.name, (z) => ({ ...z, display_name: display.value.trim() }));
      });
      // The vocabulary half. It is edited here rather than on the robot because
      // this is where the rooms get their names in the first place: a build
      // arrives as `zone_01`..`zone_07`, and "which of these is the kitchen,
      // and what else do we call it" is answered by looking at the map.
      const kind = document.createElement('select');
      kind.title = 'what kind of place this is (zone/v0)';
      for (const option of ZONE_KINDS) {
        const node = document.createElement('option');
        node.value = option;
        node.textContent = option;
        kind.append(node);
      }
      kind.value = zone.kind || 'area';
      kind.addEventListener('change', () => {
        this._update(zone.name, (z) => ({ ...z, kind: kind.value }));
        this.mapView.draw();
      });
      const aliases = document.createElement('input');
      aliases.value = formatAliases(zone.aliases);
      aliases.placeholder = 'aliases, comma separated';
      aliases.title = 'other spellings a dispatcher may use';
      aliases.addEventListener('change', () => {
        const parsed = parseAliases(aliases.value);
        aliases.value = formatAliases(parsed);
        this._update(zone.name, (z) => ({ ...z, aliases: parsed }));
      });
      const pose = document.createElement('button');
      pose.type = 'button';
      pose.textContent = '⌖';
      pose.className = this._placing === zone.name ? 'armed' : '';
      pose.title =
        typeof zone.x === 'number'
          ? 'place this zone’s pose: click the map'
          : 'this zone has no pose — click here, then click the map';
      pose.addEventListener('click', (event) => {
        event.stopPropagation();
        this.placePose(zone.name);
      });
      const del = document.createElement('button');
      del.type = 'button';
      del.textContent = '×';
      del.title = 'delete zone';
      del.addEventListener('click', () => {
        this.zones = this.zones.filter((z) => z.name !== zone.name);
        if (this.selected === zone.name) this.selected = null;
        if (this._placing === zone.name) this._placing = null;
        this._renderRows();
        this.mapView.draw();
      });
      row.addEventListener('click', () => {
        if (this.selected !== zone.name) {
          this.selected = zone.name;
          this._renderRows();
          this.mapView.draw();
        }
      });
      row.append(name, display, kind, aliases, pose, del);
      rows.append(row);
    }
  }

  // -- drawing ------------------------------------------------------------

  _draw(ctx) {
    if (!this.active || !this.mapView.map) return;
    const view = this.mapView.view;
    const toScreen = (x, y) => {
      const pixel = worldToPixel(this.mapView.map, x, y);
      return { x: pixel.x * view.scale + view.tx, y: pixel.y * view.scale + view.ty };
    };
    for (const zone of this.zones) {
      const selected = zone.name === this.selected;
      ctx.save();
      ctx.strokeStyle = EDIT_STROKE;
      ctx.fillStyle = selected ? SELECTED_FILL : EDIT_FILL;
      ctx.lineWidth = selected ? 2.5 : 1.5;
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
        for (const [x, y] of zone.polygon) {
          const point = toScreen(x, y);
          ctx.fillStyle = HANDLE;
          ctx.fillRect(point.x - 4, point.y - 4, 8, 8);
        }
      }
      if (typeof zone.x === 'number') {
        const point = toScreen(zone.x, zone.y);
        ctx.strokeStyle = HANDLE;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(point.x - 7, point.y);
        ctx.lineTo(point.x + 7, point.y);
        ctx.moveTo(point.x, point.y - 7);
        ctx.lineTo(point.x, point.y + 7);
        ctx.stroke();
        ctx.font = '11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillStyle = HANDLE;
        ctx.fillText(zone.name, point.x, point.y - 10);
      }
      ctx.restore();
    }
  }
}
