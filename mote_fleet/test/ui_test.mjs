// The dashboard's load-bearing pure pieces, under node.
//
// Everything else in the UI is DOM and canvas, which a browser is the only
// honest place to test. These are not: the MQTT codec decides whether the read
// path works at all, the world→pixel transform decides whether a robot is drawn
// where it actually is, the zone editor's geometry decides what an edit does,
// and the review pane's route builders decide *which map* is on the canvas at
// all. Each is exported from a file the browser loads unchanged — `.mjs` is what
// lets node import them with no package.json and no build step.
//
// The seams the stylesheet and the markup share with all of that fail silently
// rather than loudly, so they are read out of those files and asserted here too.
//
//     node --test mote_fleet/test/ui_test.mjs

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';

import {
  CONNACK,
  PUBLISH,
  SUBACK,
  encodeConnect,
  encodeLength,
  encodeSubscribe,
  parsePackets,
  parseTopic,
} from '../server/ui/mqtt.mjs';
import {
  fitView,
  pinchSpan,
  pinchUpdate,
  pixelToWorld,
  worldToPixel,
  zoneLabel,
  zoneOutline,
} from '../server/ui/map.mjs';
import { NARROW_MAX_PX } from '../server/ui/layout.mjs';
import { detailFacts, detailSections, rosterSubline } from '../server/ui/robot.mjs';

const read = (name) => readFileSync(new URL(`../server/ui/${name}`, import.meta.url), 'utf8');

// -- the MQTT codec ------------------------------------------------------

test('remaining length uses MQTT variable-length encoding', () => {
  assert.deepEqual(encodeLength(0), [0x00]);
  assert.deepEqual(encodeLength(127), [0x7f]);
  assert.deepEqual(encodeLength(128), [0x80, 0x01]);
  assert.deepEqual(encodeLength(16383), [0xff, 0x7f]);
});

test('CONNECT names protocol 3.1.1 and a clean session', () => {
  const packet = encodeConnect('mote-ui-test', 30);
  assert.equal(packet[0] >> 4, 1);
  assert.equal(new TextDecoder().decode(packet.subarray(4, 8)), 'MQTT');
  assert.equal(packet[8], 4); // protocol level
  assert.equal(packet[9], 0x02); // clean session, no will, no credentials
});

test('SUBSCRIBE carries the mandatory 0x02 flags', () => {
  const packet = encodeSubscribe(1, ['mote/v1/+/health']);
  assert.equal(packet[0], (8 << 4) | 0x02);
});

test('a PUBLISH is decoded with its topic, payload and retain flag', () => {
  const topic = 'mote/v1/mote-01/health';
  const payload = JSON.stringify({ schema: 1, state: 'ok' });
  const body = [
    0,
    topic.length,
    ...new TextEncoder().encode(topic),
    0,
    7, // packet id, because this is QoS 1
    ...new TextEncoder().encode(payload),
  ];
  const wire = new Uint8Array([
    (PUBLISH << 4) | 0x03, // qos 1, retain
    ...encodeLength(body.length),
    ...body,
  ]);
  const { packets, rest } = parsePackets(wire);
  assert.equal(rest.length, 0);
  assert.equal(packets.length, 1);
  assert.deepEqual(
    { topic: packets[0].topic, qos: packets[0].qos, retain: packets[0].retain },
    { topic, qos: 1, retain: true },
  );
  assert.equal(JSON.parse(packets[0].payload).state, 'ok');
});

test('a packet split across WebSocket frames is held until it is whole', () => {
  const whole = new Uint8Array([
    (CONNACK << 4),
    2,
    0,
    0,
    (SUBACK << 4),
    3,
    0,
    1,
    1,
  ]);
  // A WebSocket frame is not an MQTT packet boundary: the client must buffer.
  const first = parsePackets(whole.subarray(0, 3));
  assert.equal(first.packets.length, 0);
  assert.equal(first.rest.length, 3);

  const merged = new Uint8Array([...first.rest, ...whole.subarray(3)]);
  const second = parsePackets(merged);
  assert.deepEqual(
    second.packets.map((packet) => packet.type),
    [CONNACK, SUBACK],
  );
  assert.equal(second.rest.length, 0);
});

test('several packets in one frame all come out', () => {
  const wire = new Uint8Array([(CONNACK << 4), 2, 0, 0, (CONNACK << 4), 2, 0, 0]);
  assert.equal(parsePackets(wire).packets.length, 2);
});

test('a refusing CONNACK keeps its return code', () => {
  const { packets } = parsePackets(new Uint8Array([(CONNACK << 4), 2, 0, 5]));
  assert.equal(packets[0].returnCode, 5);
});

test('topics parse into a robot id and a leaf', () => {
  assert.deepEqual(parseTopic('mote/v1/mote-01/task/status'), {
    robotId: 'mote-01',
    leaf: 'task/status',
  });
  assert.equal(parseTopic('mote/v2/mote-01/health'), null);
  assert.equal(parseTopic('something/else'), null);
});

// -- the Q5 transform ----------------------------------------------------

// The office_world site bundle, so the numbers are a real floor's.
const map = { resolution: 0.05, origin: [-10.935, -5.958, 0], width: 500, height: 300 };

test('the map origin is the bottom-left pixel', () => {
  const pixel = worldToPixel(map, map.origin[0], map.origin[1]);
  assert.deepEqual(pixel, { x: 0, y: map.height });
});

test('image y runs top-down while the map frame runs up', () => {
  const low = worldToPixel(map, 0, 0);
  const high = worldToPixel(map, 0, 1);
  assert.ok(high.y < low.y, 'a metre further along +y is further up the image');
  assert.equal(low.y - high.y, 1 / map.resolution);
});

test('a metre is resolution pixels', () => {
  const origin = worldToPixel(map, 0, 0);
  const east = worldToPixel(map, 1, 0);
  assert.equal(east.x - origin.x, 20);
});

test('pixelToWorld is the inverse, so clicking the map reads out metres', () => {
  const world = { x: 3.25, y: -1.75 };
  const pixel = worldToPixel(map, world.x, world.y);
  const back = pixelToWorld(map, pixel.x, pixel.y);
  assert.ok(Math.abs(back.x - world.x) < 1e-9);
  assert.ok(Math.abs(back.y - world.y) < 1e-9);
});

test('fit centres the whole basemap in the canvas', () => {
  const view = fitView(map, 1000, 400, 0);
  assert.equal(view.scale, 400 / map.height); // height is the binding dimension
  assert.equal(view.ty, 0);
  assert.equal(view.tx, (1000 - map.width * view.scale) / 2);
});

// -- taught places on the basemap ---------------------------------------

test('a polygon zone becomes basemap pixels through the same transform', () => {
  const map = { resolution: 0.05, origin: [-10, -5, 0], width: 400, height: 200 };
  const outline = zoneOutline(map, {
    name: 'ward',
    polygon: [
      [0, 0],
      [1, 0],
      [1, 1],
    ],
  });
  assert.equal(outline.kind, 'polygon');
  assert.deepEqual(outline.points[0], worldToPixel(map, 0, 0));
  assert.equal(outline.points.length, 3);
});

test('a radius zone becomes a circle in pixels, not in metres', () => {
  const map = { resolution: 0.05, origin: [0, 0, 0], width: 100, height: 100 };
  const outline = zoneOutline(map, { name: 'kitchen', x: 1, y: 1, radius: 1.5 });
  assert.equal(outline.kind, 'circle');
  assert.equal(outline.radius, 30); // 1.5 m at 0.05 m/px
  assert.deepEqual(outline.centre, worldToPixel(map, 1, 1));
});

test('a zone is drawn under the name a person reads', () => {
  // `display_name` is the half meant for reading and the machine name is the
  // half meant for typing, so a zone given one is labelled with it — on the
  // fleet map and under the editor's own handles alike, or a zone would answer
  // to one name in the list and another the moment it was being edited.
  assert.equal(zoneLabel({ name: 'zone_01', display_name: 'Kitchen' }), 'Kitchen');
  assert.equal(zoneLabel({ name: 'zone_01', display_name: '' }), 'zone_01');
  assert.equal(zoneLabel({ name: 'zone_01' }), 'zone_01');
  assert.equal(zoneLabel(null), '');
});

test('the browser is told which theme to draw its own controls in', () => {
  // A `select`'s dropdown, a checkbox and a scrollbar are the browser's to
  // paint. Left to the *system* preference while the page follows its own, a
  // dark page grows a white dropdown list — which is what a select in the zone
  // editor looked like.
  const css = read('style.css');
  const dark = css.slice(css.indexOf(':root {'));
  assert.match(dark.slice(0, dark.indexOf('}')), /color-scheme:\s*dark/);
  const light = css.slice(css.indexOf('@media (prefers-color-scheme: light)'));
  assert.match(light.slice(0, light.indexOf('}')), /color-scheme:\s*light/);
  // And the element itself, which the shared control rule used to miss.
  assert.match(css, /input,\s*select,\s*button,\s*\.button \{/);
});

test('a bare waypoint has no outline to draw', () => {
  const map = { resolution: 0.05, origin: [0, 0, 0], width: 100, height: 100 };
  assert.equal(zoneOutline(map, { name: 'pickup', x: 1, y: 1 }), null);
});

// -- pinch to zoom -------------------------------------------------------

// The touch gesture the wheel handler has no equivalent for. Its arithmetic is
// pure for the same reason the transform above is: on a phone, a sign or a
// division wrong here is a map that leaps off screen with no way to tell why.

test('fingers moving apart zoom in, and together zoom out', () => {
  const before = pinchSpan({ x: 100, y: 100 }, { x: 200, y: 100 });
  const wider = pinchSpan({ x: 50, y: 100 }, { x: 250, y: 100 });
  assert.equal(pinchUpdate(before, wider).factor, 2);
  assert.equal(pinchUpdate(wider, before).factor, 0.5);
});

test('the zoom is taken about the midpoint of the two fingers', () => {
  const span = pinchSpan({ x: 10, y: 20 }, { x: 30, y: 60 });
  assert.deepEqual(span.centre, { x: 20, y: 40 });
  assert.equal(span.distance, Math.hypot(20, 40));
});

test('fingers that move together carry the map with them', () => {
  const before = pinchSpan({ x: 0, y: 0 }, { x: 100, y: 0 });
  const after = pinchSpan({ x: 30, y: 12 }, { x: 130, y: 12 });
  const update = pinchUpdate(before, after);
  assert.equal(update.factor, 1); // same span: a pan, not a zoom
  assert.deepEqual(update.pan, { x: 30, y: 12 });
});

test('two pointers reported at one place leave the scale alone', () => {
  // Not defensive: a factor of n/0 is Infinity and 0/0 is NaN, and either one
  // in the view scale blanks the map permanently.
  const together = pinchSpan({ x: 5, y: 5 }, { x: 5, y: 5 });
  assert.equal(together.distance, 0);
  assert.equal(pinchUpdate(together, pinchSpan({ x: 0, y: 0 }, { x: 40, y: 0 })).factor, 1);
});

// -- what the page says about a robot ------------------------------------

// Each state is meant to be said once, by the strongest idiom the page has: the
// roster's dot carries "ok", so its sub-line must not, and a heading over an
// empty div claims a fact the robot does not have. Neither rule fails loudly —
// the page just repeats itself, or reserves a row for nothing — so the
// decisions are pure functions and are held here.

const NOW = Date.parse('2026-08-22T12:00:00Z');
const stamp = (secondsAgo) => new Date(NOW - secondsAgo * 1000).toISOString();

// A robot doing nothing, in perfect health, reporting now. The common case.
const idle = () => ({
  id: 'mote-01',
  registry: { robot_id: 'mote-01', name: 'mote-01' },
  presence: { online: true, stamp: stamp(1) },
  health: {
    state: 'ok',
    summary: 'all subsystems nominal',
    subsystems: [{ name: 'lidar', state: 'ok', message: '10.0 Hz' }],
    stamp: stamp(2),
    site: 'depot',
    floor: 'ground',
    version: '0.5.0',
  },
  statuses: [],
});

test('a healthy idle robot gets no roster sub-line at all', () => {
  // The dot is green and the state column reads `ok`; "all subsystems nominal"
  // under them is the same fact a third time.
  assert.equal(rosterSubline(idle(), NOW), '');
});

test('a degraded robot says what is wrong', () => {
  const record = idle();
  record.health.state = 'degraded';
  record.health.summary = 'slip detected while turning';
  assert.equal(rosterSubline(record, NOW), 'slip detected while turning');
});

test('a robot that dropped off says why, not what it last claimed', () => {
  const record = idle();
  record.presence = { online: false, reason: 'connection lost', stamp: stamp(90) };
  assert.match(rosterSubline(record, NOW), /^offline \(connection lost\)/);
});

test('health that has stopped arriving is an exception too', () => {
  const record = idle();
  record.health.stamp = stamp(300);
  assert.match(rosterSubline(record, NOW), /^no health for/);
});

test('a robot running a task says so — the dot cannot', () => {
  const record = idle();
  record.health.task = { state: 'running', command: 'goto kitchen' };
  assert.equal(rosterSubline(record, NOW), 'running: goto kitchen');
});

const factOf = (record, key, canonical = null) =>
  Object.fromEntries(detailFacts(record, canonical, NOW))[key];

test('the detail health row is the state alone while the state is ok', () => {
  assert.equal(factOf(idle(), 'health'), 'ok');
});

test('a degraded state keeps its message, which is the whole point of it', () => {
  const record = idle();
  record.health.state = 'fault';
  record.health.summary = 'lidar stopped';
  assert.equal(factOf(record, 'health'), 'fault — lidar stopped');
});

test('health from before the robot went quiet is marked as such', () => {
  const record = idle();
  record.presence = { online: false, reason: 'stopped', stamp: stamp(90) };
  assert.equal(factOf(record, 'health'), 'ok  (last known)');
});

test('battery is n/a: nothing on this robot measures it', () => {
  // The power bank exposes no telemetry. A sentence saying so costs a row of
  // the pane every time anyone looks at any robot.
  assert.equal(factOf(idle(), 'battery'), 'n/a');
});

test('the map row names the fleet canonical only when it differs', () => {
  const record = idle();
  record.health.map = { site: 'depot', floor: 'ground', revision: 'r1' };
  assert.equal(factOf(record, 'map', 'r1'), 'r1');
  assert.match(factOf(record, 'map', 'r2'), /r1 {2}\(fleet canonical: r2\)/);
});

test('a section with nothing under it is not a section', () => {
  const record = idle();
  assert.deepEqual(detailSections(record), {
    subsystems: true,
    dispatch: true,
    statuses: false, // never given a task: `task status` has nothing to show
  });
  record.statuses = [{ state: 'accepted', command: 'goto kitchen', stamp: stamp(3) }];
  assert.equal(detailSections(record).statuses, true);
  record.health.subsystems = [];
  assert.equal(detailSections(record).subsystems, false);
});

test('with no robot selected the detail pane has no sections at all', () => {
  assert.deepEqual(detailSections(null), {
    subsystems: false,
    dispatch: false,
    statuses: false,
  });
});

test('every heading in the detail pane can be hidden with its content', () => {
  // `renderSections` hides a heading by id. One added without an id is a
  // heading that stays on screen over an empty div, and nothing says so.
  const html = read('index.html');
  const pane = html.slice(html.indexOf('data-pane="detail"'), html.indexOf('</main>'));
  const headings = [...pane.matchAll(/<h3([^>]*)>/g)].map((match) => match[1]);
  assert.ok(headings.length >= 3, `found ${headings.length} headings in the detail pane`);
  for (const attributes of headings) {
    assert.match(attributes, /id="[^"]+"/, `<h3${attributes}> has no id to hide it by`);
  }
  const app = read('app.mjs');
  for (const [, id] of pane.matchAll(/<h3[^>]*id="([^"]+)"/g)) {
    assert.ok(app.includes(`'${id}'`), `app.mjs never binds #${id}`);
  }
});

test('the dispatch box does not teach the grammar', () => {
  // The zone picker beside it already writes a valid command and the grammar
  // reference is in the docs, so a placeholder spelling it out is a third copy
  // — one that goes stale the first time the task layer learns a word.
  const html = read('index.html');
  const input = html.slice(html.indexOf('id="command"'));
  const placeholder = /placeholder="([^"]*)"/.exec(input.slice(0, input.indexOf('/>')))[1];
  assert.equal(placeholder, 'send this robot somewhere');
});

// -- the narrow layout ---------------------------------------------------

// Below the breakpoint the stylesheet shows one pane at a time and app.mjs
// navigates to the map on a selection. Neither half fails loudly if they
// disagree about where "narrow" starts, so the seams are checked here.

test('the stylesheet switches to one pane at the width layout.mjs uses', () => {
  const css = read('style.css');
  const widths = [...css.matchAll(/@media \(max-width: (\d+)px\)/g)].map((m) => Number(m[1]));
  assert.ok(
    widths.includes(NARROW_MAX_PX),
    `style.css has no @media (max-width: ${NARROW_MAX_PX}px); found ${widths.join(', ')}`,
  );
});

test('the tab bar and the panes address each other by the same names', () => {
  const html = read('index.html');
  const panes = [...html.matchAll(/class="pane[^"]*" data-pane="([^"]+)"/g)].map((m) => m[1]);
  const tabs = [...html.matchAll(/<button[^>]*data-pane="([^"]+)"/g)].map((m) => m[1]);
  assert.deepEqual(panes, ['roster', 'map', 'review', 'detail']);
  // A pane with no tab is simply unreachable on a phone, and nothing says so.
  assert.deepEqual([...tabs].sort(), [...panes].sort());
});

test('exactly one pane starts active, so a phone opens on something', () => {
  const html = read('index.html');
  const active = [...html.matchAll(/class="pane [^"]*\bactive\b[^"]*"/g)];
  assert.equal(active.length, 1);
});

test('both map canvases take their own touch gestures', () => {
  // Without `touch-action: none` the browser consumes the drag and the pinch
  // before the canvas sees a single pointer event.
  const css = read('style.css');
  for (const id of ['#map-canvas {', '#review-canvas {']) {
    const canvas = css.slice(css.indexOf(id));
    assert.match(canvas.slice(0, canvas.indexOf('}')), /touch-action:\s*none/, id);
  }
});

// -- the zone editor ------------------------------------------------------
//
// Every edit operation is a pure function over zones in world metres, so the
// editor's geometry is tested here with no canvas: the browser only adds
// pointer events on top of exactly these.

const {
  CONSTRAINT_KINDS,
  NAME_RE,
  POINT_KINDS,
  ZONE_KINDS,
  ambiguities,
  cursorFor,
  formatList,
  freshZone,
  hitTest,
  isAreaKind,
  isGeneratedName,
  poseFor,
  nearestEdge,
  nearestVertex,
  navigableByDefault,
  normaliseAlias,
  parseList,
  pointInPolygon,
  slugify,
  zoneSummary,
  snapDelta,
  snapToPixel,
  translated,
  withInsertedVertex,
  withKind,
  withPose,
  withVertex,
  withoutVertex,
  zonesPayload,
} = await import('../server/ui/zone_editor.mjs');

const square = {
  name: 'kitchen',
  x: 1,
  y: 1,
  polygon: [
    [0, 0],
    [2, 0],
    [2, 2],
    [0, 2],
  ],
};

test('pointInPolygon handles a concave outline (the hallway shape)', () => {
  const ell = [
    [0, 0],
    [3, 0],
    [3, 1],
    [1, 1],
    [1, 3],
    [0, 3],
  ];
  assert.ok(pointInPolygon(ell, 0.5, 2.5)); // in the upright of the L
  assert.ok(pointInPolygon(ell, 2.5, 0.5)); // in the foot
  assert.ok(!pointInPolygon(ell, 2.5, 2.5)); // in the notch: outside
});

test('vertex and edge hit-tests find the nearest, with distances', () => {
  assert.deepEqual(nearestVertex(square, 0.1, -0.1), { index: 0, distance: Math.hypot(0.1, 0.1) });
  const edge = nearestEdge(square, 1, -0.2);
  assert.equal(edge.index, 0); // the bottom edge, which starts at vertex 0
  assert.ok(Math.abs(edge.distance - 0.2) < 1e-9);
});

test('vertex edits replace, insert after the edge start, and refuse a triangle collapse', () => {
  assert.deepEqual(withVertex(square, 0, -1, -1).polygon[0], [-1, -1]);
  const grown = withInsertedVertex(square, 0, 1, -0.5);
  assert.deepEqual(grown.polygon[1], [1, -0.5]);
  assert.equal(grown.polygon.length, 5);
  const triangle = { ...square, polygon: square.polygon.slice(0, 3) };
  assert.equal(withoutVertex(triangle, 0), null);
});

test('the hit test claims a vertex, then a pose, then the zone under it', () => {
  // One function, because the hover highlight, the cursor and the drag all read
  // it: an operator who cannot tell which of the three a press will take — or
  // that it will pan the map instead — is guessing, and three copies of this
  // ordering would eventually disagree about the answer.
  const zones = [square, { name: 'home', x: 5, y: 5 }];
  assert.deepEqual(hitTest(zones, 0.05, 0.05, 0.2), {
    kind: 'vertex',
    zone: 'kitchen',
    index: 0,
  });
  // The pose sits inside its own footprint; the pose wins.
  assert.deepEqual(hitTest(zones, 1, 1, 0.2), { kind: 'pose', zone: 'kitchen' });
  assert.deepEqual(hitTest(zones, 1.6, 0.4, 0.2), { kind: 'zone', zone: 'kitchen' });
  // A waypoint zone is reachable with no footprint at all, and beyond it is the
  // map: `null` is what makes a drag pan rather than edit.
  assert.deepEqual(hitTest(zones, 5.1, 5, 0.2), { kind: 'pose', zone: 'home' });
  assert.equal(hitTest(zones, 40, 40, 0.2), null);
});

test('the cursor says which of those it is before anything is pressed', () => {
  assert.equal(cursorFor(null), ''); // the stylesheet's grab: this pans
  assert.equal(cursorFor({ kind: 'vertex' }), 'crosshair');
  assert.equal(cursorFor({ kind: 'zone' }), 'move');
  assert.equal(cursorFor({ kind: 'pose' }), 'move');
  // Arming a pose overrides everything: the next click lands wherever it is.
  assert.equal(cursorFor(null, true), 'crosshair');
  assert.equal(cursorFor({ kind: 'zone' }, true), 'crosshair');
});

test('edits land on pixel centres, and never move more than half a pixel', () => {
  // The map is the limit of the precision available: a vertex anywhere inside a
  // pixel covers the same cells as one at its centre, so free-hand coordinates
  // are digits the map cannot back — and two zones meant to share a wall land a
  // few millimetres apart, differently every time.
  const grid = { resolution: 0.05, origin: [-10.935, -5.958, 0], width: 500, height: 300 };
  const centred = (value, origin) => {
    const cells = (value - origin) / grid.resolution - 0.5;
    return Math.abs(cells - Math.round(cells)) < 1e-9;
  };
  for (const [x, y] of [
    [0, 0],
    [-10.9, -5.9],
    [3.14159, -2.71828],
    [-10.935, -5.958], // the origin itself, which is a pixel *edge*
  ]) {
    const snapped = snapToPixel(grid, x, y);
    assert.ok(centred(snapped.x, grid.origin[0]), `${snapped.x} is not a pixel centre`);
    assert.ok(centred(snapped.y, grid.origin[1]), `${snapped.y} is not a pixel centre`);
    assert.ok(Math.abs(snapped.x - x) <= grid.resolution / 2 + 1e-9);
    assert.ok(Math.abs(snapped.y - y) <= grid.resolution / 2 + 1e-9);
    // Idempotent, or a zone would creep every time it was touched.
    assert.deepEqual(snapToPixel(grid, snapped.x, snapped.y), snapped);
  }

  // A body drag moves by whole pixels instead, so the shape it moves keeps its
  // shape: snapping every vertex would pull a traced room off its walls.
  assert.equal(snapDelta(grid, 0.03), 0.05);
  assert.equal(snapDelta(grid, 0.02), 0);
  assert.equal(snapDelta(grid, -0.12), -0.1);
  const room = { ...square, polygon: [[0.013, 0.017], [2.013, 0.017], [2.013, 2.017]] };
  const nudged = translated(room, snapDelta(grid, 0.31), snapDelta(grid, -0.07));
  assert.deepEqual(
    nudged.polygon.map(([x, y], i) => [
      Math.round((x - room.polygon[i][0]) * 1000),
      Math.round((y - room.polygon[i][1]) * 1000),
    ]),
    [
      [300, -50],
      [300, -50],
      [300, -50],
    ],
  );
});

test('moving a zone carries footprint and pose together', () => {
  const moved = translated(square, 1, -1);
  assert.deepEqual(moved.polygon[0], [1, -1]);
  assert.equal(moved.x, 2);
  assert.equal(moved.y, 0);
  assert.deepEqual(withPose(square, 5, 5), { ...square, x: 5, y: 5 });
});

test('a fresh zone gets the first free generated name and a real footprint', () => {
  const zone = freshZone([{ name: 'zone_01' }, { name: 'zone_02' }], 0, 0);
  assert.equal(zone.name, 'zone_03');
  assert.ok(NAME_RE.test(zone.name));
  assert.equal(zone.polygon.length, 4);
  assert.ok(pointInPolygon(zone.polygon, zone.x, zone.y));
});

test('the wire payload keys by name and never echoes it inside the entry', () => {
  const payload = zonesPayload([{ ...square, display_name: '' }]);
  assert.deepEqual(Object.keys(payload), ['kitchen']);
  assert.ok(!('name' in payload.kitchen));
  assert.ok(!('display_name' in payload.kitchen)); // empty strings are dropped
});

test('the zone editor panel hides when hidden, whatever its class sets', () => {
  // The M3 promote-picker lesson: `hidden` does not hide an element whose
  // class sets `display`, so the stylesheet must say so explicitly.
  const css = read('style.css');
  assert.match(css, /\.zone-editor\[hidden\]\s*\{\s*display:\s*none/);
  // And the read-only list it stands in for, which is a flex container: with
  // both on screen there would be two disagreeing accounts of the zones.
  assert.match(css, /\.zone-rows\[hidden\]\s*\{\s*display:\s*none/);
  const html = read('index.html');
  for (const id of ['zone-editor', 'zone-rows', 'zone-save', 'zone-cancel', 'zones-edit']) {
    assert.ok(html.includes(`id="${id}"`), `index.html is missing #${id}`);
  }
});

test('an action sits at the level of the thing it acts on', () => {
  // Three buttons in one row said they were three of a kind. Two of them begin
  // and end the whole edit, and belong where `edit zones` was; the third adds
  // one item, and belongs at the end of the items.
  const html = read('index.html');
  const head = html.slice(html.indexOf('<div class="zones-head">'), html.indexOf('id="zone-rows"'));
  for (const id of ['zones-edit', 'zone-save', 'zone-cancel']) {
    assert.ok(head.includes(`id="${id}"`), `#${id} belongs with the zones heading`);
  }
  assert.ok(!html.includes('id="zone-add"'), 'adding a zone is rendered into the list');
  // And what the save says is under the save, not in the column of fields for
  // one zone — where it sat while reporting on all of them.
  const panel = html.slice(html.indexOf('<div class="zones-panel">'), html.indexOf('id="zone-editor"'));
  assert.ok(panel.includes('id="zone-note"'), 'the save\'s note belongs with the save');
  const editor = read('zone_editor.mjs');
  const rows = editor.slice(editor.indexOf('_renderRows() {'), editor.indexOf('_renderDetail() {'));
  assert.match(rows, /className = 'zone-add'/);
  assert.match(rows, /rows\.append\(add\)/);
});

test('a name the loader would refuse is marked while it is typed', () => {
  // The rule is real and the save enforces it; this is so that a field with a
  // rule does not look like free text until the save fails.
  const css = read('style.css');
  assert.match(css, /\.zone-rename\.bad \{[^}]*border-color:\s*var\(--fault\)/);
  const editor = read('zone_editor.mjs');
  assert.match(editor, /addEventListener\('input'[^)]*\)[^]*?classList\.toggle\('bad'/);
});

test('a zone row reads as something you pick, and as picked', () => {
  // Selection drives the panel and the map highlight, so a row that gave no
  // sign of being selectable left the operator clicking a text box instead —
  // and only while it is editable, since a row that cannot be picked must not
  // invite it.
  const css = read('style.css');
  const rows = css.slice(css.indexOf('.zone-rows.editing .zone-row {'));
  assert.match(rows.slice(0, rows.indexOf('}')), /cursor:\s*pointer/);
  assert.match(
    css,
    /\.zone-rows\.editing \.zone-row\.selected \{[^}]*border-color:\s*var\(--accent\)/,
  );
  // One list, one row shape: the cells must not move when the controls arrive.
  assert.match(css, /\.zone-rows \.zone-row \{[^}]*grid-template-columns/);
  assert.ok(!css.includes('#review-zones'), 'the second zone list is gone');
});

test('the editor lives in the review pane, over the revision it edits', () => {
  // Editing anywhere else edits zones against a map they may not belong to: the
  // operations canvas draws the *published* basemap, so an editor on it could
  // only ever derive from the published revision — which is what made renaming
  // a candidate's rooms require promoting the placeholder names first.
  const html = read('index.html');
  const review = html.slice(
    html.indexOf('class="pane review-pane"'),
    html.indexOf('class="pane detail-pane"'),
  );
  for (const id of ['zones-edit', 'zone-editor', 'zone-rows', 'zone-save']) {
    assert.ok(review.includes(`id="${id}"`), `#${id} must be in the review pane`);
  }
  const map = html.slice(
    html.indexOf('class="pane map-pane"'),
    html.indexOf('class="pane review-pane"'),
  );
  assert.ok(!map.includes('zone-editor'), 'the operations map must not edit zones');
});

// -- the vocabulary half --------------------------------------------------
//
// Names, not coordinates: the part of a zone that is true for every robot at
// the site. The rules are the robot's, so what these hold is that this editor
// cannot save a set the robot would refuse to load.

test('the kind decides whether a zone is a point or an area', () => {
  // The two are one decision, not two: a `charger` is a pose to dock at, and a
  // `room` is a place with walls whose whole job is answering "am I in it".
  // Edited separately, they drift into a dropoff carrying an outline nothing
  // reads and a room with no extent `zones.containing` can ever match.
  assert.ok(isAreaKind('room'));
  assert.ok(isAreaKind('keepout'));
  assert.ok(!isAreaKind('charger'));

  // Naming a bare pose an area gives it something to drag onto the walls.
  const waypoint = { name: 'home', kind: 'home', x: 2, y: 3 };
  const room = withKind(waypoint, 'room');
  assert.equal(room.polygon.length, 4);
  assert.ok(pointInPolygon(room.polygon, waypoint.x, waypoint.y));

  // And naming an area a point drops the outline, keeping the pose.
  const back = withKind(room, 'charger');
  assert.equal(back.polygon, undefined);
  assert.deepEqual([back.x, back.y], [2, 3]);
  assert.equal(withKind({ name: 'x', x: 0, y: 0, radius: 1.5 }, 'dock').radius, undefined);

  // A zone that is *only* an outline gets the pose the robot's loader would
  // derive — and if that centre is not inside it, the change is refused rather
  // than putting the pose in a wall.
  const ward = { name: 'ward', polygon: square.polygon };
  assert.deepEqual(withKind(ward, 'dock'), { name: 'ward', kind: 'dock', x: 1, y: 1 });
  const ell = {
    name: 'hall',
    polygon: [
      [0, 0],
      [6, 0],
      [6, 1],
      [1, 1],
      [1, 6],
      [0, 6],
    ],
  };
  assert.equal(poseFor(ell), null);
  assert.equal(withKind(ell, 'charger'), null);

  // An area kind never disturbs an outline that is already there.
  assert.deepEqual(withKind(square, 'corridor').polygon, square.polygon);
});

test('the kind list is the one the bundle validator will accept', () => {
  // A kind outside `bundle.ZONE_KINDS` is refused at the parse, so a dropdown
  // offering one would be an option that fails on save. Read out of the python
  // rather than duplicated in a fixture, so adding a kind there fails here.
  const bundle = readFileSync(
    new URL('../../mote_bringup/mote_bringup/bundle.py', import.meta.url),
    'utf8',
  );
  const kinds = (source, name) =>
    source
      .slice(source.indexOf(`${name} = (`) + name.length + 4)
      .split(')')[0]
      .match(/"[a-z]+"/g)
      .map((quoted) => quoted.slice(1, -1));
  assert.deepEqual(ZONE_KINDS, kinds(bundle, 'ZONE_KINDS'));
  const constraints = bundle
    .slice(bundle.indexOf('CONSTRAINT_KINDS = frozenset(('))
    .split('))')[0]
    .match(/"[a-z]+"/g)
    .map((quoted) => quoted.slice(1, -1));
  assert.deepEqual([...CONSTRAINT_KINDS].sort(), constraints.sort());

  // And the point/area split, which decides what geometry an edit gives a zone.
  const points = bundle
    .slice(bundle.indexOf('POINT_KINDS = frozenset(('))
    .split('))')[0]
    .match(/"[a-z]+"/g)
    .map((quoted) => quoted.slice(1, -1));
  assert.deepEqual([...POINT_KINDS].sort(), points.sort());
});

test('aliases and tags round-trip through one comma-separated field', () => {
  assert.deepEqual(parseList(' galley , The Kitchen ,, galley '), [
    'galley',
    'The Kitchen',
  ]);
  assert.equal(formatList(['galley', 'The Kitchen']), 'galley, The Kitchen');
  assert.deepEqual(parseList(''), []);
  // The resolver's comparison, so collision detection agrees with it.
  assert.equal(normaliseAlias('The  Kitchen'), 'the kitchen');
});

test('the machine name a display name implies', () => {
  // Three name fields is two too many to *type*: an operator names a room, and
  // the identifier follows. It follows as a proposal — put in the name field,
  // visible and editable — because `goto` takes it and a rename nobody asked
  // for breaks the command that names the place.
  assert.equal(slugify('The Kitchen'), 'the_kitchen');
  assert.equal(slugify('Café'), 'cafe'); // the letter survives, not just the accent
  assert.equal(slugify('  spare  room '), 'spare_room');
  assert.equal(slugify('Ward 3B'), 'ward_3b');
  // A name has to start with a letter, so there is nothing to propose here and
  // the placeholder is left alone rather than mangled into one.
  assert.equal(slugify('3rd floor'), '');
  assert.equal(slugify(''), '');
  for (const text of ['The Kitchen', 'Café', 'Ward 3B']) {
    assert.ok(NAME_RE.test(slugify(text)), `${text} did not produce a usable name`);
  }

  // And it only ever replaces a name nobody chose: what `segment-map` mints for
  // a room it found, and what `add zone` mints for a new one.
  assert.ok(isGeneratedName('zone_01'));
  assert.ok(isGeneratedName('zone_07'));
  assert.ok(!isGeneratedName('kitchen'));
  assert.ok(!isGeneratedName('zone_kitchen'));
});

test('two zones answering one query are caught before they are saved', () => {
  // The robot's loader refuses an ambiguous vocabulary outright rather than
  // resolving `goto` by dict order, so a save that produced one would produce a
  // map no robot will load. Display names are not queries and do not collide.
  const clash = ambiguities([
    { name: 'kitchen', aliases: [] },
    { name: 'galley', aliases: ['Kitchen'] },
  ]);
  assert.equal(clash.length, 1);
  assert.match(clash[0], /kitchen/);
  assert.deepEqual(
    ambiguities([
      { name: 'kitchen', display_name: 'The Kitchen', aliases: ['galley'] },
      { name: 'office', display_name: 'The Kitchen', aliases: [] },
    ]),
    [],
  );
});

test('navigable is written only when it deviates from the kind', () => {
  // Every zone arrives from the server with `navigable` filled in from its kind
  // (`bundle.zone_term`), so writing it back verbatim would carry a keepout's
  // `false` onto a zone just changed to `room` — a room nothing can be sent to,
  // with nothing on screen to say why.
  assert.ok(navigableByDefault('room'));
  assert.ok(!navigableByDefault('keepout'));
  const payload = zonesPayload([
    { name: 'kitchen', kind: 'room', navigable: true, x: 0, y: 0, aliases: [] },
    { name: 'sluice', kind: 'keepout', navigable: false, x: 1, y: 1, tags: [] },
    { name: 'workshop', kind: 'room', navigable: false, x: 2, y: 2 },
  ]);
  assert.ok(!('navigable' in payload.kitchen));
  assert.ok(!('navigable' in payload.sluice));
  assert.equal(payload.workshop.navigable, false); // deviant, so it was meant
  // Empty lists say nothing the default does not.
  assert.ok(!('aliases' in payload.kitchen));
  assert.ok(!('tags' in payload.sluice));
});

// -- candidate review -----------------------------------------------------
//
// The read half of the review pane. Its canvas needs a browser like every other
// canvas here, but the part that decides *which map is on it* does not: these
// are the route builders, the list ordering and the promotable verdict, and a
// regression in the first of them is the specific failure the whole view exists
// to remove — the canonical basemap drawn under a candidate's label.

const {
  defaultRevision,
  floorKey,
  floorPath,
  formatBytes,
  orderedRevisions,
  parseFloorKey,
  promotability,
  provenanceRows,
  revisionPath,
  zoneSource,
} = await import('../server/ui/review.mjs');

test('a candidate is read from its own routes, never the canonical basemap', () => {
  for (const leaf of ['map.json', 'map.png', 'zones.json']) {
    const path = revisionPath('home', 'ground', '20260802T145731', leaf);
    assert.equal(path, `/v1/sites/home/floors/ground/revisions/20260802T145731/${leaf}`);
    // /v1/maps/<site>/<floor>/<leaf> serves whatever is *published*. A review
    // view reading those would show the map the operator already has.
    assert.ok(!path.startsWith('/v1/maps/'), `${leaf} must not come from /v1/maps`);
    assert.ok(path.includes('20260802T145731'), `${leaf} must name the revision`);
  }
});

test('a floor is addressed directly, not through a robot', () => {
  assert.equal(floorPath('home', 'ground'), '/v1/sites/home/floors/ground');
  assert.equal(floorKey('home', 'ground'), 'home/ground');
  assert.deepEqual(parseFloorKey('home/ground'), { site: 'home', floor: 'ground' });
  assert.equal(parseFloorKey('home'), null);
  assert.equal(parseFloorKey(''), null);
});

const canonicalRevision = {
  revision: '20260726T120000',
  canonical: true,
  ok: true,
  errors: [],
  warnings: [],
};
const goodRevision = {
  revision: '20260802T145731',
  canonical: false,
  ok: true,
  errors: [],
  warnings: [],
};
const warnedRevision = {
  revision: '20260802T150000',
  canonical: false,
  ok: true,
  errors: [],
  warnings: ['no map.posegraph'],
};
const brokenRevision = {
  revision: '20260803T090000',
  canonical: false,
  ok: false,
  errors: ['the map has no free space'],
  warnings: [],
};

test('revisions are listed newest first, canonical included', () => {
  const detail = { revisions: [canonicalRevision, goodRevision, brokenRevision] };
  assert.deepEqual(
    orderedRevisions(detail).map((entry) => entry.revision),
    [brokenRevision.revision, goodRevision.revision, canonicalRevision.revision],
  );
  assert.deepEqual(orderedRevisions(null), []);
});

test('a floor opens on the newest promotable candidate', () => {
  assert.equal(
    defaultRevision({ revisions: [canonicalRevision, goodRevision] }).revision,
    goodRevision.revision,
  );
  // A floor whose only candidate is broken still opens on it: "why can I not
  // promote this" is the question that brought the operator here.
  assert.equal(
    defaultRevision({ revisions: [canonicalRevision, brokenRevision] }).revision,
    brokenRevision.revision,
  );
  // Nothing but the published map: there is still something to look at.
  assert.equal(
    defaultRevision({ revisions: [canonicalRevision] }).revision,
    canonicalRevision.revision,
  );
  assert.equal(defaultRevision({ revisions: [] }), null);
});

test('the promote button follows the validator, not the view', () => {
  assert.equal(promotability(goodRevision).promotable, true);
  assert.equal(promotability(warnedRevision).promotable, true);
  assert.deepEqual(promotability(warnedRevision).notes, ['no map.posegraph']);
  // Refused by the server too, so refusing the click is honest rather than
  // opinionated — and the reason shown is the server's own.
  assert.equal(promotability(brokenRevision).promotable, false);
  assert.deepEqual(promotability(brokenRevision).notes, ['the map has no free space']);
  assert.equal(promotability(canonicalRevision).promotable, false);
  assert.equal(promotability(null).promotable, false);
});

test('the verdict is a state, not a sentence', () => {
  // This is a control panel: a status is a word beside a coloured dot, the way
  // the roster and the subsystem list say one. It was briefly a question in the
  // heading answered by "yes — no errors. These warnings do not block it:",
  // which said the right thing in the wrong register — and wrapped onto a
  // second line ending in a dangling colon.
  for (const revision of [goodRevision, warnedRevision, brokenRevision, canonicalRevision]) {
    const { verdict } = promotability(revision);
    assert.ok(verdict.split(' ').length <= 3, `"${verdict}" is a sentence, not a state`);
    assert.doesNotMatch(verdict, /[:.]$/);
    assert.doesNotMatch(verdict, /^(yes|no)\b/);
  }
  assert.equal(promotability(goodRevision).verdict, 'promotable');
  assert.equal(promotability(brokenRevision).verdict, 'not promotable');
  assert.equal(promotability(canonicalRevision).verdict, 'already published');
});

test('the state drives the dot, using the classes the page already has', () => {
  assert.equal(promotability(goodRevision).state, 'ok');
  assert.equal(promotability(warnedRevision).state, 'ok');
  assert.equal(promotability(brokenRevision).state, 'fault');
  assert.equal(promotability(canonicalRevision).state, 'unknown');
  // A state string that maps to no styling is an invisible state. `unknown` is
  // the exception by design: it is the base `.dot` colour, so it is expressed
  // by the absence of a modifier rather than by a rule of its own.
  const css = read('style.css');
  for (const state of ['ok', 'fault']) {
    assert.match(css, new RegExp(`\\.dot\\.${state}\\b`), `style.css has no .dot.${state}`);
  }
  assert.match(css, /\.dot \{[^}]*background:\s*var\(--unknown\)/);
});

test('the notes list is captioned with whether it blocks the button', () => {
  // The bar is "no errors". That belongs on the list — it says what the list
  // *is* — rather than inside a verdict standing in for the state.
  assert.match(promotability(warnedRevision).notesLabel, /^warnings\b/);
  assert.match(promotability(warnedRevision).notesLabel, /do not block/);
  assert.match(promotability(brokenRevision).notesLabel, /^errors\b/);
  assert.match(promotability(brokenRevision).notesLabel, /block/);
  // No caption over an empty list, or it becomes a heading for nothing and the
  // break it provides lands in the wrong place.
  assert.equal(promotability(goodRevision).notesLabel, '');
  assert.equal(promotability(canonicalRevision).notesLabel, '');
});

test('provenance is read off the payload the registry already sends', () => {
  const rows = Object.fromEntries(
    provenanceRows({
      ...goodRevision,
      robot_id: 'mote-01',
      uploaded_at: '2026-08-02T14:57:31Z',
      bytes: 2048,
      sha256: 'a'.repeat(64),
      map: { width: 500, height: 300, resolution: 0.05 },
      occupancy: { total: 150000, free: 0.79, occupied: 0.11, unknown: 0.1 },
      meta: { saved: '2026-08-02T14:57:31' },
      zones: ['kitchen', 'ward'],
      files: { 'map.posegraph': 12 },
    }),
  );
  assert.equal(rows.revision, goodRevision.revision);
  assert.equal(rows.from, 'mote-01');
  assert.equal(rows.size, '500x300 px at 0.05 m/px');
  assert.match(rows.occupancy, /79\.0% free/);
  assert.equal(rows.posegraph, 'yes');
  assert.equal(rows['zones in bundle'], '2: kitchen, ward');
  // A revision seeded on disk rather than uploaded has no provenance at all,
  // and saying so beats an empty cell that reads like a missing value.
  const bare = Object.fromEntries(provenanceRows({ ...goodRevision }));
  assert.match(bare.uploaded, /seeded on disk/);
  assert.match(bare.posegraph, /cannot be extended/);
});

test('bytes are readable at every size a revision comes in', () => {
  assert.equal(formatBytes(512), '512 B');
  assert.equal(formatBytes(2048), '2.0 kB');
  assert.equal(formatBytes(5 * 1024 * 1024), '5.0 MB');
  assert.equal(formatBytes(undefined), '—');
});

test('a zone row says which of the three footprints it is', () => {
  assert.equal(
    zoneSummary({ name: 'ward', polygon: [[0, 0], [1, 0], [1, 1]] }),
    'polygon, 3 vertices',
  );
  assert.equal(zoneSummary({ name: 'kitchen', x: 1, y: 2, radius: 1.5 }), 'circle, r 1.5 m');
  assert.equal(zoneSummary({ name: 'pickup', x: 1, y: 2 }), 'waypoint 1.00, 2.00');
});

test('inherited zones are called inherited, and the ordinary case is silent', () => {
  // A revision carrying no zones.yaml inherits the floor's, taught in another
  // SLAM session's frame. They draw perfectly and are wrong by however far the
  // two map origins differ — invisible on the canvas, so it is said.
  assert.equal(zoneSource('floor', 3).tag, 'inherited');
  assert.match(zoneSource('floor', 3).title, /taught on another map/);
  assert.equal(zoneSource('revision', 0).tag, 'none');
  // And zones that belong to the map they are drawn on are just zones: a label
  // for the absence of a problem is a label nobody can act on.
  assert.equal(zoneSource('revision', 3), null);
});

test('review is a mode: opening it stands the operations panes down', () => {
  // Two maps side by side — one canonical with robots on it, one a candidate
  // without — is the confusion a dedicated pane exists to remove, so the rule
  // holds at every width rather than only on a phone.
  const css = read('style.css');
  assert.match(
    css,
    /main:has\(\.review-pane\.active\) > \.pane:not\(\.review-pane\)\s*\{\s*display:\s*none/,
  );
  assert.match(css, /\.review-pane\.active\s*\{\s*display:\s*flex/);
});

test('the review pane has a way out, and it leads to the map', () => {
  const html = read('index.html');
  const review = html.slice(
    html.indexOf('class="pane review-pane"'),
    html.indexOf('class="pane detail-pane"'),
  );
  assert.ok(review.includes('id="review-back"'), 'the review pane has no exit control');

  // An exit only if it names a pane that is *not* this one: `show('review')`
  // on a button labelled `back` looks right in the markup and does nothing.
  const app = read('app.mjs');
  const leave = app.slice(app.indexOf('function onReviewBack('));
  assert.match(leave.slice(0, leave.indexOf('\n}')), /panes\.show\('map'\)/);
  assert.match(app, /dom\.reviewBack\.addEventListener\('click', onReviewBack\)/);
  const key = app.slice(app.indexOf('function onKey('));
  const body = key.slice(0, key.indexOf('\n}'));
  assert.match(body, /event\.key !== 'Escape'/);
  assert.match(body, /onReviewBack\(\)/);
});

test('a promotion only stands the pane down for the floor on screen', () => {
  // The operations map takes its floor from the selected robot, so promoting
  // any other floor and leaving lands the operator on an unrelated map — with
  // the note that says what happened hidden behind the switch.
  const app = read('app.mjs');
  const promoted = app.slice(app.indexOf('function onPromoted('));
  const body = promoted.slice(0, promoted.indexOf('\n}'));
  assert.match(body, /state\.mapKey === `\$\{site\}\/\$\{floor\}`/);
  assert.match(body, /if \(announced && showing\) panes\.show\('map'\)/);
});

test('an edit in progress holds the exit, as it holds the revision list', () => {
  const source = read('review.mjs');
  const controls = source.slice(source.indexOf('renderEditControls()'));
  assert.match(controls.slice(0, controls.indexOf('\n  }')), /this\.dom\.back\.disabled = this\.editing/);
  assert.match(source, /leavable\(\) \{\s*return !this\.editing;/);
});

test('the review pane has every element app.mjs binds to it', () => {
  const html = read('index.html');
  for (const id of [
    'review-jump',
    'review-floors',
    'review-floor',
    'review-canonical',
    'review-revisions',
    'review-verdict',
    'review-verdict-notes',
    'review-notes-label',
    'review-provenance',
    'review-zone-source',
    'review-canvas',
    'review-map-label',
    'review-promote',
    'review-back',
    'review-fit',
    'review-note',
  ]) {
    assert.ok(html.includes(`id="${id}"`), `index.html is missing #${id}`);
  }
  // The map pane's promote picker moved here wholesale: a promote button next
  // to the canonical basemap is a promotion made without seeing the map.
  assert.ok(!html.includes('id="revision"'), 'the map pane still has a promote picker');
});
