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
  zoneOutline,
} from '../server/ui/map.mjs';
import { NARROW_MAX_PX } from '../server/ui/layout.mjs';

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

const { pointInPolygon, nearestVertex, nearestEdge, withVertex, withInsertedVertex, withoutVertex, translated, withPose, freshZone, zonesPayload, NAME_RE } = await import(
  '../server/ui/zone_editor.mjs'
);

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
  const html = read('index.html');
  for (const id of ['zone-editor', 'zone-rows', 'zone-add', 'zone-save', 'zone-cancel', 'zones-edit']) {
    assert.ok(html.includes(`id="${id}"`), `index.html is missing #${id}`);
  }
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
  zoneSummary,
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

test('the verdict answers the question and says what the bar is', () => {
  // The heading asks "can this be promoted?". A verdict that only classifies
  // the revision ("valid, with warnings") over a list of complaints answers a
  // different question and reads as its own counter-evidence — an operator
  // looking at the first build of this pane asked what the answer was.
  assert.match(promotability(goodRevision).verdict, /^yes\b/);
  assert.match(promotability(warnedRevision).verdict, /^yes\b/);
  assert.match(promotability(brokenRevision).verdict, /^no\b/);
  // The bar is "no errors", and warnings are explicitly not part of it.
  assert.match(promotability(warnedRevision).verdict, /no errors/);
  assert.match(promotability(warnedRevision).verdict, /do not block/);
  // A list with nothing in it must not be introduced as though it had
  // something in it.
  assert.doesNotMatch(promotability(goodRevision).verdict, /:$/);
  assert.match(promotability(warnedRevision).verdict, /:$/);
  assert.match(promotability(brokenRevision).verdict, /:$/);
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

test('inherited zones are called inherited, because coordinates cannot say so', () => {
  // A revision carrying no zones.yaml inherits the floor's, taught in another
  // SLAM session's frame. They draw perfectly and are wrong by however far the
  // two map origins differ — invisible on the canvas, so it is said in words.
  assert.match(zoneSource('floor', 3), /inherited from the floor/);
  assert.match(zoneSource('revision', 3), /own frame/);
  assert.match(zoneSource('revision', 0), /carries no zones/);
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
    'review-provenance',
    'review-zones',
    'review-zone-source',
    'review-canvas',
    'review-map-label',
    'review-promote',
    'review-fit',
    'review-note',
  ]) {
    assert.ok(html.includes(`id="${id}"`), `index.html is missing #${id}`);
  }
  // The map pane's promote picker moved here wholesale: a promote button next
  // to the canonical basemap is a promotion made without seeing the map.
  assert.ok(!html.includes('id="revision"'), 'the map pane still has a promote picker');
});
