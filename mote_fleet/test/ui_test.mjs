// The dashboard's two load-bearing pure pieces, under node.
//
// Everything else in the UI is DOM and canvas, which a browser is the only
// honest place to test. These two are not: the MQTT codec decides whether the
// read path works at all, and the world→pixel transform decides whether a robot
// is drawn where it actually is. Both are exported from files the browser loads
// unchanged — `.mjs` is what lets node import them with no package.json and no
// build step.
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
  assert.deepEqual(panes, ['roster', 'map', 'detail']);
  // A pane with no tab is simply unreachable on a phone, and nothing says so.
  assert.deepEqual([...tabs].sort(), [...panes].sort());
});

test('exactly one pane starts active, so a phone opens on something', () => {
  const html = read('index.html');
  const active = [...html.matchAll(/class="pane [^"]*\bactive\b[^"]*"/g)];
  assert.equal(active.length, 1);
});

test('the map canvas takes its own touch gestures', () => {
  // Without `touch-action: none` the browser consumes the drag and the pinch
  // before the canvas sees a single pointer event.
  const css = read('style.css');
  const canvas = css.slice(css.indexOf('#map-canvas {'));
  assert.match(canvas.slice(0, canvas.indexOf('}')), /touch-action:\s*none/);
});
