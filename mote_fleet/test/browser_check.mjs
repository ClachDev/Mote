// Drive a real browser at a running fleet server and check what it renders.
//
// Not a pytest: this one needs the whole stack up — a websockets broker, the
// fleet server, and at least one robot publishing — so it is an operator's
// verification tool rather than something CI can run. The unit-level halves
// (the MQTT codec, the world→pixel transform) are `ui_test.mjs`, which does run
// in CI; what only a browser can answer is whether the page actually connects
// to the broker over WebSockets, draws the basemap, and dispatches.
//
// `pixi run fleet-ui-check` builds that stack around this file — a broker, a
// server, a basemap and a fake fleet on ports nobody else is using — and is how
// to run these checks without one. Point it at a stack of your own with:
//
//     node mote_fleet/test/browser_check.mjs http://localhost:8080 [token] [out.png] [site/floor]
//
// The trailing `site/floor` is a floor with candidates and no published map; if
// it is given, the bootstrap checks run against it.
//
// It speaks the Chrome DevTools Protocol over node's built-in WebSocket, so it
// needs no npm install: only a chrome/chromium on PATH.

import { spawn } from 'node:child_process';
import { accessSync, constants, mkdtempSync, writeFileSync } from 'node:fs';
import { createServer } from 'node:net';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const url = process.argv[2] || 'http://localhost:8080';
const token = process.argv[3] || '';
const shot = process.argv[4] || 'fleet-ui.png';
const unpublished = process.argv[5] || '';

function onPath(name) {
  return (process.env.PATH || '').split(':').some((dir) => {
    try {
      accessSync(join(dir, name), constants.X_OK);
      return true;
    } catch {
      return false;
    }
  });
}

const CHROME =
  process.env.CHROME ||
  ['google-chrome', 'chromium', 'chromium-browser'].find(onPath) ||
  'google-chrome';

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

// An ephemeral debugging port, so two of these can run at once — a fixed one
// makes a second run attach to the first run's browser.
const freePort = () =>
  new Promise((resolve) => {
    const server = createServer();
    server.listen(0, '127.0.0.1', () => {
      const { port } = server.address();
      server.close(() => resolve(port));
    });
  });

async function devtools(port) {
  for (let attempt = 0; attempt < 50; attempt += 1) {
    try {
      const response = await fetch(`http://127.0.0.1:${port}/json/list`);
      const targets = await response.json();
      const page = targets.find((target) => target.type === 'page');
      if (page) return page.webSocketDebuggerUrl;
    } catch {
      /* chrome is still starting */
    }
    await sleep(200);
  }
  throw new Error('chrome never exposed a debugging target');
}

class Session {
  constructor(socket) {
    this.socket = socket;
    this.id = 0;
    this.waiting = new Map();
    socket.addEventListener('message', (event) => {
      const message = JSON.parse(event.data);
      const pending = this.waiting.get(message.id);
      if (pending) {
        this.waiting.delete(message.id);
        if (message.error) pending.reject(new Error(JSON.stringify(message.error)));
        else pending.resolve(message.result);
      }
    });
  }

  send(method, params = {}) {
    this.id += 1;
    const id = this.id;
    this.socket.send(JSON.stringify({ id, method, params }));
    return new Promise((resolve, reject) => this.waiting.set(id, { resolve, reject }));
  }

  async evaluate(expression) {
    const result = await this.send('Runtime.evaluate', {
      expression,
      awaitPromise: true,
      returnByValue: true,
    });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.text);
    return result.result.value;
  }
}

const checks = [];
function check(name, ok, detail = '') {
  checks.push({ name, ok, detail });
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail ? `  — ${detail}` : ''}`);
}

// Every assertion below is about something that arrives — a WebSocket
// handshake, a retained message, a basemap decode, a robot's reply — so each
// one polls to a deadline instead of sleeping a guessed interval. A fixed
// sleep is either longer than it needs to be or, on a loaded machine, a red
// result about code that is fine; that difference is what decides whether this
// could ever be a gate rather than an operator's tool.
const DEADLINE_MS = 20000;

// How much of a canvas has been drawn on. The only honest answer to "is the map
// there" — every other signal (a decoded image, a loaded payload) is upstream of
// the one thing that can silently go wrong, which is fitting into a canvas that
// had no size when it was fitted.
const paintedPixels = (id) => `(() => {
      const canvas = document.getElementById('${id}');
      const ctx = canvas.getContext('2d');
      const data = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
      let painted = 0;
      for (let i = 3; i < data.length; i += 4) if (data[i] > 0) painted += 1;
      return painted;
    })()`;

async function settle(session, expression, satisfied, timeout = DEADLINE_MS) {
  const deadline = Date.now() + timeout;
  for (;;) {
    const value = await session.evaluate(expression);
    if (satisfied(value) || Date.now() > deadline) return value;
    await sleep(150);
  }
}

const profile = mkdtempSync(join(tmpdir(), 'mote-ui-'));
const debugPort = await freePort();
const chrome = spawn(CHROME, [
  '--headless=new',
  `--remote-debugging-port=${debugPort}`,
  `--user-data-dir=${profile}`,
  '--window-size=1600,900',
  '--no-first-run',
  '--disable-gpu',
  'about:blank',
]);
chrome.on('error', (error) => {
  console.error(`could not start ${CHROME}: ${error.message}`);
  process.exit(2);
});

try {
  const socket = new WebSocket(await devtools(debugPort));
  await new Promise((resolve) => socket.addEventListener('open', resolve));
  const session = new Session(socket);
  await session.send('Runtime.enable');
  await session.send('Page.enable');
  await session.send('Log.enable');

  if (token) {
    await session.send('Page.navigate', { url });
    await settle(session, `!!document.getElementById('broker-state')`, (up) => up);
    await session.evaluate(`localStorage.setItem('mote.operator.token', '${token}')`);
  }
  await session.send('Page.navigate', { url });

  const broker = await settle(
    session,
    `(document.getElementById('broker-state') || {}).className || ''`,
    (className) => className.includes('connected'),
  );
  check(
    'the browser connected to the broker over WebSockets',
    broker.includes('connected'),
    await session.evaluate(
      `(document.getElementById('broker-state') || {}).textContent || 'no page'`,
    ),
  );

  const roster = await settle(
    session,
    `[...document.querySelectorAll('.robot-id')].map(n => n.textContent).join(',')`,
    (ids) => ids.includes('mote-01'),
  );
  check('the roster came from retained MQTT state', roster.includes('mote-01'), roster);

  const health = await settle(
    session,
    `[...document.querySelectorAll('.robot-state')].map(n => n.textContent).join(',')`,
    (states) => /ok|degraded|fault/.test(states),
  );
  check('health states are rendered', /ok|degraded|fault/.test(health), health);

  const mapLabel = await settle(
    session,
    `document.getElementById('map-label').textContent`,
    (label) => label.includes('/'),
  );
  check('a basemap was resolved for the selected robot', mapLabel.includes('/'), mapLabel);

  const drawn = await settle(session, paintedPixels('map-canvas'), (painted) => painted > 10000);
  check('the map canvas has pixels on it', drawn > 10000, `${drawn} painted pixels`);

  const subsystems = await settle(
    session,
    `document.querySelectorAll('#subsystems .subsystem').length`,
    (rows) => rows > 0,
  );
  check('the health roll-up lists subsystems', subsystems > 0, `${subsystems} rows`);

  if (token) {
    await session.evaluate(`(() => {
      document.getElementById('command').value = 'goto dropoff';
      document.getElementById('dispatch').requestSubmit();
    })()`);
    const dispatched = await settle(
      session,
      `document.getElementById('dispatch-note').textContent`,
      (note) => note.startsWith('dispatched'),
    );
    check('dispatch went through the fleet API', dispatched.startsWith('dispatched'), dispatched);

    const statuses = await settle(
      session,
      `[...document.querySelectorAll('#status-log .status-state')].map(n => n.textContent).join(',')`,
      (states) => /succeeded|failed|rejected/.test(states),
      // The assertion is `accepted`; this shorter deadline only buys the rest
      // of the lifecycle when it is cheap (a fake robot's task takes seconds).
      // A real robot's goto takes minutes and must not hold the run open.
      5000,
    );
    check('the robot answered on task/status', statuses.includes('accepted'), statuses);
  }

  const screenshot = await session.send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(shot, Buffer.from(screenshot.data, 'base64'));
  console.log(`screenshot: ${shot}`);

  // -- candidate review ---------------------------------------------------
  //
  // M4's rule is that uploading is not publishing, which leaves an operator a
  // decision to make; until this pane existed the only thing on screen to make
  // it with was a timestamp. Whether they can actually *see* the candidate is a
  // question about a canvas, so only a browser can answer it.

  const signpost = await settle(
    session,
    `(() => {
      const button = document.getElementById('review-jump');
      return button && !button.hidden ? button.textContent : '';
    })()`,
    (text) => text.includes('candidate'),
  );
  check('the map pane signposts the floor’s candidates', signpost.includes('candidate'), signpost);

  await session.evaluate(`document.getElementById('review-jump').click()`);
  const reviewed = await settle(
    session,
    `document.getElementById('review-map-label').textContent`,
    (label) => /\d{8}T\d{6}/.test(label),
  );
  check('the review pane opened on a candidate revision', /\d{8}T\d{6}/.test(reviewed), reviewed);

  const candidateDrawn = await settle(
    session,
    paintedPixels('review-canvas'),
    (painted) => painted > 10000,
  );
  check(
    'the candidate’s own map is drawn on the review canvas',
    candidateDrawn > 10000,
    `${candidateDrawn} painted pixels`,
  );

  // The way out, which above 760 px is the pane's own control and nothing else:
  // the tab bar the phone leaves by is hidden here, and review stands every
  // other pane down, so a pane with no exit of its own is a trap that ends in a
  // window resize or a reload (what shipped with the pane, 2026-08-11).
  const exits = await session.evaluate(`(() => {
    const active = () => [...document.querySelectorAll('.pane')]
      .filter(pane => pane.classList.contains('active'))
      .map(pane => pane.dataset.pane).join(',');
    const out = { width: innerWidth, tabs: getComputedStyle(document.querySelector('.panes')).display };
    document.getElementById('review-back').click();
    out.button = active();
    // The operations panes are only *displayed* again if the review mode rule
    // has let go: the class is what that rule keys on, so both are read.
    out.shown = getComputedStyle(document.querySelector('.map-pane')).display;
    document.getElementById('review-jump').click();
    out.reopened = active();
    document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }));
    out.escape = active();
    return out;
  })()`);
  check(
    'the review pane can be left at desk width, by its button and by Escape',
    exits.tabs === 'none' &&
      exits.button === 'map' &&
      exits.shown !== 'none' &&
      exits.reopened === 'review' &&
      exits.escape === 'map',
    JSON.stringify(exits),
  );

  // Back in, for the rest of the review checks.
  await session.evaluate(`document.getElementById('review-jump').click()`);
  await settle(
    session,
    `document.getElementById('review-map-label').textContent`,
    (label) => /\d{8}T\d{6}/.test(label),
  );

  // The fixture's candidate is the published map mirrored, so a review pane
  // that fetched the canonical image — the defect this replaces — would draw a
  // perfectly convincing map. The URL is what separates the two.
  const source = await session.evaluate(
    `[...performance.getEntriesByType('resource')]
       .map(entry => new URL(entry.name).pathname)
       .filter(path => path.endsWith('/map.png')).join(' ')`,
  );
  check(
    'the review pane fetched a revision’s image, not the canonical basemap',
    /\/revisions\/\d{8}T\d{6}\/map\.png/.test(source),
    source,
  );

  const zoneRows = await settle(
    session,
    `document.querySelectorAll('#zone-rows .zone-row').length`,
    (rows) => rows > 0,
  );
  check('the candidate’s zones are listed beside it', zoneRows > 0, `${zoneRows} rows`);

  const verdict = await session.evaluate(`(() => ({
    verdict: document.getElementById('review-verdict').textContent,
    enabled: !document.getElementById('review-promote').disabled,
    facts: document.querySelectorAll('#review-provenance .fact').length,
  }))()`);
  check(
    'the pane says why the revision is promotable, and offers to',
    verdict.enabled && verdict.facts > 0,
    JSON.stringify(verdict),
  );

  const reviewShot = shot.replace(/(\.png)?$/, '-review.png');
  const reviewPng = await session.send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(reviewShot, Buffer.from(reviewPng.data, 'base64'));
  console.log(`screenshot: ${reviewShot}`);

  // -- editing the candidate's zones --------------------------------------
  //
  // The other half of the review decision: a build's rooms arrive as `zone_01`
  // ..`zone_07`, and naming them is done here, against the candidate's own map,
  // by an operator who can see which room is which. Saving derives a *new*
  // candidate from the one on screen — so the map the coordinates were drawn on
  // is the map they are stored against, and nothing published moves.

  if (token) {
    // The map must not move when the editor opens: the editing surface is
    // taller than the list it replaces, and with the canvas taking whatever
    // height was left, clicking `edit zones` resized the thing being edited.
    const shapeOf = `(() => {
      const rows = [...document.querySelectorAll('#zone-rows .zone-row')];
      return JSON.stringify({
        map: Math.round(document.getElementById('review-canvas').getBoundingClientRect().height),
        rows: rows.map(row => Math.round(row.getBoundingClientRect().height)),
        top: rows.length ? Math.round(rows[0].getBoundingClientRect().top) : 0,
        cells: rows.length ? rows[0].children.length : 0,
      });
    })()`;
    const shapeBefore = await session.evaluate(shapeOf);
    const mapBefore = JSON.parse(shapeBefore).map;
    const editing = await session.evaluate(`(() => {
      document.getElementById('zones-edit').click();
      const rows = document.getElementById('zone-rows');
      const shown = (id) => !document.getElementById(id).hidden;
      return {
        rows: document.querySelectorAll('#zone-rows .zone-row').length,
        // One list in both modes: the rows that were text now hold controls.
        controls: document.querySelectorAll('#zone-rows select').length,
        listLocked: [...document.querySelectorAll('#review-revisions button')]
          .every(button => button.disabled),
        promoteLocked: document.getElementById('review-promote').disabled,
        // The edit ends where it began, and adding a zone is the end of the
        // list rather than a third button beside the two that end the edit.
        ends: !shown('zones-edit') && shown('zone-save') && shown('zone-cancel'),
        adds: rows.lastElementChild && rows.lastElementChild.className === 'zone-add',
      };
    })()`);
    check(
      'editing opens on the selected revision’s zones and holds it still',
      editing.rows > 0 &&
        editing.controls === editing.rows &&
        editing.listLocked &&
        editing.promoteLocked,
      JSON.stringify(editing),
    );
    check(
      'the edit ends where it began, and `add zone` is part of the list',
      editing.ends && editing.adds,
      JSON.stringify({ ends: editing.ends, adds: editing.adds }),
    );

    // The way out is held with them. An edit has no autosave, so leaving would
    // strand it on a canvas nobody can see; `cancel` is how an edit ends.
    const held = await session.evaluate(`(() => {
      document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape' }));
      document.getElementById('review-back').click();
      return {
        disabled: document.getElementById('review-back').disabled,
        pane: [...document.querySelectorAll('.pane')]
          .filter(pane => pane.classList.contains('active'))
          .map(pane => pane.dataset.pane).join(','),
      };
    })()`);
    check(
      'an edit in progress holds the exit, by button and by Escape',
      held.disabled && held.pane === 'review',
      JSON.stringify(held),
    );

    // One list, one shape: the rows are the same rows, in the same place, the
    // same height, with the same cells — `edit zones` puts controls in them and
    // changes nothing else. Every part of that has been wrong at least once: a
    // second list with its own columns, a box drawn round the editing one, and
    // a selection border that grew every row by two pixels.
    const shapeDuring = await session.evaluate(shapeOf);
    check(
      'the list and the map are the same shape, edited or not',
      shapeBefore === shapeDuring && JSON.parse(shapeBefore).map > 0,
      `${shapeBefore} then ${shapeDuring}`,
    );

    // A row carries identity and shape; everything else is a form for the one
    // selected zone, which is what lets zone/v0 grow a field without giving
    // every row another column.
    const selected = await session.evaluate(`(() => {
      document.querySelectorAll('#zone-rows .zone-name')[0].click();
      const row = document.querySelectorAll('#zone-rows .zone-row')[0];
      return {
        cells: row.children.length,
        marked: row.classList.contains('selected'),
        inputsInRow: row.querySelectorAll('input').length,
        fields: [...document.querySelectorAll('#zone-detail .zone-field-name')]
          .map(node => node.textContent).join(','),
      };
    })()`);
    check(
      'picking a zone by name marks the row and opens its fields',
      selected.cells === 5 &&
        selected.marked &&
        selected.inputsInRow === 0 &&
        selected.fields.startsWith('name,'),
      JSON.stringify(selected),
    );

    // A vocabulary the robot's loader would *refuse* — two zones answering one
    // query — must not leave the browser: it would be stored as a candidate
    // that looks fine and cannot be loaded. The alias is set through the panel
    // the row selects, which is where every field but identity and shape lives.
    const refused = await session.evaluate(`(() => {
      const rows = [...document.querySelectorAll('#zone-rows .zone-row')];
      const first = rows[0].querySelector('.zone-name').textContent;
      rows[1].querySelector('.zone-name').click();
      const field = [...document.querySelectorAll('#zone-detail .zone-field')]
        .find(row => row.textContent.startsWith('also called'));
      const aliases = field.querySelector('input');
      aliases.value = first.toUpperCase();
      aliases.dispatchEvent(new Event('change'));
      const note = document.getElementById('zone-note');
      const top = () => Math.round(document.querySelector('#zone-rows .zone-row').getBoundingClientRect().top);
      const before = { top: top(), note: Math.round(note.getBoundingClientRect().height) };
      document.getElementById('zone-save').click();
      return {
        note: note.textContent,
        field: !!field,
        before,
        // The message takes its room from the list, which is the one thing in
        // the box that scrolls — so nothing above it moves, and an empty one
        // leaves no gap over the first zone.
        after: { top: top(), note: Math.round(note.getBoundingClientRect().height) },
        whole: note.scrollHeight <= note.clientHeight,
      };
    })()`);
    check(
      'an ambiguous vocabulary is refused in the browser, not stored',
      /both answer to/.test(refused.note),
      refused.note,
    );
    check(
      'the refusal is readable in full and moves nothing above it',
      refused.before.note === 0 &&
        refused.after.note > 0 &&
        refused.before.top === refused.after.top &&
        refused.whole,
      JSON.stringify({ ...refused.before, ...refused.after, whole: refused.whole }),
    );

    await session.evaluate(`(() => {
      const detailField = (label) =>
        [...document.querySelectorAll('#zone-detail .zone-field')]
          .find(row => row.textContent.startsWith(label))
          .querySelector('input, select');
      const set = (label, value) => {
        const control = detailField(label);
        control.value = value;
        control.dispatchEvent(new Event('change'));
      };
      set('also called', '');                       // undo the clash above
      const rows = [...document.querySelectorAll('#zone-rows .zone-row')];
      rows[0].querySelector('.zone-name').click();  // the row is a list, so pick
      const name = document.querySelector('.zone-rename');
      const renamed = name.value + '_named';
      name.value = renamed;                         // and rename in the panel
      name.dispatchEvent(new Event('change'));
      const fresh = [...document.querySelectorAll('#zone-rows .zone-row')][0];
      fresh.querySelector('select').value = 'room';
      fresh.querySelector('select').dispatchEvent(new Event('change'));
      set('display name', 'A Named Room');
      document.getElementById('zone-save').click();
      return renamed;
    })()`);
    const derived = await settle(
      session,
      `(() => ({
        // The line under the zones, which is where the work was: what the save
        // did outlives the save, rather than a flash of 'saving...' here with
        // the result reported in the far column.
        note: document.getElementById('zone-note').textContent,
        label: document.getElementById('review-map-label').textContent,
        // Hidden means the zones belong to the revision they are drawn on,
        // which is the ordinary case and says nothing.
        inherited: !document.getElementById('review-zone-source').hidden,
        zones: [...document.querySelectorAll('#zone-rows .zone-row')]
          .map(row => row.textContent).join(' '),
        editorHidden: document.getElementById('zone-editor').hidden,
        // And the head is back to the one control that starts an edit.
        editVisible: !document.getElementById('zones-edit').hidden,
      }))()`,
      (state) => /saved from \d{8}T\d{6}/.test(state.note),
    );
    check(
      'saving derives a candidate from the revision under review',
      /saved from \d{8}T\d{6}/.test(derived.note),
      derived.note,
    );
    // The edited set is on screen afterwards because it was *read back* from the
    // new candidate — not held over as an overlay of what was typed. And the
    // zone renamed above went in as a bare waypoint and comes back as a
    // polygon, because calling it a `room` is what gave it an outline: the kind
    // and the geometry are one decision, made once.
    check(
      'the pane then shows the saved candidate’s own zones',
      derived.editorHidden &&
        derived.editVisible &&
        derived.zones.includes('A Named Room') &&
        !derived.inherited,
      JSON.stringify(derived),
    );
    check(
      'calling a bare pose a room gave it an outline, all the way to the server',
      /A Named Roomroom\s*polygon/.test(derived.zones),
      derived.zones,
    );

    // And that outline is on the map's own pixel grid. A vertex anywhere inside
    // a pixel covers the same cells as one at its centre, so free-hand
    // coordinates are precision the map cannot back — and two zones meant to
    // share a wall land millimetres apart, differently each time.
    const onGrid = await session.evaluate(`(async () => {
      const label = document.getElementById('review-map-label').textContent;
      const [floor, revision] = label.split(' · ');
      const base = '/v1/sites/' + floor.replace('/', '/floors/') + '/revisions/' + revision;
      const [map, zones] = await Promise.all([
        fetch(base + '/map.json').then(r => r.json()),
        fetch(base + '/zones.json').then(r => r.json()),
      ]);
      const centred = (value, origin) => {
        const cells = (value - origin) / map.resolution - 0.5;
        return Math.abs(cells - Math.round(cells)) < 1e-6;
      };
      const drawn = zones.zones.filter(zone => zone.polygon);
      const off = drawn.flatMap(zone => (zone.polygon || [])
        .filter(([x, y]) => !(centred(x, map.origin[0]) && centred(y, map.origin[1])))
        .map(point => zone.name + ' ' + point.join(',')));
      return { outlines: drawn.length, off };
    })()`);
    check(
      'the outline it drew sits on the map’s pixel grid',
      onGrid.outlines > 0 && onGrid.off.length === 0,
      JSON.stringify(onGrid),
    );
    check(
      'the edited revision is still a candidate: nothing published moved',
      !new RegExp(`${derived.label.split(' · ')[1]}`).test(
        await session.evaluate(`document.getElementById('review-canonical').textContent`),
      ),
      `canonical: ${await session.evaluate(
        `document.getElementById('review-canonical').textContent`,
      )}`,
    );

    const editShot = shot.replace(/(\.png)?$/, '-zones.png');
    const editPng = await session.send('Page.captureScreenshot', { format: 'png' });
    writeFileSync(editShot, Buffer.from(editPng.data, 'base64'));
    console.log(`screenshot: ${editShot}`);
  }

  // -- the first promotion on a floor -------------------------------------
  //
  // A floor whose only revisions are candidates. Its detail used to be fetched
  // only after a basemap had loaded — behind an early return — so it listed no
  // candidates at all and its first promotion could never be made here.

  if (unpublished && token) {
    const opened = await session.evaluate(`(() => {
      const select = document.getElementById('review-floor');
      const option = [...select.options].find(o => o.value === ${JSON.stringify(unpublished)});
      if (!option) return '';
      select.value = option.value;
      select.dispatchEvent(new Event('change'));
      return option.value;
    })()`);
    check(`${unpublished} is in the floor picker`, opened === unpublished, opened || 'not listed');

    // Settled on the *label*, not on the row count: the previous floor's rows
    // are still on screen while this one's detail is in flight, so a predicate
    // of "some rows exist" is satisfied before anything has changed and the
    // promote below then fires at the wrong floor's revision.
    const listed = await settle(
      session,
      `(() => ({
        canonical: document.getElementById('review-canonical').textContent,
        label: document.getElementById('review-map-label').textContent,
        rows: document.querySelectorAll('#review-revisions .revision-row').length,
        promotable: !document.getElementById('review-promote').disabled,
      }))()`,
      (state) => state.label.startsWith(unpublished),
    );
    check(
      'a floor with nothing published still lists its candidates',
      listed.rows > 0 && listed.promotable && /nothing published/.test(listed.canonical),
      JSON.stringify(listed),
    );

    await session.evaluate(`document.getElementById('review-promote').click()`);
    const promoted = await settle(
      session,
      `document.getElementById('review-note').textContent`,
      (note) => /is on \d{8}T\d{6}/.test(note),
    );
    check('the first promotion on a floor goes through', /is on /.test(promoted), promoted);

    // And the pane it was made in stands down: the decision it exists for has
    // been made, and the operations map is what an operator wants next.
    const landed = await session.evaluate(`[...document.querySelectorAll('.pane')]
      .filter(pane => pane.classList.contains('active'))
      .map(pane => pane.dataset.pane).join(',')`);
    check('a promotion hands the screen back to the operations map', landed === 'map', landed);
  }

  // -- the phone ----------------------------------------------------------
  //
  // The off-LAN client is a phone, and a phone is not a narrow desktop: it has
  // one pane's worth of screen and no wheel to zoom the map with. Emulated
  // here — a real device is still the acceptance (README §9), because emulation
  // gets the viewport and the touch points right and the thumb wrong.

  await session.send('Emulation.setDeviceMetricsOverride', {
    width: 390,
    height: 844,
    deviceScaleFactor: 3,
    mobile: true,
  });
  await session.send('Emulation.setTouchEmulationEnabled', {
    enabled: true,
    maxTouchPoints: 5,
  });
  await session.send('Page.navigate', { url });
  // The phone checks click through the tab bar and select a robot, so the
  // reload has to have got as far as a populated roster — waited for, not
  // slept through, for the reason `settle` exists.
  await settle(
    session,
    `document.querySelectorAll('.robot').length`,
    (rows) => rows > 0,
  );

  check(
    'a coarse pointer is what the page thinks it has',
    await session.evaluate(`window.matchMedia('(pointer: coarse)').matches`),
  );

  const onePane = await session.evaluate(`(() => ({
    tabs: getComputedStyle(document.querySelector('.panes')).display,
    shown: [...document.querySelectorAll('.pane')]
      .filter(p => getComputedStyle(p).display !== 'none').length,
  }))()`);
  check(
    'one pane at a time, with a tab bar to move between them',
    onePane.tabs === 'flex' && onePane.shown === 1,
    JSON.stringify(onePane),
  );

  // Nothing may scroll sideways: a pane wider than the screen hides whatever is
  // off its right edge — which is where the map's follow and fit buttons are.
  const sideways = await session.evaluate(`(() => {
    const out = [];
    for (const tab of document.querySelectorAll('.panes [data-pane]')) {
      tab.click();
      const pane = document.querySelector('.pane.active');
      if (pane.scrollWidth > pane.clientWidth ||
          document.documentElement.scrollWidth > window.innerWidth) out.push(tab.dataset.pane);
    }
    return out.join(',');
  })()`);
  check('no pane scrolls sideways on a phone', sideways === '', sideways);

  // The review canvas is hidden at *every* width until its pane is opened, so
  // unlike the fleet map it is never fitted at load. If `shown()` is not wired
  // through the tab bar it measures 0x0, fits to a scale of 0, and stays blank
  // for good — with nothing on screen to say why.
  await session.evaluate(`document.querySelector('.panes [data-pane="review"]').click()`);
  const phoneReview = await settle(
    session,
    paintedPixels('review-canvas'),
    (painted) => painted > 10000,
  );
  check(
    'the review canvas fits itself when its pane is opened on a phone',
    phoneReview > 10000,
    `${phoneReview} painted pixels`,
  );
  const reviewPhoneShot = shot.replace(/(\.png)?$/, '-review-phone.png');
  const reviewPhonePng = await session.send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(reviewPhoneShot, Buffer.from(reviewPhonePng.data, 'base64'));
  console.log(`screenshot: ${reviewPhoneShot}`);

  // Switching panes changes the canvas's height but not its width. Resizing on
  // width alone leaves a backing store `clearRect` cannot fully reach, and the
  // previous frame stays visible along the bottom.
  const backing = await session.evaluate(`(() => {
    document.querySelector('.panes [data-pane="detail"]').click();
    document.querySelector('.panes [data-pane="map"]').click();
    const c = document.getElementById('map-canvas');
    const r = c.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    return { w: c.width, h: c.height, want: [Math.round(r.width * ratio), Math.round(r.height * ratio)] };
  })()`);
  check(
    'the canvas backing store follows the pane it is in',
    backing.w === backing.want[0] && backing.h === backing.want[1],
    `${backing.w}x${backing.h} for ${backing.want.join('x')}`,
  );

  const jumped = await session.evaluate(`(() => {
    document.querySelector('.panes [data-pane="roster"]').click();
    document.querySelector('.robot').click();
    return document.querySelector('.pane.active').dataset.pane;
  })()`);
  check('picking a robot in the roster shows it on the map', jumped === 'map', jumped);

  // Pinch. The wheel handler has no touch equivalent, so this is the gesture
  // that decides whether the map is usable at all on a phone.
  const canvas = await session.evaluate(`(() => {
    const r = document.getElementById('map-canvas').getBoundingClientRect();
    return { x: r.x + r.width / 2, y: r.y + r.height / 2 };
  })()`);
  const paintHash = `(() => {
      const c = document.getElementById('map-canvas');
      const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
      let h = 0;
      for (let i = 0; i < d.length; i += 997) h = (h * 31 + d[i]) >>> 0;
      return h;
    })()`;
  const touch = (type, points) =>
    session.send('Input.dispatchTouchEvent', {
      type,
      touchPoints: points.map(([x, y], id) => ({ x, y, id })),
    });
  const before = await session.evaluate(paintHash);
  await touch('touchStart', [
    [canvas.x - 40, canvas.y],
    [canvas.x + 40, canvas.y],
  ]);
  await touch('touchMove', [
    [canvas.x - 100, canvas.y],
    [canvas.x + 100, canvas.y],
  ]);
  await touch('touchMove', [
    [canvas.x - 160, canvas.y],
    [canvas.x + 160, canvas.y],
  ]);
  await touch('touchEnd', []);
  // The redraw is a frame away, not a fixed interval away.
  const after = await settle(session, paintHash, (hash) => hash !== before, 3000);
  check('two fingers zoom the map', after !== before);

  const phoneShot = shot.replace(/(\.png)?$/, '-phone.png');
  const phonePng = await session.send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(phoneShot, Buffer.from(phonePng.data, 'base64'));
  console.log(`screenshot: ${phoneShot}`);

  const errors = await session.evaluate(`window.__errors ? window.__errors.length : 0`);
  check('no uncaught page errors', errors === 0, String(errors));
} finally {
  chrome.kill();
}

const failed = checks.filter((entry) => !entry.ok);
console.log(`\n${checks.length - failed.length}/${checks.length} checks passed`);
process.exit(failed.length ? 1 : 0);
