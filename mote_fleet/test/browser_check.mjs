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
//     node mote_fleet/test/browser_check.mjs http://localhost:8080 [token] [out.png]
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

  const drawn = await settle(
    session,
    `(() => {
      const canvas = document.getElementById('map-canvas');
      const ctx = canvas.getContext('2d');
      const data = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
      let painted = 0;
      for (let i = 3; i < data.length; i += 4) if (data[i] > 0) painted += 1;
      return painted;
    })()`,
    (painted) => painted > 10000,
  );
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
