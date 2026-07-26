// Drive a real browser at a running fleet server and check what it renders.
//
// Not a pytest: this one needs the whole stack up — a websockets broker, the
// fleet server, and at least one robot publishing — so it is an operator's
// verification tool rather than something CI can run. The unit-level halves
// (the MQTT codec, the world→pixel transform) are `ui_test.mjs`, which does run
// in CI; what only a browser can answer is whether the page actually connects
// to the broker over WebSockets, draws the basemap, and dispatches.
//
//     node mote_fleet/test/browser_check.mjs http://localhost:8080 [token] [out.png]
//
// It speaks the Chrome DevTools Protocol over node's built-in WebSocket, so it
// needs no npm install: only a chrome/chromium on PATH.

import { spawn } from 'node:child_process';
import { mkdtempSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const url = process.argv[2] || 'http://localhost:8080';
const token = process.argv[3] || '';
const shot = process.argv[4] || 'fleet-ui.png';
const CHROME =
  process.env.CHROME ||
  ['google-chrome', 'chromium', 'chromium-browser'].find(Boolean);

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

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

const profile = mkdtempSync(join(tmpdir(), 'mote-ui-'));
const chrome = spawn(CHROME, [
  '--headless=new',
  '--remote-debugging-port=9333',
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
  const socket = new WebSocket(await devtools(9333));
  await new Promise((resolve) => socket.addEventListener('open', resolve));
  const session = new Session(socket);
  await session.send('Runtime.enable');
  await session.send('Page.enable');
  await session.send('Log.enable');

  if (token) {
    await session.send('Page.navigate', { url });
    await sleep(1000);
    await session.evaluate(`localStorage.setItem('mote.operator.token', '${token}')`);
  }
  await session.send('Page.navigate', { url });
  // Long enough for the config fetch, the WebSocket handshake, the retained
  // messages and the basemap decode.
  await sleep(4000);

  check(
    'the browser connected to the broker over WebSockets',
    (await session.evaluate(`document.getElementById('broker-state').className`)).includes(
      'connected',
    ),
    await session.evaluate(`document.getElementById('broker-state').textContent`),
  );

  const roster = await session.evaluate(
    `[...document.querySelectorAll('.robot-id')].map(n => n.textContent).join(',')`,
  );
  check('the roster came from retained MQTT state', roster.includes('mote-01'), roster);

  const health = await session.evaluate(
    `[...document.querySelectorAll('.robot-state')].map(n => n.textContent).join(',')`,
  );
  check('health states are rendered', /ok|degraded|fault/.test(health), health);

  const mapLabel = await session.evaluate(
    `document.getElementById('map-label').textContent`,
  );
  check('a basemap was resolved for the selected robot', mapLabel.includes('/'), mapLabel);

  const drawn = await session.evaluate(`(() => {
    const canvas = document.getElementById('map-canvas');
    const ctx = canvas.getContext('2d');
    const data = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
    let painted = 0;
    for (let i = 3; i < data.length; i += 4) if (data[i] > 0) painted += 1;
    return painted;
  })()`);
  check('the map canvas has pixels on it', drawn > 10000, `${drawn} painted pixels`);

  const subsystems = await session.evaluate(
    `document.querySelectorAll('#subsystems .subsystem').length`,
  );
  check('the health roll-up lists subsystems', subsystems > 0, `${subsystems} rows`);

  if (token) {
    const dispatched = await session.evaluate(`(async () => {
      document.getElementById('command').value = 'goto dropoff';
      document.getElementById('dispatch').requestSubmit();
      await new Promise(r => setTimeout(r, 1500));
      return document.getElementById('dispatch-note').textContent;
    })()`);
    check('dispatch went through the fleet API', dispatched.startsWith('dispatched'), dispatched);

    await sleep(2500);
    const statuses = await session.evaluate(
      `[...document.querySelectorAll('#status-log .status-state')].map(n => n.textContent).join(',')`,
    );
    check('the robot answered on task/status', statuses.includes('accepted'), statuses);
  }

  const screenshot = await session.send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(shot, Buffer.from(screenshot.data, 'base64'));
  console.log(`screenshot: ${shot}`);

  const errors = await session.evaluate(`window.__errors ? window.__errors.length : 0`);
  check('no uncaught page errors', errors === 0, String(errors));
} finally {
  chrome.kill();
}

const failed = checks.filter((entry) => !entry.ok);
console.log(`\n${checks.length - failed.length}/${checks.length} checks passed`);
process.exit(failed.length ? 1 : 0);
