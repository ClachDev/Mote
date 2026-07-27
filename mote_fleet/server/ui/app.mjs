// The thin fleet dashboard: roster, live map, and dispatch.
//
// The read path is the broker — every robot's presence, health, pose and task
// status arrives over MQTT-over-WebSockets, retained, so this page shows the
// state of the fleet the instant it loads and never polls (fleet.md Q5). The
// write path is the fleet API: `POST /v1/robots/<id>/dispatch` authorizes the
// operator and writes an audit line before anything reaches `task/command`.
//
// The two paths are deliberately different, and the browser holds no credential
// that can publish to the broker. Since M7 that is enforced twice over: this
// page's MQTT client has no PUBLISH encoder, *and* the broker credential it is
// given — the operator's own, from /v1/config — has no write rule in the
// broker's ACL. Both credentials arrive together and die together, because
// revoking the operator removes the row both are read from.
//
// What this view is *not* is a Foxglove replacement. It answers "where is every
// robot, what is each doing, send one somewhere" and hands the single-robot
// deep view (3D, sensors, teleop) to Foxglove by deep link.

import { BrokerReader, parseTopic } from './mqtt.mjs';
import { MapView } from './map.mjs';

const TOKEN_KEY = 'mote.operator.token';

// A pose older than this is drawn hollow: retained state is the *last known*
// position, which is not the same claim as "the robot is there now".
const POSE_STALE_S = 20;

// The same rule for health, and it matters more: an offline robot's retained
// health is whatever it last claimed, and eight green subsystems next to a robot
// that is not there is the one lie this view must not tell. The agent publishes
// every 5 s by default, so this is several missed heartbeats rather than a blip.
const HEALTH_STALE_S = 30;

const state = {
  config: null,
  robots: new Map(),
  selected: null,
  operator: null,
  mapKey: null,
};

const dom = {};
let mapView = null;
let pending = false;
let reader = null;

// -- small helpers -------------------------------------------------------

function el(tag, attributes = {}, children = []) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attributes)) {
    if (key === 'class') node.className = value;
    else if (key === 'text') node.textContent = value;
    else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
    else if (value !== null && value !== undefined) node.setAttribute(key, value);
  }
  for (const child of [].concat(children)) {
    if (child) node.append(child);
  }
  return node;
}

function ageSeconds(stamp) {
  if (!stamp) return null;
  const parsed = Date.parse(stamp);
  return Number.isNaN(parsed) ? null : (Date.now() - parsed) / 1000;
}

function ageText(stamp) {
  const age = ageSeconds(stamp);
  if (age === null) return '—';
  if (age < 60) return `${Math.max(0, Math.round(age))}s ago`;
  if (age < 3600) return `${Math.round(age / 60)}m ago`;
  return `${Math.round(age / 3600)}h ago`;
}

function robotRecord(robotId) {
  if (!state.robots.has(robotId)) {
    state.robots.set(robotId, { id: robotId, statuses: [] });
  }
  return state.robots.get(robotId);
}

// A robot's single roll-up state: offline beats whatever health it last
// claimed, because that health is by definition from before it dropped.
function robotState(record) {
  if (record.presence && record.presence.online === false) return 'offline';
  if (!record.health) return 'unknown';
  // Health that has stopped arriving is reported as stale rather than as the
  // last thing it said — the contract has a state for exactly this.
  if (!healthIsCurrent(record)) return 'stale';
  return record.health.state || 'unknown';
}

function robotLabel(record) {
  const name = record.registry && record.registry.name;
  return name && name !== record.id ? `${record.id} · ${name}` : record.id;
}

// Is what we know about this robot's health still a claim about *now*? Offline
// robots and silent ones both fail this, and the answer drives every green dot
// on the page.
function healthIsCurrent(record) {
  if (!record.health) return false;
  if (record.presence && record.presence.online === false) return false;
  const age = ageSeconds(record.health.stamp);
  return age === null || age <= HEALTH_STALE_S;
}

// What to say instead of a health summary when it is not current.
function staleReason(record) {
  if (record.presence && record.presence.online === false) {
    return `offline (${record.presence.reason || 'no reason given'}) — last seen ${ageText(
      record.presence.stamp,
    )}`;
  }
  if (!record.health) return 'never reported';
  return `no health for ${ageText(record.health.stamp).replace(' ago', '')}`;
}

// -- data in -------------------------------------------------------------

async function api(path, options = {}) {
  const headers = Object.assign({}, options.headers);
  const token = localStorage.getItem(TOKEN_KEY);
  if (token) headers.Authorization = `Bearer ${token}`;
  const response = await fetch(path, Object.assign({}, options, { headers }));
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.error || `${response.status} ${response.statusText}`);
  return body;
}

async function loadRoster() {
  const body = await api('/v1/robots');
  for (const robot of body.robots) {
    Object.assign(robotRecord(robot.robot_id), { registry: robot });
  }
  scheduleRender();
}

function onBrokerMessage(topic, payload) {
  const parsed = parseTopic(topic, state.config.topics.root);
  if (!parsed) return;
  let message;
  try {
    message = JSON.parse(payload);
  } catch (error) {
    console.warn('unparseable payload on', topic, error);
    return;
  }
  // A payload version this page does not know is refused, not guessed at —
  // the same rule protocol.check applies on the robot side.
  if (message.schema !== 1) {
    console.warn('ignoring schema', message.schema, 'on', topic);
    return;
  }
  const record = robotRecord(parsed.robotId);
  if (parsed.leaf === state.config.topics.presence) record.presence = message;
  else if (parsed.leaf === state.config.topics.health) record.health = message;
  else if (parsed.leaf === state.config.topics.pose) record.pose = message;
  else if (parsed.leaf === state.config.topics.status) {
    record.status = message;
    const last = record.statuses[record.statuses.length - 1];
    if (!last || last.stamp !== message.stamp || last.state !== message.state) {
      record.statuses.push(message);
      if (record.statuses.length > 50) record.statuses.shift();
    }
  }
  scheduleRender();
}

// -- basemap -------------------------------------------------------------

async function ensureMap(record) {
  const pose = record && record.pose;
  const site = (pose && pose.site) || (record && record.health && record.health.site);
  const floor = (pose && pose.floor) || (record && record.health && record.health.floor);
  const key = site && floor ? `${site}/${floor}` : null;
  if (key === state.mapKey) return;
  state.mapKey = key;
  if (!key) {
    mapView.clearMap();
    dom.mapLabel.textContent = 'no floor reported';
    return;
  }
  dom.mapLabel.textContent = key;
  try {
    const meta = await api(`/v1/maps/${site}/${floor}/map.json`);
    const image = new Image();
    image.src = meta.image_url;
    await image.decode();
    if (state.mapKey === key) mapView.setMap(meta, image);
  } catch (error) {
    mapView.clearMap();
    dom.mapLabel.textContent = `${key} — no basemap on the fleet server (${error.message})`;
  }
}

// -- rendering -----------------------------------------------------------

function scheduleRender() {
  if (pending) return;
  pending = true;
  requestAnimationFrame(() => {
    pending = false;
    render();
  });
}

function render() {
  // At the gate there is nothing to render, and the periodic re-render must not
  // replace "paste a token" with "no robots" — which is a different claim.
  if (!state.config) return;
  const records = [...state.robots.values()].sort((a, b) => a.id.localeCompare(b.id));
  if (!state.selected && records.length) state.selected = records[0].id;
  renderRoster(records);
  const selected = state.robots.get(state.selected);
  renderDetail(selected);
  ensureMap(selected); // sets state.mapKey synchronously, then loads
  mapView.select(state.selected);
  mapView.setRobots(
    records
      .filter((record) => record.pose && onDisplayedFloor(record))
      .map((record) => ({
        id: record.id,
        x: record.pose.x,
        y: record.pose.y,
        yaw: record.pose.yaw,
        state: robotState(record),
        label: record.id,
        stale:
          (ageSeconds(record.pose.stamp) || 0) > POSE_STALE_S ||
          !healthIsCurrent(record),
      })),
  );
}

// Only robots on the basemap being shown: a pose from another floor is a
// different map frame, and drawing it here would place a robot somewhere it is
// not. Keyed on the floor actually on screen rather than on the selected
// robot's pose, so selecting a robot that has not localised yet still shows the
// floor its health reports, with whoever else is on it.
function onDisplayedFloor(record) {
  return `${record.pose.site}/${record.pose.floor}` === state.mapKey;
}

function renderRoster(records) {
  dom.roster.replaceChildren(
    ...records.map((record) => {
      const health = record.health || {};
      const task = health.task;
      return el(
        'button',
        {
          class: `robot ${record.id === state.selected ? 'selected' : ''}`,
          onclick: () => {
            state.selected = record.id;
            state.mapKey = null; // re-resolve: the new robot may be on another floor
            scheduleRender();
          },
        },
        [
          el('div', { class: 'robot-head' }, [
            el('span', { class: `dot ${robotState(record)}` }),
            el('span', { class: 'robot-id', text: record.id }),
            el('span', {
              class: 'robot-state',
              text: robotState(record),
            }),
          ]),
          el('div', {
            class: 'robot-sub',
            text: record.registry ? record.registry.name : 'not enrolled here',
          }),
          el('div', {
            class: `robot-sub ${healthIsCurrent(record) ? '' : 'stale'}`,
            text: healthIsCurrent(record)
              ? task
                ? `${task.state}: ${task.command}`
                : health.summary || '—'
              : staleReason(record),
          }),
        ],
      );
    }),
  );
  if (!records.length) {
    dom.roster.replaceChildren(
      el('p', { class: 'empty', text: 'no robots — enroll one, then start its agent' }),
    );
  }
}

function renderDetail(record) {
  if (!record) {
    dom.detailName.textContent = 'no robot selected';
    dom.detailMeta.replaceChildren();
    dom.subsystems.replaceChildren();
    dom.statusLog.replaceChildren();
    dom.dispatch.hidden = true;
    dom.detailStale.hidden = true;
    return;
  }
  dom.dispatch.hidden = false;
  dom.detailName.textContent = robotLabel(record);

  const health = record.health || {};
  const pose = record.pose;
  const current = healthIsCurrent(record);
  // Everything below is retained state, i.e. the last thing the robot said. When
  // that is no longer a claim about now, say so once, loudly, at the top —
  // rather than letting eight green dots imply otherwise.
  dom.detailStale.textContent = current ? '' : `NOT CURRENT — ${staleReason(record)}`;
  dom.detailStale.hidden = current;
  dom.subsystems.className = `subsystems ${current ? '' : 'stale'}`;
  const facts = [
    ['presence', record.presence ? (record.presence.online ? 'online' : `offline (${record.presence.reason || '—'})`) : 'never seen'],
    [
      'health',
      health.state
        ? `${health.state} — ${health.summary}${current ? '' : '  (last known)'}`
        : '—',
    ],
    ['task', health.task ? `${health.task.state}: ${health.task.command}` : 'idle'],
    ['site', health.site || (pose && pose.site) ? `${health.site || pose.site}/${health.floor || pose.floor}` : '—'],
    ['pose', pose ? `x ${pose.x.toFixed(2)}  y ${pose.y.toFixed(2)}  yaw ${pose.yaw.toFixed(2)}  (${ageText(pose.stamp)})` : 'not localised'],
    ['version', health.version || '—'],
    ['uptime', health.uptime_s ? `${Math.round(health.uptime_s / 3600)}h` : '—'],
    ['battery', 'unmeasurable on this hardware'],
    ['reported', ageText(health.stamp)],
  ];
  dom.detailMeta.replaceChildren(
    ...facts.map(([key, value]) =>
      el('div', { class: 'fact' }, [
        el('span', { class: 'fact-key', text: key }),
        el('span', { class: 'fact-value', text: value }),
      ]),
    ),
  );

  dom.subsystems.replaceChildren(
    ...(health.subsystems || []).map((subsystem) =>
      el('div', { class: 'subsystem' }, [
        el('span', { class: `dot ${subsystem.state}` }),
        el('span', { class: 'subsystem-name', text: subsystem.name }),
        el('span', { class: 'subsystem-message', text: subsystem.message }),
      ]),
    ),
  );

  dom.statusLog.replaceChildren(
    ...[...record.statuses]
      .reverse()
      .map((status) =>
        el('div', { class: `status ${status.state}` }, [
          el('span', { class: 'status-time', text: status.stamp.slice(11, 19) }),
          el('span', { class: 'status-state', text: status.state }),
          el('span', {
            class: 'status-command',
            text: `${status.command}${status.detail ? ` — ${status.detail}` : ''}`,
          }),
          el('span', { class: 'status-source', text: status.source }),
        ]),
      ),
  );

  if (state.config.foxglove_url) {
    dom.foxglove.href = state.config.foxglove_url.replace('{robot_id}', record.id);
    dom.foxglove.hidden = false;
  }
}

// -- dispatch ------------------------------------------------------------

async function onDispatch(event) {
  event.preventDefault();
  const command = dom.command.value.trim();
  if (!command || !state.selected) return;
  dom.dispatchNote.textContent = 'dispatching…';
  dom.dispatchNote.className = 'note';
  try {
    const body = await api(`/v1/robots/${state.selected}/dispatch`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ schema: 1, command }),
    });
    // Nothing is echoed into the log from here: the robot's own task/status is
    // the truth about what happened, and it arrives over the broker in
    // milliseconds. Reporting "sent" as if it were "accepted" would be a lie
    // the operator has no way to check.
    dom.dispatchNote.textContent = `dispatched ${body.id}`;
    dom.command.value = '';
  } catch (error) {
    dom.dispatchNote.textContent = error.message;
    dom.dispatchNote.className = 'note error';
  }
}

async function onToken(event) {
  event.preventDefault();
  const token = dom.token.value.trim();
  localStorage.setItem(TOKEN_KEY, token);
  dom.token.value = '';
  await start();
}

// Since M7 there is no read-only mode to fall back to: /v1/config is itself
// operator-only, because it hands out this operator's broker credential. So the
// page has exactly two states — signed in, or asking to be — and this is the
// asking one. It is not an error; a wall display that has been logged out looks
// like this until somebody pastes a token back in.
function showGate(reason) {
  state.operator = null;
  state.config = null;
  dom.operator.textContent = reason;
  dom.operator.className = 'operator anonymous';
  dom.brokerState.textContent = 'not connected — no operator token';
  dom.brokerState.className = 'broker offline';
  dom.roster.replaceChildren(
    el('p', {
      class: 'empty',
      text:
        'Paste an operator token to see the fleet. Mint one on the fleet box: ' +
        'fleetctl operator new --name <you>',
    }),
  );
  renderDetail(null);
}

// -- boot ----------------------------------------------------------------

function bind() {
  for (const [key, id] of Object.entries({
    roster: 'roster',
    detailName: 'detail-name',
    detailMeta: 'detail-meta',
    detailStale: 'detail-stale',
    subsystems: 'subsystems',
    statusLog: 'status-log',
    dispatch: 'dispatch',
    command: 'command',
    dispatchNote: 'dispatch-note',
    foxglove: 'foxglove',
    brokerState: 'broker-state',
    operator: 'operator',
    token: 'token',
    mapLabel: 'map-label',
    canvas: 'map-canvas',
    follow: 'follow',
    fit: 'fit',
  })) {
    dom[key] = document.getElementById(id);
  }
}

function brokerUrl(config) {
  const host = config.broker.ws_host || location.hostname;
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${host}:${config.broker.ws_port}/`;
}

// Everything that needs a credential. Called at load, and again whenever a
// token is pasted — so signing in never needs a page reload, and a token that
// stops working puts the page back at the gate rather than into a silent
// half-state.
async function start() {
  if (!localStorage.getItem(TOKEN_KEY)) {
    showGate('read-only — paste an operator token');
    return;
  }
  try {
    state.config = await api('/v1/config');
  } catch (error) {
    showGate(`token refused: ${error.message}`);
    return;
  }
  state.operator = localStorage.getItem(TOKEN_KEY);
  dom.operator.textContent = 'operator token accepted';
  dom.operator.className = 'operator ok';
  document.getElementById('contract').textContent = state.config.contract;
  await loadRoster().catch((error) => console.warn('roster unavailable', error));

  if (reader) reader.close();
  const { root, presence, health, pose, status } = state.config.topics;
  reader = new BrokerReader({
    url: brokerUrl(state.config),
    topics: [presence, health, pose, status].map((leaf) => `${root}/+/${leaf}`),
    // The broker is authenticated since M7, and this credential is
    // subscribe-only by the broker's own ACL — see docs/fleet/security.md.
    credential: state.config.broker.username
      ? { username: state.config.broker.username, password: state.config.broker.password }
      : null,
    onMessage: onBrokerMessage,
    onState: (status_, detail) => {
      dom.brokerState.textContent =
        status_ === 'connected' ? `broker connected` : `broker ${status_}: ${detail}`;
      dom.brokerState.className = `broker ${status_}`;
    },
  });
  reader.connect();
  scheduleRender();
}

export async function boot() {
  bind();
  mapView = new MapView(dom.canvas, {
    onSelect: (robotId) => {
      state.selected = robotId;
      scheduleRender();
    },
  });
  dom.fit.addEventListener('click', () => {
    mapView.follow(null);
    dom.follow.checked = false;
    mapView.fit();
    mapView.draw();
  });
  dom.follow.addEventListener('change', () => {
    mapView.follow(dom.follow.checked ? state.selected : null);
    mapView.draw();
  });
  dom.dispatch.addEventListener('submit', onDispatch);
  document.getElementById('token-form').addEventListener('submit', onToken);

  await start();

  // Ages are the only thing on the page that changes without a message.
  setInterval(scheduleRender, 5000);
}

boot().catch((error) => {
  document.getElementById('broker-state').textContent = `failed to start: ${error.message}`;
});
