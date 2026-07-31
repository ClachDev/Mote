// The thin fleet dashboard: roster, live map, and dispatch.
//
// The read path is the broker — every robot's presence, health, pose and task
// status arrives over MQTT-over-WebSockets, retained, so this page shows the
// state of the fleet the instant it loads and never polls (fleet.md Q5). The
// write path is the fleet API: `POST /v1/robots/<id>/dispatch` authorizes the
// operator and writes an audit line before anything reaches `task/command`.
// The two paths are deliberately different, and the browser holds no credential
// that can publish to the broker.
//
// What this view is *not* is a Foxglove replacement. It answers "where is every
// robot, what is each doing, send one somewhere" and hands the single-robot
// deep view (3D, sensors, teleop) to Foxglove by deep link.

import { BrokerReader, parseTopic } from './mqtt.mjs';
import { MapView } from './map.mjs';
import { setupPanes } from './layout.mjs';

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
  floor: null, // the registry's view of the floor on screen: revisions, candidates
  zones: [], // the floor's taught places, for the map and the dispatch picker
};

const dom = {};
let mapView = null;
let panes = null;
let pending = false;

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
  state.floor = null;
  setZones([]);
  renderRevisions();
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
    if (state.mapKey !== key) return;
    mapView.setMap(meta, image);
  } catch (error) {
    mapView.clearMap();
    dom.mapLabel.textContent = `${key} — no basemap on the fleet server (${error.message})`;
    return;
  }
  // Taught places, in the same frame as the basemap. A floor may have none,
  // which is a 404 and not an error worth showing.
  api(`/v1/maps/${site}/${floor}/zones.json`)
    .then((body) => state.mapKey === key && setZones(body.zones))
    .catch(() => state.mapKey === key && setZones([]));
  loadFloor(site, floor, key);
}

// Taught places go two ways: onto the basemap, and into the dispatch picker.
// Both are the floor's, so they arrive and are cleared together.
function setZones(zones) {
  state.zones = zones || [];
  mapView.setZones(state.zones);
  renderZones();
}

// -- the map registry ----------------------------------------------------

async function loadFloor(site, floor, key) {
  try {
    const body = await api(`/v1/sites/${site}/floors/${floor}`);
    if (state.mapKey !== key) return;
    state.floor = body;
  } catch (error) {
    state.floor = null;
  }
  renderRevisions();
}

// Which revision the floor is on, and what else could be promoted onto it. A
// candidate is a map some robot published that changes nothing until an
// operator says so, so this is where that decision is made.
function renderRevisions() {
  const floor = state.floor;
  if (!floor) {
    dom.revisions.hidden = true;
    dom.mapRevision.textContent = '';
    return;
  }
  dom.mapRevision.textContent = floor.canonical || 'no published map';
  const candidates = floor.revisions.filter(
    (revision) => !revision.canonical && revision.ok,
  );
  dom.revisions.hidden = candidates.length === 0;
  dom.revision.replaceChildren(
    ...candidates.map((revision) =>
      el('option', {
        value: revision.revision,
        text: `${revision.revision}${revision.robot_id ? ` · ${revision.robot_id}` : ''}`,
      }),
    ),
  );
}

// The zones of the floor on screen, as a `goto` the operator does not have to
// type. It writes the command rather than sending it: the send button stays the
// one place a task leaves this page.
function renderZones() {
  const names = state.zones.map((zone) => zone.name).sort((a, b) => a.localeCompare(b));
  dom.zone.hidden = names.length === 0;
  dom.zone.replaceChildren(
    el('option', { value: '', text: 'go to a taught zone…' }),
    ...names.map((name) => el('option', { value: name, text: name })),
  );
}

function onZone() {
  const name = dom.zone.value;
  if (!name) return;
  dom.command.value = `goto ${name}`;
  dom.dispatchNote.textContent = '';
  dom.dispatchNote.className = 'note';
}

async function onPromote(event) {
  event.preventDefault();
  const revision = dom.revision.value;
  if (!state.floor || !revision) return;
  const { site, floor } = state.floor;
  dom.promoteNote.textContent = `promoting ${revision}…`;
  dom.promoteNote.className = 'note';
  try {
    const body = await api(
      `/v1/sites/${site}/floors/${floor}/revisions/${revision}/promote`,
      { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{"schema":1}' },
    );
    dom.promoteNote.textContent = body.announced
      ? `${site}/${floor} is on ${body.revision}; robots will pull it`
      : `promoted, but not announced: ${body.detail}`;
    // Re-read the floor rather than assume: the basemap on screen is now a
    // different map.
    state.mapKey = null;
    scheduleRender();
  } catch (error) {
    dom.promoteNote.textContent = error.message;
    dom.promoteNote.className = 'note error';
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
            // Where the desktop layout has three panes at once, a phone has to
            // be taken there: picking a robot means asking where it is.
            panes.show('map');
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
  // On a phone the detail pane is behind a tab, so the tab is where the
  // selection is visible at all.
  dom.tabDetail.textContent = record ? record.id : 'robot';
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
    [
      'map',
      health.map && health.map.revision
        ? `${health.map.revision}${
            state.floor && state.floor.canonical &&
            state.floor.canonical !== health.map.revision
              ? '  (fleet canonical: ' + state.floor.canonical + ')'
              : ''
          }`
        : '—',
    ],
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
  await checkOperator();
  dom.token.value = '';
}

// The one call that tells us whether this token is any good — the audit route
// is operator-only, so a 200 is proof and a 401 is the reason to say so.
async function checkOperator() {
  const token = localStorage.getItem(TOKEN_KEY);
  if (!token) {
    state.operator = null;
    dom.operator.textContent = 'read-only — paste an operator token to dispatch';
    dom.operator.className = 'operator anonymous';
    return;
  }
  try {
    await api('/v1/audit?limit=1');
    state.operator = token;
    dom.operator.textContent = 'operator token accepted';
    dom.operator.className = 'operator ok';
  } catch (error) {
    state.operator = null;
    dom.operator.textContent = `token refused: ${error.message}`;
    dom.operator.className = 'operator error';
  }
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
    zone: 'zone',
    tabDetail: 'tab-detail',
    dispatchNote: 'dispatch-note',
    foxglove: 'foxglove',
    brokerState: 'broker-state',
    operator: 'operator',
    token: 'token',
    mapLabel: 'map-label',
    mapRevision: 'map-revision',
    revisions: 'revisions',
    revision: 'revision',
    promoteNote: 'promote-note',
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

export async function boot() {
  bind();
  mapView = new MapView(dom.canvas, {
    onSelect: (robotId) => {
      state.selected = robotId;
      scheduleRender();
    },
  });
  // The canvas has no size until its pane is on screen, so the map is told when
  // it becomes visible rather than fitting into a hidden 0x0 box.
  panes = setupPanes({ onShow: (name) => name === 'map' && mapView.shown() });
  dom.zone.addEventListener('change', onZone);
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
  dom.revisions.addEventListener('submit', onPromote);
  document.getElementById('token-form').addEventListener('submit', onToken);

  state.config = await api('/v1/config');
  document.getElementById('contract').textContent = state.config.contract;
  await checkOperator();
  await loadRoster().catch((error) => console.warn('roster unavailable', error));

  const { root, presence, health, pose, status } = state.config.topics;
  const reader = new BrokerReader({
    url: brokerUrl(state.config),
    topics: [presence, health, pose, status].map((leaf) => `${root}/+/${leaf}`),
    onMessage: onBrokerMessage,
    onState: (status_, detail) => {
      dom.brokerState.textContent =
        status_ === 'connected' ? `broker connected` : `broker ${status_}: ${detail}`;
      dom.brokerState.className = `broker ${status_}`;
    },
  });
  reader.connect();

  // Ages are the only thing on the page that changes without a message.
  setInterval(scheduleRender, 5000);
}

boot().catch((error) => {
  document.getElementById('broker-state').textContent = `failed to start: ${error.message}`;
});
