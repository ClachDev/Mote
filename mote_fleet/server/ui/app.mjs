// The thin fleet dashboard: roster, live map, and dispatch.
//
// The read path is the broker — every robot's presence, health, pose, mission
// status and capability set arrives over MQTT-over-WebSockets, retained, so
// this page shows the state of the fleet the instant it loads and never polls
// (fleet.md Q5). The write path is the fleet API:
// `POST /v1/robots/<id>/dispatch` authorizes the operator and writes an audit
// line before anything reaches `mission/command`. The two paths are
// deliberately different, and the browser holds no credential that can publish
// to the broker.
//
// The dispatch form is *generated* from the selected robot's capability set.
// It used to be a text box the operator typed a sentence into, which meant the
// page had to know the grammar and the operator had to know it better. Now the
// keys come from the document the robot publishes, and a field is a zone
// picker exactly when its schema $refs zone/v0's zone reference — so this file
// contains no list of capabilities and no list of which inputs are places.
//
// What this view is *not* is a Foxglove replacement. It answers "where is every
// robot, what is each doing, send one somewhere" and hands the single-robot
// deep view (3D, sensors, teleop) to Foxglove by deep link.

import { BrokerReader, parseTopic } from './mqtt.mjs';
import { MapView } from './map.mjs';
import { ReviewView } from './review.mjs';
import { setupPanes } from './layout.mjs';
// Every decision about *what* to say about a robot lives there, pure, so that
// `ui_test.mjs` can hold it under node. This file only puts the answer in a div.
import {
  POSE_STALE_S,
  ageSeconds,
  detailFooter,
  detailHeadline,
  detailSections,
  healthBanner,
  healthIsCurrent,
  robotState,
  rosterSubline,
  staleReason,
  missionLine,
} from './robot.mjs';

const TOKEN_KEY = 'mote.operator.token';

const state = {
  config: null,
  robots: new Map(),
  selected: null,
  operator: null,
  noted: null, // whose capability summary the dispatch note is showing
  mapKey: null,
  floor: null, // the registry's view of the floor on screen: revisions, candidates
  zones: [], // the floor's bound places, for the map and the dispatch picker
};

const dom = {};
let mapView = null;
let review = null;
let panes = null;
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

function robotRecord(robotId) {
  if (!state.robots.has(robotId)) {
    state.robots.set(robotId, { id: robotId, statuses: [] });
  }
  return state.robots.get(robotId);
}

// -- data in -------------------------------------------------------------

function authHeaders(headers = {}) {
  const token = localStorage.getItem(TOKEN_KEY);
  return token ? Object.assign({}, headers, { Authorization: `Bearer ${token}` }) : headers;
}

async function api(path, options = {}) {
  const headers = authHeaders(options.headers);
  const response = await fetch(path, Object.assign({}, options, { headers }));
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.error || `${response.status} ${response.statusText}`);
  return body;
}

// The basemap is a gated route like the transform beside it, and an `<img src>`
// carries no Authorization header — so the pixels are fetched with the token
// and handed to the decoder as a blob. The object URL is released once decoding
// has finished: the bitmap outlives it, the URL does not need to.
async function loadImage(path) {
  const response = await fetch(path, { headers: authHeaders() });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const objectUrl = URL.createObjectURL(await response.blob());
  try {
    const image = new Image();
    image.src = objectUrl;
    await image.decode();
    return image;
  } finally {
    URL.revokeObjectURL(objectUrl);
  }
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
  else if (parsed.leaf === state.config.topics.capabilities) record.capabilities = message;
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
  // What revisions the floor has does not depend on it having a published map,
  // and must not be fetched as though it did. This used to sit after the
  // basemap fetch, behind its early return, so a floor whose only revisions
  // were candidates reported no candidates — and the first promotion on any
  // floor could never be made from the browser (observed live, 2026-08-02).
  loadFloor(site, floor, key);
  try {
    const meta = await api(`/v1/maps/${site}/${floor}/map.json`);
    const image = await loadImage(meta.image_url);
    if (state.mapKey !== key) return;
    mapView.setMap(meta, image);
  } catch (error) {
    mapView.clearMap();
    dom.mapLabel.textContent = `${key} — no basemap on the fleet server (${error.message})`;
    return;
  }
  // Bound places, in the same frame as the basemap. A floor may have none,
  // which is a 404 and not an error worth showing.
  api(`/v1/maps/${site}/${floor}/zones.json`)
    .then((body) => state.mapKey === key && setZones(body.zones))
    .catch(() => state.mapKey === key && setZones([]));
}

// Bound places go two ways: onto the basemap, and into the dispatch picker.
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

// Which revision the floor is on, and whether anything is waiting to replace
// it. The decision is deliberately *not* made here: this canvas draws live
// robots on the **canonical** basemap, so a promote button beside it would be
// a promotion made without ever seeing the map being promoted — which is what
// the review pane exists to end. This is the signpost to it.
//
// Candidates the validator refused are counted too: "why can I not promote the
// map my robot just published" is a question the review pane can answer and a
// filtered-out row cannot.
function renderRevisions() {
  const floor = state.floor;
  dom.mapRevision.textContent = floor ? floor.canonical || 'no published map' : '';
  const candidates = floor
    ? floor.revisions.filter((revision) => !revision.canonical)
    : [];
  // Always reachable, and *not* conditional on this floor having candidates.
  // The tab bar it shares the job with is hidden above 760 px, so while this
  // was, a desk-width operator could reach the review pane only through a
  // floor a robot was reporting — which is precisely the floor review is least
  // needed for. The floor mapped last week by a robot since switched off, and
  // the floor a build lands on with no robot near it, were both unreachable.
  dom.reviewJump.hidden = false;
  dom.reviewJump.textContent = candidates.length
    ? `${candidates.length} candidate${candidates.length === 1 ? '' : 's'} — review`
    : 'review maps';
}

// The zones of the floor on screen feed every zone-typed mission input, so the
// operator picks a place rather than spelling one. Redrawing the form is how
// they get there: the fields are generated, and the zone list is one of the two
// things they are generated from.
function renderZones() {
  renderDispatch(state.robots.get(state.selected));
}

// -- the dispatch form, generated from the capability set ------------------

const ZONE_REF = 'https://spec.augereai.com/zone/v0/zone-ref.schema.json';

function selectedCapability(record) {
  const offered = (record && record.capabilities && record.capabilities.capabilities) || [];
  return offered.find((item) => item.key === dom.capability.value) || offered[0] || null;
}

function renderDispatch(record) {
  const offered = (record && record.capabilities && record.capabilities.capabilities) || [];
  const wanted = dom.capability.value;
  dom.capability.replaceChildren(
    ...offered.map((item) =>
      el('option', { value: item.key, text: item.display_name || item.key }),
    ),
  );
  if (offered.some((item) => item.key === wanted)) dom.capability.value = wanted;
  if (!offered.length) {
    // Retained, so an empty set means the robot has never advertised one —
    // its task server is not running, or it predates the capability topic.
    // Either way there is nothing this page can honestly offer to send.
    dom.capability.hidden = true;
    dom.missionInput.replaceChildren(
      el('p', { class: 'note', text: 'this robot has advertised no capabilities' }),
    );
    dom.dispatchSend.disabled = true;
    return;
  }
  dom.capability.hidden = false;
  dom.dispatchSend.disabled = false;
  const capability = selectedCapability(record);
  const schema = (capability && capability.input_schema) || {};
  const properties = schema.properties || {};
  const required = schema.required || [];
  const names = state.zones.map((zone) => zone.name).sort((a, b) => a.localeCompare(b));
  dom.missionInput.replaceChildren(
    ...Object.entries(properties).map(([key, sub]) =>
      el('label', { class: 'mission-field', title: sub.description || '' }, [
        el('span', { class: 'mission-field-name', text: key + (required.includes(key) ? ' *' : '') }),
        sub.$ref === ZONE_REF && names.length
          ? el('select', { 'data-input': key }, [
              el('option', { value: '', text: 'a zone…' }),
              ...names.map((name) => el('option', { value: name, text: name })),
            ])
          : el('input', {
              'data-input': key,
              type: 'text',
              placeholder: sub.$ref === ZONE_REF ? 'a zone name' : sub.description || key,
              autocomplete: 'off',
              autocapitalize: 'off',
              spellcheck: 'false',
            }),
      ]),
    ),
  );
  // One line carries two things: what the selected capability does, and what
  // the last dispatch did. So the summary is written when the *selection*
  // changes and not on every render — otherwise an outcome an operator has just
  // read is replaced by a description of the form, by whatever arrives next.
  const note = `${state.selected}:${capability && capability.key}`;
  if (capability && state.noted !== note) {
    state.noted = note;
    dom.dispatchNote.textContent = capability.summary || '';
    dom.dispatchNote.className = 'note';
  }
}

function onCapability() {
  renderDispatch(state.robots.get(state.selected));
}

// Into the review pane, on the floor already on screen. Promotion lives there
// because that is where the candidate's own map is drawn.
function onReviewJump() {
  panes.show('review');
  if (state.mapKey) review.open(state.mapKey);
}

// Out of it again, by the pane's own control or by Escape: above 760 px the tab
// bar is hidden, so these are the only exits.
function onReviewBack() {
  if (!review.leavable()) return;
  panes.show('map');
}

function onKey(event) {
  if (event.key !== 'Escape') return;
  if (panes.current() !== 'review') return;
  onReviewBack();
}

// A promotion happened in the review pane: this pane's basemap is now a
// different map, so re-resolve it rather than keep drawing the old one.
//
// The review is then over — but only stand the pane down when this pane is on
// the floor that was promoted, which is the one case where the promotion is
// visible here. `ensureMap` takes its floor from the *selected robot*, so any
// other floor lands the operator on an unrelated map, having taken the note
// that says what happened off screen with it. The floor with no robot on it is
// exactly what the review pane is for.
function onPromoted(site, floor, revision, announced) {
  const showing = state.mapKey === `${site}/${floor}`;
  state.mapKey = null;
  scheduleRender();
  if (announced && showing) panes.show('map');
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
  // At the gate there is nothing to draw, and the five-second re-render must
  // not replace "paste a token" with "no robots" — a different claim.
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
      // Said only when there is something to say: the dot carries the state.
      const subline = rosterSubline(record);
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
          subline &&
            el('div', {
              class: `robot-sub ${healthIsCurrent(record) ? '' : 'stale'}`,
              text: subline,
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
  renderSections(record);

  // The headline. It is written for a missing robot too: the dot and the name
  // are the pane's title, not one of its facts.
  const headline = detailHeadline(record);
  dom.detailDot.className = `dot ${headline.state}`;
  dom.detailName.textContent = headline.label;
  dom.detailReported.textContent = headline.reported;

  if (!record) {
    dom.detailStale.hidden = true;
    dom.detailHealth.hidden = true;
    dom.missionLine.replaceChildren();
    dom.subsystems.replaceChildren();
    dom.statusLog.replaceChildren();
    dom.detailFooter.textContent = '';
    // Without this the link keeps the last robot's id, so `open in Foxglove`
    // on an empty pane opens somebody else's robot.
    dom.foxglove.hidden = true;
    return;
  }
  renderDispatch(record);

  const health = record.health || {};
  const current = healthIsCurrent(record);
  // Everything below is retained state, i.e. the last thing the robot said. When
  // that is no longer a claim about now, say so once, loudly, at the top —
  // rather than letting eight green dots imply otherwise.
  dom.detailStale.textContent = current ? '' : `NOT CURRENT — ${staleReason(record)}`;
  dom.detailStale.hidden = current;
  dom.subsystems.className = `subsystems ${current ? '' : 'stale'}`;
  // And the health state itself, when it is current and not `ok`. It is a
  // banner rather than a row because a fault is not a thing to go looking for.
  const banner = healthBanner(record);
  dom.detailHealth.textContent = banner || '';
  dom.detailHealth.className = `stale-banner health-banner ${banner ? health.state : ''}`;
  dom.detailHealth.hidden = !banner;

  const mission = missionLine(record);
  dom.missionLine.replaceChildren(
    ...[
      mission.capability &&
        el('span', { class: 'mission-capability', text: mission.capability }),
      el('span', { class: 'mission-meta', text: mission.meta }),
    ].filter(Boolean),
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
          el('span', { class: 'status-command', text: statusText(status) }),
          el('span', { class: 'status-source', text: status.source }),
        ]),
      ),
  );

  dom.detailFooter.textContent = detailFooter(record);

  if (state.config.foxglove_url) {
    dom.foxglove.href = state.config.foxglove_url.replace('{robot_id}', record.id);
    dom.foxglove.hidden = false;
  }
}

// A heading over an empty div reserves space for a fact the robot does not
// have — `subsystems` on a robot whose health monitor is not running. Each
// heading is hidden with its content, the way the dispatch form already was.
function renderSections(record) {
  const sections = detailSections(record);
  dom.missionHead.hidden = !sections.mission;
  dom.missionLine.hidden = !sections.mission;
  dom.statusLog.hidden = !sections.statuses;
  dom.dispatchHead.hidden = !sections.dispatch;
  dom.dispatch.hidden = !sections.dispatch;
  dom.subsystemsHead.hidden = !sections.subsystems;
  dom.subsystems.hidden = !sections.subsystems;
}

// What one status line says. The failure class leads, because it is the part
// an operator acts on: `busy` means wait, `invalid_input` means fix the
// mission, `obstructed` means look at the corridor. The sentence after it is
// the detail, not the verdict.
function statusText(status) {
  const failure = status.failure;
  if (failure) {
    const retry = failure.recoverable ? 'retryable' : 'not retryable';
    return `${status.capability} — ${failure.class} (${retry}): ${failure.detail}`;
  }
  const warnings = (status.warnings || []).join('; ');
  const note = status.detail || '';
  return `${status.capability}${note ? ` — ${note}` : ''}${warnings ? `  ! ${warnings}` : ''}`;
}

// -- dispatch ------------------------------------------------------------

async function onDispatch(event) {
  event.preventDefault();
  if (!state.selected) return;
  const capability = dom.capability.value;
  if (!capability) return;
  const payload = {};
  for (const field of dom.missionInput.querySelectorAll('[data-input]')) {
    const value = field.value.trim();
    // Absent, not empty: a property left blank is one the operator did not
    // supply, and sending "" would fail the schema for a different reason than
    // the true one.
    if (value) payload[field.dataset.input] = value;
  }
  dom.dispatchNote.textContent = 'dispatching…';
  dom.dispatchNote.className = 'note';
  try {
    const body = await api(`/v1/robots/${state.selected}/dispatch`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ schema: 1, capability, input: payload }),
    });
    // Nothing is echoed into the log from here: the robot's own task/status is
    // the truth about what happened, and it arrives over the broker in
    // milliseconds. Reporting "sent" as if it were "accepted" would be a lie
    // the operator has no way to check.
    dom.dispatchNote.textContent = `dispatched ${body.id}`;
    for (const field of dom.missionInput.querySelectorAll('[data-input]')) {
      field.value = '';
    }
  } catch (error) {
    dom.dispatchNote.textContent = error.message;
    dom.dispatchNote.className = 'note error';
  }
}

async function onToken(event) {
  event.preventDefault();
  localStorage.setItem(TOKEN_KEY, dom.token.value.trim());
  dom.token.value = '';
  await start();
}

// There is no read-only mode to fall back to: `/v1/config` is itself
// operator-only, so the page has two states — signed in, or asking to be — and
// this is the asking one. It is not an error. A wall display whose token has
// been revoked looks like this until somebody pastes another in.
function showGate(reason) {
  state.operator = null;
  state.config = null;
  state.robots.clear();
  if (reader) {
    reader.close();
    reader = null;
  }
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
    detailDot: 'detail-dot',
    detailReported: 'detail-reported',
    detailStale: 'detail-stale',
    detailHealth: 'detail-health',
    detailFooter: 'detail-footer',
    missionHead: 'mission-head',
    missionLine: 'mission-line',
    subsystems: 'subsystems',
    subsystemsHead: 'subsystems-head',
    statusLog: 'status-log',
    dispatch: 'dispatch',
    dispatchHead: 'dispatch-head',
    capability: 'capability',
    missionInput: 'mission-input',
    tabDetail: 'tab-detail',
    dispatchNote: 'dispatch-note',
    foxglove: 'foxglove',
    brokerState: 'broker-state',
    operator: 'operator',
    token: 'token',
    mapLabel: 'map-label',
    mapRevision: 'map-revision',
    reviewJump: 'review-jump',
    canvas: 'map-canvas',
    follow: 'follow',
    fit: 'fit',
    reviewFloors: 'review-floors',
    reviewFloor: 'review-floor',
    reviewCanonical: 'review-canonical',
    reviewRevisions: 'review-revisions',
    reviewVerdict: 'review-verdict',
    reviewVerdictNotes: 'review-verdict-notes',
    reviewNotesLabel: 'review-notes-label',
    reviewProvenance: 'review-provenance',
    reviewZoneSource: 'review-zone-source',
    reviewCanvas: 'review-canvas',
    reviewMapLabel: 'review-map-label',
    reviewPromote: 'review-promote',
    reviewBack: 'review-back',
    reviewFit: 'review-fit',
    reviewNote: 'review-note',
    zonesEdit: 'zones-edit',
    zoneRows: 'zone-rows',
    zoneDetail: 'zone-detail',
    zoneSave: 'zone-save',
    zoneCancel: 'zone-cancel',
    zoneNote: 'zone-note',
  })) {
    dom[key] = document.getElementById(id);
  }
}

function brokerUrl(config) {
  const host = config.broker.ws_host || location.hostname;
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${host}:${config.broker.ws_port}/`;
}

// Everything that needs a credential, in one function: called at load and again
// whenever a token is pasted, so signing in never needs a reload and a token
// that has stopped working puts the page back at the gate rather than into a
// silent half-state. `/v1/config` is the check — it is operator-only like every
// other route, and it is the one the rest of the page is built out of.
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
  // The registry's floors, not the fleet's: a floor worth reviewing may have no
  // robot reporting it at all.
  await review.loadFloors().catch((error) => console.warn('registry unavailable', error));

  if (reader) reader.close();
  const { root, presence, health, pose, status, capabilities } = state.config.topics;
  reader = new BrokerReader({
    url: brokerUrl(state.config),
    topics: [presence, health, pose, status, capabilities].map(
      (leaf) => `${root}/+/${leaf}`,
    ),
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
  review = new ReviewView({
    api,
    loadImage,
    onPromoted,
    dom: {
      canvas: dom.reviewCanvas,
      floors: dom.reviewFloors,
      floor: dom.reviewFloor,
      canonical: dom.reviewCanonical,
      revisions: dom.reviewRevisions,
      verdict: dom.reviewVerdict,
      verdictNotes: dom.reviewVerdictNotes,
      notesLabel: dom.reviewNotesLabel,
      provenance: dom.reviewProvenance,
      zoneSource: dom.reviewZoneSource,
      mapLabel: dom.reviewMapLabel,
      promote: dom.reviewPromote,
      back: dom.reviewBack,
      fit: dom.reviewFit,
      note: dom.reviewNote,
      // The zone editor's own controls. It lives in this pane because it edits
      // the *selected revision's* zones over that revision's own map.
      zonesEdit: dom.zonesEdit,
      editorRows: dom.zoneRows,
      editorDetail: dom.zoneDetail,
      editorNote: dom.zoneNote,
      zoneSave: dom.zoneSave,
      zoneCancel: dom.zoneCancel,
    },
  });
  // A canvas has no size until its pane is on screen, so each map is told when
  // it becomes visible rather than fitting into a hidden 0x0 box. The review
  // pane's canvas is hidden at *every* width until the pane is opened, so this
  // is the only moment it can be fitted at all.
  panes = setupPanes({
    onShow: (name) => {
      if (name === 'map') mapView.shown();
      if (name === 'review') review.shown();
    },
  });
  dom.reviewJump.addEventListener('click', onReviewJump);
  dom.reviewBack.addEventListener('click', onReviewBack);
  document.addEventListener('keydown', onKey);
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
  dom.capability.addEventListener('change', onCapability);
  dom.dispatchSend = dom.dispatch.querySelector('button[type="submit"]');
  document.getElementById('token-form').addEventListener('submit', onToken);

  await start();

  // Ages are the only thing on the page that changes without a message.
  setInterval(scheduleRender, 5000);
}

boot().catch((error) => {
  document.getElementById('broker-state').textContent = `failed to start: ${error.message}`;
});
