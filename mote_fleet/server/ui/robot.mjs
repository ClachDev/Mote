// What the page says about one robot.
//
// Every function here turns a robot record — the retained presence, health,
// pose and status the broker delivered — into the text beside it. They are pure
// and DOM-free, so `ui_test.mjs` holds them under node: this is where the page
// decides whether to speak at all, and a green dot with "ok" written next to it
// is the duplication the module exists to prevent.

// A pose older than this is drawn hollow: retained state is the *last known*
// position, which is not the same claim as "the robot is there now".
export const POSE_STALE_S = 20;

// The same rule for health, and it matters more: an offline robot's retained
// health is whatever it last claimed, and eight green subsystems next to a robot
// that is not there is the one lie this view must not tell. The agent publishes
// every 5 s by default, so this is several missed heartbeats rather than a blip.
export const HEALTH_STALE_S = 30;

export function ageSeconds(stamp, now = Date.now()) {
  if (!stamp) return null;
  const parsed = Date.parse(stamp);
  return Number.isNaN(parsed) ? null : (now - parsed) / 1000;
}

export function ageText(stamp, now = Date.now()) {
  const age = ageSeconds(stamp, now);
  if (age === null) return '—';
  if (age < 60) return `${Math.max(0, Math.round(age))}s ago`;
  if (age < 3600) return `${Math.round(age / 60)}m ago`;
  return `${Math.round(age / 3600)}h ago`;
}

// Is what we know about this robot's health still a claim about *now*? Offline
// robots and silent ones both fail this, and the answer drives every green dot
// on the page.
export function healthIsCurrent(record, now = Date.now()) {
  if (!record || !record.health) return false;
  if (record.presence && record.presence.online === false) return false;
  const age = ageSeconds(record.health.stamp, now);
  return age === null || age <= HEALTH_STALE_S;
}

// A robot's single roll-up state: offline beats whatever health it last
// claimed, because that health is by definition from before it dropped.
export function robotState(record, now = Date.now()) {
  if (record.presence && record.presence.online === false) return 'offline';
  if (!record.health) return 'unknown';
  // Health that has stopped arriving is reported as stale rather than as the
  // last thing it said — the contract has a state for exactly this.
  if (!healthIsCurrent(record, now)) return 'stale';
  return record.health.state || 'unknown';
}

// What to say instead of a health summary when it is not current.
export function staleReason(record, now = Date.now()) {
  if (record.presence && record.presence.online === false) {
    return `offline (${record.presence.reason || 'no reason given'}) — last seen ${ageText(
      record.presence.stamp,
      now,
    )}`;
  }
  if (!record.health) return 'never reported';
  return `no health for ${ageText(record.health.stamp, now).replace(' ago', '')}`;
}

export function robotLabel(record) {
  const name = record.registry && record.registry.name;
  return name && name !== record.id ? `${record.id} · ${name}` : record.id;
}

// The roster's one line of prose per robot. The dot beside the id already
// carries the state, so this speaks only where the dot cannot: why the robot has
// stopped answering, what is wrong with it, or what it is busy doing. A healthy
// idle robot gets nothing, and the row is a line shorter for it.
export function rosterSubline(record, now = Date.now()) {
  if (!healthIsCurrent(record, now)) return staleReason(record, now);
  const health = record.health;
  if (health.state && health.state !== 'ok') return health.summary || health.state;
  if (health.task) return `${health.task.state}: ${health.task.command}`;
  return '';
}

// -- the detail pane -----------------------------------------------------
//
// The pane is ranked by what an operator needs first rather than laid out as a
// flat table: who this is and whether it is still current, then what it is
// doing, then how to change that, then what is wrong with it, then the numbers
// nobody reads until something else has already gone wrong. Each piece below is
// one of those bands.

// The headline. Every other thing in the pane is retained state — the last
// thing the robot said — so the age of that state is read *before* it, beside
// the name, rather than in the last row of a table.
export function detailHeadline(record, now = Date.now()) {
  if (!record) return { state: 'unknown', label: 'no robot selected', reported: '' };
  return {
    state: robotState(record, now),
    label: robotLabel(record),
    reported: record.health ? `reported ${ageText(record.health.stamp, now)}` : 'never reported',
  };
}

// A health state worth a sentence of its own. `ok` says nothing the dot has not
// already said, and health that is no longer current is the stale banner's to
// report — two banners disagreeing about one robot is worse than one.
export function healthBanner(record, now = Date.now()) {
  if (!record || !record.health) return null;
  if (!healthIsCurrent(record, now)) return null;
  const { state, summary } = record.health;
  if (!state || state === 'ok') return null;
  return summary ? `${state.toUpperCase()} — ${summary}` : state.toUpperCase();
}

// When the in-flight task started. `health.task` carries no stamp of its own
// (protocol.py: id, command, state), so the age comes from the status that
// began the run — the oldest one still belonging to it. The scan stops at the
// last terminal status, so a command issued twice is not aged from the first
// time it ran.
export function taskStartStamp(record) {
  const task = record && record.health && record.health.task;
  if (!task) return null;
  const statuses = (record && record.statuses) || [];
  let stamp = null;
  for (let i = statuses.length - 1; i >= 0; i -= 1) {
    const status = statuses[i];
    if (status.terminal) break;
    const belongs = task.id ? status.id === task.id : status.command === task.command;
    if (!belongs) break;
    stamp = status.stamp;
  }
  return stamp;
}

// The task band's first line: the command itself, and beside it the state and
// how long it has been in it. An idle robot says one dim word — the section
// exists either way, because "is it busy" is the question the pane is opened
// with and an absent section is not an answer to it.
export function taskLine(record, now = Date.now()) {
  const task = record && record.health && record.health.task;
  if (!task) return { command: '', meta: 'idle' };
  const started = taskStartStamp(record);
  const age = started === null ? null : ageText(started, now).replace(' ago', '');
  return {
    command: task.command || '—',
    meta: age ? `${task.state} · ${age}` : task.state,
  };
}

// The footer: the two numbers that are worth keeping on screen and worth no
// more room than one dim line at the bottom of the pane. Battery is a constant
// — the power bank exposes no telemetry, so nothing on this robot measures it
// — and it is here rather than deleted so that the absence is stated once
// instead of asked about again.
export function detailFooter(record) {
  const health = (record && record.health) || {};
  const uptime = health.uptime_s ? `${Math.round(health.uptime_s / 3600)} h` : '—';
  return `uptime ${uptime} · battery n/a`;
}

// Which of the detail pane's sections have anything under them. A heading over
// an empty div reserves space for a fact the robot does not have; each of these
// gates its heading and its content together, as the dispatch form already was.
export function detailSections(record) {
  const health = (record && record.health) || {};
  return {
    task: !!record,
    dispatch: !!record,
    subsystems: (health.subsystems || []).length > 0,
    statuses: !!(record && record.statuses && record.statuses.length),
  };
}
