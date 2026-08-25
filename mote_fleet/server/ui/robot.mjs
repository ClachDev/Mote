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

// The state is the fact. The summary is worth its width only when it says
// something the state does not, which for `ok` it never does.
function healthValue(health, current) {
  if (!health.state) return '—';
  const state =
    health.state === 'ok' || !health.summary
      ? health.state
      : `${health.state} — ${health.summary}`;
  return current ? state : `${state}  (last known)`;
}

// The revision this robot is running, and the fleet's only if they differ —
// which is the whole reason the row carries a revision at all.
function mapValue(health, canonical) {
  const revision = health.map && health.map.revision;
  if (!revision) return '—';
  return canonical && canonical !== revision
    ? `${revision}  (fleet canonical: ${canonical})`
    : revision;
}

// The detail pane's key/value rows, in order.
export function detailFacts(record, canonical = null, now = Date.now()) {
  const health = record.health || {};
  const pose = record.pose;
  const current = healthIsCurrent(record, now);
  const site = health.site || (pose && pose.site);
  const floor = health.floor || (pose && pose.floor);
  return [
    [
      'presence',
      record.presence
        ? record.presence.online
          ? 'online'
          : `offline (${record.presence.reason || '—'})`
        : 'never seen',
    ],
    ['health', healthValue(health, current)],
    ['task', health.task ? `${health.task.state}: ${health.task.command}` : 'idle'],
    ['site', site ? `${site}/${floor}` : '—'],
    [
      'pose',
      pose
        ? `x ${pose.x.toFixed(2)}  y ${pose.y.toFixed(2)}  yaw ${pose.yaw.toFixed(2)}  (${ageText(
            pose.stamp,
            now,
          )})`
        : 'not localised',
    ],
    ['map', mapValue(health, canonical)],
    ['version', health.version || '—'],
    ['uptime', health.uptime_s ? `${Math.round(health.uptime_s / 3600)}h` : '—'],
    // The power bank exposes no telemetry, so nothing on this robot measures it.
    ['battery', 'n/a'],
    ['reported', ageText(health.stamp, now)],
  ];
}

// Which of the detail pane's sections have anything under them. A heading over
// an empty div reserves space for a fact the robot does not have; each of these
// gates its heading and its content together, as the dispatch form already was.
export function detailSections(record) {
  const health = (record && record.health) || {};
  return {
    subsystems: (health.subsystems || []).length > 0,
    dispatch: !!record,
    statuses: !!(record && record.statuses && record.statuses.length),
  };
}
