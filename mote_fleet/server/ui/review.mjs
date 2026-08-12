// Candidate review: see the map you are about to promote, then promote it.
//
// M4's rule is that uploading is not publishing, which puts a decision in front
// of an operator — and until this view existed, the only thing on screen to
// make it with was a timestamp in a `<select>`. Selecting a candidate showed
// nothing, because the canvas beside the picker was always the *canonical*
// basemap. Promotion was therefore an act of faith (mapping-pipeline.md stage
// 3, and an operator asking "how am I meant to verify it?" on 2026-08-02).
//
// Three things make this a pane of its own rather than a mode on the map pane:
//
//   - The map pane is fleet operations: live robots on the *published* basemap,
//     follow, fit, dispatch. A candidate is a different map frame with no
//     robots in it. Two meanings on one canvas is the confusion this avoids.
//   - It addresses a floor **directly**. The map pane derives the floor on
//     screen from the selected robot's pose or health, which is wrong here: the
//     interesting floor is often one no robot is reporting — mapped by a robot
//     since switched off, or (once the build stage lands) built on the fleet
//     box with no robot involved at all.
//   - It has room to grow. The carry-forward report, the build report, and the
//     zone editing that turns a placeholder `zone_03` into `kitchen` all render
//     here. The zone list below is deliberately a row per zone with its own
//     cells, because that is where those controls go.
//
// Two writes leave this pane, and both are audited operator actions: the
// promote M4 already had, and the zone edit beside it (`zone_editor.mjs`),
// which derives a *new* candidate rather than touching the revision on screen.
// Nothing here changes a floor until the promote.

import { MapView } from './map.mjs';
import { ZoneEditor } from './zone_editor.mjs';

// -- routes ---------------------------------------------------------------

// A revision's own leaves, which is the whole point: `/v1/maps/<site>/<floor>/`
// serves whatever is *canonical*, so a review view built on those routes would
// draw the map the operator already has and label it with the candidate's id —
// the precise failure this replaces.
export function revisionPath(site, floor, revision, leaf) {
  return `/v1/sites/${site}/floors/${floor}/revisions/${revision}/${leaf}`;
}

export function floorPath(site, floor) {
  return `/v1/sites/${site}/floors/${floor}`;
}

export function floorKey(site, floor) {
  return `${site}/${floor}`;
}

export function parseFloorKey(key) {
  const [site, floor] = String(key || '').split('/');
  return site && floor ? { site, floor } : null;
}

// -- what the list says ---------------------------------------------------

// Newest first: the candidate someone has come to look at is the one that just
// arrived. The canonical revision stays in the list rather than being filtered
// out of it — "what am I replacing" is half of the decision, and on a floor
// with nothing published it is the answer "nothing yet".
export function orderedRevisions(detail) {
  return [...((detail && detail.revisions) || [])].reverse();
}

// Which revision to open a floor on. The newest promotable candidate, because
// that is what a review is for; failing that the newest anything, so a floor
// whose only candidates are broken still shows why.
export function defaultRevision(detail) {
  const ordered = orderedRevisions(detail);
  return (
    ordered.find((revision) => !revision.canonical && revision.ok) ||
    ordered.find((revision) => !revision.canonical) ||
    ordered[0] ||
    null
  );
}

// Whether this revision may be promoted, said before the click rather than
// discovered by it. The verdict is the *validator's* — the same report the
// server re-runs at promotion — so this can never encourage a promotion the
// server will refuse, nor discourage one it would accept.
//
// **The bar is exactly "no errors", and that has to be legible**, because the
// list underneath is not the reason for the verdict: warnings are what is
// imperfect about a revision that passes anyway (a missing posegraph navigates
// perfectly and simply cannot be extended). A bare "valid, with warnings" over
// three complaints reads as a claim with its own evidence against it, and an
// operator looking at the first build of this pane asked what the answer was.
//
// The answer is a **state**, not a sentence. It was briefly written as a
// question in the heading answered by "yes — no errors. These warnings do not
// block it:", which says the right thing in the wrong register: this is a
// control panel, its other headings are nouns, and a wrapped sentence dangling
// on a colon is not how a status reads. So the state is one word beside a
// coloured dot — the idiom the roster and the subsystem list already use — and
// what the list *is* moves into a caption on the list, which is where it
// belongs and which also stops the bullets running into the provenance.
export function promotability(revision) {
  if (!revision) {
    return {
      promotable: false,
      verdict: 'no revision selected',
      state: 'unknown',
      notes: [],
      notesLabel: '',
    };
  }
  const warnings = revision.warnings || [];
  const warningLabel = 'warnings — these do not block promotion';
  if (!revision.ok) {
    return {
      promotable: false,
      verdict: 'not promotable',
      state: 'fault',
      notes: revision.errors || [],
      notesLabel: 'errors — these block promotion',
    };
  }
  if (revision.canonical) {
    return {
      promotable: false,
      verdict: 'already published',
      state: 'unknown',
      notes: warnings,
      notesLabel: warnings.length ? warningLabel : '',
    };
  }
  return {
    promotable: true,
    verdict: 'promotable',
    state: 'ok',
    notes: warnings,
    notesLabel: warnings.length ? warningLabel : '',
  };
}

function percent(fraction) {
  return typeof fraction === 'number' ? `${(fraction * 100).toFixed(1)}%` : '—';
}

export function formatBytes(count) {
  if (typeof count !== 'number') return '—';
  if (count < 1024) return `${count} B`;
  if (count < 1024 * 1024) return `${(count / 1024).toFixed(1)} kB`;
  return `${(count / 1024 / 1024).toFixed(1)} MB`;
}

// Everything the registry already knows about a revision, which is everything
// needed to judge one: where it came from, what the validator measured in it,
// and what it would replace. No new payload — `detail()` has carried all of
// this since M4; there was simply nowhere to show it.
export function provenanceRows(revision) {
  if (!revision) return [];
  const map = revision.map || {};
  const occupancy = revision.occupancy || {};
  const meta = revision.meta || {};
  const zones = revision.zones || [];
  return [
    ['revision', revision.revision],
    ['uploaded', revision.uploaded_at || 'not by this server (seeded on disk)'],
    ['from', revision.robot_id || revision.uploaded_by || '—'],
    ['mapped', meta.saved || '—'],
    ['bag', meta.bag || '—'],
    [
      'size',
      map.width && map.height
        ? `${map.width}x${map.height} px at ${map.resolution} m/px`
        : '—',
    ],
    [
      'occupancy',
      occupancy.total
        ? `${percent(occupancy.free)} free · ${percent(occupancy.occupied)} occupied · ${percent(
            occupancy.unknown,
          )} unknown`
        : '—',
    ],
    // A revision with no posegraph navigates perfectly and simply cannot be
    // mapped further, which the validator reports as a warning rather than an
    // error. Saying which it is here is cheaper than reading the warning.
    ['posegraph', revision.files && revision.files['map.posegraph'] ? 'yes' : 'no — cannot be extended'],
    // The zones *inside* the bundle, which is not always the set drawn beside
    // it — see `zoneSource` for the difference and why it matters.
    ['zones in bundle', zones.length ? `${zones.length}: ${zones.join(', ')}` : 'none'],
    ['bytes', formatBytes(revision.bytes)],
    ['sha256', revision.sha256 ? `${revision.sha256.slice(0, 16)}…` : '—'],
  ];
}

// Where the zones on screen came from, which the coordinates cannot say. A
// revision that carries none inherits the floor's — taught in a *previous* SLAM
// session's frame, and so wrong for this map by however far the two origins
// differ. That is a reason to look before promoting, which is what this pane is
// for, so it is said out loud rather than left to the validator's warning list.
export function zoneSource(source, count) {
  if (!count) return 'this revision carries no zones';
  if (source === 'floor') {
    return 'inherited from the floor — this revision carries none, so these were taught on another map frame';
  }
  return 'taught in this revision’s own frame';
}

// One zone's geometry in a phrase. The vocabulary half (kind, aliases) is shown
// in its own cells; this is the binding half, which is the half that is only
// true against the map beside it.
export function zoneSummary(zone) {
  if (zone.polygon && zone.polygon.length >= 3) {
    return `polygon, ${zone.polygon.length} vertices`;
  }
  if (typeof zone.radius === 'number') return `circle, r ${zone.radius} m`;
  if (zone.x !== undefined && zone.y !== undefined) {
    return `waypoint ${zone.x.toFixed(2)}, ${zone.y.toFixed(2)}`;
  }
  return 'no footprint';
}

// -- the view -------------------------------------------------------------

function el(tag, attributes = {}, children = []) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attributes)) {
    if (key === 'class') node.className = value;
    else if (key === 'text') node.textContent = value;
    else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
    else if (value !== null && value !== undefined) node.setAttribute(key, value);
  }
  for (const child of [].concat(children)) if (child) node.append(child);
  return node;
}

export class ReviewView {
  constructor({ api, dom, onPromoted = () => {} }) {
    this.api = api;
    this.dom = dom;
    this.onPromoted = onPromoted;
    this.floors = [];
    this.key = null;
    this.detail = null;
    this.selected = null;
    // Every load is stamped, and a load whose stamp is stale drops its result
    // on the floor. Selecting a floor fires three fetches (detail, map, zones)
    // and an image decode; without this, clicking through two candidates draws
    // whichever finished last rather than the one selected.
    this.epoch = 0;
    // The zones last loaded for the selected revision, which is what an edit
    // starts from and what the map goes back to if the edit is cancelled.
    this.zones = [];
    this.editing = false;
    this.map = new MapView(dom.canvas);
    this.editor = new ZoneEditor(this.map, {
      panel: dom.editor,
      rows: dom.editorRows,
      note: dom.editorNote,
    });
    dom.floor.addEventListener('change', () => this.open(dom.floor.value));
    dom.promote.addEventListener('click', () => this.promote());
    dom.fit.addEventListener('click', () => {
      this.map.fit();
      this.map.draw();
    });
    dom.zonesEdit.addEventListener('click', () => this.beginEdit());
    dom.zoneAdd.addEventListener('click', () => this.editor.addZone());
    dom.zoneSave.addEventListener('click', () => this.saveZones());
    dom.zoneCancel.addEventListener('click', () => this.endEdit());
  }

  // The pane's canvas has no size until the pane is on screen, so a fit done
  // before that yields a scale of 0 and a map that stays blank for ever.
  shown() {
    this.map.shown();
  }

  // -- loading ----------------------------------------------------------

  // Floors come from the registry's own list, not from what a robot is
  // reporting: a floor nobody is standing on is exactly the one to review.
  async loadFloors() {
    try {
      const body = await this.api('/v1/sites');
      this.floors = body.sites || [];
    } catch (error) {
      this.floors = [];
      this.note(`could not list floors: ${error.message}`, true);
    }
    this.renderFloors();
    if (!this.key && this.floors.length) {
      await this.open(floorKey(this.floors[0].site, this.floors[0].floor));
    }
  }

  async open(key, revision = null) {
    const parsed = parseFloorKey(key);
    if (!parsed) return;
    // An edit in progress owns the pane: re-opening a floor would reload the
    // zones under it and drop the edit on the floor. The controls that lead
    // here are disabled while editing; this covers the ones that arrive from
    // elsewhere (the map pane's jump button).
    if (this.editing) return;
    const epoch = (this.epoch += 1);
    this.key = key;
    this.dom.floor.value = key;
    this.note('');
    try {
      this.detail = await this.api(floorPath(parsed.site, parsed.floor));
    } catch (error) {
      this.detail = null;
      this.note(error.message, true);
    }
    if (epoch !== this.epoch) return;
    this.renderRevisions();
    const wanted =
      (revision &&
        orderedRevisions(this.detail).find((row) => row.revision === revision)) ||
      defaultRevision(this.detail);
    await this.select(wanted, epoch);
  }

  async select(revision, epoch = null) {
    if (epoch === null) epoch = (this.epoch += 1);
    this.selected = revision;
    this.renderRevisions();
    this.renderVerdict();
    if (!revision) {
      this.map.clearMap();
      this.dom.mapLabel.textContent = this.detail
        ? 'no revisions on this floor'
        : 'no floor selected';
      this.renderZones([]);
      return;
    }
    const { site, floor } = parseFloorKey(this.key);
    this.dom.mapLabel.textContent = `${this.key} · ${revision.revision}`;
    try {
      const meta = await this.api(revisionPath(site, floor, revision.revision, 'map.json'));
      const image = new Image();
      image.src = meta.image_url;
      await image.decode();
      if (epoch !== this.epoch) return;
      // Two revisions of one floor are compared by switching between them, so
      // the viewport is kept — unless the maps are different sizes, where
      // keeping it would leave the new one off screen.
      const sized = this.map.map;
      const refit =
        !sized || sized.width !== meta.width || sized.height !== meta.height;
      this.map.setMap(meta, image, refit);
    } catch (error) {
      if (epoch !== this.epoch) return;
      this.map.clearMap();
      this.dom.mapLabel.textContent = `${this.key} · ${revision.revision} — ${error.message}`;
    }
    // A revision may legitimately carry no zones, which is a 404 and not a
    // failure worth a red note.
    try {
      const body = await this.api(
        revisionPath(site, floor, revision.revision, 'zones.json'),
      );
      if (epoch !== this.epoch) return;
      this.map.setZones(body.zones || []);
      this.renderZones(body.zones || [], body.source);
    } catch (error) {
      if (epoch !== this.epoch) return;
      this.map.setZones([]);
      this.renderZones([]);
    }
  }

  // -- editing ----------------------------------------------------------

  // Editable when there is a revision selected and its map is on screen: the
  // coordinates being dragged mean nothing except against that image, and a
  // revision whose map failed to load has none.
  editable() {
    return Boolean(this.selected && this.map.map);
  }

  // Editing is a mode on the selected revision, so while it is on, the things
  // that would swap that revision out from under it are disabled rather than
  // racing it. There is no autosave: an unsaved edit is lost to `cancel`, and
  // nothing else can reach it.
  renderEditControls() {
    this.dom.zonesEdit.disabled = this.editing || !this.editable();
    this.dom.zonesEdit.hidden = !this.editable() && !this.editing;
    this.dom.floor.disabled = this.editing;
    this.dom.zones.hidden = this.editing;
    this.dom.zoneSource.hidden = this.editing;
    for (const row of this.dom.revisions.querySelectorAll('button')) {
      row.disabled = this.editing;
    }
    if (this.editing) this.dom.promote.disabled = true;
  }

  beginEdit() {
    if (!this.editable() || this.editing) return;
    this.editing = true;
    // The editor draws its own zones, handles and pose crosses; leaving the
    // read-only set under them would double every outline.
    this.map.setZones([]);
    this.editor.begin(this.zones);
    this.renderEditControls();
    this.note(
      this.selected.canonical
        ? 'editing the published map’s zones — saving derives a new candidate'
        : `editing candidate ${this.selected.revision} — saving derives a new one`,
    );
  }

  endEdit() {
    this.editing = false;
    this.editor.note('');
    this.editor.end();
    this.map.setZones(this.zones);
    this.renderEditControls();
    this.renderVerdict();
  }

  // Saving does not write the revision on screen: the server packs that
  // revision's map bytes with the submitted zones and stores the result as an
  // ordinary candidate. So the pane then *selects* the new candidate, and the
  // zones on screen afterwards are the saved ones read back from the server —
  // rather than the frozen overlay this needed when the editor lived on the
  // operations map, where the only thing to re-render was the stale set the
  // edit was made from (which read as data loss, 2026-08-02).
  async saveZones() {
    if (!this.editing || !this.selected) return;
    const problem = this.editor.problems();
    if (problem) {
      this.editor.note(problem, true);
      return;
    }
    const { site, floor } = parseFloorKey(this.key);
    const from = this.selected.revision;
    this.editor.note('saving…');
    let body;
    try {
      body = await this.api(`/v1/sites/${site}/floors/${floor}/zones`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ schema: 1, revision: from, zones: this.editor.payload() }),
      });
    } catch (error) {
      this.editor.note(error.message, true);
      return;
    }
    this.endEdit();
    await this.loadFloors();
    await this.open(this.key, body.revision);
    this.note(`candidate ${body.revision} saved from ${from}; promote it when it looks right`);
  }

  // -- rendering --------------------------------------------------------

  renderFloors() {
    const current = this.key;
    this.dom.floor.replaceChildren(
      ...this.floors.map((entry) => {
        const key = floorKey(entry.site, entry.floor);
        const candidates = (entry.candidates || []).length;
        return el('option', {
          value: key,
          text: candidates ? `${key} — ${candidates} candidate${candidates > 1 ? 's' : ''}` : key,
        });
      }),
    );
    if (current) this.dom.floor.value = current;
    this.dom.floors.hidden = this.floors.length === 0;
  }

  renderRevisions() {
    const revisions = orderedRevisions(this.detail);
    this.dom.canonical.textContent = this.detail
      ? this.detail.canonical || 'nothing published yet'
      : '—';
    this.dom.revisions.replaceChildren(
      ...revisions.map((revision) =>
        el(
          'button',
          {
            type: 'button',
            class: `revision-row ${
              this.selected && this.selected.revision === revision.revision ? 'selected' : ''
            } ${revision.ok ? '' : 'bad'}`,
            onclick: () => this.select(revision),
          },
          [
            el('span', { class: 'revision-id', text: revision.revision }),
            el('span', {
              class: 'revision-tag',
              text: revision.canonical ? 'published' : 'candidate',
            }),
            el('span', {
              class: 'revision-sub',
              text: `${revision.robot_id || revision.uploaded_by || 'unattributed'} · ${
                revision.ok ? 'valid' : revision.errors[0] || 'invalid'
              }`,
            }),
          ],
        ),
      ),
    );
    if (!revisions.length) {
      this.dom.revisions.replaceChildren(
        el('p', { class: 'empty', text: 'no revisions on this floor' }),
      );
    }
    this.renderEditControls();
  }

  renderVerdict() {
    const { promotable, verdict, state, notes, notesLabel } = promotability(this.selected);
    this.dom.verdict.replaceChildren(
      el('span', { class: `dot ${state}` }),
      el('span', { text: verdict }),
    );
    this.dom.notesLabel.textContent = notesLabel;
    this.dom.notesLabel.hidden = !notesLabel;
    this.dom.verdictNotes.replaceChildren(
      ...notes.map((note) => el('li', { text: note })),
    );
    this.dom.promote.disabled = !promotable;
    this.dom.provenance.replaceChildren(
      ...provenanceRows(this.selected).map(([key, value]) =>
        el('div', { class: 'fact' }, [
          el('span', { class: 'fact-key', text: key }),
          el('span', { class: 'fact-value', text: String(value) }),
        ]),
      ),
    );
  }

  // One row per zone, three cells — what the revision says its places are.
  // `edit zones` replaces this list with the editable one; it is the same set
  // of rows with inputs in them.
  renderZones(zones, source = '') {
    this.zones = zones;
    this.dom.zoneSource.textContent = zoneSource(source, zones.length);
    this.dom.zones.replaceChildren(
      ...zones.map((zone) =>
        el('div', { class: 'zone-row' }, [
          el('span', { class: 'zone-name', text: zone.display_name || zone.name }),
          el('span', { class: 'zone-kind', text: zone.kind || 'area' }),
          el('span', { class: 'dim', text: zoneSummary(zone) }),
        ]),
      ),
    );
    this.renderEditControls();
  }

  note(text, bad = false) {
    this.dom.note.textContent = text;
    this.dom.note.className = `note ${bad ? 'error' : ''}`;
  }

  // -- the one write ----------------------------------------------------

  async promote() {
    if (!this.selected || !this.key) return;
    const { site, floor } = parseFloorKey(this.key);
    const revision = this.selected.revision;
    this.note(`promoting ${revision}…`);
    try {
      const body = await this.api(
        `${floorPath(site, floor)}/revisions/${revision}/promote`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: '{"schema":1}',
        },
      );
      // Re-read rather than assume: this floor's canonical revision, its
      // basemap and every robot's "is it running the fleet's map" answer have
      // all just changed. The result is reported *after* the re-read, because
      // opening a floor clears this note — say it first and the one message
      // that reports what happened is wiped by the refresh that follows it.
      await this.loadFloors();
      await this.open(this.key, revision);
      this.note(
        body.announced
          ? `${site}/${floor} is on ${body.revision}; robots will pull it`
          : `promoted, but not announced: ${body.detail}`,
        !body.announced,
      );
      this.onPromoted(site, floor, revision);
    } catch (error) {
      this.note(error.message, true);
    }
  }
}
