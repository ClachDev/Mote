// The palette, for the parts of this page a stylesheet cannot reach.
//
// A canvas gets no cascade: `style.css` decides how every element looks and
// reaches nothing inside `ctx.fillStyle`. So each canvas module carried its own
// copy of the colours — seventeen literals across `map.mjs` and
// `zone_editor.mjs` — and the copies drifted from the tokens sitting beside
// them. One of those copies was a bug rather than a mismatch: a robot's id was
// drawn in the dark theme's near-white with a black halo, so on a page asked
// for the light theme the label over a robot read as an outline of itself.
//
// The fix is the pattern the M3 scale bar already used, made the only one:
// every colour a canvas needs is a custom property on `:root`, defined in both
// theme blocks, and this is the one module that reads them. `ui_test.mjs`
// enforces both halves — no colour literal outside this file, and no token
// asked for here that `style.css` does not define in both themes.
//
// Reading is not free (`getComputedStyle` flushes style, and one frame wants a
// dozen colours), so the palette is read once and cached. The cache is dropped
// when the theme changes and the subscribers are told, because a canvas has no
// other reason to repaint: nothing in the DOM has changed.

//: What each drawing module asks for, and the custom property behind it. The
//: state names double as `robot.state` values — see `stateColour`.
export const TOKENS = {
  accent: '--accent',
  dim: '--dim',
  zoneStroke: '--zone-stroke',
  zoneFill: '--zone-fill',
  labelInk: '--label-ink',
  labelHalo: '--label-halo',
  edit: '--edit',
  editStroke: '--edit-stroke',
  editFill: '--edit-fill',
  editSelected: '--edit-selected',
  editHover: '--edit-hover',
  editRing: '--edit-ring',
  ok: '--ok',
  degraded: '--degraded',
  fault: '--fault',
  stale: '--stale',
  unknown: '--unknown',
  offline: '--offline',
};

// The dark theme's values, for the one case a token cannot answer: a draw that
// happens before the stylesheet has loaded, where `getPropertyValue` returns an
// empty string and an empty `fillStyle` silently keeps the previous colour.
// These are the only colour literals in the UI, and the reason this file is the
// one exception the literal scan makes.
export const FALLBACK = {
  accent: '#58a6ff',
  dim: '#8b949e',
  zoneStroke: 'rgba(88, 166, 255, 0.75)',
  zoneFill: 'rgba(88, 166, 255, 0.10)',
  labelInk: '#e6edf3',
  labelHalo: 'rgba(0, 0, 0, 0.65)',
  edit: 'rgba(240, 130, 34, 1)',
  editStroke: 'rgba(240, 130, 34, 0.9)',
  editFill: 'rgba(240, 130, 34, 0.08)',
  editSelected: 'rgba(240, 130, 34, 0.18)',
  editHover: 'rgba(240, 130, 34, 0.3)',
  editRing: 'rgba(13, 17, 23, 0.9)',
  ok: '#3fb950',
  degraded: '#d29922',
  fault: '#f85149',
  stale: '#8b949e',
  unknown: '#8b949e',
  offline: '#484f58',
};

// Pure, so the fallback rule is testable without a browser: anything the
// stylesheet does not answer for keeps the value above.
export function readTokens(style) {
  const palette = {};
  for (const [key, token] of Object.entries(TOKENS)) {
    const value = (style.getPropertyValue(token) || '').trim();
    palette[key] = value || FALLBACK[key];
  }
  return palette;
}

// A health state is not an arbitrary key: matching `robot.state` straight
// against the palette would let an unexpected value name `accent` or `edit` and
// draw a robot in it.
const STATES = ['ok', 'degraded', 'fault', 'stale', 'unknown', 'offline'];

export function stateColour(palette, state) {
  return STATES.includes(state) ? palette[state] : palette.unknown;
}

let cache = null;
const listeners = new Set();
let watching = false;

export function theme() {
  if (!cache) {
    cache =
      typeof document === 'undefined'
        ? { ...FALLBACK }
        : readTokens(getComputedStyle(document.documentElement));
  }
  return cache;
}

// The theme changed under the page: re-read, and tell whoever draws.
export function refresh() {
  cache = null;
  const palette = theme();
  for (const listener of listeners) listener(palette);
}

export function onThemeChange(listener) {
  listeners.add(listener);
  watch();
  return () => listeners.delete(listener);
}

// One matchMedia listener for the whole page, attached on the first subscriber
// rather than at import, so this module stays importable under node.
function watch() {
  if (watching || typeof window === 'undefined' || !window.matchMedia) return;
  watching = true;
  window.matchMedia('(prefers-color-scheme: light)').addEventListener('change', refresh);
}
