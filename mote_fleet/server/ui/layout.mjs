// One pane at a time, on a viewport too narrow for three.
//
// The desktop layout puts roster, map and detail side by side, and gets one
// thing for free by doing so: picking a robot in the roster shows it on the map
// without the operator asking. A phone cannot show three panes, so this module
// supplies the missing half — a tab bar, and the rule that selecting a robot
// moves you to the map.
//
// The breakpoint is the seam. CSS decides which panes are *displayed*; this
// decides when a selection should navigate. Disagree and nothing fails: you get
// a tab bar over three stacked panes, or a phone where choosing a robot appears
// to do nothing because the map it selected on is off screen. So the number
// lives here, the stylesheet is required to match it, and `ui_test.mjs` reads
// both files and asserts they do.

export const NARROW_MAX_PX = 760;

export const NARROW_MEDIA = `(max-width: ${NARROW_MAX_PX}px)`;

// Wire the tab bar to the panes. Returns `show(name)`, which is a no-op on a
// wide viewport in effect but not in fact: it still moves `.active`, so a
// window narrowed later opens on the pane the operator last chose.
export function setupPanes({ root = document, onShow = () => {} } = {}) {
  const tabs = [...root.querySelectorAll('.panes [data-pane]')];
  const panes = [...root.querySelectorAll('.pane[data-pane]')];

  function show(name) {
    if (!panes.some((pane) => pane.dataset.pane === name)) return;
    for (const pane of panes) pane.classList.toggle('active', pane.dataset.pane === name);
    for (const tab of tabs) {
      const selected = tab.dataset.pane === name;
      tab.classList.toggle('active', selected);
      tab.setAttribute('aria-selected', String(selected));
    }
    onShow(name);
  }

  for (const tab of tabs) tab.addEventListener('click', () => show(tab.dataset.pane));

  const narrow = window.matchMedia(NARROW_MEDIA);
  // Crossing the breakpoint changes which panes exist on screen. The map is the
  // one that cares: its canvas has no size while hidden, so it needs telling.
  narrow.addEventListener('change', () => onShow(current()));

  function current() {
    const active = panes.find((pane) => pane.classList.contains('active'));
    return active ? active.dataset.pane : null;
  }

  return { show, current, isNarrow: () => narrow.matches };
}
