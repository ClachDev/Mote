// The fleet map: a floor's basemap PNG with live robots drawn on it.
//
// A pannable, zoomable 2D canvas, not a thumbnail (fleet.md Q5). It owns
// exactly one job — fleet-wide situational awareness in the map frame — and
// deliberately renders no 3D, no point clouds and no costmaps: those are the
// per-robot deep view, which the roster deep-links to Foxglove for.
//
// Two coordinate systems meet here and the order matters:
//
//   map frame (metres)  --worldToPixel-->  basemap pixels  --view--> screen
//
// The first hop is the floor's own `map.yaml` (`resolution`, `origin`) and is
// the reason a pose payload carries its site and floor: a map frame's origin is
// an accident of where SLAM started, so metres from one floor mean nothing on
// another. The second hop is pan/zoom and belongs to the viewer alone.

// What to call a zone on a map. `display_name` is the half meant for reading —
// "The Kitchen" — and the machine name is the half meant for typing, so a zone
// that has been given one is drawn with it. The editor draws its own overlay
// and reads this too: a zone should not answer to one name in the list and
// another the moment it is being edited.
export function zoneLabel(zone) {
  return (zone && (zone.display_name || zone.name)) || '';
}

// A zone as pixels on the basemap: a polygon's vertices, or a circle. Pure, so
// the placement of a taught place is testable the same way a robot's is.
export function zoneOutline(map, zone) {
  if (zone.polygon && zone.polygon.length >= 3) {
    return {
      kind: 'polygon',
      points: zone.polygon.map(([x, y]) => worldToPixel(map, x, y)),
    };
  }
  if (typeof zone.radius === 'number' && zone.x !== undefined) {
    return {
      kind: 'circle',
      centre: worldToPixel(map, zone.x, zone.y),
      radius: zone.radius / map.resolution,
    };
  }
  return null;
}

// The Q5 transform. Image y runs top-down while the map frame's runs up.
export function worldToPixel(map, x, y) {
  return {
    x: (x - map.origin[0]) / map.resolution,
    y: map.height - (y - map.origin[1]) / map.resolution,
  };
}

export function pixelToWorld(map, px, py) {
  return {
    x: px * map.resolution + map.origin[0],
    y: (map.height - py) * map.resolution + map.origin[1],
  };
}

// -- pinch ---------------------------------------------------------------

// Two touches, as one gesture: how far apart they are and where their midpoint
// is. Pure, because the alternative to testing this is discovering on a phone
// that the map flies off screen.
export function pinchSpan(a, b) {
  return {
    distance: Math.hypot(a.x - b.x, a.y - b.y),
    centre: { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 },
  };
}

// What one move of a two-finger gesture asks for: a zoom factor, the point to
// zoom about, and how far the fingers carried the map with them.
//
// The zero guard is load-bearing rather than defensive. Two pointers reported
// at the same position give distance 0, and a factor of 0/0 or n/0 puts NaN or
// Infinity into the view scale — from which every subsequent draw is blank,
// with nothing on screen to say why.
export function pinchUpdate(before, after) {
  const factor = before.distance > 0 ? after.distance / before.distance : 1;
  return {
    factor,
    centre: after.centre,
    pan: {
      x: after.centre.x - before.centre.x,
      y: after.centre.y - before.centre.y,
    },
  };
}

// The view that shows the whole basemap, centred, with a small margin.
export function fitView(map, width, height, margin = 16) {
  const scale = Math.min(
    (width - margin * 2) / map.width,
    (height - margin * 2) / map.height,
  );
  return {
    scale,
    tx: (width - map.width * scale) / 2,
    ty: (height - map.height * scale) / 2,
  };
}

// Taught places are context, not the subject: drawn under the robots, in one
// muted colour, so a `goto <zone>` target can be read off the map without
// competing with where anything actually is.
const ZONE_STROKE = 'rgba(88, 166, 255, 0.75)';
const ZONE_FILL = 'rgba(88, 166, 255, 0.10)';

const STATE_COLOURS = {
  ok: '#3fb950',
  degraded: '#d29922',
  fault: '#f85149',
  stale: '#8b949e',
  unknown: '#8b949e',
  offline: '#484f58',
};

export class MapView {
  constructor(canvas, { onSelect } = {}) {
    this.canvas = canvas;
    this.context = canvas.getContext('2d');
    this.onSelect = onSelect || (() => {});
    this.map = null;
    this.image = null;
    this.robots = [];
    this.zones = [];
    this.view = { scale: 1, tx: 0, ty: 0 };
    this.followId = null;
    this.selectedId = null;
    // A caller-supplied pass drawn over everything (the zone editor's
    // handles); null means no overlay and costs nothing.
    this.overlay = null;
    this._drag = null;
    this._pointers = new Map();
    this._pinch = null;
    // A fit needs a sized canvas, and on a narrow viewport this one spends its
    // early life in a hidden pane with no size at all. Track whether the map on
    // screen has ever actually been fitted, so `shown()` can finish the job.
    this._fitted = false;
    this._bind();
  }

  // -- data -------------------------------------------------------------

  // `refit` overrides the default test for a caller that knows better. The
  // fleet map changes basemap only when it changes floor, so that is the test;
  // the review view swaps between revisions *of* one floor, where holding the
  // viewport is how two candidates get compared — but a revision of a different
  // size has to be re-fitted or it lands off screen.
  setMap(map, image, refit = null) {
    const changed =
      refit === null
        ? !this.map || this.map.site !== map.site || this.map.floor !== map.floor
        : refit;
    this.map = map;
    this.image = image;
    if (changed) {
      this._fitted = false;
      this.fit();
    }
    this.draw();
  }

  clearMap() {
    this.map = null;
    this.image = null;
    this.zones = [];
    this._fitted = false;
    this.draw();
  }

  setRobots(robots) {
    this.robots = robots;
    this.draw();
  }

  // Zones belong to the floor, not to a robot: they are coordinates in this
  // basemap's frame, which is why they arrive with the map and are cleared
  // with it.
  setZones(zones) {
    this.zones = zones || [];
    this.draw();
  }

  follow(robotId) {
    this.followId = robotId;
    this.draw();
  }

  select(robotId) {
    this.selectedId = robotId;
    this.draw();
  }

  // -- view -------------------------------------------------------------

  fit() {
    if (!this.map) return;
    const { width, height } = this._size();
    // A hidden pane's canvas measures 0x0, and fitting into that yields a scale
    // of 0 or NaN — a blank map that stays blank once the pane is shown. Leave
    // the view alone and let `shown()` fit when there is something to fit into.
    if (!width || !height) return;
    this.view = fitView(this.map, width, height);
    this._fitted = true;
  }

  // The map pane has become visible (a tab switch, or the viewport crossing the
  // breakpoint). This is the first moment its canvas has a size.
  shown() {
    if (!this._fitted) this.fit();
    this.draw();
  }

  zoomBy(factor, screenX, screenY) {
    const next = Math.max(0.05, Math.min(60, this.view.scale * factor));
    const ratio = next / this.view.scale;
    this.view = {
      scale: next,
      tx: screenX - (screenX - this.view.tx) * ratio,
      ty: screenY - (screenY - this.view.ty) * ratio,
    };
    this.draw();
  }

  _size() {
    const rect = this.canvas.getBoundingClientRect();
    return { width: rect.width, height: rect.height };
  }

  _toScreen(worldX, worldY) {
    return this._screenOf(worldToPixel(this.map, worldX, worldY));
  }

  // Canvas-relative coordinates. Not `offsetX`: during a pointer capture — and
  // for the second finger of a pinch — that is measured against whatever the
  // event happens to be over, which is not always this canvas.
  _local(event) {
    const rect = this.canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  _bind() {
    const canvas = this.canvas;
    canvas.addEventListener('pointerdown', (event) => {
      canvas.setPointerCapture(event.pointerId);
      this._pointers.set(event.pointerId, this._local(event));

      // A second finger means a pinch, whatever the first one had started.
      if (this._pointers.size === 2) {
        this._drag = null;
        canvas.classList.remove('dragging');
        this._pinch = this._span();
        return;
      }
      if (this._pointers.size > 2) return;

      const point = this._local(event);
      const hit = this._robotAt(point.x, point.y);
      if (hit) {
        this.onSelect(hit.id);
        return;
      }
      // Dragging the map is a deliberate pan, so it stops following: an
      // operator who grabs the canvas wants to look somewhere else.
      this.followId = null;
      this._drag = point;
      canvas.classList.add('dragging');
    });
    canvas.addEventListener('pointermove', (event) => {
      if (!this._pointers.has(event.pointerId)) return;
      const point = this._local(event);
      this._pointers.set(event.pointerId, point);

      if (this._pinch) {
        const span = this._span();
        if (!span) return;
        const { factor, centre, pan } = pinchUpdate(this._pinch, span);
        this._pinch = span;
        // Fingers carry the map with them as well as scaling it, so the pan
        // lands before the zoom is taken about where they now are.
        this.view.tx += pan.x;
        this.view.ty += pan.y;
        this.zoomBy(factor, centre.x, centre.y);
        return;
      }

      if (!this._drag) return;
      this.view.tx += point.x - this._drag.x;
      this.view.ty += point.y - this._drag.y;
      this._drag = point;
      this.draw();
    });
    const release = (event) => {
      this._pointers.delete(event.pointerId);
      if (canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
      if (this._pointers.size < 2 && this._pinch) {
        this._pinch = null;
        // One finger left: hand it the pan rather than freezing until it lifts.
        // Its position is where the drag resumes, so the map does not jump.
        const [remaining] = [...this._pointers.values()];
        this._drag = remaining || null;
        if (this._drag) canvas.classList.add('dragging');
        return;
      }
      if (!this._pointers.size) {
        this._drag = null;
        canvas.classList.remove('dragging');
      }
    };
    canvas.addEventListener('pointerup', release);
    canvas.addEventListener('pointercancel', release);
    canvas.addEventListener(
      'wheel',
      (event) => {
        event.preventDefault();
        this.zoomBy(Math.exp(-event.deltaY * 0.002), event.offsetX, event.offsetY);
      },
      { passive: false },
    );
    window.addEventListener('resize', () => {
      if (!this._fitted) this.fit();
      this.draw();
    });
  }

  _span() {
    const [a, b] = [...this._pointers.values()];
    return a && b ? pinchSpan(a, b) : null;
  }

  _robotAt(screenX, screenY) {
    if (!this.map) return null;
    // A fingertip is not a mouse pointer: the same 14 px target that is
    // comfortable with a cursor is most of the way to unhittable by touch.
    const reach =
      typeof window !== 'undefined' && window.matchMedia('(pointer: coarse)').matches ? 24 : 14;
    for (const robot of this.robots) {
      if (robot.x === null || robot.x === undefined) continue;
      const point = this._toScreen(robot.x, robot.y);
      if (Math.hypot(point.x - screenX, point.y - screenY) <= reach) return robot;
    }
    return null;
  }

  // -- drawing ----------------------------------------------------------

  draw() {
    const { width, height } = this._size();
    const ratio = window.devicePixelRatio || 1;
    // Height as well as width: the two change independently, and on a phone the
    // height changes often — a tab switch, the toolbar rewrapping, the URL bar
    // sliding away. Resizing on width alone leaves the backing store at its old
    // height, and `clearRect` (which works in CSS pixels) then cannot reach the
    // bottom of it: the previous frame's scale bar stays on screen under the
    // new one.
    const backing = { w: Math.round(width * ratio), h: Math.round(height * ratio) };
    if (this.canvas.width !== backing.w || this.canvas.height !== backing.h) {
      this.canvas.width = backing.w;
      this.canvas.height = backing.h;
    }
    const ctx = this.context;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    if (!this.map) return;

    if (this.followId) {
      const followed = this.robots.find((robot) => robot.id === this.followId);
      if (followed && followed.x !== null && followed.x !== undefined) {
        const pixel = worldToPixel(this.map, followed.x, followed.y);
        this.view.tx = width / 2 - pixel.x * this.view.scale;
        this.view.ty = height / 2 - pixel.y * this.view.scale;
      }
    }

    if (this.image) {
      // Occupancy pixels are data, not a photo: past 1:1 they should look like
      // cells rather than a blur.
      ctx.imageSmoothingEnabled = this.view.scale < 1.5;
      ctx.drawImage(
        this.image,
        this.view.tx,
        this.view.ty,
        this.map.width * this.view.scale,
        this.map.height * this.view.scale,
      );
    }

    for (const zone of this.zones) this._drawZone(ctx, zone);
    for (const robot of this.robots) {
      if (robot.x === null || robot.x === undefined) continue;
      this._drawRobot(ctx, robot);
    }
    this._drawScaleBar(ctx, height);
    if (this.overlay) this.overlay(ctx);
  }

  _drawZone(ctx, zone) {
    const outline = zoneOutline(this.map, zone);
    const label =
      zone.x === undefined || zone.y === undefined
        ? outline && outline.kind === 'polygon'
          ? outline.points[0]
          : null
        : worldToPixel(this.map, zone.x, zone.y);

    ctx.save();
    ctx.strokeStyle = ZONE_STROKE;
    ctx.fillStyle = ZONE_FILL;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    if (outline && outline.kind === 'polygon') {
      ctx.beginPath();
      outline.points.forEach((point, index) => {
        const screen = this._screenOf(point);
        if (index === 0) ctx.moveTo(screen.x, screen.y);
        else ctx.lineTo(screen.x, screen.y);
      });
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    } else if (outline && outline.kind === 'circle') {
      const centre = this._screenOf(outline.centre);
      ctx.beginPath();
      ctx.arc(centre.x, centre.y, outline.radius * this.view.scale, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    } else if (label) {
      // A waypoint with no footprint is still a place you can send a robot to.
      const point = this._screenOf(label);
      ctx.beginPath();
      ctx.setLineDash([]);
      ctx.moveTo(point.x - 4, point.y);
      ctx.lineTo(point.x + 4, point.y);
      ctx.moveTo(point.x, point.y - 4);
      ctx.lineTo(point.x, point.y + 4);
      ctx.stroke();
    }
    ctx.restore();

    if (!label) return;
    const point = this._screenOf(label);
    ctx.font = '11px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillStyle = ZONE_STROKE;
    // display_name is what an operator calls the place; the machine name is
    // what they would type. Prefer the former on the map, where this is a
    // label rather than a thing to copy.
    ctx.fillText(zoneLabel(zone), point.x, point.y + 14);
  }

  _screenOf(pixel) {
    return {
      x: pixel.x * this.view.scale + this.view.tx,
      y: pixel.y * this.view.scale + this.view.ty,
    };
  }

  _drawRobot(ctx, robot) {
    const point = this._toScreen(robot.x, robot.y);
    const colour = STATE_COLOURS[robot.state] || STATE_COLOURS.unknown;
    const radius = 7;

    // Heading first, so the marker sits on top of its own wedge.
    ctx.save();
    ctx.translate(point.x, point.y);
    ctx.rotate(-robot.yaw); // map yaw is counter-clockwise; screen y is down
    ctx.beginPath();
    ctx.moveTo(radius - 1, 0);
    ctx.lineTo(radius + 12, 0);
    ctx.strokeStyle = colour;
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.restore();

    ctx.beginPath();
    ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = robot.stale ? 'transparent' : colour;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = colour;
    ctx.stroke();

    if (robot.id === this.selectedId) {
      ctx.beginPath();
      ctx.arc(point.x, point.y, radius + 5, 0, Math.PI * 2);
      ctx.strokeStyle = '#58a6ff';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.lineWidth = 3;
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.65)';
    ctx.strokeText(robot.label, point.x, point.y - radius - 8);
    ctx.fillStyle = '#e6edf3';
    ctx.fillText(robot.label, point.x, point.y - radius - 8);
  }

  _drawScaleBar(ctx, height) {
    // Pick the roundest number of metres that fits in ~120 px at this zoom.
    const pixelsPerMetre = this.view.scale / this.map.resolution;
    const candidates = [0.5, 1, 2, 5, 10, 20, 50];
    const metres =
      candidates.find((value) => value * pixelsPerMetre > 90) ||
      candidates[candidates.length - 1];
    const length = metres * pixelsPerMetre;
    const x = 16;
    const y = height - 18;
    // A canvas gets no cascade, so the stylesheet's light theme cannot reach in
    // here: drawn in the dark theme's near-white this is invisible on a phone
    // that has asked for light, over a map whose free space is white. `--dim`
    // is chosen for legibility against either background.
    const ink = getComputedStyle(this.canvas).getPropertyValue('--dim').trim() || '#8b949e';
    ctx.strokeStyle = ink;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y - 5);
    ctx.lineTo(x, y);
    ctx.lineTo(x + length, y);
    ctx.lineTo(x + length, y - 5);
    ctx.stroke();
    ctx.font = '11px ui-monospace, monospace';
    ctx.textAlign = 'left';
    ctx.fillStyle = ink;
    ctx.fillText(`${metres} m`, x + 4, y - 8);
  }
}
