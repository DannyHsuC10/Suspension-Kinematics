const background = '#111d31';
const gridColor = '#2a3d56';
const textColor = '#e3edf8';
const mutedColor = '#b4c5d9';
const movingColor = '#59d9c3';
const fixedColor = '#b8cbe1';
const selectionColors = ['#ffbd76', '#d6aeff'];
const seriesColors = ['#59d9c3', '#93b7ff', '#ffbd76', '#d6aeff', '#f291a7', '#c6db88'];
const font = '13px "Segoe UI", system-ui, sans-serif';
const tau = 2 * Math.PI;

const add = (a, b) => a.map((value, i) => value + b[i]);
const subtract = (a, b) => a.map((value, i) => value - b[i]);
const multiply = (a, factor) => a.map(value => value * factor);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const unit = vector => multiply(vector, 1 / Math.hypot(...vector));
const clamp = (number, min, max) => Math.max(min, Math.min(number, max));
const directions = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const orientations = {
  orbit: [.68, .48],
  iso: [.68, .48],
  front: [Math.PI / 2, 0],
  side: [0, 0],
  top: [-Math.PI / 2, Math.PI / 2],
};
const tireTrig = Array.from({ length: 32 }, (_, index) => {
  const angle = tau * index / 32;
  return [Math.cos(angle), Math.sin(angle)];
});

// Work in CSS pixels so picking and text stay consistent on high-DPI displays.
function canvasSurface(canvas) {
  const { width, height } = canvas.getBoundingClientRect();
  if (width < 1 || height < 1) return null;
  const ratio = window.devicePixelRatio || 1;
  const pixelWidth = Math.round(width * ratio);
  const pixelHeight = Math.round(height * ratio);
  if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
  if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
  const context = canvas.getContext('2d');
  context.setTransform(pixelWidth / width, 0, 0, pixelHeight / height, 0, 0);
  context.fillStyle = background;
  context.fillRect(0, 0, width, height);
  context.lineCap = 'round';
  context.lineJoin = 'round';
  context.font = font;
  return { context, width, height };
}

function eventPosition(canvas, event) {
  const rect = canvas.getBoundingClientRect();
  return [event.clientX - rect.left, event.clientY - rect.top];
}

function stroke(context, a, b, color, width = 1) {
  context.strokeStyle = color;
  context.lineWidth = width;
  context.beginPath();
  context.moveTo(a[0], a[1]);
  context.lineTo(b[0], b[1]);
  context.stroke();
}

function polygon(context, points, color, fill = null) {
  context.beginPath();
  for (const [index, [x, y]] of points.entries()) {
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  }
  context.closePath();
  if (fill) {
    context.fillStyle = fill;
    context.fill();
  }
  context.strokeStyle = color;
  context.lineWidth = 1;
  context.stroke();
}

function circle(context, [x, y], radius, color, fill = null) {
  context.beginPath();
  context.arc(x, y, radius, 0, tau);
  if (fill) {
    context.fillStyle = fill;
    context.fill();
  }
  context.strokeStyle = color;
  context.lineWidth = 1.4;
  context.stroke();
}

function message(context, width, height, value) {
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillStyle = mutedColor;
  context.fillText(value, width / 2, height / 2, Math.max(1, width - 28));
}

function observer(canvas, draw) {
  const resize = new ResizeObserver(draw);
  resize.observe(canvas);
  return resize;
}

function memberStyle(link) {
  if (link.deformable || link.kind === 'deformable') return ['Deformable member', '#ff9f9c'];
  if (link.name === 'FRONT_RACK_BAR') return ['Rack', '#c6db88'];
  if (link.name.endsWith('_RACK-TOE')) return ['Tie rod', '#8db5ff'];
  if (link.name.endsWith('_PUSH_ROD')) return ['Pushrod', '#e3a3ee'];
  return null;
}

function legend(context, entries, left, top, right) {
  let x = left;
  let y = top;
  context.textAlign = 'left';
  context.textBaseline = 'middle';
  for (const { name, color, dashed } of entries) {
    const textWidth = Math.min(context.measureText(name).width, Math.max(1, right - left - 25));
    const itemWidth = textWidth + 40;
    if (x > left && x + itemWidth > right) {
      x = left;
      y += 20;
    }
    context.setLineDash(dashed ? [4, 3] : []);
    stroke(context, [x, y], [x + 15, y], color, 2);
    context.setLineDash([]);
    context.fillStyle = textColor;
    context.fillText(name, x + 21, y, textWidth);
    x += itemWidth;
  }
  return entries.length ? y + 14 : top;
}

function geometryBounds(points, wheels, bounds) {
  if (bounds && bounds.length === 2) return bounds;
  const low = [Infinity, Infinity, Infinity];
  const high = [-Infinity, -Infinity, -Infinity];
  const include = (point, extent) => {
    for (let axis = 0; axis < 3; axis += 1) {
      low[axis] = Math.min(low[axis], point[axis] - extent);
      high[axis] = Math.max(high[axis], point[axis] + extent);
    }
  };
  for (const point of Object.values(points)) include(point, 0);
  for (const wheel of wheels) {
    if (points[wheel.center]) include(points[wheel.center], Math.hypot(wheel.width / 2, wheel.radius));
  }
  return [low, high];
}

function segmentDistance(point, a, b) {
  const segment = subtract(b, a);
  const offset = subtract(point, a);
  const squaredLength = dot(segment, segment);
  const fraction = squaredLength ? clamp(dot(offset, segment) / squaredLength, 0, 1) : 0;
  return Math.hypot(...subtract(offset, multiply(segment, fraction)));
}

export class MechanismViewport {
  constructor(canvas, onPick) {
    this.canvas = canvas;
    this.onPick = onPick;
    this.model = null;
    this.view = 'orbit';
    this.selectedPoints = [];
    this.selectedRods = [];
    this.referenceVisible = false;
    this.labelsVisible = false;
    this.drawnPoints = [];
    this.drawnRods = [];
    this.drag = null;
    this.resetCamera();
    this.resize = observer(canvas, () => this.draw());

    canvas.style.touchAction = 'none';
    canvas.addEventListener('contextmenu', event => event.preventDefault());
    canvas.addEventListener('pointerdown', event => this.startDrag(event));
    canvas.addEventListener('pointermove', event => this.moveDrag(event));
    canvas.addEventListener('pointerup', event => this.endDrag(event));
    canvas.addEventListener('pointercancel', () => { this.drag = null; });
    canvas.addEventListener('lostpointercapture', () => { this.drag = null; });
    canvas.addEventListener('wheel', event => this.onWheel(event), { passive: false });
  }

  setModel(model) {
    this.model = model;
    this.selectedPoints = [];
    this.selectedRods = [];
    this.points = model?.points;
    this.wheels = model?.wheels ?? [];
    if (this.points && Object.keys(this.points).length) {
      this.fixed = new Set(model.fixed ?? []);
      this.members = (model.links ?? []).filter(link => this.points[link.a] && this.points[link.b]);
      this.setBounds(model.bounds);
    }
    this.resetCamera();
  }

  setBounds(bounds) {
    if (!this.model || !Object.keys(this.model.points).length) return;
    const [low, high] = geometryBounds(this.model.points, this.model.wheels ?? [], bounds);
    this.center = multiply(add(low, high), .5);
    this.span = Math.max(.001, ...subtract(high, low));
    this.fitCorners = [low[0], high[0]].flatMap(x =>
      [low[1], high[1]].flatMap(y => [low[2], high[2]].map(z => [x, y, z])));
    const half = multiply(subtract(high, low), .5).map(value => Math.max(value, .15 * this.span));
    const [x0, y0] = subtract(this.center, half);
    const [x1, y1] = add(this.center, half);
    this.floor = [];
    for (let step = 0; step <= 8; step += 1) {
      const x = x0 + (x1 - x0) * step / 8;
      const y = y0 + (y1 - y0) * step / 8;
      this.floor.push([[x, y0, low[2]], [x, y1, low[2]]], [[x0, y, low[2]], [x1, y, low[2]]]);
    }
    this.draw();
  }

  setFrame(points = this.model?.points, wheels = this.model?.wheels ?? []) {
    this.points = points;
    this.wheels = wheels;
    this.draw();
  }

  setView(view) {
    if (!Object.hasOwn(orientations, view)) return;
    this.view = view;
    this.resetCamera();
  }

  resetCamera() {
    [this.azimuth, this.elevation] = orientations[this.view];
    this.zoom = 1;
    this.pan = [0, 0];
    this.draw();
  }

  setReferenceVisible(visible) {
    this.referenceVisible = Boolean(visible);
    this.draw();
  }

  setLabelsVisible(visible) {
    this.labelsVisible = Boolean(visible);
    this.draw();
  }

  setSelection(points = [], rods = []) {
    if (this.selectedPoints.length === points.length && this.selectedRods.length === rods.length &&
      points.every((point, i) => this.selectedPoints[i] === point) &&
      rods.every((rod, i) => this.selectedRods[i] === rod)) return;
    this.selectedPoints = points;
    this.selectedRods = rods;
    this.draw();
  }

  startDrag(event) {
    if (!this.model || this.drag || ![0, 1, 2].includes(event.button)) return;
    event.preventDefault();
    this.canvas.setPointerCapture(event.pointerId);
    const at = eventPosition(this.canvas, event);
    this.drag = { id: event.pointerId, at, origin: at, moved: false, pan: event.button > 0 || event.shiftKey };
  }

  moveDrag(event) {
    const drag = this.drag;
    if (!drag || drag.id !== event.pointerId) return;
    const at = eventPosition(this.canvas, event);
    if (!drag.moved && Math.hypot(...subtract(at, drag.origin)) < 4) return;
    drag.moved = true;
    const [dx, dy] = subtract(at, drag.at);
    drag.at = at;
    if (drag.pan) {
      this.pan = add(this.pan, [dx, dy]);
    } else {
      this.azimuth = (this.azimuth + dx * .01) % tau;
      this.elevation = clamp(this.elevation + dy * .01, -Math.PI / 2, Math.PI / 2);
    }
    this.draw();
  }

  endDrag(event) {
    if (this.drag?.id !== event.pointerId) return;
    const pick = !this.drag.pan && !this.drag.moved;
    this.drag = null;
    if (pick) this.pick(eventPosition(this.canvas, event));
  }

  onWheel(event) {
    if (!this.model) return;
    event.preventDefault();
    const { width, height } = this.canvas.getBoundingClientRect();
    const delta = event.deltaY * (event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? height : 1);
    const zoom = clamp(this.zoom * Math.exp(-delta * .002), .25, 10);
    const origin = add([width / 2, height / 2], this.pan);
    const offset = subtract(eventPosition(this.canvas, event), origin);
    this.pan = add(this.pan, multiply(offset, 1 - zoom / this.zoom));
    this.zoom = zoom;
    this.draw();
  }

  pick(at) {
    const hits = this.drawnPoints.flatMap(({ name, screen, radius, label }) => {
      const [x, y] = at;
      const distance = Math.hypot(...subtract(at, screen));
      if (distance <= radius + 5) {
        return [{ name, kind: 'point', priority: distance <= radius + 1 ? 0 : 2, distance }];
      }
      if (label && x >= label.x && x <= label.right && y >= label.top && y <= label.y) {
        return [{ name, kind: 'point', priority: 3, distance }];
      }
      return [];
    });
    for (const { name, a, b } of this.drawnRods) {
      const distance = segmentDistance(at, a, b);
      if (distance <= 6) hits.push({ name, kind: 'rod', priority: 1, distance });
    }
    hits.sort((a, b) => a.priority - b.priority || a.distance - b.distance);
    if (hits.length) this.onPick?.(hits[0].kind, hits[0].name);
  }

  drawTires(context, project, toward) {
    const wheels = this.wheels.filter(wheel => this.points[wheel.center] &&
      wheel.normal?.length === 3 && wheel.normal.every(Number.isFinite) &&
      Math.hypot(...wheel.normal) > 1e-10 && wheel.width > 0 && wheel.radius > 0);
    wheels.sort((a, b) => dot(subtract(this.points[a.center], this.points[b.center]), toward));
    for (const wheel of wheels) {
      const center = this.points[wheel.center];
      let normal = unit(wheel.normal);
      if (dot(normal, toward) < 0) normal = multiply(normal, -1);
      const anchor = directions.reduce((best, axis) =>
        Math.abs(dot(normal, axis)) < Math.abs(dot(normal, best)) ? axis : best);
      const radial = unit(cross(normal, anchor));
      const tangent = cross(normal, radial);
      const width = multiply(normal, wheel.width / 2);
      const ring = tireTrig.map(([cosine, sine]) =>
        multiply(add(multiply(radial, cosine), multiply(tangent, sine)), wheel.radius));
      const back = ring.map(point => project(add(subtract(center, width), point)));
      const front = ring.map(point => project(add(add(center, width), point)));
      polygon(context, back, '#526a85');
      ring.forEach((point, index) => {
        const next = (index + 1) % ring.length;
        if (dot(add(point, ring[next]), toward) > 0) {
          polygon(context, [back[index], back[next], front[next], front[index]], '#526a8554', '#6585a52e');
        }
      });
      polygon(context, front, '#819db9', '#6e90b13d');
    }
  }

  drawReference(context, project) {
    const { points } = this.model;
    context.setLineDash([4, 4]);
    for (const { a, b } of this.members) {
      if (!this.fixed.has(a) || !this.fixed.has(b)) {
        stroke(context, project(points[a]), project(points[b]), '#7189a4', 1.2);
      }
    }
    context.setLineDash([]);
    for (const [name, point] of Object.entries(points)) {
      if (!this.fixed.has(name)) circle(context, project(point), 3, '#7189a4');
    }
  }

  drawAxes(context, width, height, right, up) {
    const origin = [width - 44, height - 40];
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    for (const [index, axis] of directions.entries()) {
      const delta = multiply([dot(axis, right), -dot(axis, up)], 24);
      if (Math.hypot(...delta) < 3) continue;
      const color = ['#f4a2a5', '#8ddbb1', '#99beff'][index];
      const last = add(origin, delta);
      const label = add(origin, multiply(delta, 1.35));
      stroke(context, origin, last, color, 2);
      circle(context, last, 2, color, color);
      context.fillStyle = color;
      context.fillText('XYZ'[index], ...label);
    }
    circle(context, origin, 2, textColor, textColor);
  }

  draw() {
    const surface = canvasSurface(this.canvas);
    this.drawnPoints = [];
    this.drawnRods = [];
    if (!surface) return;
    const { context, width, height } = surface;
    if (!this.model || !this.points || !Object.keys(this.points).length) {
      message(context, width, height, 'Import hardpoints to view the mechanism.');
      return;
    }
    const right = [Math.cos(this.azimuth), Math.sin(this.azimuth), 0];
    const toward = [
      Math.sin(this.azimuth) * Math.cos(this.elevation),
      -Math.cos(this.azimuth) * Math.cos(this.elevation),
      Math.sin(this.elevation),
    ];
    const up = cross(toward, right);
    const fit = axis => {
      const projections = this.fitCorners.map(point => dot(subtract(point, this.center), axis));
      return Math.max(Math.max(...projections) - Math.min(...projections), this.span * .12);
    };
    const scale = this.zoom * Math.min(Math.max(1, width - 76) / fit(right), Math.max(1, height - 88) / fit(up));
    const origin = add([width / 2, height / 2], this.pan);
    const project = point => {
      const centered = subtract(point, this.center);
      return add(origin, multiply([dot(centered, right), -dot(centered, up)], scale));
    };
    const depth = point => dot(subtract(point, this.center), toward);

    for (const [a, b] of this.floor) stroke(context, project(a), project(b), gridColor);
    this.drawTires(context, project, toward);
    if (this.referenceVisible) this.drawReference(context, project);
    const entries = Object.entries(this.points).sort(([, a], [, b]) => depth(a) - depth(b));
    const screen = Object.fromEntries(entries.map(([name, point]) => [name, project(point)]));
    const visibleRods = this.members.filter(({ a, b }) => screen[a] && screen[b]).map(link => ({
      link,
      depth: (depth(this.points[link.a]) + depth(this.points[link.b])) / 2,
      color: selectionColors[this.selectedRods.indexOf(link.name)],
    }));
    visibleRods.sort((a, b) => Number(Boolean(a.color)) - Number(Boolean(b.color)) || a.depth - b.depth);
    for (const { link, color } of visibleRods) {
      const { name, a, b } = link;
      const style = memberStyle(link);
      const fixed = this.fixed.has(a) && this.fixed.has(b);
      const weight = color ? 4 : style ? 2.8 : 1.8;
      stroke(context, screen[a], screen[b], background, weight + 2);
      stroke(context, screen[a], screen[b], color ?? style?.[1] ?? (fixed ? '#6b82a0' : '#95bbd9'), weight);
      this.drawnRods.push({ name, a: screen[a], b: screen[b] });
    }

    context.textAlign = 'left';
    context.textBaseline = 'bottom';
    for (const [name] of entries) {
      const [x, y] = screen[name];
      const highlight = selectionColors[this.selectedPoints.indexOf(name)];
      const radius = highlight ? 6 : 4;
      if (this.fixed.has(name)) {
        context.fillStyle = background;
        context.fillRect(x - radius, y - radius, 2 * radius, 2 * radius);
        context.strokeStyle = highlight ?? fixedColor;
        context.lineWidth = 1.6;
        context.strokeRect(x - radius, y - radius, 2 * radius, 2 * radius);
      } else {
        circle(context, [x, y], radius, background, highlight ?? movingColor);
      }
      this.drawnPoints.push({ name, screen: [x, y], radius, highlight });
    }
    for (const entry of this.drawnPoints) {
      if (!this.labelsVisible && !entry.highlight) continue;
      const x = entry.screen[0] + 8;
      const y = entry.screen[1] - 5;
      context.fillStyle = entry.highlight ?? textColor;
      context.strokeStyle = background;
      context.lineWidth = 3;
      context.strokeText(entry.name, x, y);
      context.fillText(entry.name, x, y);
      entry.label = { x, y, right: x + context.measureText(entry.name).width, top: y - 14 };
    }
    this.drawAxes(context, width, height, right, up);
  }
}

function dataDomain(values, initial = [Infinity, -Infinity]) {
  const domain = initial.slice();
  for (const value of values) {
    if (!Number.isFinite(value)) continue;
    domain[0] = Math.min(domain[0], value);
    domain[1] = Math.max(domain[1], value);
  }
  return domain;
}

function expandedDomain(domain, pad = .055) {
  const [minimum, maximum] = domain;
  if (!Number.isFinite(minimum)) return [0, 1];
  if (minimum === maximum) {
    const span = Math.max(Math.abs(minimum) * .08, .5);
    return [minimum - span, maximum + span];
  }
  const span = (maximum - minimum) * pad;
  return [minimum - span, maximum + span];
}

function ticksFor([min, max], count) {
  const estimate = (max - min) / count;
  const place = 10 ** Math.floor(Math.log10(estimate));
  const step = ([1, 2, 2.5, 5, 10].find(size => size * place >= estimate) ?? 10) * place;
  if (!(step > 0) || !Number.isFinite(step)) return [min, max];
  const first = Math.ceil(min / step);
  const last = Math.floor(max / step);
  return Array.from({ length: clamp(last - first + 1, 0, 16) }, (_, i) => (first + i) * step);
}

const formatter = new Intl.NumberFormat('en', { maximumSignificantDigits: 6, useGrouping: false });

function tickLabel(value) {
  const magnitude = Math.abs(value);
  return magnitude > 0 && (magnitude < 1e-3 || magnitude >= 1e5)
    ? value.toExponential(2) : formatter.format(value);
}

export class LineChart {
  constructor(canvas, onSeek) {
    this.canvas = canvas;
    this.onSeek = onSeek;
    this.data = null;
    this.index = null;
    this.plot = null;
    this.resize = observer(canvas, () => this.draw());
    canvas.style.touchAction = 'pan-y';
    if (!canvas.hasAttribute('tabindex')) canvas.tabIndex = 0;
    canvas.addEventListener('click', event => this.seek(eventPosition(canvas, event)));
    canvas.addEventListener('keydown', event => this.seekKey(event));
    this.draw();
  }

  setData({ x, xLabel, xRange, yLabel, series, secondaryLabel }) {
    const lines = series.map((line, index) => ({ color: seriesColors[index % seriesColors.length], ...line }));
    const secondary = line => line.axis === 1 || line.axis === 'right' || line.axis === 'secondary';
    const xDomain = expandedDomain(dataDomain(x.length ? x : xRange ?? []), 0);
    const leftDomain = expandedDomain(lines.filter(line => !secondary(line))
      .reduce((range, line) => dataDomain(line.values, range), [Infinity, -Infinity]));
    const hasSecondary = lines.some(secondary);
    const rightDomain = expandedDomain(lines.filter(secondary)
      .reduce((range, line) => dataDomain(line.values, range), [Infinity, -Infinity]));
    this.data = { x, xLabel, yLabel, secondaryLabel, lines, secondary, xDomain, leftDomain, rightDomain, hasSecondary };
    this.draw();
  }

  setCursor(index) {
    this.index = index;
    this.draw();
  }

  clear() {
    this.data = null;
    this.index = null;
    this.plot = null;
    this.draw();
  }

  destroy() {
    this.resize?.disconnect();
  }

  seek([screenX, screenY]) {
    if (!this.plot || !this.data || screenY < this.plot.top || screenY > this.plot.bottom ||
      screenX < this.plot.left || screenX > this.plot.right) return;
    const target = this.plot.xInverse(screenX);
    let nearest = -1;
    let distance = Infinity;
    this.data.x.forEach((value, index) => {
      const delta = Math.abs(value - target);
      if (Number.isFinite(value) && delta < distance) {
        nearest = index;
        distance = delta;
      }
    });
    if (nearest !== -1) this.onSeek?.(nearest);
  }

  seekKey(event) {
    if (!this.data?.x.length || !['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const last = this.data.x.length - 1;
    const delta = event.key === 'ArrowLeft' ? -1 : 1;
    const next = event.key === 'Home' ? 0 : event.key === 'End' ? last : clamp((this.index ?? 0) + delta, 0, last);
    this.onSeek?.(next);
  }

  draw() {
    const surface = canvasSurface(this.canvas);
    if (!surface) return;
    const { context, width, height } = surface;
    this.plot = null;
    if (!this.data) {
      message(context, width, height, 'Run an analysis to plot results.');
      return;
    }
    const { x, xLabel, yLabel, secondaryLabel, lines, secondary, xDomain, leftDomain, rightDomain, hasSecondary } = this.data;
    const tickCount = clamp(Math.floor(height / 68), 2, 7);
    const leftTicks = ticksFor(leftDomain, tickCount);
    const rightTicks = hasSecondary ? ticksFor(rightDomain, tickCount) : [];
    const tickWidth = ticks => Math.max(22, ...ticks.map(value => context.measureText(tickLabel(value)).width));
    const left = tickWidth(leftTicks) + (yLabel ? 36 : 17);
    const right = width - (hasSecondary ? tickWidth(rightTicks) + (secondaryLabel ? 36 : 17) : 22);
    const top = legend(context, lines, 16, 18, width - 16) + 4;
    const bottom = height - (xLabel ? 49 : 27);
    if (right - left < 35 || bottom - top < 30) return;

    const xScale = value => left + (value - xDomain[0]) * (right - left) / (xDomain[1] - xDomain[0]);
    const xInverse = value => xDomain[0] + (value - left) * (xDomain[1] - xDomain[0]) / (right - left);
    const vertical = ([min, max]) => value => bottom - (value - min) * (bottom - top) / (max - min);
    const leftScale = vertical(leftDomain);
    const rightScale = vertical(rightDomain);
    this.plot = { left, right, top, bottom, xInverse };

    context.textAlign = 'center';
    context.textBaseline = 'top';
    const xTicks = ticksFor(xDomain, clamp(Math.floor((right - left) / 82), 2, 9));
    for (const value of xTicks) {
      const screenX = xScale(value);
      stroke(context, [screenX, top], [screenX, bottom], gridColor);
      context.fillStyle = mutedColor;
      context.fillText(tickLabel(value), screenX, bottom + 7);
    }
    context.textAlign = 'right';
    context.textBaseline = 'middle';
    for (const value of leftTicks) {
      const y = leftScale(value);
      stroke(context, [left, y], [right, y], gridColor);
      context.fillStyle = mutedColor;
      if (lines.length) context.fillText(tickLabel(value), left - 8, y);
    }
    context.textAlign = 'left';
    for (const value of rightTicks) {
      const y = rightScale(value);
      context.fillStyle = mutedColor;
      context.fillText(tickLabel(value), right + 8, y);
    }
    context.strokeStyle = '#5c728e';
    context.lineWidth = 1;
    context.strokeRect(left, top, right - left, bottom - top);
    context.fillStyle = textColor;
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    if (xLabel) context.fillText(xLabel, (left + right) / 2, height - 13, right - left);
    for (const [label, position, rotation] of [
      [yLabel, 13, -Math.PI / 2],
      [secondaryLabel, width - 13, Math.PI / 2],
    ]) {
      if (!label) continue;
      context.save();
      context.translate(position, (top + bottom) / 2);
      context.rotate(rotation);
      context.fillText(label, 0, 0, bottom - top);
      context.restore();
    }

    context.save();
    context.beginPath();
    context.rect(left - 1, top - 1, right - left + 2, bottom - top + 2);
    context.clip();
    const cursorX = x[this.index];
    if (Number.isFinite(cursorX)) {
      const screenX = xScale(cursorX);
      stroke(context, [screenX, top], [screenX, bottom], '#cadaec', 1.1);
    }
    for (const line of lines) {
      const yScale = secondary(line) ? rightScale : leftScale;
      context.strokeStyle = line.color;
      context.lineWidth = 2;
      context.setLineDash(line.dashed ? [6, 4] : []);
      let previous = null;
      for (let i = 0; i <= x.length; i += 1) {
        const value = line.values[i];
        if (!Number.isFinite(value) || !Number.isFinite(x[i])) {
          if (previous?.length === 1) {
            context.save();
            context.setLineDash([]);
            circle(context, previous[0], 2, line.color, line.color);
            context.restore();
          }
          else if (previous) context.stroke();
          previous = null;
          continue;
        }
        const point = [xScale(x[i]), yScale(value)];
        if (!previous) {
          context.beginPath();
          context.moveTo(...point);
          previous = [point];
        } else {
          context.lineTo(...point);
          if (previous.length === 1) previous.push(point);
        }
      }
      context.setLineDash([]);
      const selected = line.values[this.index];
      if (Number.isFinite(cursorX) && Number.isFinite(selected)) {
        circle(context, [xScale(cursorX), yScale(selected)], 3.5, background, line.color);
      }
    }
    context.restore();
  }
}
