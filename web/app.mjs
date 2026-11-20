import { MechanismViewport, LineChart } from './visuals.mjs';
import { download, zipCsv } from './downloads.mjs';

const $ = id => document.getElementById(id);
const all = (selector, root = document) => [...root.querySelectorAll(selector)];
const on = (id, event, fn) => $(id).addEventListener(event, fn);
const value = id => $(id).value;
const number = id => $(id).valueAsNumber;
const corners = { FL: 'Front left', FR: 'Front right', RL: 'Rear left', RR: 'Rear right' };
const axes = ['X', 'Y', 'Z'];
const colors = ['#60e0c6', '#f5ba76', '#95baff', '#e6a6f2', '#ff9c99', '#d2de83'];
const profiles = [['pulse', 'Pulse'], ['sine', 'Sine'], ['ramp', 'Ramp']];
const mounts = [['LCA', 'Lower control arm'], ['UCA', 'Upper control arm'], ['UPRIGHT', 'Upright']];

let model;
let result;
let tab = 'motion';
let sweepType = 'suspension';
const rackRanges = { suspension: [0, 0], front_axle: [-8, 8] };
let damperType = 'history';
let sweepData;
let damperSweep;
let ratioData;
let comparison = false;
let lastPick;
let sample = 0;
let playback;
let activePlot;
let busy = false;
let needsSave = false;
let pendingId = 0;
const pending = new Map();
let worker;
const selected = { point: [], rod: [] };
const signals = {};
const clearSignals = () => ['point', 'rod', 'damper'].forEach(kind => { signals[kind] = Object.create(null); });
clearSignals();
const signalMetric = { point: 'position', rod: 'azimuth_deg' };

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, val] of Object.entries(attrs)) {
    if (val !== undefined && val !== null) node.setAttribute(key, val);
  }
  node.append(...children);
  return node;
}

function selectOptions(node, choices, preferred = node.value) {
  node.replaceChildren(...choices.map(item => {
    const [key, label] = typeof item === 'string' ? [item, item] : item;
    return new Option(label, key);
  }));
  if (choices.some(item => (typeof item === 'string' ? item : item[0]) === preferred)) node.value = preferred;
}

function labeled(label, input) {
  return el('label', {}, label, input);
}

function inputNumber(value, options = {}) {
  return el('input', { type: 'number', step: 'any', value, required: '', ...options });
}

function display(value, digits = 4) {
  if (typeof value === 'string') return value;
  if (!Number.isFinite(value)) return '—';
  if (value !== 0 && (Math.abs(value) < 10 ** -digits || Math.abs(value) >= 1e6)) return value.toExponential(3);
  return Number(value.toFixed(digits)).toString();
}

const viewport = new MechanismViewport($('viewport'), pick);

function rpc(operation, payload = {}) {
  if (!worker) {
    worker = new Worker(new URL('./worker.js', import.meta.url));
    worker.onmessage = ({ data: { id, result, error } }) => {
      const job = pending.get(id);
      if (!job) return;
      pending.delete(id);
      if (error) job.reject(new Error(error));
      else job.resolve(result);
    };
    worker.onerror = event => {
      for (const { reject } of pending.values()) reject(new Error(event.message || 'The solver worker stopped.'));
      pending.clear();
      worker.terminate();
      worker = undefined;
    };
  }
  const id = ++pendingId;
  return new Promise((resolve, reject) => {
    pending.set(id, { resolve, reject });
    worker.postMessage({ id, operation, payload });
  });
}

function status(message, error = false) {
  $('status').textContent = message;
  $('status').toggleAttribute('data-error', error);
}

function refreshButtons() {
  all('[data-engine]').forEach(button => { button.disabled = busy || !model; });
  $('motion-fields').disabled = busy || !model;
  $('calculate-ratio').disabled = busy || !result || model.point_names.length < 2;
  $('calculate-sweep').disabled = busy || !(sweepType === 'front_axle' ? model?.has_front_axle : model?.sweep_corners.length);
  $('calculate-damper').disabled = busy || !model?.damper_axles.length || !model?.deformable_members.length;
  all('[data-export]').forEach(button => {
    const kind = button.dataset.export;
    button.disabled = busy || !result || (kind === 'ratio' && !ratioData) ||
      (kind === 'damper' && !model?.deformable_members.length);
  });
}

async function run(message, task, onError = error => status(error.message, true)) {
  if (busy) return;
  busy = true;
  $('operation-progress').hidden = false;
  status(message);
  refreshButtons();
  try {
    await task();
  } catch (error) {
    onError(error);
  } finally {
    busy = false;
    $('operation-progress').hidden = true;
    refreshButtons();
    void refreshView();
  }
}

function projectTitle() {
  $('project-name').textContent = (model?.name || 'Four-wheel suspension') + (needsSave ? ' *' : '');
}

function markDirty() {
  needsSave = true;
  projectTitle();
}

function clearResults() {
  pause();
  result = sweepData = damperSweep = ratioData = undefined;
  clearSignals();
  sample = 0;
}

function visiblePoints() {
  return model.point_names.filter(name => !model.internal_points.includes(name));
}

function showModel(next, resetSelection = false) {
  model = next;
  clearResults();
  if (resetSelection) {
    selected.point = [];
    selected.rod = [];
    lastPick = undefined;
  }
  const pointNames = visiblePoints();
  const rodNames = next.links.map(link => link.name);
  selected.point = selected.point.filter(name => pointNames.includes(name));
  selected.rod = selected.rod.filter(name => rodNames.includes(name));
  viewport.setModel({
    points: framePoints(),
    fixed: next.fixed,
    links: next.links.map(link => ({ ...link, deformable: next.deformable_members.some(member => member.name === link.name) })),
    wheels: next.wheels
  });
  projectTitle();
  const [input, output] = ['FL_WC', 'FL_PUSH_IN'].map((name, i) => pointNames.includes(name) ? name : pointNames[i]);
  selectOptions($('ratio-in'), pointNames, resetSelection ? input : value('ratio-in') || input);
  selectOptions($('ratio-out'), pointNames, resetSelection ? output : value('ratio-out') || output);
  selectOptions($('ratio-drive'), [['', 'Along time history'], ...next.drives.filter(drive => !drive.locked)
    .map(drive => [drive.name, drive.name + ' (others fixed)'])]);
  selectOptions($('sweep-corner'), next.sweep_corners.map(corner => [corner, corners[corner]]));
  selectOptions($('damper-axle'), next.damper_axles.map(axle => [axle, axle === 'F' ? 'Front' : 'Rear']));
  const members = next.deformable_members.map(member => member.name);
  selectOptions($('damper-a'), members);
  selectOptions($('damper-b'), [['', 'None'], ...members]);
  setSweepType(sweepType, false);
  setDamperType(damperType, false);
  setSample(0);
}

function renderMotionForm() {
  $('duration').value = model.motion.duration;
  $('samples').value = model.motion.samples;
  $('motion-title').textContent = model.mechanism ? 'Motion inputs' : 'Wheel travel & steering';
  const drives = model.drives.filter(drive => !drive.locked);
  $('drive-settings').replaceChildren(...drives.map((drive, index) => {
    const name = drive.name;
    const title = name === 'FRONT_RACK' ? 'Front rack' :
      name.endsWith('_WC_Z') ? corners[name.slice(0, 2)] + ' wheel' : name;
    const profile = el('select', { id: 'profile-' + index });
    selectOptions(profile, profiles, model.motion.profiles[name]);
    return el('div', { class: 'drive-row' }, el('strong', {}, title),
      el('div', { class: 'field-grid' },
        labeled('Travel (mm)', inputNumber(model.motion.amplitudes_mm[name], { id: 'amplitude-' + index })),
        labeled('Profile', profile)));
  }));
  const preset = model.reference_settings?.analysis ?? model.reference_settings;
  $('reference-presets').hidden = !Number.isFinite(preset?.bump_mm) || !Number.isFinite(preset?.droop_mm);
  if (!$('reference-presets').hidden) {
    $('bump-preset').textContent = '+' + Math.abs(preset.bump_mm) + ' mm bump';
    $('droop-preset').textContent = '−' + Math.abs(preset.droop_mm) + ' mm droop';
  }
}

function motionSettings() {
  const names = model.drives.filter(drive => !drive.locked).map(drive => drive.name);
  return {
    duration: number('duration'), samples: number('samples'),
    amplitudes_mm: Object.fromEntries(names.map((name, i) => [name, number('amplitude-' + i)])),
    profiles: Object.fromEntries(names.map((name, i) => [name, value('profile-' + i)]))
  };
}

function editMotion() {
  renderMotionForm();
  $('motion-error').textContent = '';
  $('motion-dialog').showModal();
}

function applyMotion() {
  const motion = motionSettings();
  const current = model.motion;
  if (motion.duration === current.duration && motion.samples === current.samples &&
    Object.keys(motion.amplitudes_mm).every(name => motion.amplitudes_mm[name] === current.amplitudes_mm[name] &&
      motion.profiles[name] === current.profiles[name])) {
    $('motion-dialog').close();
    return;
  }
  return applyEditor('motion', { motion }, 'Motion settings applied. Solve kinematics to update the time history.');
}

function applyPreset(sign) {
  const preset = model.reference_settings.analysis ?? model.reference_settings;
  const amount = sign * Math.abs(sign > 0 ? preset.bump_mm : preset.droop_mm);
  model.drives.filter(drive => !drive.locked).forEach((drive, i) => {
    if (drive.name.endsWith('_WC_Z')) $('amplitude-' + i).value = amount;
  });
}

function mayReplaceProject() {
  return !needsSave || window.confirm('Replace this project? Download it first if you want to keep your changes.');
}

function loadProject(payload) {
  return run(model ? 'Loading geometry…' : 'Loading the Python solver and NumPy…', async () => {
    const next = await rpc('load', payload);
    needsSave = false;
    showModel(next, true);
    if (payload.example === 'damper') {
      setTab('dampers');
      setDamperType('scan');
    } else setTab('motion');
    status(visiblePoints().length + ' hardpoints · ' + next.links.length + ' links · Ready to solve');
  }, error => {
    status(error.message, true);
    $('retry').hidden = Boolean(model);
  });
}

async function importFiles(event, mechanismOnly = false) {
  const files = [...event.target.files];
  event.target.value = '';
  if (!files.length || (!mechanismOnly && !mayReplaceProject())) return;
  const csv = files.find(file => /\.csv$/i.test(file.name));
  const json = files.find(file => /\.json$/i.test(file.name));
  if (files.length > 2 || files.length === 2 && (!csv || !json)) {
    status('Select one project, or one hardpoint CSV with one mechanism JSON.', true);
    return;
  }
  try {
    if (mechanismOnly) {
      const mechanism = JSON.parse(await files[0].text());
      await run('Applying mechanism…', async () => {
        showModel(await rpc('update', { mechanism }), true);
        markDirty();
        status('Custom mechanism loaded.');
      });
    } else {
      const file = csv || json || files[0];
      await loadProject({ filename: file.name, text: await file.text(), mechanism: csv && json ? await json.text() : undefined });
    }
  } catch (error) {
    status(error.message, true);
  }
}

function saveProject(kind) {
  return run('Preparing project download…', async () => {
    const { name, text } = await rpc('project_' + kind);
    download(name, text, kind === 'json' ? 'application/json' : 'text/csv;charset=utf-8');
    needsSave = false;
    projectTitle();
    status('Downloaded ' + name + '.');
  });
}

function framePoints(index = sample) {
  const position = result?.position[index] ?? model.reference;
  return Object.fromEntries(model.point_names.flatMap((name, i) =>
    model.internal_points.includes(name) ? [] : [[name, position[i]]]));
}

function setSample(index, scroll = false) {
  sample = Math.min(Math.max(Number(index), 0), (result?.time.length ?? 1) - 1);
  $('sample').max = (result?.time.length ?? 1) - 1;
  $('sample').value = sample;
  $('clock').textContent = result ? result.time[sample].toFixed(3) + ' s' : 'Static pose';
  ['sample', 'play', 'rewind'].forEach(id => { $(id).disabled = !result; });
  if (model && tab === 'motion') {
    const wheels = model.wheels.map(wheel => ({
      ...wheel, normal: result?.wheel_normals?.[wheel.corner]?.[sample] ?? wheel.normal
    }));
    viewport.setFrame(framePoints(), wheels);
    syncSelection();
  }
  if (activePlot?.timeline) activePlot.seek(sample, scroll);
}

function pause() {
  if (playback) cancelAnimationFrame(playback);
  playback = undefined;
  $('play').textContent = 'Play';
}

function play() {
  if (!result || playback) return pause();
  const times = result.time;
  if (sample === times.length - 1) setSample(0, true);
  const startingTime = times[sample];
  const start = performance.now();
  $('play').textContent = 'Pause';
  const tick = now => {
    let time = startingTime + (now - start) * Number(value('play-speed')) / 1000;
    const end = times.at(-1);
    if (time >= end && !$('loop').checked) {
      setSample(times.length - 1, true);
      pause();
      return;
    }
    if (time >= end) time %= end;
    let low = 0;
    let high = times.length - 1;
    while (low < high) {
      const mid = Math.ceil((low + high) / 2);
      if (times[mid] <= time) low = mid;
      else high = mid - 1;
    }
    if (low !== sample) setSample(low, true);
    playback = requestAnimationFrame(tick);
  };
  playback = requestAnimationFrame(tick);
}

function pick(kind, name) {
  const list = selected[kind];
  if (list.includes(name)) selected[kind] = list.filter(item => item !== name);
  else selected[kind] = comparison ? [...list, name].slice(-2) : [name];
  lastPick = kind;
  syncSelection();
}

function setComparison(enabled) {
  comparison = enabled;
  $('compare-3d').checked = $('compare-signals').checked = enabled;
  if (!enabled) {
    selected.point = selected.point.slice(0, 1);
    selected.rod = selected.rod.slice(0, 1);
  }
  syncSelection();
  if (tab === 'points' || tab === 'rods') void refreshView();
}

function syncSelection() {
  if (!model) return;
  viewport.setSelection(selected.point, selected.rod);
  const points = framePoints();
  const kinds = lastPick === 'rod' ? ['rod', 'point'] : ['point', 'rod'];
  const kind = kinds.find(key => selected[key].length);
  $('open-selection').hidden = !kind;
  $('geometry-detail').textContent = kind ? selected[kind].map((name, i) => {
    const prefix = comparison ? (i ? 'B · ' : 'A · ') : '';
    if (kind === 'point') {
      const baseline = model.reference[model.point_names.indexOf(name)];
      const delta = result ? ' · Δ(' + points[name].map((x, j) => display((x - baseline[j]) * 1000, 2)).join(', ') + ') mm' : '';
      return prefix + name + ' · XYZ (' + points[name].map(x => display(x * 1000, 2)).join(', ') + ') mm' + delta;
    }
    const rod = model.links.find(link => link.name === name);
    return prefix + name + ' · ' + display(Math.hypot(...points[rod.a].map((x, j) => x - points[rod.b][j])) * 1000, 3) + ' mm';
  }).join('  |  ') : 'Select a point or rod to inspect its coordinates or length.';
  if (kind) $('open-selection').dataset.kind = kind;
}

function solve() {
  return run('Solving position, velocity, and acceleration…', async () => {
    pause();
    const next = await rpc('solve');
    result = next;
    clearSignals();
    ratioData = undefined;
    const bounds = [[Infinity, Infinity, Infinity], [-Infinity, -Infinity, -Infinity]];
    const wheelRadii = new Map(model.wheels.map(wheel =>
      [model.point_names.indexOf(wheel.center), Math.hypot(wheel.width / 2, wheel.radius)]));
    for (const frame of result.position) frame.forEach((point, i) => {
      const radius = wheelRadii.get(i) ?? 0;
      point.forEach((coordinate, axis) => {
        bounds[0][axis] = Math.min(bounds[0][axis], coordinate - radius);
        bounds[1][axis] = Math.max(bounds[1][axis], coordinate + radius);
      });
    });
    viewport.setBounds(bounds);
    setSample(0);
    status('Solved ' + result.time.length + ' samples over ' + result.time.at(-1) + ' s.');
  });
}

const pointMetrics = {
  position: ['Position', 'mm', 1000, axes],
  displacement: ['Displacement', 'mm', 1000, [...axes, 'Magnitude']],
  velocity: ['Velocity', 'mm/s', 1000, [...axes, 'Magnitude']],
  acceleration: ['Acceleration', 'mm/s²', 1000, [...axes, 'Magnitude']]
};
const rodMetrics = {
  azimuth_deg: ['Azimuth', '°', 1], elevation_deg: ['Elevation', '°', 1],
  angle_deg: ['Angle from static axis', '°', 1],
  angular_velocity: ['Angular velocity', 'rad/s', 1, [...axes, 'Magnitude']],
  angular_acceleration: ['Angular acceleration', 'rad/s²', 1, [...axes, 'Magnitude']],
  length: ['Length', 'mm', 1000], compression: ['Axial compression', 'mm', 1000]
};
const damperMetrics = {
  compression: ['Compression', 'mm', 1000],
  velocity: ['Compression speed', 'mm/s', 1000],
  acceleration: ['Compression acceleration', 'mm/s²', 1000]
};
const wheelMetrics = {
  camber_deg: ['Camber', '°'], camber_gain: ['Camber gain', '°/mm'],
  toe_deg: ['Toe', '°'], bump_steer_deg: ['Bump steer', '°'],
  bump_steer_rate: ['Toe change rate', '°/mm'], track_mm: ['Track width', 'mm'],
  wheelbase_mm: ['Wheelbase', 'mm']
};
const axleMetrics = {
  steer_deg: ['Steering angle', '°'], yaw_deg: ['Absolute wheel heading', '°'],
  difference_deg: ['Left − right steer angle', '°'], ackermann_pct: ['Ackermann', '%']
};
const optionsFor = metrics => Object.entries(metrics).map(([key, [label]]) => [key, label]);
const positive = value => value === null ? null : Math.abs(value);
const scaled = (value, scale) => value === null || value === undefined ? null : value * scale;
let renderId = 0;

function clearPlot() {
  activePlot?.chart.destroy();
  activePlot = undefined;
}

function column(name, values, color = colors[0], dashed = false, axis = undefined) {
  return { name, values, color, dashed, axis };
}

function renderPlot(id, spec) {
  const { x = [], xLabel, xRange, label, unit, series = [], columns = series, note, timeline = false,
    secondaryLabel, details } = spec;
  const yLabel = unit ? label + ' (' + unit + ')' : label;
  const chartElement = el('canvas', { class: 'plot-canvas', tabindex: x.length ? 0 : -1,
    'aria-label': yLabel + ' versus ' + xLabel });
  const table = el('table', { 'aria-label': label + ' data' },
    el('thead', {}, el('tr', {}, ...[xLabel, ...columns.map(item => item.name)].map(header => el('th', { scope: 'col' }, header)))));
  const body = el('tbody');
  const rows = x.map((item, index) => el('tr', { 'data-sample': index },
    ...[item, ...columns.map(next => next.values[index])].map(cell => el('td', {}, display(cell)))));
  body.append(...rows);
  table.append(body);
  const tableArea = el('div', { class: x.length ? 'table-scroll' : 'table-scroll empty-table',
    tabindex: x.length ? 0 : null, 'aria-label': label + ' samples' }, table);
  const area = el('div', { class: 'plot-grid' },
    el('div', { class: 'visual' }, el('div', { class: 'visual-toolbar' }, el('strong', {}, yLabel)), chartElement), tableArea);
  const notes = [note, details].filter(Boolean).map(text => el('p', { class: 'help plot-note' }, text));
  $(id).replaceChildren(...notes, area);
  let cursor = -1;
  const seek = (index, scroll = false) => {
    rows[cursor]?.removeAttribute('data-current');
    cursor = index;
    rows[index]?.setAttribute('data-current', '');
    chart.setCursor(index);
    if (scroll && rows[index]) {
      const row = rows[index].getBoundingClientRect();
      const area = tableArea.getBoundingClientRect();
      if (row.bottom > area.bottom) tableArea.scrollTop += row.bottom - area.bottom;
      else if (row.top < area.top + table.tHead.offsetHeight) tableArea.scrollTop += row.top - area.top - table.tHead.offsetHeight;
    }
  };
  const onSeek = index => {
    if (timeline) {
      pause();
      setSample(index, true);
    } else seek(index, true);
  };
  const chart = new LineChart(chartElement, onSeek);
  chart.setData({ x, xLabel, xRange, yLabel, series, secondaryLabel });
  activePlot = { chart, timeline, seek };
  if (!x.length) return;
  seek(timeline ? sample : 0, true);
  tableArea.addEventListener('click', event => {
    const row = event.target.closest('[data-sample]');
    if (row) onSeek(Number(row.dataset.sample));
  });
  tableArea.addEventListener('keydown', event => {
    const movement = { ArrowUp: -1, ArrowDown: 1 }[event.key];
    if (!movement) return;
    event.preventDefault();
    onSeek(Math.max(0, Math.min(rows.length - 1, cursor + movement)));
  });
}

async function getSignals(kind, names) {
  const cache = signals[kind];
  const missing = names.filter(name => !Object.hasOwn(cache, name));
  if (missing.length) Object.assign(cache, await rpc('signals', { kind, names: missing }));
  return names.map(name => cache[name]);
}

function chooseSignalOptions(kind) {
  const names = model ? (kind === 'point' ? visiblePoints() : model.links.map(link => link.name)) : [];
  const picks = selected[kind];
  const noun = kind === 'point' ? 'Point' : 'Rod';
  $('signal-name-a').textContent = noun + ' A';
  $('signal-name-b').textContent = noun + ' B';
  $('signal-b-field').hidden = !comparison;
  selectOptions($('signal-a'), names, picks[0] ?? (names.includes('FL_WC') ? 'FL_WC' : names[0]));
  selectOptions($('signal-b'), [['', 'None'], ...names.filter(name => name !== value('signal-a'))], picks[1] ?? '');
  const metrics = kind === 'point' ? pointMetrics : rodMetrics;
  selectOptions($('signal-type'), optionsFor(metrics), signalMetric[kind]);
  return [value('signal-a'), comparison ? value('signal-b') : ''].filter(Boolean);
}

async function showSignals(token) {
  const kind = tab === 'points' ? 'point' : 'rod';
  const names = chooseSignalOptions(kind);
  const metric = value('signal-type');
  const [label, unit, scale, components] = (kind === 'point' ? pointMetrics : rodMetrics)[metric];
  if (!result || !names.length) {
    const note = !model ? 'Load a project to inspect its geometry.' :
      !names.length ? 'This mechanism has no ' + kind + ' signals.' : 'Solve kinematics to view the time history.';
    return renderPlot('signals-output', {
      xLabel: 'Time (s)', xRange: [0, model?.motion.duration], label, unit,
      columns: (components ?? [label]).map(name => column(name, [])), note
    });
  }
  const data = await getSignals(kind, names);
  if (token !== renderId) return;
  const plotted = data.flatMap((record, i) => {
    const prefix = names.length > 1 ? (i ? 'B · ' : 'A · ') : '';
    const values = record[metric];
    if (!components) return [column(prefix + names[i], values.map(next => scaled(next, scale)), colors[0], Boolean(i))];
    return components.map((component, j) => column(prefix + component, values.map(vector => {
      if (!vector || vector.some(item => item === null)) return null;
      return (j === 3 ? Math.hypot(...vector) : vector[j]) * scale;
    }), colors[j], Boolean(i)));
  });
  renderPlot('signals-output', { x: result.time, xLabel: 'Time (s)', label, unit, series: plotted, timeline: true,
    note: names.length > 1 ? 'A · ' + names[0] + '   |   B · ' + names[1] + ' (dashed)' : names[0] });
}

function ratioSettings() {
  return {
    input_point: value('ratio-in'), output_point: value('ratio-out'),
    input_direction: value('ratio-in-dir').replaceAll('−', '-'),
    output_direction: value('ratio-out-dir').replaceAll('−', '-'),
    drive: value('ratio-drive') || null
  };
}

function showRatio() {
  if (!ratioData) return renderPlot('ratio-output', {
    xLabel: 'Time (s)', xRange: [0, model?.motion.duration], label: 'Motion ratio',
    columns: [column('Signed motion ratio', []), column('Mechanical advantage', [])],
    note: result ? 'Choose two points and a derivative basis, then calculate the ratio.' :
      'Solve kinematics before calculating a motion ratio.'
  });
  const data = ratioData;
  const series = [column('Signed motion ratio', data.motion_ratio), column('Mechanical advantage', data.mechanical_advantage, colors[1])];
  const rateUnit = data.drive ? ' per mm' : ' (mm/s)';
  const mapScaled = values => values.map(next => scaled(next, 1000));
  const columns = [...series, column('Input travel (mm)', mapScaled(data.input_displacement)),
    column('Output travel (mm)', mapScaled(data.output_displacement)),
    column('Input rate' + rateUnit, data.drive ? data.input_rate : mapScaled(data.input_rate)),
    column('Output rate' + rateUnit, data.drive ? data.output_rate : mapScaled(data.output_rate)),
    column('Status', data.status)];
  renderPlot('ratio-output', {
    x: data.time_s, xLabel: 'Time (s)', label: 'Motion ratio', series, columns, timeline: true,
    note: data.drive ? 'Local derivative with respect to ' + data.drive + ' · Other drives fixed' :
      'Derivative along time history · Undefined projections appear as gaps.'
  });
}

function setSweepType(type, invalidate = true) {
  if (invalidate && type !== sweepType) {
    rackRanges[sweepType] = [value('rack-min'), value('rack-max')];
    [$('rack-min').value, $('rack-max').value] = rackRanges[type];
    sweepData = undefined;
  }
  sweepType = type;
  const axle = type === 'front_axle';
  all('[data-sweep]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.sweep === type)));
  for (const [id, hidden] of [['wheel-sweep-fields', axle], ['axle-sweep-fields', !axle],
    ['rack-sweep-fields', !axle && !model?.steering_corners.includes(value('sweep-corner'))]]) {
    $(id).hidden = hidden;
    all('input, select', $(id)).forEach(field => { field.disabled = hidden; });
  }
  selectOptions($('sweep-metric'), optionsFor(axle ? axleMetrics : wheelMetrics));
  refreshButtons();
}

function calculateSweep() {
  const type = sweepType;
  const fields = () => all('input, select', $('sweep-form')).map(input => input.value).join('|');
  const snapshot = fields();
  return run('Calculating suspension sweep…', async () => {
    const rack = [number('rack-min'), number('rack-max')];
    const settings = type === 'front_axle' ? {
      heave_mm: number('axle-heave'), roll_deg: number('axle-roll'), rack_limits: rack,
      wheelbase_mm: value('axle-wheelbase') === '' ? null : number('axle-wheelbase')
    } : {
      corner: value('sweep-corner'), travel_limits: [number('travel-min'), number('travel-max')],
      rack_limits: $('rack-sweep-fields').hidden ? [0, 0] : rack
    };
    const data = await rpc(type, settings);
    if (type === sweepType && snapshot === fields()) sweepData = data;
    status('Suspension sweep calculated.');
  });
}

function showSweep() {
  const axle = sweepType === 'front_axle';
  const metrics = axle ? axleMetrics : wheelMetrics;
  const metric = value('sweep-metric');
  const [label, unit] = metrics[metric] ?? Object.values(metrics)[0];
  const xLabel = axle ? 'Rack displacement (mm)' : 'Wheel travel (mm)';
  if (!sweepData) {
    const supported = axle ? model?.has_front_axle : model?.sweep_corners.length;
    return renderPlot('sweep-output', {
      xLabel, xRange: axle ? [number('rack-min'), number('rack-max')] : [number('travel-min'), number('travel-max')],
      label, unit, columns: [column(label + ' (' + unit + ')', [])],
      note: supported ? 'Set the range and calculate the sweep.' :
        (axle ? 'This sweep requires both front wheel centers, rack points, and tie rods.' :
          'This sweep requires a standard suspension with a wheel center.')
    });
  }
  const data = sweepData;
  const series = axle ? (['steer_deg', 'yaw_deg'].includes(metric) ?
    data[metric].map((values, i) => column(i ? 'Right wheel' : 'Left wheel', values, colors[i], Boolean(i))) :
    [column(label, data[metric])]) :
    data.rack_mm.map((rack, i) => column('Rack ' + display(rack) + ' mm', data[metric][i], colors[i]));
  const columns = axle ? [
    ...data.steer_deg.map((items, i) => column(i ? 'Right steer (°)' : 'Left steer (°)', items)),
    column('Difference (°)', data.difference_deg), column('Ackermann (%)', data.ackermann_pct),
    ...data.yaw_deg.map((items, i) => column(i ? 'Right heading (°)' : 'Left heading (°)', items))
  ] : Object.entries(wheelMetrics).flatMap(([key, [title, units]]) =>
    data.rack_mm.map((rack, i) => column(title + ' (' + units + ') · ' + display(rack) + ' mm rack', data[key][i])));
  renderPlot('sweep-output', {
    x: axle ? data.rack_mm : data.travel_mm, xLabel,
    label, unit, series, columns,
    note: axle ? 'Fixed wheel travel · Left ' + display(data.travel_mm[0]) + ' mm · Right ' + display(data.travel_mm[1]) + ' mm' :
      'Positive travel is bump. Each curve holds the rack at a fixed position.'
  });
}

function setDamperType(type, redraw = true) {
  damperType = type;
  const scan = type === 'scan';
  all('[data-damper]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.damper === type)));
  $('damper-form').hidden = $('damper-abs-field').hidden = !scan;
  const roll = value('damper-mode') === 'ROLL';
  $('damper-fixed-label').textContent = roll ? 'Fixed heave (mm)' : 'Fixed roll (mm)';
  if (roll) $('damper-min').min = 0;
  else $('damper-min').removeAttribute('min');
  selectOptions($('damper-metric'), scan ? [['compression', 'Compression'],
    ...(roll ? [['difference', 'Left/right difference']] : [])] : optionsFor(damperMetrics));
  if (redraw && tab === 'dampers') void refreshView();
}

function calculateDamper() {
  const fields = () => all('input, select', $('damper-form')).map(input => input.value).join('|');
  const snapshot = fields();
  return run('Calculating damper sweep…', async () => {
    const data = await rpc('damper_sweep', {
      axle: value('damper-axle'), mode: value('damper-mode'),
      limits_mm: [number('damper-min'), number('damper-max')], fixed_mm: number('damper-fixed')
    });
    if (snapshot === fields()) damperSweep = data;
    status('Damper sweep calculated.');
  });
}

async function showDamper(token) {
  const names = [...new Set([value('damper-a'), value('damper-b')].filter(Boolean))];
  const metric = value('damper-metric') || 'compression';
  const history = damperType === 'history';
  const difference = metric === 'difference';
  const [label, unit] = difference ? ['Left/right difference', '%'] : damperMetrics[metric];
  const xLabel = history ? 'Time (s)' : value('damper-mode') === 'ROLL' ? 'Roll magnitude (mm)' : 'Heave sum (mm)';
  if (!names.length || !(history ? result : damperSweep)) {
    const note = !names.length ? 'Add deformable members or load the damper example to analyze damper travel.' :
      history ? 'Solve kinematics to view damper travel, speed, and acceleration.' :
        'Choose an axle, damper, and range, then calculate the sweep.';
    return renderPlot('damper-output', {
      xLabel, xRange: history ? [0, model?.motion.duration] : [number('damper-min'), number('damper-max')],
      label, unit, columns: [column(label + ' (' + unit + ')', [])], note
    });
  }
  if (history) {
    const data = await getSignals('damper', names);
    if (token !== renderId) return;
    const toSeries = (key, item, i, name) => column(name, item[key].map(next => scaled(next, 1000)), colors[i], Boolean(i));
    const series = data.map((item, i) => toSeries(metric, item, i, names[i]));
    const columns = data.flatMap((item, i) => Object.entries(damperMetrics).map(([key, [label, unit]]) =>
      toSeries(key, item, i, names[i] + ' · ' + label + ' (' + unit + ')')));
    renderPlot('damper-output', { x: result.time, xLabel, label, unit, series, columns, timeline: true,
      note: 'Positive compression means the member is shorter than in the static pose.' });
    return;
  }
  const data = damperSweep;
  const roll = Boolean(data.roll_difference_pct);
  const abs = $('damper-abs').checked;
  const compress = names.flatMap((name, i) => data.compression_mm[data.member_names.indexOf(name)].map((values, j) =>
    column(name + (roll ? (j ? ' · Right roll' : ' · Left roll') : ''),
      abs ? values.map(positive) : values, colors[i + j * 2], Boolean(i))));
  const differences = roll ? names.map((name, i) => column(name + ' · Difference (%)',
    data.roll_difference_pct[data.member_names.indexOf(name)], colors[i], Boolean(i))) : [];
  const wheelColumns = data.wheel_travel_mm.flatMap((caseValues, j) => caseValues.map((values, i) =>
    column((roll ? (j ? 'Right roll · ' : 'Left roll · ') : '') + (i ? 'Right wheel (mm)' : 'Left wheel (mm)'), values)));
  renderPlot('damper-output', { x: data.input_mm, xLabel, label, unit,
    series: difference ? differences : compress, columns: [...compress, ...differences, ...wheelColumns],
    note: roll ? 'Opposite roll cases share the same wheel-travel difference magnitude.' :
      'Heave is the sum of left and right wheel travel.' });
}

async function refreshView() {
  const token = ++renderId;
  clearPlot();
  $('playback').hidden = tab === 'sweeps' || (tab === 'dampers' && damperType === 'scan');
  try {
    if (tab === 'motion') setSample(sample);
    else if (tab === 'points' || tab === 'rods') await showSignals(token);
    else if (tab === 'ratio') showRatio();
    else if (tab === 'sweeps') showSweep();
    else await showDamper(token);
  } catch (error) {
    if (token === renderId) status(error.message, true);
  }
}

function setTab(next) {
  tab = next;
  all('[data-tab]').forEach(button => {
    const active = button.dataset.tab === next;
    button.setAttribute('aria-selected', String(active));
    button.tabIndex = active ? 0 : -1;
  });
  all('[role=tabpanel]').forEach(section => { section.hidden = true; });
  const view = ['points', 'rods'].includes(next) ? 'signals' : next;
  $('view-' + view).hidden = false;
  $('view-' + view).setAttribute('aria-labelledby', 'tab-' + next);
  if (tab === 'sweeps' || tab === 'dampers' && damperType === 'scan') pause();
  void refreshView();
}

function tableWithHeadings(labels, rows, name) {
  return el('div', { class: 'table-scroll', tabindex: '0' }, el('table', { 'aria-label': name },
    el('thead', {}, el('tr', {}, ...labels.map(label => el('th', { scope: 'col' }, label)))),
    el('tbody', {}, ...rows)));
}

function sectionWithTitle(title, ...children) {
  return el('section', { class: 'settings-group' }, el('h3', {}, title), ...children);
}

function editGeometry() {
  const rows = Object.entries(model.points).map(([name, coords]) => el('tr', { 'data-point': name },
    el('th', { scope: 'row' }, name),
    ...coords.map((coordinate, i) => el('td', {}, inputNumber(coordinate * 1000,
      { 'aria-label': name + ' ' + axes[i] + ' (mm)' })))));
  const groups = [];
  for (const [corner, tire] of Object.entries(model.tires)) {
    const fields = [
      ['Width (mm)', 'width_mm', { min: '0.0001' }],
      ['Radius (mm)', 'radius_mm', { min: '0.0001' }],
      ['Toe (°)', 'toe_deg', { required: null, placeholder: 'From geometry' }],
      ['Camber (°)', 'camber_deg', {}]
    ].map(([label, key, extra]) => labeled(label, inputNumber(tire[key], { 'data-tire': key, ...extra })));
    const mount = model.pushrod_mounts[corner];
    const pushrod = el('select', { 'data-mount': corner });
    if (mount) selectOptions(pushrod, mounts, mount);
    groups.push(sectionWithTitle(corners[corner], el('div', { class: 'tire-grid', 'data-corner': corner }, ...fields),
      ...(mount ? [labeled('Pushrod outboard mount', pushrod)] : [])));
  }
  $('geometry-fields').replaceChildren(
    sectionWithTitle('Hardpoints', tableWithHeadings(['Point', ...axes.map(axis => axis + ' (mm)')], rows, 'Hardpoint coordinates')),
    ...(groups.length ? [el('p', { class: 'help settings-group' },
      'Toe-in and top-out camber are positive. Leave toe blank to use the reference wheel heading.'), ...groups] : [])
  );
  $('geometry-error').textContent = '';
  $('geometry-dialog').showModal();
}

function applyGeometry() {
  const points = Object.fromEntries(all('[data-point]').map(row =>
    [row.dataset.point, all('input', row).map(input => input.valueAsNumber / 1000)]));
  const tires = Object.fromEntries(all('[data-corner]').map(group => [group.dataset.corner,
    Object.fromEntries(all('[data-tire]', group).map(input =>
      [input.dataset.tire, input.value === '' ? null : input.valueAsNumber]))]));
  const pushrod_mounts = Object.fromEntries(all('[data-mount]').map(input => [input.dataset.mount, input.value]));
  return applyEditor('geometry', { points, tires, pushrod_mounts }, 'Geometry applied.');
}

function memberCard(member) {
  const name = el('input', { 'data-member-name': '', value: member.name, required: '' });
  const remove = el('button', { type: 'button' }, 'Remove');
  const card = el('section', { class: 'member-card' }, el('div', { class: 'member-title' }, labeled('Member name', name), remove));
  const choices = [['CHASSIS', 'Chassis'], ...Object.keys(model.pushrod_mounts).map(corner => [corner, corner + ' rocker'])];
  for (const end of ['a', 'b']) {
    const mount = el('select');
    selectOptions(mount, choices, member[end].mount);
    const axisInputs = member[end].xyz.map((coordinate, i) => labeled(axes[i] + ' (mm)', inputNumber(coordinate * 1000)));
    card.append(el('div', { class: 'endpoint', 'data-end': end },
      labeled('End ' + end.toUpperCase() + ' mount', mount), ...axisInputs));
  }
  remove.addEventListener('click', () => card.remove());
  return card;
}

function editMembers() {
  $('member-fields').replaceChildren(...model.deformable_members.map(memberCard));
  $('members-error').textContent = '';
  $('members-dialog').showModal();
}

function addMember() {
  const origin = { xyz: [0, 0, 0], mount: 'CHASSIS' };
  const card = memberCard({ name: '', a: origin, b: origin });
  $('member-fields').append(card);
  card.querySelector('input').focus();
}

function applyMembers() {
  const deformable_members = all('.member-card').map(card => {
    const name = card.querySelector('[data-member-name]').value.trim();
    const ends = Object.fromEntries(all('[data-end]', card).map(group => [group.dataset.end, {
      mount: group.querySelector('select').value,
      xyz: all('input', group).map(input => input.valueAsNumber / 1000)
    }]));
    return { name, ...ends };
  });
  return applyEditor('members', { deformable_members }, 'Deformable members applied.');
}

function applyEditor(kind, changes, message) {
  const form = $(kind + '-form');
  const submit = form.querySelector('button.primary');
  $(kind + '-error').textContent = '';
  submit.disabled = true;
  return run('Validating the model…', async () => {
    try {
      showModel(await rpc('update', changes));
      markDirty();
      $(kind + '-dialog').close();
      status(message);
    } finally {
      submit.disabled = false;
    }
  }, error => {
    $(kind + '-error').textContent = error.message;
    status(error.message, true);
  });
}

function exportResults(kind) {
  return run('Preparing CSV files…', async () => {
    const ratio = ratioData ? ratioSettings() : undefined;
    const files = await rpc('export', { kind, ratio, include_ratio: Boolean(ratio) });
    if (kind === 'all') download('suspension_kinematics_results.zip', zipCsv(files), 'application/zip');
    else for (const { name, text } of files) download(name, text, 'text/csv;charset=utf-8');
    $('results-menu').open = false;
    status('Downloaded ' + files.length + (files.length === 1 ? ' CSV file.' : ' CSV files.'));
  });
}

function submit(id, action) {
  on(id, 'submit', event => {
    event.preventDefault();
    void action();
  });
}

on('solve', 'click', solve);
on('edit-motion', 'click', editMotion);
submit('motion-form', applyMotion);
on('bump-preset', 'click', () => applyPreset(1));
on('droop-preset', 'click', () => applyPreset(-1));
on('import-project', 'click', () => $('project-file').click());
on('import-mechanism', 'click', () => $('mechanism-file').click());
on('project-file', 'change', event => { void importFiles(event); });
on('mechanism-file', 'change', event => { void importFiles(event, true); });
on('download-json', 'click', () => { void saveProject('json'); });
on('download-project-csv', 'click', () => { void saveProject('csv'); });
on('load-example', 'click', () => { if (mayReplaceProject()) void loadProject({}); });
on('load-damper-example', 'click', () => { if (mayReplaceProject()) void loadProject({ example: 'damper' }); });
on('retry', 'click', () => window.location.reload());
all('.file-menu button').forEach(button => button.addEventListener('click', () => { button.closest('details').open = false; }));
all('[data-export]').forEach(button => button.addEventListener('click', () => { void exportResults(button.dataset.export); }));
document.addEventListener('click', event => {
  all('.file-menu[open]').forEach(menu => { if (!menu.contains(event.target)) menu.open = false; });
});
window.addEventListener('beforeunload', event => {
  if (needsSave) event.preventDefault();
});

on('edit-geometry', 'click', editGeometry);
on('edit-members', 'click', editMembers);
on('add-member', 'click', addMember);
all('[data-close]').forEach(button => button.addEventListener('click', () => $(button.dataset.close).close()));
submit('geometry-form', applyGeometry);
submit('members-form', applyMembers);

all('[data-tab]').forEach(button => button.addEventListener('click', () => setTab(button.dataset.tab)));
on('result-tabs', 'keydown', event => {
  const buttons = all('[data-tab]');
  const index = buttons.indexOf(event.target);
  const delta = { ArrowLeft: -1, ArrowRight: 1 }[event.key];
  if (index < 0 || !delta && event.key !== 'Home' && event.key !== 'End') return;
  event.preventDefault();
  const next = event.key === 'Home' ? 0 : event.key === 'End' ? buttons.length - 1 :
    (index + delta + buttons.length) % buttons.length;
  setTab(buttons[next].dataset.tab);
  buttons[next].focus();
});
on('camera', 'change', () => viewport.setView(value('camera')));
on('reset-camera', 'click', () => viewport.resetCamera());
on('show-reference', 'change', () => viewport.setReferenceVisible($('show-reference').checked));
on('show-labels', 'change', () => viewport.setLabelsVisible($('show-labels').checked));
on('compare-3d', 'change', () => setComparison($('compare-3d').checked));
on('compare-signals', 'change', () => setComparison($('compare-signals').checked));
on('open-selection', 'click', () => setTab($('open-selection').dataset.kind === 'point' ? 'points' : 'rods'));
on('play', 'click', play);
on('rewind', 'click', () => { pause(); setSample(0, true); });
on('sample', 'input', () => { pause(); setSample(value('sample'), true); });
on('play-speed', 'change', () => { if (playback) { pause(); play(); } });

['signal-a', 'signal-b'].forEach(id => on(id, 'change', () => {
  const kind = tab === 'points' ? 'point' : 'rod';
  const a = value('signal-a');
  const b = value('signal-b');
  selected[kind] = [a, b !== a && comparison ? b : ''].filter(Boolean);
  lastPick = kind;
  syncSelection();
  void refreshView();
}));
on('signal-type', 'change', () => {
  signalMetric[tab === 'points' ? 'point' : 'rod'] = value('signal-type');
  void refreshView();
});
submit('ratio-form', () => run('Calculating motion ratio…', async () => {
  const settings = ratioSettings();
  const data = await rpc('ratio', settings);
  if (JSON.stringify(settings) === JSON.stringify(ratioSettings())) ratioData = data;
  status('Motion ratio calculated.');
}));
on('ratio-form', 'input', () => { ratioData = undefined; refreshButtons(); void refreshView(); });
all('[data-sweep]').forEach(button => button.addEventListener('click', () => {
  setSweepType(button.dataset.sweep);
  void refreshView();
}));
submit('sweep-form', calculateSweep);
on('sweep-form', 'input', () => {
  sweepData = undefined;
  setSweepType(sweepType, false);
  void refreshView();
});
on('sweep-metric', 'change', () => { void refreshView(); });
all('[data-damper]').forEach(button => button.addEventListener('click', () => {
  if (button.dataset.damper === 'scan') pause();
  setDamperType(button.dataset.damper);
}));
submit('damper-form', calculateDamper);
on('damper-form', 'input', () => {
  damperSweep = undefined;
  setDamperType(damperType);
});
['damper-a', 'damper-b', 'damper-metric', 'damper-abs'].forEach(id => on(id, 'change', () => { void refreshView(); }));

void loadProject({});
