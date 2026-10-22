// Mission planner: step editor, save/load, kinematic preview, run/pause/abort with live progress.
import { $, api, store, on, h, fmt, selId, toast, mmss } from '../core.js';
import { MapView } from '../mapview.js';
import { setPlan } from './map.js';
import { paramForm } from '../form.js';

const DIRS = ['forward', 'back', 'left', 'right', 'up', 'down'];
const DEFS = {
  takeoff: [], land: [],
  move: [['dir', 'select', 'forward', DIRS], ['cm', 'number', 100, [20, 500]]],
  rotate: [['dir', 'select', 'cw', ['cw', 'ccw']], ['deg', 'number', 90, [1, 360]]],
  go: [['x', 'number', 100, [-500, 500]], ['y', 'number', 0, [-500, 500]], ['z', 'number', 0, [-500, 500]], ['speed', 'number', 50, [10, 100]]],
  speed: [['cms', 'number', 50, [10, 100]]],
  flip: [['dir', 'select', 'f', ['f', 'b', 'l', 'r']]],
  wait: [['s', 'number', 2, [0, 600]]],
  hover: [['s', 'number', 3, [0, 600]]],
  snapshot: [],
  record: [['on', 'bool', true]],
  wait_detection: [['plugin', 'plugin', 'red_triangle'], ['label', 'text', ''], ['timeout', 'number', 10, [1, 600]], ['on_timeout', 'select', 'continue', ['continue', 'abort', 'land']]],
  scan: [['deg', 'number', 360, [30, 720]], ['step', 'number', 45, [5, 180]], ['dwell', 'number', 1.0, [0.1, 10]], ['plugin', 'plugin', ''], ['label', 'text', '']],
  command: [['text', 'text', 'battery?']],
};
const HINT = {
  move: 'relative move in body frame', rotate: 'yaw in place', go: 'body-frame offset (cm) at speed (cm/s)',
  wait_detection: 'hold until a vision plugin sees the target', scan: 'yaw in steps looking for a target', record: 'start/stop video',
  command: 'raw SDK text', hover: 'rc 0 0 0 0 for N s', snapshot: 'save a JPEG',
};

let mission = { name: 'new_mission', description: '', steps: [{ type: 'takeoff' }, { type: 'move', dir: 'forward', cm: 100 }, { type: 'land' }] };
let plugins = [];
let current = { state: 'idle', step: -1 };
const mini = new MapView($('#msMap'), { mode: '2d' });

$('#msAddType').replaceChildren(...Object.keys(DEFS).map(k => h('option', { value: k }, k)));

function stepDefaults(type) {
  const s = { type };
  for (const [k, , def] of DEFS[type]) s[k] = def;
  return s;
}

function field(st, [k, type, def, opt]) {
  const val = st[k] ?? def;
  const set = (v) => { st[k] = v; changed(); };
  if (type === 'select' || type === 'plugin') {
    const options = type === 'plugin' ? ['', ...plugins] : opt;
    return h('select', { title: k, onchange: (e) => set(e.target.value) }, ...options.map(o => h('option', { value: o, selected: o === val }, o || '(any)')));
  }
  if (type === 'bool') return h('select', { title: k, onchange: (e) => set(e.target.value === 'on') }, h('option', { value: 'on', selected: !!val }, 'on'), h('option', { value: 'off', selected: !val }, 'off'));
  if (type === 'number') return h('label', { class: 'small muted' }, k + ' ', h('input', { type: 'number', value: val, min: opt?.[0], max: opt?.[1], step: k === 'dwell' ? 0.1 : 1, onchange: (e) => set(Number(e.target.value)) }));
  return h('input', { value: val, placeholder: k, onchange: (e) => set(e.target.value) });
}

function render() {
  $('#msName').value = mission.name || '';
  const list = $('#msSteps');
  list.replaceChildren(...mission.steps.map((st, i) => {
    const li = h('li', { class: i === current.step && ['running', 'paused'].includes(current.state) ? 'cur' : (current.state !== 'idle' && i < current.step ? 'done' : '') },
      h('span', { class: 'ty', title: HINT[st.type] || '' }, st.type),
      ...(DEFS[st.type] || []).map(f => field(st, f)),
      h('span', { class: 'sp' }),
      h('button', { class: 'btn xs ghost', title: 'up', onclick: () => move(i, -1) }, '↑'),
      h('button', { class: 'btn xs ghost', title: 'down', onclick: () => move(i, 1) }, '↓'),
      h('button', { class: 'btn xs ghost', title: 'duplicate', onclick: () => { mission.steps.splice(i + 1, 0, structuredClone(st)); changed(true); } }, '⧉'),
      h('button', { class: 'btn xs ghost', title: 'delete', onclick: () => { mission.steps.splice(i, 1); changed(true); } }, '×'));
    return li;
  }));
}
function move(i, d) {
  const j = i + d; if (j < 0 || j >= mission.steps.length) return;
  [mission.steps[i], mission.steps[j]] = [mission.steps[j], mission.steps[i]]; changed(true);
}

let tmr = null;
function changed(rerender = false) {
  if (rerender) render();
  clearTimeout(tmr); tmr = setTimeout(previewNow, 200);
}
async function previewNow() {
  try {
    const p = await api('/missions/preview', mission);
    mini.layers.plan = p.points; mini.layers.fence = store.settings.safety?.geofence_enabled ? { r: store.settings.safety.geofence_radius_m } : null;
    mini.fit(p.points);
    setPlan(p.points);
    $('#msStats').replaceChildren(
      h('span', {}, 'steps'), h('span', {}, mission.steps.length),
      h('span', {}, 'distance'), h('span', {}, `${fmt(p.distance_m, 1)} m`),
      h('span', {}, 'est. time'), h('span', {}, mmss(p.est_time_s)),
      h('span', {}, 'max range / alt'), h('span', { style: { color: p.fence_ok ? '' : 'var(--danger)' } }, `${fmt(p.max_range_m)} / ${fmt(p.max_alt_m)} m ${p.fence_ok ? '✓ fence' : '⚠ outside geofence'}`),
      ...(p.error ? [h('span', {}, 'error'), h('span', { style: { color: 'var(--danger)' } }, p.error)] : []));
  } catch (e) { console.warn(e); }
}

async function refreshList() {
  const list = await api('/missions');
  $('#msLoad').replaceChildren(h('option', { value: '' }, `Load… (${list.length})`), ...list.map(m => h('option', { value: m.file }, `${m.name} · ${m.steps} steps`)));
}

$('#msName').onchange = (e) => { mission.name = e.target.value; };
$('#msAdd').onclick = () => { mission.steps.push(stepDefaults($('#msAddType').value)); changed(true); };
$('#msNew').onclick = () => { mission = { name: 'new_mission', steps: [{ type: 'takeoff' }, { type: 'land' }] }; changed(true); };
$('#msLoad').onchange = async (e) => { if (!e.target.value) return; mission = await api('/missions/' + encodeURIComponent(e.target.value)); changed(true); e.target.value = ''; };
$('#msSave').onclick = async () => { try { const r = await api('/missions', mission); toast(`Saved ${r.file}`, 'success'); refreshList(); } catch (e) { toast(e.message, 'error'); } };
$('#msDel').onclick = async () => { const f = (mission.name || '') + '.json'; if (confirm(`Delete ${f}?`)) { await api('/missions/' + encodeURIComponent(f), undefined, 'DELETE'); refreshList(); } };
$('#msExport').onclick = () => {
  const a = h('a', { href: URL.createObjectURL(new Blob([JSON.stringify(mission, null, 2)], { type: 'application/json' })), download: `${mission.name || 'mission'}.json` });
  a.click();
};
$('#msImport').onclick = () => $('#msFile').click();
$('#msFile').onchange = async (e) => { const f = e.target.files[0]; if (!f) return; try { mission = JSON.parse(await f.text()); changed(true); } catch { toast('Invalid JSON', 'error'); } e.target.value = ''; };

const targets = () => ($('#fleetSync').checked ? Object.keys(store.drones) : [selId()]);
$('#msRun').onclick = async () => {
  if (current.state === 'paused') { for (const id of targets()) await api(`/drones/${id}/mission/resume`, {}).catch(e => toast(e.message, 'error')); return; }
  for (const id of targets()) {
    try { await api(`/drones/${id}/mission/start`, mission); toast(`Mission started on ${id}`, 'success'); }
    catch (e) { toast(`${id}: ${e.message}`, 'error'); }
  }
};
$('#msPause').onclick = () => targets().forEach(id => api(`/drones/${id}/mission/pause`, {}));
$('#msAbort').onclick = () => targets().forEach(id => api(`/drones/${id}/mission/abort`, {}));

function showStatus(st) {
  current = st || { state: 'idle', step: -1 };
  const col = { running: 'var(--info)', paused: 'var(--warn)', done: 'var(--ok)', failed: 'var(--danger)', aborted: 'var(--warn)' }[current.state] || 'var(--muted)';
  $('#msStatus').replaceChildren(h('b', { style: { color: col } }, current.state.toUpperCase()),
    current.name ? ` · ${current.name}` : '', current.step >= 0 ? ` · step ${current.step + 1}/${current.total}` : '',
    current.msg ? ` · ${current.msg}` : '', current.elapsed ? ` · ${mmss(current.elapsed)}` : '');
  $('#msRun').textContent = current.state === 'paused' ? '▶ Resume' : '▶ Run';
  render();
}
on('mission', (d) => { if (d.drone === selId()) showStatus(d); });
on('select', () => showStatus(store.drones[selId()]?.mission));
on('hello', async () => {
  try { plugins = (await api('/vision')).map(p => p.name); } catch {}
  refreshList(); render(); previewNow();
});
on('tab', (t) => { if (t === 'mission') { mini.draw(); previewNow(); } });

// ------------------------------------------------------------ search pattern generator
let patterns = {}, patVals = {};
function renderPatternForm() {
  const name = $('#msPattern').value, spec = patterns[name];
  if (!spec) return;
  patVals[name] ||= Object.fromEntries(Object.entries(spec.params).map(([k, v]) => [k, v.default]));
  $('#msPatternForm').replaceChildren(paramForm(spec.params, patVals[name], (k, v) => { patVals[name][k] = v; }));
}
$('#msPattern').onchange = renderPatternForm;
$('#msGen').onclick = async () => {
  const name = $('#msPattern').value;
  if (mission.steps.length > 3 && !confirm('Replace the current steps with the generated pattern?')) return;
  try {
    mission = await api(`/patterns/${name}`, patVals[name] || {});
    changed(true);
    toast(`Generated ${patterns[name].title}: ${mission.steps.length} steps — check the preview, then Save / Run`, 'success');
  } catch (e) { toast(e.message, 'error'); }
};
on('hello', async () => {
  try {
    patterns = await api('/patterns');
    $('#msPattern').replaceChildren(...Object.entries(patterns).map(([k, v]) => h('option', { value: k }, v.title)));
    renderPatternForm();
  } catch {}
});
