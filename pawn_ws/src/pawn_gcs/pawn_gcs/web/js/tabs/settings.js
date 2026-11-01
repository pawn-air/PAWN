// Settings tab (every runtime setting) + Safety tab (the few limits that act on the sticks).
import { $, api, store, on, emit, h, toast } from '../core.js';

const LABELS = {
  stick_max: 'Stick limit ± (0-100)', deadman_ms: 'Dead-man: sticks → 0 after no input (ms)',
  rate_hz: 'rc packet rate (Hz)', timeout_s: 'State link shown lost after (s)',
  jpeg_quality: 'Browser JPEG quality', max_fps: 'Browser max fps', relay_udp: 'Raw H.264 relay (host:port, comma)',
  log_on_connect: 'Log while connected',
};

// After a change the field gives up focus, so W/A/S/D and the arrows go to the sticks again
// (a focused number field would otherwise eat the arrow keys and step its own value).
const done = (fn) => (e) => { fn(e); e.target.blur(); };
function input(v, set) {
  if (typeof v === 'boolean') return h('input', { type: 'checkbox', checked: v, onchange: done((e) => set(e.target.checked)) });
  if (typeof v === 'number') return h('input', { type: 'number', value: v, step: 'any', onchange: done((e) => set(Number(e.target.value))),
    onkeydown: (e) => { if (e.key === 'ArrowUp' || e.key === 'ArrowDown') e.preventDefault(); if (e.key === 'Enter') e.target.blur(); } });
  if (Array.isArray(v)) return h('input', { value: v.join(', '), onchange: done((e) => set(e.target.value.split(',').map(x => x.trim()).filter(Boolean))) });
  return h('input', { value: v, onchange: done((e) => set(e.target.value)) });
}

// ----------------------------------------------------------- settings tab: edit, then Save
let draft = {};
function buildSettings() {
  const s = store.settings; draft = structuredClone(s);
  const box = $('#settingsForm'); box.replaceChildren();
  for (const [sec, vals] of Object.entries(s)) {
    box.append(h('h4', {}, sec));
    for (const [k, v] of Object.entries(vals)) box.append(h('label', { title: `${sec}.${k}` }, LABELS[k] || k), input(v, (x) => { draft[sec][k] = x; }));
  }
}
$('#setSave').onclick = async () => {
  try { store.settings = await api('/settings', draft); toast('Settings saved', 'success'); emit('settings', store.settings); }
  catch (e) { toast(e.message, 'error'); }
};
$('#setReset').onclick = async () => {
  if (!confirm('Reset all settings to defaults?')) return;
  store.settings = await api('/settings/reset', {}); emit('settings', store.settings); toast('Defaults restored');
};

// ----------------------------------------------------------- safety tab: applies immediately
const SAFETY = [['safety', 'stick_max'], ['safety', 'deadman_ms'], ['link', 'timeout_s']];
function buildSafety() {
  const s = store.settings;
  $('#safetyForm').replaceChildren(...SAFETY.flatMap(([sec, k]) => [
    h('label', { title: `${sec}.${k}` }, LABELS[k]),
    input(s[sec]?.[k], async (v) => {
      try { store.settings = await api('/settings', { [sec]: { [k]: v } }); emit('settings', store.settings); toast(`${sec}.${k} = ${v}`, 'success', 1800); }
      catch (e) { toast(e.message, 'error'); }
    }),
  ]));
}

function build() { buildSettings(); buildSafety(); }
on('hello', build);
on('settings', build);
