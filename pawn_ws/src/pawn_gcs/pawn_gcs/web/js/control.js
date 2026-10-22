// Pilot input: keyboard, gamepad (Gamepad API), on-screen touch sticks, and flight buttons.
// Sticks are streamed to the GCS at 20 Hz; the GCS forwards them as `rc` packets and
// zeroes them itself if the stream stops (dead-man), so a closed tab can't leave the drone drifting.
import { $, $$, api, store, sel, selId, sendCmd, toast, wsSend, clamp, select, on, h } from './core.js';
import { rth, bookmark } from './tabs/map.js';
import { toggleVoice } from './voice.js';

const keys = new Set();
const vstick = { L: [0, 0], R: [0, 0], active: false };
let gpIndex = null, gpPrev = [];
let lastNonZero = 0;

const armed = () => $('#armSticks').checked;
const rcMax = () => Number($('#rcMax').value);
const fleet = () => $('#fleetSync').checked;
const typing = () => ['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName) && document.activeElement.type !== 'checkbox' && document.activeElement.type !== 'range';

// -------------------------------------------------------------- actions
export const actions = {
  takeoff: () => sendCmd('takeoff', { fleet: fleet() }),
  land: () => sendCmd('land', { fleet: fleet() }),
  hover: async () => {
    const ids = fleet() ? Object.keys(store.drones) : [selId()];
    for (const id of ids) await api(`/drones/${id}/hover`, {}).catch(() => {});
    toast('Hover: missions/AUTO stopped, sticks zero', 'warn');
  },
  emergency: async () => {
    // always to ALL drones: an emergency is never about only one aircraft in tandem flight
    await Promise.all(Object.keys(store.drones).map(id => api(`/drones/${id}/emergency`, {}).catch(() => {})));
    toast('EMERGENCY — motors stopped', 'error', 6000);
  },
  snapshot: () => api(`/drones/${selId()}/snapshot`, {}).then(r => toast(`Snapshot ${r.file}`, 'success')).catch(e => toast(e.message, 'warn')),
  toggleVideo: () => {
    const on = !sel()?.recording?.video;
    api(`/drones/${selId()}/record`, { what: 'video', on }).catch(e => toast(e.message, 'error'));
  },
  toggleLog: () => {
    const on = !sel()?.recording?.logging;
    api(`/drones/${selId()}/record`, { what: 'log', on }).catch(e => toast(e.message, 'error'));
  },
};

$('#btnTakeoff').onclick = actions.takeoff;
$('#btnLand').onclick = actions.land;
$('#btnHover').onclick = actions.hover;
$('#btnEmergency').onclick = actions.emergency;
$('#btnSnap').onclick = actions.snapshot;
$('#btnRecVid').onclick = actions.toggleVideo;
$('#btnRecLog').onclick = actions.toggleLog;
$('#btnConnect').onclick = async () => {
  $('#btnConnect').disabled = true;
  try { const r = await api(`/drones/${selId()}/connect`, {}); toast(r.ok ? 'Connected' : 'Connect failed — see events', r.ok ? 'success' : 'error'); }
  catch (e) { toast(e.message, 'error'); }
  $('#btnConnect').disabled = false;
};
$('#btnDisconnect').onclick = () => api(`/drones/${selId()}/disconnect`, {});
$('#stepCm').oninput = (e) => $('#stepOut').value = e.target.value;
$('#rcMax').oninput = (e) => $('#rcMaxOut').value = e.target.value;
$$('[data-mv]').forEach(b => b.onclick = () => sendCmd(`${b.dataset.mv} ${$('#stepCm').value}`, { fleet: fleet() }));
$$('[data-rot]').forEach(b => b.onclick = () => sendCmd(`${b.dataset.rot} ${$('#rotDeg').value}`, { fleet: fleet() }));
$$('[data-flip]').forEach(b => b.onclick = () => sendCmd(`flip ${b.dataset.flip}`, { fleet: fleet() }));
$('#btnSpd').onclick = () => sendCmd(`speed ${$('#spd').value}`, { fleet: fleet() });

on('hello', (d) => { if (d.settings?.rc?.max) { $('#rcMax').value = d.settings.rc.max; $('#rcMaxOut').value = d.settings.rc.max; } });

// -------------------------------------------------------------- keyboard
const STICK_KEYS = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'];
addEventListener('keydown', (e) => {
  if (typing()) return;
  const k = e.key.toLowerCase();
  if (k === ' ' && e.shiftKey) { e.preventDefault(); actions.emergency(); return; }
  if (k === 'h' && e.shiftKey && !e.repeat) { rth(); return; }
  if (e.repeat && !STICK_KEYS.includes(k)) return;
  if (STICK_KEYS.includes(k)) { e.preventDefault(); keys.add(k); return; }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  switch (k) {
    case ' ': e.preventDefault(); actions.hover(); break;
    case 't': actions.takeoff(); break;
    case 'l': actions.land(); break;
    case 'f': actions.snapshot(); break;
    case 'r': actions.toggleVideo(); break;
    case '?': $('#helpDlg').showModal(); break;
    case 'm': bookmark(); break;
    case 'k': { const c = $('#markOn'); c.checked = !c.checked; c.dispatchEvent(new Event('change')); toast(c.checked ? '✚ Mark mode: click the target on the video' : 'Mark mode off'); break; }
    case 'v': toggleVoice(); break;
    case '1': case '2': case '3': case '4': select(Object.keys(store.drones)[Number(k) - 1]); break;
  }
});
addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
addEventListener('blur', () => keys.clear());
$('#btnHelp').onclick = () => $('#helpDlg').showModal();

function keyboardSticks(boost) {
  const m = boost ? 100 : rcMax();
  const ax = (p, n) => (keys.has(p) ? m : 0) - (keys.has(n) ? m : 0);
  return [ax('d', 'a'), ax('w', 's'), ax('arrowup', 'arrowdown'), ax('arrowright', 'arrowleft')];
}

// -------------------------------------------------------------- gamepad
addEventListener('gamepadconnected', (e) => { gpIndex = e.gamepad.index; toast(`Gamepad: ${e.gamepad.id.slice(0, 40)}`, 'success'); updatePadInfo(); });
addEventListener('gamepaddisconnected', () => { gpIndex = null; updatePadInfo(); });
function updatePadInfo() {
  const gp = gpIndex != null ? navigator.getGamepads()[gpIndex] : null;
  $('#padInfo').textContent = gp ? `🎮 ${gp.id.slice(0, 48)}` : 'Keyboard: WASD move · ↑↓ alt · ←→ yaw · gamepad auto-detected';
}
function shape(v) {
  const dz = store.settings.rc?.deadzone ?? 0.08, expo = store.settings.rc?.expo ?? 0.3;
  if (Math.abs(v) < dz) return 0;
  const x = Math.sign(v) * (Math.abs(v) - dz) / (1 - dz);
  return expo * x * x * x + (1 - expo) * x;
}
// axis mapping (per browser): function -> {axis, inv}. Mode 2 defaults for the standard gamepad layout.
const FUNCS = [['roll', 'Roll (right X)'], ['pitch', 'Pitch (right Y)'], ['throttle', 'Throttle (left Y)'], ['yaw', 'Yaw (left X)']];
const DEFAULT_MAP = { roll: { axis: 2, inv: false }, pitch: { axis: 3, inv: true }, throttle: { axis: 1, inv: true }, yaw: { axis: 0, inv: false } };
let axisMap = structuredClone(DEFAULT_MAP);
try { const m = JSON.parse(localStorage.getItem('pawn.axisMap') || 'null'); if (m) axisMap = { ...axisMap, ...m }; } catch {}
const saveMap = () => { try { localStorage.setItem('pawn.axisMap', JSON.stringify(axisMap)); } catch {} };
const ax = (a, f) => { const m = axisMap[f]; const v = a[m.axis] || 0; return m.inv ? -v : v; };

function gamepadSticks() {
  if (gpIndex == null) return null;
  const gp = navigator.getGamepads()[gpIndex];
  if (!gp) return null;
  const b = gp.buttons.map(x => x.pressed);
  const edge = (i) => b[i] && !gpPrev[i];
  if (edge(0)) actions.takeoff();
  if (edge(1)) actions.land();
  if (edge(2)) actions.hover();
  if (edge(3)) actions.snapshot();
  if (b[4] && b[5] && edge(9)) actions.emergency();
  gpPrev = b;
  const m = b[5] ? 100 : rcMax();
  const a = gp.axes;
  const out = [shape(ax(a, 'roll')) * m, shape(ax(a, 'pitch')) * m, shape(ax(a, 'throttle')) * m, shape(ax(a, 'yaw')) * m];
  return out.some(v => Math.abs(v) > 0.5) ? out : null;
}

// -------------------------------------------------------------- touch sticks
function bindStick(el, key) {
  const knob = el.querySelector('.knob');
  let pid = null;
  const set = (x, y) => { vstick[key] = [x, y]; knob.style.left = `${36 + x * 36}%`; knob.style.top = `${36 - y * 36}%`; };
  const move = (e) => {
    const r = el.getBoundingClientRect();
    const x = clamp((e.clientX - r.left) / r.width * 2 - 1, -1, 1), y = clamp(1 - (e.clientY - r.top) / r.height * 2, -1, 1);
    const n = Math.hypot(x, y); set(n > 1 ? x / n : x, n > 1 ? y / n : y);
  };
  el.addEventListener('pointerdown', (e) => { pid = e.pointerId; el.setPointerCapture(pid); vstick.active = true; move(e); });
  el.addEventListener('pointermove', (e) => { if (e.pointerId === pid) move(e); });
  const up = (e) => { if (e.pointerId !== pid) return; pid = null; set(0, 0); vstick.active = !!(vstick.L[0] || vstick.R[0] || vstick.L[1] || vstick.R[1]); };
  el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up);
  return set;
}
const setL = bindStick($('#stickL'), 'L'), setR = bindStick($('#stickR'), 'R');

// -------------------------------------------------------------- 20 Hz loop
setInterval(() => {
  const id = selId();
  if (!id) return;
  let v = gamepadSticks();
  const boost = keys.has('shift');
  if (!v && vstick.active) {
    const m = rcMax();
    v = [vstick.R[0] * m, vstick.R[1] * m, vstick.L[1] * m, vstick.L[0] * m];
  }
  if (!v) v = keyboardSticks(boost || [...keys].includes('shift'));
  // mirror keyboard/gamepad onto the on-screen sticks (visual feedback)
  if (!vstick.active) { const m = 100; setL(v[3] / m, v[2] / m); setR(v[0] / m, v[1] / m); }
  if (!armed()) return;
  const nz = v.some(x => Math.abs(x) >= 1);
  if (nz) lastNonZero = performance.now();
  if (nz || performance.now() - lastNonZero < 300) {
    const payload = v.map(x => Math.round(x));
    if (fleet()) for (const did of Object.keys(store.drones)) wsSend({ type: 'sticks', drone: did, v: payload });
    else wsSend({ type: 'sticks', drone: id, v: payload });
  }
}, 50);
addEventListener('keydown', (e) => { if (e.key === 'Shift') keys.add('shift'); });
addEventListener('keyup', (e) => { if (e.key === 'Shift') keys.delete('shift'); });

// -------------------------------------------------------------- mapping dialog (gamepad / RC radio as USB joystick)
let learning = null, rest = null, dlgTimer = null;
function renderPadForm() {
  $('#padMapForm').replaceChildren(...FUNCS.flatMap(([f, label]) => [
    h('label', {}, label),
    h('div', { class: 'row gap' },
      h('span', { class: 'mono' }, `axis ${axisMap[f].axis}`),
      h('label', { class: 'switch' }, h('input', { type: 'checkbox', checked: axisMap[f].inv, onchange: (e) => { axisMap[f].inv = e.target.checked; saveMap(); } }), h('span', {}, 'invert')),
      h('button', { class: `btn xs${learning === f ? ' primary' : ''}`, onclick: () => { const gp = gpIndex != null ? navigator.getGamepads()[gpIndex] : null; if (!gp) { toast('Connect a gamepad / radio and press any button first', 'warn'); return; } learning = f; rest = [...gp.axes]; renderPadForm(); } }, learning === f ? 'move stick…' : 'Learn')),
  ]));
}
function padTick() {
  const gp = gpIndex != null ? navigator.getGamepads()[gpIndex] : null;
  const box = $('#padAxes');
  if (!gp) { box.replaceChildren(h('span', { class: 'muted' }, 'No gamepad detected — connect it and press a button.')); return; }
  box.replaceChildren(h('b', { style: { gridColumn: '1 / -1' } }, gp.id.slice(0, 60)), ...gp.axes.flatMap((v, i) => {
    const used = FUNCS.filter(([f]) => axisMap[f].axis === i).map(([f]) => f).join(', ');
    return [h('span', {}, `axis ${i}`), h('div', { class: 'bar' }, h('i', { style: { left: `${Math.min(50, 50 + v * 50)}%`, width: `${Math.abs(v) * 50}%` } })), h('span', { class: 'muted' }, used || '')];
  }));
  if (learning && rest) {
    let best = -1, bd = 0.5;
    gp.axes.forEach((v, i) => { const d = Math.abs(v - (rest[i] || 0)); if (d > bd) { bd = d; best = i; } });
    if (best >= 0) {
      const v = gp.axes[best] - (rest[best] || 0);
      // the user pushes the stick RIGHT (roll/yaw) or UP (pitch/throttle): that direction must come out positive
      axisMap[learning] = { axis: best, inv: v < 0 };
      toast(`${learning} → axis ${best}${axisMap[learning].inv ? ' (inverted)' : ''}`, 'success');
      learning = null; rest = null; saveMap(); renderPadForm();
    }
  }
}
$('#btnPadMap').onclick = () => { renderPadForm(); $('#padDlg').showModal(); dlgTimer = setInterval(padTick, 60); };
$('#padDlg').addEventListener('close', () => { clearInterval(dlgTimer); learning = null; });
$('#padReset').onclick = () => { axisMap = structuredClone(DEFAULT_MAP); saveMap(); renderPadForm(); };
