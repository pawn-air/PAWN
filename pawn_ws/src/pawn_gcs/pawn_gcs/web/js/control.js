// Pilot input: keyboard (Mode 2) and the flight buttons.
// Sticks are streamed to the GCS at 20 Hz; the GCS clamps them to safety.stick_max, forwards them as
// `rc` packets and zeroes them itself when the stream stops (dead-man), so a closed tab means hover.
import { $, api, store, sel, selId, sendCmd, toast, wsSend, select, on, h } from './core.js';

const keys = new Set();
let lastNonZero = 0;

const armed = () => $('#armSticks').checked;
const stickMax = () => Number(store.settings.safety?.stick_max ?? 60);
const typing = () => ['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName) && document.activeElement.type !== 'checkbox';

// -------------------------------------------------------------- actions
export const actions = {
  takeoff: () => sendCmd('takeoff'),
  land: () => sendCmd('land'),
  hover: () => api(`/drones/${selId()}/hover`, {}).catch(e => toast(e.message, 'error')),
  emergency: async () => {
    // always to ALL drones: an emergency is never about only one aircraft in tandem flight
    await Promise.all(Object.keys(store.drones).map(id => api(`/drones/${id}/emergency`, {}).catch(() => {})));
    toast('EMERGENCY sent', 'error', 6000);
  },
  snapshot: () => api(`/drones/${selId()}/snapshot`, {}).then(r => toast(`Snapshot ${r.file}`, 'success')).catch(e => toast(e.message, 'warn')),
  toggleVideo: () => api(`/drones/${selId()}/record`, { what: 'video', on: !sel()?.recording?.video }).catch(e => toast(e.message, 'error')),
  toggleLog: () => api(`/drones/${selId()}/record`, { what: 'log', on: !sel()?.recording?.logging }).catch(e => toast(e.message, 'error')),
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
  try { const r = await api(`/drones/${selId()}/connect`, {}); toast(r.ok ? 'Connected' : 'Connect failed — see log', r.ok ? 'success' : 'error'); }
  catch (e) { toast(e.message, 'error'); }
  $('#btnConnect').disabled = false;
};
$('#btnDisconnect').onclick = () => api(`/drones/${selId()}/disconnect`, {});
$('#btnHelp').onclick = () => $('#helpDlg').showModal();

// -------------------------------------------------------------- keyboard
const STICK_KEYS = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'];
addEventListener('keydown', (e) => {
  if (e.key === 'Escape') { document.activeElement?.blur?.(); return; }
  if (e.key === 'Meta') { keys.clear(); return; }
  if (typing()) return;
  const k = e.key.toLowerCase();
  if (k === ' ' && e.shiftKey) { e.preventDefault(); actions.emergency(); return; }
  if (STICK_KEYS.includes(k)) { e.preventDefault(); keys.add(k); return; }
  if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
  switch (k) {
    case ' ': e.preventDefault(); actions.hover(); break;
    case 't': actions.takeoff(); break;
    case 'l': actions.land(); break;
    case 'f': actions.snapshot(); break;
    case 'r': actions.toggleVideo(); break;
    case '?': $('#helpDlg').showModal(); break;
    case '1': case '2': case '3': case '4': select(Object.keys(store.drones)[Number(k) - 1]); break;
  }
});
addEventListener('keyup', (e) => { keys.delete(e.key.toLowerCase()); if (e.key === 'Meta') keys.clear(); });
// Never leave a key "held": macOS sends no keyup for keys released while Cmd is down, and a hidden
// tab or a focus change can swallow the keyup too. Then the drone would keep the last stick values.
addEventListener('blur', () => keys.clear());
document.addEventListener('visibilitychange', () => { if (document.hidden) keys.clear(); });

// Mode 2: W/S + A/D = left stick (throttle / yaw), arrows = right stick (pitch / roll). Order = rc a b c d.
function keyboardSticks() {
  const m = stickMax();
  const ax = (p, n) => (keys.has(p) ? m : 0) - (keys.has(n) ? m : 0);
  return [ax('arrowright', 'arrowleft'), ax('arrowup', 'arrowdown'), ax('w', 's'), ax('d', 'a')];
}

// -------------------------------------------------------------- stick display
// knobs = your keyboard input right now; "rc sent" = what the GCS last sent to the drone
function knob(el, x, y) { const k = el.querySelector('.knob'); k.style.left = `${36 + x * 36}%`; k.style.top = `${36 - y * 36}%`; }
function showInput(v) {
  const m = stickMax() || 1;
  knob($('#stickL'), v[3] / m, v[2] / m); knob($('#stickR'), v[0] / m, v[1] / m);
}
function renderRcInfo() {
  const d = sel(), v = keyboardSticks();
  const why = !armed() ? 'sticks not armed' : typing() ? 'keys go to the text field — press Esc'
    : d && !d.connected ? 'not connected' : d && d.motors_running === false ? 'motors off → no rc sent' : '';
  $('#rcInfo').replaceChildren(
    h('span', {}, 'keys'), h('span', {}, v.join(' ')),
    h('span', {}, 'rc sent'), h('span', {}, (d?.rc || [0, 0, 0, 0]).join(' ')),
    h('span', {}, 'stick limit'), h('span', {}, `±${stickMax()}`),
    ...(why ? [h('span', {}, 'note'), h('span', { class: 'warn-t' }, why)] : []));
}
on('telemetry', (d) => { if (d.drone === selId() && sel()) sel().rc = d.rc; });
setInterval(renderRcInfo, 100);

// -------------------------------------------------------------- 20 Hz loop
setInterval(() => {
  const id = selId();
  const v = armed() ? keyboardSticks() : [0, 0, 0, 0];
  showInput(v);
  if (!id || !armed()) return;
  const nz = v.some(x => x !== 0);
  if (nz) lastNonZero = performance.now();
  if (nz || performance.now() - lastNonZero < 300) wsSend({ type: 'sticks', drone: id, v });
}, 50);
