// Bootstrap + shell: tabs, fleet selector, top-bar stats, link card, raw state table, log.
// New tab module: create web/js/tabs/<name>.js, fill #pane-<name>, import it below.
import { $, $$, api, store, on, emit, h, hhmmss, fmt, connectWS, select, sel, selId, toast } from './core.js';
import { videoStats } from './video.js';
import './control.js';
import './tabs/console.js';
import './tabs/telemetry.js';
import './tabs/review.js';
import './tabs/settings.js';

// ------------------------------------------------------------- tabs
$$('.tab').forEach(t => t.onclick = () => {
  $$('.tab').forEach(x => x.classList.toggle('active', x === t));
  $$('.pane').forEach(p => p.classList.toggle('active', p.id === 'pane-' + t.dataset.tab));
  try { localStorage.setItem('pawn.tab', t.dataset.tab); } catch {}
  emit('tab', t.dataset.tab);
});
try { const t = localStorage.getItem('pawn.tab'); if (t) setTimeout(() => $(`.tab[data-tab="${t}"]`)?.click(), 0); } catch {}

// ------------------------------------------------------------- fleet chips
function renderFleet() {
  $('#fleet').replaceChildren(...Object.values(store.drones).map((d, i) => {
    const dot = !d.connected ? '' : d.link_alive ? 'ok' : 'bad';
    return h('button', { class: `drone-chip${d.id === selId() ? ' active' : ''}`, style: { '--c': d.color }, title: `${d.name} · ${d.ip} (key ${i + 1})`, onclick: () => select(d.id) },
      h('span', { class: `dot ${dot}` }), h('span', { style: { color: d.color } }, d.name || d.id));
  }));
}

// ------------------------------------------------------------- link card + top stats
function renderLink() {
  const d = sel(); if (!d) return;
  $('#linkDot').className = 'dot ' + (!d.connected ? '' : d.link_alive ? 'ok' : 'bad');
  const kv = [
    ['drone', `${d.name} @ ${d.ip}`], ['bind', d.bind_ip], ['sdk?', d.info?.sdk ?? '–'], ['sn?', d.info?.sn ?? '–'], ['wifi?', d.info?.wifi ?? '–'],
    ['state', `${fmt(d.state_hz, 0)} Hz · ${d.state_packets ?? 0} packets`], ['video', `${fmt(d.video_fps, 0)} fps · ${fmt(d.video_kbps / 1000, 2)} Mbps`],
    ['resolution', d.video_size?.[0] ? `${d.video_size[0]}×${d.video_size[1]}` : '–'], ['decode errors', d.decode_errors ?? 0],
    ['last cmd rtt', d.rtt != null ? `${Math.round(d.rtt * 1000)} ms` : '–'],
  ];
  if (d.open_error) kv.push(['error', d.open_error]);
  $('#linkInfo').replaceChildren(...kv.flatMap(([k, v]) => [h('span', {}, k), h('span', {}, v)]));
  $('#tsHz').textContent = d.connected ? `${fmt(d.state_hz, 0)}Hz` : '–';
  $('#tsVid').textContent = d.connected ? `${fmt(videoStats().fps, 0)}fps ${fmt((d.video_kbps || 0) / 1000, 1)}M` : '–';
  $('#btnRecVid').classList.toggle('on', !!d.recording?.video);
  $('#btnRecLog').classList.toggle('on', !!d.recording?.logging);
  $('#btnRecVid').textContent = d.recording?.video ? `● REC ${d.recording.frames}f` : '● REC';
  $('#btnRecLog').textContent = d.recording?.logging ? '● LOG ON' : '● LOG';
  $('#btnConnect').textContent = d.connected ? 'Reconnect' : 'Connect';
}

// ------------------------------------------------------------- raw state table (right column)
// rows = exactly the fields in the drone's packets (a stock Tello has no mission-pad fields, an EDU does),
// most used first; KEY_FIELDS are highlighted
const KEY_FIELDS = ['bat', 'h', 'tof', 'temph', 'templ', 'time'];
const ORDER = [...KEY_FIELDS, 'yaw', 'pitch', 'roll', 'vgx', 'vgy', 'vgz', 'baro', 'agx', 'agy', 'agz'];
const rank = (k) => { const i = ORDER.indexOf(k); return i < 0 ? ORDER.length : i; };
let shown = '';
function buildTable(keys) {
  $('#teleTable').replaceChildren(...keys.map(k =>
    h('tr', { class: KEY_FIELDS.includes(k) ? 'key' : '', title: store.fieldMeta[k]?.label || k },
      h('td', {}, k), h('td', { class: 'v', id: `tv-${k}` }, '–'), h('td', { class: 'u' }, store.fieldMeta[k]?.unit ?? ''))));
}
let lastTel = 0;
on('telemetry', (d) => {
  if (d.drone !== selId()) return;
  lastTel = performance.now();
  const keys = Object.keys(d.state).sort((a, b) => rank(a) - rank(b));
  if (keys.join() !== shown) { shown = keys.join(); buildTable(keys); emit('fields', keys); }
  for (const k in d.state) { const el = document.getElementById(`tv-${k}`); if (el) el.textContent = d.state[k]; }
});

setInterval(() => {
  const age = lastTel ? (performance.now() - lastTel) / 1000 : null;
  $('#stateAge').textContent = age == null ? 'no packets' : `last ${fmt(age, 1)} s ago`;
  $('#stateAge').style.color = age != null && age > 1 ? 'var(--danger)' : '';
}, 250);

// ------------------------------------------------------------- log
const evBox = $('#events');
const evLines = [];
function text(e) {
  if (e.type === 'command') return `> ${e.cmd}  <  ${e.resp ?? 'no reply (timeout)'}  ${Math.round((e.dt || 0) * 1000)} ms`;
  if (e.type === 'tello') return `drone: ${e.msg}`;
  return e.msg;
}
function evLine(e) {
  const f = $('#evFilter').value;
  if (f && e.type !== f) return;
  const multi = Object.keys(store.drones).length > 1;
  const cls = e.type === 'command' ? (e.ok ? 'cmd' : 'cmd bad') : e.type === 'tello' ? 'tello' : e.level;
  evBox.append(h('div', { class: `ev ${cls}` }, h('span', { class: 't' }, hhmmss(e.t || Date.now() / 1000)),
    multi && e.drone ? h('span', { class: 'd' }, e.drone) : null, text(e)));
  while (evBox.children.length > 1000) evBox.firstChild.remove();
  evBox.scrollTop = evBox.scrollHeight;
}
function addEvent(e) { evLines.push(e); if (evLines.length > 5000) evLines.shift(); evLine(e); }
$('#evFilter').onchange = () => { evBox.replaceChildren(); evLines.forEach(evLine); };
$('#evExport').onclick = () => {
  const txt = evLines.map(e => `${new Date((e.t || 0) * 1000).toISOString()}\t${e.type}\t${e.drone || '-'}\t${text(e)}`).join('\n');
  h('a', { href: URL.createObjectURL(new Blob([txt], { type: 'text/plain' })), download: `pawn-log-${Date.now()}.log` }).click();
};

// ------------------------------------------------------------- wiring
on('hello', async (d) => {
  $('#simBadge').classList.toggle('hidden', !d.sim);
  evBox.replaceChildren(); evLines.length = 0;
  (d.events || []).forEach(addEvent);
  renderFleet(); renderLink();
  store.fieldMeta = (await api('/meta')).state_fields; shown = '';
});
on('select', () => { renderFleet(); renderLink(); shown = ''; });
on('event', (e) => {
  addEvent({ type: 'event', ...e, t: Date.now() / 1000 });
  if (e.level === 'error') toast(`${e.drone ? e.drone + ': ' : ''}${e.msg}`, 'error', 5000);
});
on('command', (c) => addEvent({ type: 'command', ...c }));
on('tello', (m) => addEvent({ type: 'tello', ...m }));
on('link', () => { renderFleet(); renderLink(); });
on('recording', (r) => { const d = store.drones[r.drone]; if (d) d.recording = r; renderLink(); });
on('ws', (ok) => { $('#tsWs').textContent = ok ? '…' : 'OFFLINE'; $('#tsWs').style.color = ok ? '' : 'var(--danger)'; if (!ok) toast('Lost connection to GCS server — retrying', 'error'); });
on('latency', (ms) => { $('#tsWs').textContent = `${Math.round(ms)}ms`; $('#tsWs').style.color = ''; });

connectWS();
