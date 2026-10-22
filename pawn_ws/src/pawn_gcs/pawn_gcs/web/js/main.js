// Bootstrap + shell: tabs, fleet selector, top-bar stats, link/pre-flight cards, event log.
import { $, $$, api, store, on, emit, h, hhmmss, mmss, fmt, connectWS, select, sel, selId, toast } from './core.js';
import { videoStats } from './video.js';
import './control.js';
import './voice.js';
import './tabs/map.js';
import './tabs/mission.js';
import './tabs/vision.js';
import './tabs/console.js';
import './tabs/telemetry.js';
import './tabs/review.js';

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
    const st = store.tel[d.id]?.state || {};
    const dot = !d.connected ? '' : d.link_alive ? 'ok' : 'bad';
    return h('button', { class: `drone-chip${d.id === selId() ? ' active' : ''}`, style: { '--c': d.color }, title: `${d.name} · ${d.ip} (key ${i + 1})`, onclick: () => select(d.id) },
      h('span', { class: `dot ${dot}` }), h('span', { style: { color: d.color } }, d.name || d.id),
      h('span', { class: 'bat' }, st.bat != null ? `${st.bat}%` : ''), d.flying ? h('span', { title: 'airborne' }, '✈') : null);
  }));
}

// ------------------------------------------------------------- link card + top stats
function renderLink() {
  const d = sel(); if (!d) return;
  $('#linkDot').className = 'dot ' + (!d.connected ? '' : d.link_alive ? 'ok' : 'bad');
  const kv = [
    ['drone', `${d.name} @ ${d.ip}`], ['bind', d.bind_ip], ['sdk', d.info?.sdk ?? '–'], ['serial', d.info?.sn ?? '–'],
    ['telemetry', `${fmt(d.state_hz, 0)} Hz`], ['video', `${fmt(d.video_fps, 0)} fps · ${fmt(d.video_kbps / 1000, 2)} Mbps`],
    ['resolution', d.video_size?.[0] ? `${d.video_size[0]}×${d.video_size[1]}` : '–'], ['cmd rtt', d.rtt != null ? `${Math.round(d.rtt * 1000)} ms` : '–'],
  ];
  if (d.open_error) kv.push(['error', d.open_error]);
  $('#linkInfo').replaceChildren(...kv.flatMap(([k, v]) => [h('span', {}, k), h('span', {}, v)]));
  $('#tsHz').textContent = d.connected ? `${fmt(d.state_hz, 0)}Hz` : '–';
  $('#tsVid').textContent = d.connected ? `${fmt(videoStats().fps, 0)}fps ${fmt((d.video_kbps || 0) / 1000, 1)}M` : '–';
  $('#tsSnr').textContent = d.info?.wifi ?? '–';
  $('#tsFt').textContent = mmss(d.flight_time || 0);
  const mb = $('#modeBadge'); mb.textContent = d.mode || 'MANUAL'; mb.className = `badge mode-${d.mode}`;
  $('#btnRecVid').classList.toggle('on', !!d.recording?.video);
  $('#btnRecLog').classList.toggle('on', !!d.recording?.logging);
  $('#btnRecVid').textContent = d.recording?.video ? `● REC ${mmss(d.recording.frames / 30)}` : '● REC';
  $('#btnRecLog').textContent = d.recording?.logging ? '● LOG ON' : '● LOG';
  $('#btnConnect').textContent = d.connected ? 'Reconnect' : 'Connect';
}

// ------------------------------------------------------------- pre-flight
async function preflight() {
  if (!selId()) return;
  try {
    const r = await api(`/drones/${selId()}/preflight`);
    $('#preflight').replaceChildren(...r.checks.map(c => h('li', {}, h('span', { class: `dot ${c.ok ? 'ok' : 'bad'}` }), h('b', {}, c.name), h('span', {}, c.detail))));
    $('#btnTakeoff').title = r.blockers.length ? 'Blocked: ' + r.blockers.join('; ') : 'Takeoff (T)';
  } catch {}
}
$('#btnPreflight').onclick = preflight;
setInterval(() => { if (!sel()?.flying) preflight(); }, 2000);

// ------------------------------------------------------------- events
const evBox = $('#events');
const evLines = [];
function evLine(e) {
  const f = $('#evFilter').value;
  if (f === 'warn' && !['warn', 'error'].includes(e.level)) return;
  if (f === 'cmd' && e.level !== 'cmd') return;
  const multi = Object.keys(store.drones).length > 1;
  evBox.append(h('div', { class: `ev ${e.level}` }, h('span', { class: 't' }, hhmmss(e.t || Date.now() / 1000)),
    multi && e.drone ? h('span', { class: 'd' }, e.drone) : null, e.msg));
  while (evBox.children.length > 400) evBox.firstChild.remove();
  evBox.scrollTop = evBox.scrollHeight;
}
function addEvent(e) { evLines.push(e); if (evLines.length > 2000) evLines.shift(); evLine(e); }
$('#evFilter').onchange = () => { evBox.replaceChildren(); evLines.forEach(evLine); };
$('#evExport').onclick = () => {
  const txt = evLines.map(e => `${new Date((e.t || 0) * 1000).toISOString()}\t${e.level}\t${e.drone || '-'}\t${e.msg}`).join('\n');
  h('a', { href: URL.createObjectURL(new Blob([txt], { type: 'text/plain' })), download: `pawn-events-${Date.now()}.log` }).click();
};

// ------------------------------------------------------------- wiring
on('hello', (d) => {
  $('#simBadge').classList.toggle('hidden', !d.sim);
  evBox.replaceChildren(); evLines.length = 0;
  (d.events || []).forEach(addEvent);
  renderFleet(); renderLink(); preflight();
});
on('select', () => { renderFleet(); renderLink(); preflight(); });
on('event', (e) => {
  addEvent({ ...e, t: Date.now() / 1000 });
  if (['warn', 'error'].includes(e.level) && e.src !== 'cmd') toast(`${e.drone ? e.drone + ': ' : ''}${e.msg}`, e.level, e.level === 'error' ? 6000 : 3500);
  if (e.level === 'success' && /TARGET/.test(e.msg)) toast(e.msg, 'success', 5000);
});
on('command', (c) => addEvent({ level: 'cmd', drone: c.drone, msg: `> ${c.cmd}  ${c.resp ?? 'timeout'}  ${Math.round((c.dt || 0) * 1000)}ms`, t: c.t }));
on('link', () => { renderFleet(); renderLink(); });
on('mode', () => renderLink());
on('recording', (r) => { const d = store.drones[r.drone]; if (d) d.recording = r; renderLink(); });
on('collision', (c) => toast(`${c.drone}: BRAKE — ${c.reason}`, 'error', 4000));
on('flight', (f) => { const d = store.drones[f.drone]; if (d) d.flying = f.flying; renderFleet(); });
on('ws', (ok) => { $('#tsWs').textContent = ok ? '…' : 'OFFLINE'; $('#tsWs').style.color = ok ? '' : 'var(--danger)'; if (!ok) toast('Lost connection to GCS server — retrying', 'error'); });
on('latency', (ms) => { $('#tsWs').textContent = `${Math.round(ms)}ms`; $('#tsWs').style.color = ''; });
let fleetT = 0;
on('telemetry', () => { const n = performance.now(); if (n - fleetT > 1000) { fleetT = n; renderFleet(); renderLink(); } });

connectWS();
