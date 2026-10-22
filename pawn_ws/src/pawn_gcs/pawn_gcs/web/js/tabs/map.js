// Map tab: live trajectories, localised targets (+uncertainty), coverage heat map, backtrack routes,
// RTH along the breadcrumb trail, bookmarks, SLAM clouds, geofence and mission plan.
import { $, $$, api, store, on, emit, h, fmt, hhmmss, selId, sel, toast } from '../core.js';
import { MapView } from '../mapview.js';

export const map = new MapView($('#map'));
export let planPoints = null;
map.follow = null;
const coverage = { res: 0.25, cells: new Map(), area: 0 };
let selectedTarget = null;

$$('#pane-map .segb').forEach(b => b.onclick = () => {
  $$('#pane-map .segb').forEach(x => x.classList.toggle('active', x === b));
  map.setMode(b.dataset.view);
  map.fit();
});
$('#mapFit').onclick = () => { $('#mapFollow').checked = false; map.fit(); };
$('#mapClear').onclick = async () => {
  if (!confirm('Clear trajectories, targets and coverage?')) return;
  await api('/map/clear', {});
  for (const t of Object.values(map.layers.tracks)) t.pts = [];
  coverage.cells.clear(); map.layers.paths = {}; selectedTarget = null;
  renderMarkers(); map.draw();
};
$('#mapHome').onclick = async () => { await api(`/drones/${selId()}/home`, {}); const t = map.layers.tracks[selId()]; if (t) t.pts = []; toast('Home reset to current position', 'info'); };
$('#mapCov').onchange = () => { map.layers.coverage = $('#mapCov').checked ? coverage : null; map.draw(); };
map.layers.coverage = coverage;

// ------------------------------------------------------------ actions
export async function rth() {
  const d = sel();
  if (!d?.flying) { toast('RTH: not flying', 'warn'); return; }
  const n = d.nav || {};
  if (!confirm(`Return home along the flown trail (${fmt(n.path_m, 1)} m, ~${fmt(n.bat_needed, 0)}% battery) and land?`)) return;
  try { const r = await api(`/drones/${selId()}/rth`, { land: true }); toast(`RTH started: ${r.steps} steps`, 'warn'); }
  catch (e) { toast(e.message, 'error'); }
}
export async function bookmark(note) {
  try { const m = await api(`/drones/${selId()}/bookmark`, { note: note || `bookmark ${hhmmss(Date.now() / 1000)}` }); toast(`⚑ ${m.label}`, 'info'); }
  catch (e) { toast(e.message, 'error'); }
}
$('#btnRth').onclick = rth;
$('#btnBookmark').onclick = () => { const n = prompt('Bookmark note', 'check here'); if (n !== null) bookmark(n); };

async function showRoute(m) {
  if (!m || m.source === 'bookmark') { delete map.layers.paths.route; map.layers.selected = null; selectedTarget = null; renderMarkers(); return; }
  selectedTarget = m.id; map.layers.selected = m.id;
  try {
    const r = await api(`/targets/${m.id}/backtrack`);
    map.layers.paths.route = { pts: r.path, color: '#fbbf24', width: 3.5 };
    toast(`Rescue route to ${m.label} #${m.id}: ${fmt(r.length_m, 1)} m from home along the flown trail`, 'info', 5000);
  } catch (e) { toast(e.message, 'warn'); }
  renderMarkers();
}
map.onPick = (m) => { if (m) showRoute(m); };

async function gotoTarget(m) {
  if (!confirm(`Fly back along the trail to where ${m.label} #${m.id} was seen?`)) return;
  try { const r = await api(`/drones/${selId()}/goto_target/${m.id}`, {}); toast(`Going to ${m.label}: ${r.steps} steps`, 'warn'); }
  catch (e) { toast(e.message, 'error'); }
}

// RTH preview path (refreshed while the checkbox is on and the drone flies)
setInterval(async () => {
  if (!$('#mapRth').checked || !sel()?.flying) { if (map.layers.paths.rth) { delete map.layers.paths.rth; map.draw(); } return; }
  try { const r = await api(`/drones/${selId()}/rth`); map.layers.paths.rth = { pts: r.path, color: '#22c55e', width: 2, dash: [8, 5] }; map.draw(); } catch {}
}, 2000);

export function setPlan(pts, idx = -1) {
  planPoints = pts;
  map.layers.plan = $('#mapPlan').checked ? pts : null;
  map.layers.planIndex = idx;
  map.draw();
}
$('#mapPlan').onchange = () => setPlan(planPoints);

function fence() {
  const s = store.settings.safety || {};
  const hm = store.drones[selId()]?.home || [0, 0];
  map.layers.fence = $('#mapFence').checked && s.geofence_enabled ? { r: s.geofence_radius_m, alt: s.geofence_max_alt_m, cx: hm[0], cy: hm[1] } : null;
}
$('#mapFence').onchange = () => { fence(); map.draw(); };

// ------------------------------------------------------------ side panel
function renderSummary() {
  const d = sel() || {}, n = d.nav || {};
  const ms = store.map.markers.filter(m => m.source !== 'bookmark');
  const conf = ms.filter(m => m.status === 'confirmed').length;
  const bat = store.tel[selId()]?.state?.bat;
  const tight = n.bat_needed != null && bat != null && bat - n.bat_needed <= (store.settings.safety?.battery_land ?? 15) + 10;
  $('#sarSummary').replaceChildren(
    h('div', {}, h('span', {}, 'explored'), h('b', {}, `${fmt(coverage.area, 1)} m²`)),
    h('div', {}, h('span', {}, 'targets'), h('b', { class: conf ? 'ok-t' : '' }, `${conf} ✓ / ${ms.length}`)),
    h('div', { title: 'Distance home along the flown trail / straight line' }, h('span', {}, 'home via trail'),
      h('b', {}, d.flying && n.path_m != null ? `${fmt(n.path_m, 1)} m` : '–'), h('small', {}, n.straight_m != null && d.flying ? ` (${fmt(n.straight_m, 1)} direct)` : '')),
    h('div', { title: `learned drain ${fmt(n.drain_pct_s, 3)} %/s` }, h('span', {}, 'RTH battery'),
      h('b', { class: tight ? 'danger-t' : '' }, d.flying && n.bat_needed != null ? `~${fmt(n.bat_needed, 0)}%` : '–')));
}

function renderMarkers() {
  const box = $('#markerList');
  const list = store.map.markers.slice().sort((a, b) => (b.status === 'confirmed') - (a.status === 'confirmed') || b.t - a.t);
  box.replaceChildren(...list.map(m => {
    const bm = m.source === 'bookmark';
    return h('div', { class: `marker-item${selectedTarget === m.id ? ' sel' : ''}`, onclick: () => showRoute(m) },
      h('div', { class: 'row gap' },
        h('b', { class: 'grow' }, `${bm ? '⚑' : '★'} ${m.label} #${m.id}`),
        bm ? null : h('span', { class: `tag ${m.status}` }, m.status || 'api'),
        h('button', { class: 'btn xs ghost', title: 'delete', onclick: (e) => { e.stopPropagation(); api(`/map/markers/${m.id}`, undefined, 'DELETE'); } }, '×')),
      h('div', { class: 'mono small' }, `(${fmt(m.x)}, ${fmt(m.y)}, ${fmt(m.z)}) m${m.sigma ? ` ±${fmt(m.sigma, 2)}` : ''}`),
      bm ? h('div', { class: 'small muted' }, `${hhmmss(m.t)} · ${m.drone || ''}`) :
        h('div', { class: 'small muted' }, `${m.method || m.source} · ${m.hits} hits${m.n_range ? ` (${m.n_range} ranged)` : ''} · ${m.source} · ${hhmmss(m.t)}`),
      bm ? null : h('div', { class: 'row gap' },
        h('button', { class: 'btn xs', title: 'show the rescue route home', onclick: (e) => { e.stopPropagation(); showRoute(m); } }, '⤺ route'),
        h('button', { class: 'btn xs', title: 'fly back along the trail to where it was seen', onclick: (e) => { e.stopPropagation(); gotoTarget(m); } }, '➜ go to')));
  }));
  if (!list.length) box.append(h('div', { class: 'small muted' }, 'Targets appear when a ★ vision plugin detects something in flight (triangulated from several positions, ranged by known size) or when you click the video with ✚ Mark. ⚑ Bookmarks (M) save notes on the map.'));
  map.layers.markers = store.map.markers;
  renderSummary();
  map.draw();
}

// ------------------------------------------------------------ wiring
on('hello', async () => {
  const m = await api('/map');
  store.map.markers = m.markers; store.map.clouds = m.clouds;
  for (const [id, pts] of Object.entries(m.tracks)) map.layers.tracks[id] = { pts, color: store.drones[id]?.color || '#22d3ee' };
  coverage.cells.clear();
  if (m.coverage) { coverage.res = m.coverage.res; coverage.area = m.coverage.area_m2; for (const [i, j, n] of m.coverage.cells) coverage.cells.set(`${i},${j}`, n); }
  map.layers.clouds = store.map.clouds;
  fence(); renderMarkers(); map.fit();
});
on('coverage', (d) => {
  if (d.cleared) coverage.cells.clear();
  coverage.res = d.res; coverage.area = d.area_m2;
  for (const [i, j, n] of d.cells) coverage.cells.set(`${i},${j}`, n);
  map.draw();
});
on('map', (d) => {
  map.layers.clouds = store.map.clouds;
  if (d.type === 'cleared') { coverage.cells.clear(); map.layers.paths = {}; }
  renderMarkers();
});
on('select', (id) => { if ($('#mapFollow').checked) map.follow = id; fence(); map.draw(); });
$('#mapFollow').onchange = (e) => { map.follow = e.target.checked ? selId() : null; map.draw(); };

let lastTrack = {};
on('telemetry', (d) => {
  const dr = store.drones[d.drone] || {};
  map.layers.drones[d.drone] = { pose: d.odom, color: dr.color || '#22d3ee', label: dr.name || d.drone };
  const t = (map.layers.tracks[d.drone] ||= { pts: [], color: dr.color || '#22d3ee' });
  const p = [d.odom.x, d.odom.y, d.odom.z], last = t.pts[t.pts.length - 1];
  if (!last || Math.abs(last[0] - p[0]) + Math.abs(last[1] - p[1]) + Math.abs(last[2] - p[2]) > 0.03) {
    t.pts.push(p); if (t.pts.length > 20000) t.pts.shift();
  }
  if ($('#mapFollow').checked) map.follow = selId();
  const now = performance.now();
  if (now - (lastTrack.t || 0) > 100) {
    lastTrack.t = now;
    if ($('#pane-map').classList.contains('active')) map.draw();
    const o = store.tel[selId()]?.odom;
    if (o) $('#mapInfo').textContent = `pos (${fmt(o.x, 2)}, ${fmt(o.y, 2)}, ${fmt(o.z, 2)}) m · yaw ${fmt(o.yaw, 0)}° · ${o.source === 'external' ? 'SLAM pose' : 'dead reckoning'} · trip ${fmt(o.dist, 1)} m`;
  }
  if (now - (lastTrack.s || 0) > 1000) { lastTrack.s = now; renderSummary(); }
});
on('settings', () => { fence(); map.draw(); });
on('tab', (t) => { if (t === 'map') map.draw(); });
