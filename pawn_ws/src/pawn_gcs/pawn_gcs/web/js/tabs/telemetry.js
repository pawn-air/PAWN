// Telemetry tab (every state field + big charts), right-column instruments, Settings tab.
import { $, api, store, on, emit, h, fmt, selId, toast } from '../core.js';
import { StripChart } from '../chart.js';

// ----------------------------------------------------------- full table
let meta = {};
on('hello', async () => {
  meta = (await api('/meta')).state_fields;
  $('#teleTable').replaceChildren(...Object.entries(meta).map(([k, m]) =>
    h('tr', {}, h('td', {}, m.label), h('td', { class: 'v', id: `tv-${k}` }, '–'), h('td', { class: 'u' }, m.unit))));
  buildSettings();
});

const big = {
  alt: new StripChart($('#teleCharts'), { title: 'ALTITUDE m', color: '#22d3ee', digits: 2 }),
  tof: new StripChart($('#teleCharts'), { title: 'TOF m', color: '#38bdf8', digits: 2 }),
  spd: new StripChart($('#teleCharts'), { title: 'GROUND SPEED m/s', color: '#a3e635', digits: 2, min: 0 }),
  vz: new StripChart($('#teleCharts'), { title: 'VERTICAL SPEED m/s', color: '#fbbf24', digits: 2 }),
  bat: new StripChart($('#teleCharts'), { title: 'BATTERY %', color: '#22c55e', digits: 0, min: 0, max: 100, warn: 30 }),
  temp: new StripChart($('#teleCharts'), { title: 'TEMP °C', color: '#f97316', digits: 0, warn: 85 }),
};
const small = {
  alt: new StripChart($('#miniCharts'), { title: 'ALT m', color: '#22d3ee', digits: 2 }),
  bat: new StripChart($('#miniCharts'), { title: 'BAT %', color: '#22c55e', digits: 0, min: 0, max: 100, warn: 30 }),
  spd: new StripChart($('#miniCharts'), { title: 'SPEED m/s', color: '#a3e635', digits: 2, min: 0 }),
};

// ----------------------------------------------------------- attitude indicator (right column)
const adi = $('#adi');
function drawADI(st) {
  const r = adi.getBoundingClientRect(), dpr = devicePixelRatio || 1;
  adi.width = r.width * dpr; adi.height = r.height * dpr;
  const c = adi.getContext('2d'), w = adi.width, hh = adi.height, R = Math.min(w, hh) / 2 - 4 * dpr;
  const cx = w / 2 - R * 0.75, cy = hh / 2;
  c.save(); c.beginPath(); c.arc(cx, cy, R, 0, 7); c.clip();
  c.translate(cx, cy); c.rotate(-(st.roll || 0) * Math.PI / 180);
  const off = (st.pitch || 0) * R / 40;
  c.fillStyle = '#0c4a6e'; c.fillRect(-R * 2, -R * 2 + off, R * 4, R * 2);
  c.fillStyle = '#4a3216'; c.fillRect(-R * 2, off, R * 4, R * 2);
  c.strokeStyle = '#fff'; c.lineWidth = 1.5 * dpr; c.beginPath(); c.moveTo(-R * 2, off); c.lineTo(R * 2, off); c.stroke();
  for (const a of [-20, -10, 10, 20]) { const y = off - a * R / 40; c.beginPath(); c.moveTo(-R * 0.25, y); c.lineTo(R * 0.25, y); c.stroke(); }
  c.restore();
  c.strokeStyle = '#fbbf24'; c.lineWidth = 3 * dpr;
  c.beginPath(); c.moveTo(cx - R * 0.55, cy); c.lineTo(cx - R * 0.15, cy); c.moveTo(cx + R * 0.15, cy); c.lineTo(cx + R * 0.55, cy); c.stroke();
  c.strokeStyle = '#2a3a4b'; c.lineWidth = 2 * dpr; c.beginPath(); c.arc(cx, cy, R, 0, 7); c.stroke();
  // compass
  const kx = w / 2 + R * 0.75, yaw = ((st.yaw || 0) * Math.PI) / 180;
  c.strokeStyle = '#2a3a4b'; c.beginPath(); c.arc(kx, cy, R, 0, 7); c.stroke();
  c.fillStyle = '#7d8fa1'; c.font = `${10 * dpr}px ui-monospace, monospace`; c.textAlign = 'center'; c.textBaseline = 'middle';
  [['0', 0], ['90', 90], ['180', 180], ['270', 270]].forEach(([t, a]) => {
    const b = (a * Math.PI) / 180 - yaw;
    c.fillText(t, kx + Math.sin(b) * R * 0.78, cy - Math.cos(b) * R * 0.78);
  });
  c.fillStyle = '#22d3ee'; c.beginPath(); c.moveTo(kx, cy - R * 0.45); c.lineTo(kx - R * 0.16, cy + R * 0.3); c.lineTo(kx, cy + R * 0.15); c.lineTo(kx + R * 0.16, cy + R * 0.3); c.fill();
  c.fillStyle = '#d7e1ea'; c.fillText(`${Math.round(((st.yaw || 0) + 360) % 360)}°`, kx, cy + R * 0.62);
}

// ----------------------------------------------------------- gauges
function gauge(id, val, cls, sub) {
  const g = $(id); g.querySelector('b').textContent = val;
  g.classList.toggle('warn', cls === 'warn'); g.classList.toggle('bad', cls === 'bad');
}

let lastDraw = 0;
on('telemetry', (d) => {
  if (d.drone !== selId()) return;
  const now = performance.now();
  if (now - lastDraw < 90) return;
  lastDraw = now;
  const st = d.state, od = d.odom, s = store.settings.safety || {};
  const bat = st.bat;
  gauge('#gBat', `${bat ?? '–'}%`, bat <= s.battery_land ? 'bad' : bat <= s.battery_warn ? 'warn' : '');
  const bar = $('#gBat .bar div'); bar.style.width = `${bat || 0}%`; bar.style.background = bat <= s.battery_land ? 'var(--danger)' : bat <= s.battery_warn ? 'var(--warn)' : 'var(--ok)';
  gauge('#gAlt', fmt(od.z, 2)); $('#gTof').textContent = `tof ${fmt((st.tof || 0) / 100, 2)} m`;
  gauge('#gSpd', fmt(od.speed, 2)); $('#gVz').textContent = `vz ${fmt(-(st.vgz || 0) / 10, 1)} m/s`;
  gauge('#gTemp', `${st.templ ?? '–'}–${st.temph ?? '–'}`, st.temph >= s.temp_warn ? 'warn' : ''); $('#gBaro').textContent = `baro ${fmt(st.baro, 2)} m`;
  gauge('#gHdg', `${Math.round(((st.yaw || 0) + 360) % 360)}°`); $('#gAtt').textContent = `p ${st.pitch ?? '–'}° r ${st.roll ?? '–'}°`;
  const rng = Math.hypot(od.x, od.y);
  gauge('#gDist', fmt(rng, 1), s.geofence_enabled && rng > s.geofence_radius_m * 0.85 ? 'warn' : ''); $('#gTrip').textContent = `trip ${fmt(od.dist, 1)} m`;
  drawADI(st);
  const hh = store.hist[d.drone];
  for (const k in small) small[k].draw(hh[k]);
  if ($('#pane-telemetry').classList.contains('active')) {
    for (const k in big) big[k].draw(hh[k]);
    for (const k in st) { const el = document.getElementById(`tv-${k}`); if (el) el.textContent = st[k]; }
    $('#teleRaw').textContent = JSON.stringify({ state: st, odom: od }, null, 0);
  }
});
on('select', () => { for (const k in small) small[k].draw(store.hist[selId()]?.[k] || []); });

// ----------------------------------------------------------- settings
const LABELS = {
  battery_warn: 'Battery warning %', battery_land: 'Battery auto-land %', temp_warn: 'Temperature warning °C', link_timeout_s: 'Link-lost timeout s',
  geofence_enabled: 'Geofence enabled', geofence_radius_m: 'Geofence radius m', geofence_max_alt_m: 'Geofence max altitude m',
  geofence_action: 'Geofence action (hover|land)', client_loss_action: 'All browsers closed (hover|land|none)', client_loss_land_s: 'Land after client loss s',
  require_preflight: 'Block takeoff on failed pre-flight', rate_hz: 'rc rate Hz', max: 'Stick limit %', expo: 'Gamepad expo 0-1', deadzone: 'Gamepad deadzone',
  deadman_ms: 'Dead-man timeout ms', velocity_scale_cm: 'vgx/vgy unit → cm/s', velocity_frame: 'Velocity frame (world|body)',
  jpeg_quality: 'Browser JPEG quality', max_fps: 'Browser max fps', relay_udp: 'Raw H.264 relay (host:port, comma)',
  auto_log_on_takeoff: 'Auto flight log on takeoff', auto_video_on_takeoff: 'Auto video on takeoff',
  collision_guard: 'Collision guard (brake on low TTC)', dynamic_obstacle_brake: 'Brake for moving obstacles in path', smart_rth: 'Smart RTH battery (off|warn|rth)',
  hfov_deg: 'Camera horizontal FOV °', tilt_deg: 'Camera tilt down °', known_height_m: 'Known target heights m (JSON)',
  confirm_hits: 'Hits to confirm a target', confirm_sigma_m: 'Max σ to confirm (m)', gate_m: 'Association gate m', max_obs_hz: 'Observations / s per target', min_conf: 'Min detection confidence',
  speed_cms: 'RTH / go-to speed cm/s', rdp_eps_m: 'Trail simplification m', reserve_pct: 'RTH battery reserve %', default_drain_pct_s: 'Default drain %/s',
  enabled: 'Enabled', cell_m: 'Cell size m', range_m: 'Camera sweep range m',
  min_move_m: 'Keyframe every … m', min_turn_deg: '… or every … ° yaw', min_interval_s: 'Min interval s', max_interval_s: 'Max interval s', min_sharpness: 'Min sharpness (Laplacian var)',
};
let draft = {};
function buildSettings() {
  const s = store.settings; draft = structuredClone(s);
  const box = $('#settingsForm'); box.replaceChildren();
  for (const [sec, vals] of Object.entries(s)) {
    box.append(h('h4', {}, sec));
    for (const [k, v] of Object.entries(vals)) {
      const set = (x) => { draft[sec][k] = x; };
      let inp;
      if (typeof v === 'boolean') inp = h('input', { type: 'checkbox', checked: v, onchange: (e) => set(e.target.checked) });
      else if (typeof v === 'number') inp = h('input', { type: 'number', value: v, step: 'any', onchange: (e) => set(Number(e.target.value)) });
      else if (Array.isArray(v)) inp = h('input', { value: v.join(', '), onchange: (e) => set(e.target.value.split(',').map(x => x.trim()).filter(Boolean)) });
      else if (v && typeof v === 'object') inp = h('input', { class: 'mono', value: JSON.stringify(v), onchange: (e) => { try { set(JSON.parse(e.target.value)); e.target.style.borderColor = ''; } catch { e.target.style.borderColor = 'var(--danger)'; } } });
      else inp = h('input', { value: v, onchange: (e) => set(e.target.value) });
      box.append(h('label', { title: `${sec}.${k}` }, LABELS[k] || k), inp);
    }
  }
}
$('#setSave').onclick = async () => {
  try { store.settings = await api('/settings', draft); toast('Settings saved', 'success'); emit('settings', store.settings); }
  catch (e) { toast(e.message, 'error'); }
};
$('#setReset').onclick = async () => {
  if (!confirm('Reset all settings to defaults?')) return;
  store.settings = await api('/settings/reset', {}); buildSettings(); emit('settings', store.settings); toast('Defaults restored');
};
