// Live FPV video (JPEG over WebSocket) + HUD overlay (attitude, tapes, detections, warnings).
import { $, store, on, sel, selId, fitCanvas, fmt, mmss, api, toast } from './core.js';

const vcv = $('#video'), hud = $('#hud'), msg = $('#videoMsg');
const vctx = vcv.getContext('2d');
let vws = null, vid = null, lastFrameT = 0, frames = [], latency = null, frameSize = [960, 720];

export const videoStats = () => ({ fps: frames.length, latency });

function connect(id) {
  if (vws) { vws.onclose = null; vws.close(); }
  vid = id;
  if (!id) return;
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/video/' + id;
  vws = new WebSocket(url);
  vws.binaryType = 'arraybuffer';
  vws.onmessage = async (ev) => {
    const buf = ev.data;
    const dv = new DataView(buf);
    const tMs = dv.getFloat64(4, true);
    const bmp = await createImageBitmap(new Blob([buf.slice(12)], { type: 'image/jpeg' }));
    if (vcv.width !== bmp.width || vcv.height !== bmp.height) { vcv.width = bmp.width; vcv.height = bmp.height; }
    vctx.drawImage(bmp, 0, 0);
    frameSize = [bmp.width, bmp.height];
    bmp.close?.();
    const now = performance.now();
    lastFrameT = now;
    frames.push(now); while (frames.length && now - frames[0] > 1000) frames.shift();
    latency = Date.now() - tMs;
    msg.classList.add('hidden');
  };
  vws.onclose = () => setTimeout(() => { if (vid === id) connect(id); }, 1000);
}

on('select', connect);

// ------------------------------------------------------------------ HUD
const C = { fg: '#e2f7ff', acc: '#22d3ee', warn: '#f59e0b', bad: '#ef4444', ok: '#22c55e' };

function txt(c, s, x, y, size, color = C.fg, align = 'left', bold = false) {
  c.font = `${bold ? '600 ' : ''}${size}px ui-monospace, Menlo, Consolas, monospace`;
  c.textAlign = align; c.textBaseline = 'middle';
  c.lineWidth = Math.max(2, size / 5); c.strokeStyle = 'rgba(0,0,0,.75)'; c.strokeText(s, x, y);
  c.fillStyle = color; c.fillText(s, x, y);
}

function drawHUD() {
  requestAnimationFrame(drawHUD);

  const { w, h, dpr } = fitCanvas(hud);
  const c = hud.getContext('2d');
  c.clearRect(0, 0, w, h);
  const id = store.selected, d = sel(), tel = store.tel[id];
  const stale = performance.now() - lastFrameT > 2000;
  if (stale) { msg.classList.remove('hidden'); }
  // video occupies the whole box (4:3 wrap + contain); compute rect anyway for other aspect ratios
  const ar = frameSize[0] / frameSize[1];
  let vw = w, vh = w / ar; if (vh > h) { vh = h; vw = h * ar; }
  const ox = (w - vw) / 2, oy = (h - vh) / 2;
  const cx = ox + vw / 2, cy = oy + vh / 2, s = Math.max(vh / 720, dpr * 0.8);
  const fs = Math.max(10, 13 * s);

  if ($('#gridOn').checked) {
    c.strokeStyle = 'rgba(255,255,255,.18)'; c.lineWidth = 1;
    for (let i = 1; i < 3; i++) {
      c.beginPath(); c.moveTo(ox + vw * i / 3, oy); c.lineTo(ox + vw * i / 3, oy + vh); c.stroke();
      c.beginPath(); c.moveTo(ox, oy + vh * i / 3); c.lineTo(ox + vw, oy + vh * i / 3); c.stroke();
    }
  }

  if ($('#detOn').checked) drawDetections(c, ox, oy, vw, vh, fs);
  if (!$('#hudOn').checked || !tel) return;
  const st = tel.state, od = tel.odom;

  // --- horizon & pitch ladder
  const pxPerDeg = vh / 70;
  c.save(); c.translate(cx, cy); c.rotate(-(st.roll || 0) * Math.PI / 180);
  const pOff = (st.pitch || 0) * pxPerDeg;
  c.strokeStyle = 'rgba(226,247,255,.85)'; c.lineWidth = 2 * s;
  c.beginPath(); c.moveTo(-vw * 0.32, pOff); c.lineTo(-vw * 0.08, pOff); c.moveTo(vw * 0.08, pOff); c.lineTo(vw * 0.32, pOff); c.stroke();
  c.lineWidth = 1 * s;
  for (const a of [-20, -10, 10, 20]) {
    const yy = pOff - a * pxPerDeg;
    c.setLineDash(a < 0 ? [6 * s, 5 * s] : []);
    c.beginPath(); c.moveTo(-vw * 0.12, yy); c.lineTo(-vw * 0.05, yy); c.moveTo(vw * 0.05, yy); c.lineTo(vw * 0.12, yy); c.stroke();
    txt(c, String(a), -vw * 0.14, yy, fs * 0.8, 'rgba(226,247,255,.8)', 'right');
  }
  c.setLineDash([]); c.restore();

  // --- aircraft reference
  c.strokeStyle = C.acc; c.lineWidth = 2.5 * s;
  c.beginPath(); c.moveTo(cx - 40 * s, cy); c.lineTo(cx - 14 * s, cy); c.lineTo(cx - 7 * s, cy + 7 * s);
  c.moveTo(cx + 40 * s, cy); c.lineTo(cx + 14 * s, cy); c.lineTo(cx + 7 * s, cy + 7 * s); c.stroke();
  c.beginPath(); c.arc(cx, cy, 2.5 * s, 0, Math.PI * 2); c.fillStyle = C.acc; c.fill();

  // --- heading tape (Tello yaw: CW+, 0 = power-on heading)
  const hdg = ((st.yaw || 0) + 360) % 360;
  const tw = vw * 0.5, tx = cx - tw / 2, ty = oy + 26 * s;
  c.save(); c.beginPath(); c.rect(tx, ty - 16 * s, tw, 34 * s); c.clip();
  c.fillStyle = 'rgba(0,0,0,.35)'; c.fillRect(tx, ty - 16 * s, tw, 34 * s);
  const pxd = tw / 90;
  for (let a = Math.floor(hdg - 50); a <= hdg + 50; a++) {
    if (a % 5) continue;
    const X = cx + (a - hdg) * pxd, n = ((a % 360) + 360) % 360;
    c.strokeStyle = C.fg; c.lineWidth = 1 * s;
    c.beginPath(); c.moveTo(X, ty + 16 * s); c.lineTo(X, ty + (n % 15 ? 10 : 4) * s); c.stroke();
    if (n % 30 === 0) txt(c, String(n).padStart(3, '0'), X, ty - 5 * s, fs * 0.8, C.fg, 'center');
  }
  c.restore();
  c.fillStyle = C.acc; c.beginPath(); c.moveTo(cx, ty + 18 * s); c.lineTo(cx - 6 * s, ty + 26 * s); c.lineTo(cx + 6 * s, ty + 26 * s); c.fill();
  txt(c, String(Math.round(hdg)).padStart(3, '0') + '°', cx, ty + 36 * s, fs, C.acc, 'center', true);

  // --- speed (left) and altitude (right) boxes
  const bx = (x, label, val, sub, align) => {
    c.fillStyle = 'rgba(0,0,0,.45)'; c.strokeStyle = 'rgba(34,211,238,.6)'; c.lineWidth = 1;
    const bw = 92 * s, bh = 46 * s, X = align === 'left' ? x : x - bw;
    c.fillRect(X, cy - bh / 2, bw, bh); c.strokeRect(X, cy - bh / 2, bw, bh);
    txt(c, label, X + 6 * s, cy - 13 * s, fs * 0.7, '#8fb7c5');
    txt(c, val, X + bw - 6 * s, cy + 3 * s, fs * 1.35, C.fg, 'right', true);
    txt(c, sub, X + bw - 6 * s, cy + 17 * s, fs * 0.7, '#8fb7c5', 'right');
  };
  bx(ox + 14 * s, 'GS m/s', fmt(od.speed, 1), `dist ${fmt(od.dist, 0)}m`, 'left');
  const vz = -(st.vgz || 0) / 10;
  bx(ox + vw - 14 * s, 'ALT m', fmt(od.z, 2), `vz ${vz >= 0 ? '+' : ''}${fmt(vz, 1)}  tof ${fmt((st.tof || 0) / 100, 1)}`, 'right');

  // --- bottom status line
  const bat = st.bat ?? 0, batC = bat <= (store.settings.safety?.battery_land ?? 15) ? C.bad : bat <= (store.settings.safety?.battery_warn ?? 30) ? C.warn : C.ok;
  const by = oy + vh - 18 * s;
  txt(c, `BAT ${bat}%`, ox + 14 * s, by, fs, batC, 'left', true);
  txt(c, `${d?.mode || ''}${d?.flying ? ' · AIR' : ' · GND'}`, ox + 110 * s, by, fs, d?.mode === 'AUTO' ? '#c4b5fd' : d?.mode === 'MISSION' ? '#93c5fd' : C.fg);
  txt(c, `T+${mmss(d?.flight_time || 0)}`, ox + 260 * s, by, fs, C.fg);
  txt(c, `${frames.length}fps ${latency != null ? Math.max(0, Math.round(latency)) + 'ms' : ''}`, ox + vw - 14 * s, by, fs * 0.85, '#8fb7c5', 'right');
  txt(c, `x ${fmt(od.x, 1)}  y ${fmt(od.y, 1)}  ${od.source === 'external' ? 'SLAM' : 'DR'}`, ox + vw - 14 * s, by - 18 * s, fs * 0.8, '#8fb7c5', 'right');

  // rc stick indicator
  const rc = d?.rc || [0, 0, 0, 0];
  const mini = (X, a, b) => {
    const r = 16 * s; c.strokeStyle = 'rgba(226,247,255,.5)'; c.lineWidth = 1; c.strokeRect(X - r, by - 46 * s - r, 2 * r, 2 * r);
    c.fillStyle = C.acc; c.beginPath(); c.arc(X + a / 100 * r, by - 46 * s - b / 100 * r, 3 * s, 0, 7); c.fill();
  };
  mini(ox + 34 * s, rc[3], rc[2]); mini(ox + 74 * s, rc[0], rc[1]);

  // --- warnings
  const warns = [];
  if (d && d.connected && !d.link_alive) warns.push(['TELEMETRY LOST', C.bad]);
  if (bat && bat <= (store.settings.safety?.battery_warn ?? 30)) warns.push([`LOW BATTERY ${bat}%`, batC]);
  if ((st.temph || 0) >= (store.settings.safety?.temp_warn ?? 85)) warns.push([`HIGH TEMP ${st.temph}°C`, C.warn]);
  const col = d?.collision || {};
  if (col.danger) warns.push([`⚠ BRAKE — ${col.reason || 'obstacle'}`, C.bad]);
  const nav = d?.nav || {};
  if (d?.flying && nav.bat_needed != null && bat - nav.bat_needed <= (store.settings.safety?.battery_land ?? 15) + 5) warns.push([`RETURN HOME — needs ~${Math.round(nav.bat_needed)}%`, C.warn]);
  if (d?.mission?.name === 'RTH' && ['running', 'paused'].includes(d?.mission?.state)) warns.push(['RTH IN PROGRESS', '#22c55e']);
  const fr = store.settings.safety?.geofence_radius_m;
  if (fr && Math.hypot(od.x, od.y) > fr * 0.85) warns.push(['NEAR GEOFENCE', C.warn]);
  warns.forEach(([t, col], i) => {
    const blink = Math.floor(performance.now() / 500) % 2 || col !== C.bad;
    if (blink) txt(c, t, cx, oy + vh * 0.28 + i * fs * 1.5, fs * 1.25, col, 'center', true);
  });
  if (col.ttc_s != null && col.ttc_s < 6) txt(c, `TTC ${fmt(col.ttc_s, 1)}s`, cx, cy + 60 * s, fs * 0.9, col.danger ? C.bad : '#8fb7c5', 'center', true);
  if (d?.flying && nav.path_m != null) txt(c, `⌂ ${fmt(nav.path_m, 1)}m trail · ~${fmt(nav.bat_needed, 0)}%`, ox + vw - 14 * s, by - 36 * s, fs * 0.8, '#8fb7c5', 'right');
  if ($('#markOn').checked) {
    txt(c, '✚ MARK MODE — click the target', cx, oy + vh - 46 * s, fs, '#fbbf24', 'center', true);
    if (mouse) { c.strokeStyle = '#fbbf24'; c.lineWidth = 1.5 * dpr; const r = 14 * s;
      c.beginPath(); c.arc(mouse[0] * dpr, mouse[1] * dpr, r, 0, 7); c.moveTo(mouse[0] * dpr - r * 1.6, mouse[1] * dpr); c.lineTo(mouse[0] * dpr + r * 1.6, mouse[1] * dpr);
      c.moveTo(mouse[0] * dpr, mouse[1] * dpr - r * 1.6); c.lineTo(mouse[0] * dpr, mouse[1] * dpr + r * 1.6); c.stroke(); }
  }
  if (d?.recording?.video) {
    if (Math.floor(performance.now() / 600) % 2) { c.fillStyle = C.bad; c.beginPath(); c.arc(ox + 22 * s, oy + 22 * s, 6 * s, 0, 7); c.fill(); }
    txt(c, 'REC', ox + 34 * s, oy + 22 * s, fs, C.bad, 'left', true);
  }
}

function drawDetections(c, ox, oy, vw, vh, fs) {
  const dets = store.det[store.selected] || {};
  const now = Date.now() / 1000;
  for (const d of Object.values(dets)) {
    if (!d.items?.length || now - d.t > 1.0) continue;
    const col = d.color || '#22d3ee';
    c.strokeStyle = col; c.lineWidth = 2;
    for (const it of d.items) {
      if (it.poly) {
        c.beginPath(); it.poly.forEach(([x, y], i) => (i ? c.lineTo : c.moveTo).call(c, ox + x * vw, oy + y * vh)); c.closePath(); c.stroke();
      }
      if (it.box) {
        const [x, y, w, h] = it.box, X = ox + x * vw, Y = oy + y * vh, W = w * vw, H = h * vh;
        if (!it.poly) c.strokeRect(X, Y, W, H);
        const extra = it.info?.range_m != null ? ` ${it.info.range_m}m` : '';
        txt(c, `${it.label} ${Math.round((it.conf ?? 1) * 100)}%${extra}`, X, Y - fs * 0.7, fs * 0.85, col);
      } else if (it.label === 'quality') {
        const q = it.info; txt(c, `Q sharp ${q.sharpness} feat ${q.orb_features} ${q.keyframe_ok ? 'KF✓' : 'KF✗'}`, ox + 14, oy + vh * 0.12, fs * 0.8, q.keyframe_ok ? C.ok : C.warn);
      }
    }
  }
}

requestAnimationFrame(drawHUD);

// ------------------------------------------------------------------ click-to-mark
let mouse = null;
/** Normalised image coordinates (0..1) of a pointer event on the video box, or null outside the image. */
export function uvFromEvent(e) {
  const r = hud.getBoundingClientRect();
  const ar = frameSize[0] / frameSize[1];
  let vw = r.width, vh = r.width / ar; if (vh > r.height) { vh = r.height; vw = r.height * ar; }
  const ox = (r.width - vw) / 2, oy = (r.height - vh) / 2;
  const u = (e.clientX - r.left - ox) / vw, v = (e.clientY - r.top - oy) / vh;
  return u >= 0 && u <= 1 && v >= 0 && v <= 1 ? [u, v] : null;
}
const wrap = $('#videoWrap');
wrap.addEventListener('pointermove', (e) => { const r = hud.getBoundingClientRect(); mouse = [e.clientX - r.left, e.clientY - r.top]; });
wrap.addEventListener('pointerleave', () => { mouse = null; });
wrap.addEventListener('click', async (e) => {
  if (!$('#markOn').checked || e.target.closest('.video-tools')) return;
  const uv = uvFromEvent(e); if (!uv) return;
  const label = (store.markLabel ||= 'victim');
  try {
    const m = await api(`/drones/${selId()}/mark`, { u: uv[0], v: uv[1], label });
    toast(`✚ ${m.label} #${m.id}: ${m.hits} ray${m.hits > 1 ? 's' : ''} · ${m.method}${m.method === 'bearing' ? ' — move sideways and mark again to triangulate' : ` ±${fmt(m.sigma, 2)} m`}`, 'info', 5000);
  } catch (err) { toast(err.message, 'error'); }
});
$('#markOn').addEventListener('change', (e) => { wrap.classList.toggle('marking', e.target.checked); });

$('#btnFull').onclick = () => {
  const el = $('#videoWrap');
  if (document.fullscreenElement) document.exitFullscreen(); else el.requestFullscreen?.();
};
