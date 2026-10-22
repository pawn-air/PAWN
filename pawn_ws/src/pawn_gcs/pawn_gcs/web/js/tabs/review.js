// Flight review: replay a recorded session (trajectory, charts, video, events, snapshots) with a time scrubber.
import { $, api, on, h, fmt, hhmmss, mmss, toast } from '../core.js';
import { MapView } from '../mapview.js';
import { StripChart } from '../chart.js';

let sessions = [], cur = null;

async function load() {
  sessions = await api('/recordings');
  $('#sessions').replaceChildren(...(sessions.length ? sessions.map(s => h('button', {
    class: `sess${cur?.id === s.id ? ' active' : ''}`, onclick: () => open(s),
  }, h('b', {}, s.id), `${s.meta.duration_s != null ? mmss(s.meta.duration_s) : 'recording…'} · ${fmt(s.meta.dist_m, 1)} m · ${s.files.includes('video.mp4') ? '🎞 ' : ''}${s.snapshots.length ? '📷' + s.snapshots.length : ''} · ${(s.size / 1e6).toFixed(1)} MB`))
    : [h('div', { class: 'small muted' }, 'No flight logs yet. Logs start automatically on takeoff (Settings → recording).')]));
}

function parseCSV(text) {
  const [head, ...rows] = text.trim().split(/\r?\n/);
  const cols = head.split(',');
  return rows.map(r => { const v = r.split(','); const o = {}; cols.forEach((c, i) => { const n = Number(v[i]); o[c] = v[i] === '' ? null : Number.isNaN(n) ? v[i] : n; }); return o; });
}

async function open(s) {
  cur = s; load();
  const base = `/api/recordings/${encodeURIComponent(s.id)}/`;
  const body = $('#reviewBody'); body.classList.remove('muted');
  body.replaceChildren(h('div', { class: 'small muted' }, 'loading…'));
  let rows = [], events = [];
  try { rows = parseCSV(await (await fetch(base + 'telemetry.csv')).text()); } catch {}
  try { events = (await (await fetch(base + 'events.jsonl')).text()).trim().split('\n').filter(Boolean).map(l => JSON.parse(l)); } catch {}
  let targets = [];
  if (s.files.includes('targets.json')) { try { targets = await (await fetch(base + 'targets.json')).json(); } catch {} }
  const hasVideo = s.files.includes('video.mp4');
  const mapCv = h('canvas');
  const video = hasVideo ? h('video', { src: base + 'video.mp4', controls: true, muted: true, preload: 'auto' }) : h('div', { class: 'muted small', style: { display: 'grid', placeItems: 'center' } }, 'no video recorded in this session');
  const charts = h('div', { class: 'charts', style: { gridTemplateColumns: 'repeat(auto-fill, minmax(220px, 1fr))' } });
  const slider = h('input', { type: 'range', min: 0, max: Math.max(1, rows.length - 1), value: 0 });
  const info = h('div', { class: 'small mono' });
  const evBox = h('div', { class: 'events', style: { maxHeight: '160px' } });
  const meta = s.meta;
  body.replaceChildren(
    h('div', { class: 'row gap wrap small', style: { marginBottom: '6px' } },
      h('b', {}, s.id), h('span', { class: 'muted' }, `${meta.name || ''} · start ${meta.start ? hhmmss(meta.start) : '?'} · bat ${meta.bat_start ?? '?'}→${meta.bat_end ?? '?'}% · max alt ${fmt(meta.max_alt_m, 2)} m · ${rows.length} samples`),
      h('span', { class: 'grow' }),
      ...s.files.map(f => h('a', { class: 'btn xs ghost', href: base + f, download: f }, '⇩ ' + f)),
      h('button', { class: 'btn xs ghost', onclick: async () => { if (confirm(`Delete ${s.id}?`)) { await api('/recordings/' + encodeURIComponent(s.id), undefined, 'DELETE'); cur = null; body.replaceChildren(); load(); } } }, 'delete')),
    h('div', { class: 'review-actions' },
      h('a', { class: 'btn xs primary', href: base + 'report.html', target: '_blank', title: 'Printable post-flight report (print → PDF)' }, '📄 Flight report'),
      h('a', { class: 'btn xs', href: base + 'export.zip', title: 'Everything in this session as one zip' }, '⇩ Session .zip'),
      s.dataset ? h('a', { class: 'btn xs', href: base + 'export.zip?what=dataset', title: 'Keyframes + transforms.json (nerfstudio / 3DGS) + COLMAP model' }, `⇩ 3D dataset (${meta.keyframes ?? '?'} keyframes)`) : null,
      targets.length ? h('span', { class: 'small muted' }, `${targets.filter(t => t.status === 'confirmed').length} confirmed / ${targets.length} targets`) : null,
      meta.coverage_m2 != null ? h('span', { class: 'small muted' }, `· explored ${fmt(meta.coverage_m2, 1)} m²`) : null),
    h('div', { class: 'review-top' }, mapCv, video),
    h('div', { class: 'row gap', style: { margin: '6px 0' } }, slider), info, charts,
    s.snapshots.length ? h('div', { class: 'snaps' }, ...s.snapshots.map(f => h('a', { href: base + 'snapshots/' + f, target: '_blank' }, h('img', { src: base + 'snapshots/' + f, loading: 'lazy', title: f })))) : null,
    h('div', { class: 'sub' }, 'Events'), evBox);

  const mv = new MapView(mapCv, { mode: '3d' });
  const pts = rows.filter(r => r.x != null).map(r => [r.x, r.y, r.z]);
  mv.layers.tracks.rec = { pts, color: '#22d3ee' };
  mv.layers.markers = [...targets, ...events.filter(e => e.type === 'snapshot' && e.pose).map((e, i) => ({ id: -1 - i, label: '📷', source: 'bookmark', x: e.pose.x, y: e.pose.y, z: e.pose.z, hits: 1 }))];
  setTimeout(() => mv.fit(pts.length ? pts : null), 50);

  const series = (k, f = (x) => x) => rows.map(r => (r[k] == null ? null : f(r[k])));
  const cs = [
    [new StripChart(charts, { title: 'ALT m', color: '#22d3ee', digits: 2 }), series('z')],
    [new StripChart(charts, { title: 'BAT %', color: '#22c55e', digits: 0, min: 0, max: 100 }), series('bat')],
    [new StripChart(charts, { title: 'SPEED m/s', color: '#a3e635', digits: 2, min: 0 }), series('speed')],
    [new StripChart(charts, { title: 'TEMP °C', color: '#f97316', digits: 0 }), series('temph')],
    [new StripChart(charts, { title: 'PITCH/ROLL °', color: '#c084fc', digits: 0 }), series('pitch')],
    [new StripChart(charts, { title: 'RC fb', color: '#fbbf24', digits: 0, min: -100, max: 100 }), series('rc_fb')],
  ];
  const vidStart = events.find(e => e.type === 'event' && /Video recording started/.test(e.msg || ''))?.t;
  const t0 = meta.start || rows[0]?.t;

  const update = (i, fromVideo = false) => {
    const r = rows[i]; if (!r) return;
    const frac = i / Math.max(1, rows.length - 1);
    for (const [c, v] of cs) c.draw(v, frac);
    mv.layers.cursor = [r.x, r.y, r.z]; mv.layers.drones.rec = { pose: { x: r.x, y: r.y, z: r.z, yaw: r.yaw_est }, color: '#fbbf24', label: '' }; mv.draw();
    info.textContent = `t+${fmt(r.t_rel, 1)} s · pos (${fmt(r.x, 2)}, ${fmt(r.y, 2)}, ${fmt(r.z, 2)}) · yaw ${fmt(r.yaw_est, 0)}° · bat ${r.bat}% · mode ${r.mode} · rc [${r.rc_lr} ${r.rc_fb} ${r.rc_ud} ${r.rc_yw}]`;
    if (hasVideo && vidStart && !fromVideo) { const vt = r.t - vidStart; if (vt >= 0 && Math.abs(video.currentTime - vt) > 0.3) video.currentTime = vt; }
  };
  slider.oninput = () => update(Number(slider.value));
  if (hasVideo && vidStart) video.ontimeupdate = () => {
    const t = vidStart + video.currentTime;
    let i = rows.findIndex(r => r.t >= t); if (i < 0) i = rows.length - 1;
    slider.value = i; update(i, true);
  };
  update(0);
  evBox.replaceChildren(...events.filter(e => e.type !== 'detections').map(e => {
    const i = rows.findIndex(r => r.t >= e.t);
    const txt = e.type === 'command' ? `> ${e.cmd}  ${e.resp ?? 'timeout'}` : e.type === 'event' ? e.msg : `${e.type} ${e.state || e.mode || e.file || ''}`;
    return h('div', { class: `ev ${e.level || (e.type === 'command' ? 'cmd' : '')}`, style: { cursor: 'pointer' }, onclick: () => { if (i >= 0) { slider.value = i; update(i); } } },
      h('span', { class: 't' }, `+${fmt((e.t - t0), 1)}s`), txt);
  }));
}

on('tab', (t) => { if (t === 'review') load().catch(e => toast(e.message, 'error')); });
on('recording', () => { if ($('#pane-review').classList.contains('active')) load(); });
